// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C Controller transceiver.
//
// - SCL-driving I3C transmitter/receiver.
// - This is presently Controller only, since HDR-BT mode is not supported.

module i3c_controller_trx
  import i3c_pkg::*;
#(
  // Number of SDA lanes; must be one presently since HDR-BT mode not supported.
  parameter int unsigned NumSDALanes = 1,
  parameter int unsigned DataWidth = 32,
  parameter int unsigned CntWidth = 16
) (
  input                         clk_i,
  input                         rst_ni,

  input                         sw_reset_i,

  input                         tx_enable_i,
  input                         rx_enable_i,
  input                         ddr_enable_i,

  // Timing parameters.
  input          [CntWidth-1:0] clkdiv_i,
  // TODO: Choose suitable dimensions.
  input          [CntWidth-1:0] tcas_i,
  input          [CntWidth-1:0] tcbp_i,

  // Control inputs.
  input                         gen_hdr_exit_i,
  input                         gen_hdr_sr_i,

  // Buffer reading.
  output logic                  buf_rd_o,
  input  i3c_dtype_e            buf_rtype_i,
  input         [DataWidth-1:0] buf_rdata_i,
  input                         buf_empty_i,

  // Buffer writing.
  output logic                  buf_wr_o,
  output      [DataWidth/8-1:0] buf_wmask_o,
  output        [DataWidth-1:0] buf_wdata_o,

  // I3C I/O signaling.
  output                        scl_o,
  output                        scl_pp_en_o,
  output                        scl_od_en_o,
  input       [NumSDALanes-1:0] sda_i,
  output      [NumSDALanes-1:0] sda_o,
  output                        sda_pp_en_o,
  output                        sda_od_en_o,

  // Pullup enables.
  output                      scl_pu_en_o,
  output                      sda_pu_en_o
);

  localparam int unsigned Log2DW = $clog2(DataWidth);

  // Current SDA output, when driving.
  logic sda_q;
  // Data buffer.
  logic [DataWidth-1:0] buf_q;

  //  Enable for transmission/reception logic; for reduced power consumption.
  wire enable = tx_enable_i | rx_enable_i;

  // SCL/SDA_<x>_En are driven directly from bits of the state variable to eliminate
  // glitches that could otherwise result from any combinational logic.
  localparam int unsigned StBit_SCL_PP_En = 1;
  localparam int unsigned StBit_SCL       = 0;

  typedef enum logic [5:0] {
    State_Idle            = 6'b0000_0_1,
    State_PreStart        = 6'b0000_1_1,
    State_Start           = 6'b0000_1_0,
    State_Stop            = 6'b0001_1_1,
    State_PreStop         = 6'b0001_1_0,

    // --- Address arbitration ---
    //
    // Following START in SDR mode, or START/Repeated START in I2C mode.
    State_AddrClkLoSetupD = 6'b0010_1_0,
    State_AddrClkHiHoldD  = 6'b0010_1_1,
    State_AddrClkHiCheckD = 6'b0011_1_1,
    State_AddrClkLoIdle   = 6'b0011_1_0,

    // --- Word Transmission ---

    // SCK low->high transition.
    State_ClkLoSetupD     = 6'b0110_1_0,
    State_ClkHiHoldD      = 6'b0110_1_1,
    // SCK high->low transition.
    State_ClkHiSetupD     = 6'b0111_1_1,
    State_ClkLoHoldD      = 6'b0111_1_0,

    // --- Word reception ---

    // SCK low->high transition
    State_ClkLoAwaitD     = 6'b1000_1_0,
    State_ClkHiSampleD    = 6'b1000_1_1,
    // SCK high->low transition
    State_ClkHiAwaitD     = 6'b1001_1_1,
    State_ClkLoSampleD    = 6'b1001_1_0,

    // --- HDR Exit/Restart ---
    State_HDRExRstHiD     = 6'b1010_1_0,
    State_HDRExRstLoD     = 6'b1011_1_0
  } state_e;

  // Transceiver state.
  state_e state_q, state_d;
  logic [CntWidth-1:0] cnt;
  logic [Log2DW-1:0] bits_left;
  wire phase_end = ~|cnt;

  // Shift buffered output data?
  wire tx_shift = |{state_q == State_ClkHiHoldD && ddr_enable_i,
                    state_q == State_ClkLoHoldD,
                    state_q == State_AddrClkLoIdle,
                    state_q == State_Start} & phase_end;
  wire rx_shift = |{state_q == State_ClkHiSampleD,
                    state_q == State_ClkLoSampleD && ddr_enable_i} & phase_end;
  wire buf_shift = tx_shift | rx_shift;

  logic starting, stopping, pre_stop, stop;
  logic last_bit;
  logic advance;
  assign advance  = enable & phase_end;
  assign last_bit = ~|bits_left;
  assign starting = &{advance, state_q == State_Idle, rx_enable_i | !buf_empty_i};
  // When the final bit has been stable long enough, move into the PreStop state.
  assign stopping = &{advance, state_q == State_ClkLoHoldD || state_q == State_ClkLoSampleD,
                      last_bit};
  // We must lower SDA before we can raise it for STOP signaling.
  assign pre_stop = &{advance, state_q == State_PreStop};
  // STOP signaling; SDA raised but SCL still driven low.
  assign stop     = &{advance, state_q == State_Stop};

  // Generation of HDR Exit/Restart signaling.
  logic [2:0] hdr_patt_cnt;
  wire gen_hdr_patt = gen_hdr_exit_i | gen_hdr_sr_i;
  wire leave_hdr_exit = gen_hdr_exit_i & ~|hdr_patt_cnt;
  wire leave_hdr_sr   = gen_hdr_sr_i   & ~|hdr_patt_cnt;

  // The two bits used in the parity/CRC-5 calculation.
  logic msbit;  // Transmitted/received first, before `lsbit`.
  logic lsbit;
  assign msbit = tx_enable_i ? buf_q[DataWidth-1] : buf_q[0];
  assign lsbit = tx_enable_i ? buf_q[DataWidth-2] : sda_i;

  // Parity calculated on transmitted/received data.
  logic       parity_sdr;
  logic [1:0] parity_q;  // {PA1, PA0}
  logic [1:0] parity_d;
  assign parity_d = parity_q ^ {msbit, lsbit};
  // SDR employs odd parity across all 8 data bits.
  assign parity_sdr = ^parity_q;

  // CRC-5 calculated on transmitted/received data.
  logic [4:0] crc5_q;
  logic [4:0] crc5_d;
  assign crc5_d = {crc5_q[2],
                   crc5_q[1] ^ msbit ^ crc5_q[4],
                   crc5_q[0] ^ lsbit ^ crc5_q[3],
                   crc5_q[4] ^ msbit,
                   crc5_q[3] ^ lsbit};

  // Shift the input data so that the first bit is ready for output.
  logic [Log2DW-1:0] d_bits_m1;
  i3c_dtype_e dtype_q;
  always_comb begin
    d_bits_m1 = Log2DW'(8);
    case (buf_rtype_i)
      I3CType_DWORD:        d_bits_m1 = '1; // TODO: Development aid.
      I3CType_CommandWord,
      I3CType_DataWord,
      I3CType_ChecksumWord: d_bits_m1 = Log2DW'(17);
      default:              d_bits_m1 = Log2DW'(8);  // includes I3CType_SDRByte/Address.
    endcase
  end

  // TODO: This is going to be recruited for the purposes of CRC-5 and parity transmission too.
  wire [Log2DW-1:0] sh_amount = Log2DW'(DataWidth - 1) - d_bits_m1;
  wire [DataWidth-1:0] buf_rdata_sh = buf_rdata_i << sh_amount;

  // TODO: Parity injection.
  wire emit_pa1 = 1'b0;
  wire emit_pa0 = 1'b0;

  // Input to data buffer.
  logic [DataWidth-1:0] buf_d;
  always_comb begin
    // Shift/load the transmit/receive buffer.
    buf_d[DataWidth-1:1] = (state_q == State_Start) ? buf_rdata_sh[DataWidth-2:0]
                                                    : buf_q[DataWidth-2:0];
    // When emitting SDR write data or DDR-HDR words, we shall need to inject the calculated parity.
    if (emit_pa1) buf_d[DataWidth-1] = parity_q[1];
    if (emit_pa0) buf_d[DataWidth-2] = parity_q[0];
    // TODO: ...and the CRC-5!

    // SDA input value, for read data but also for address arbitration; otherwise '1'.
    // TODO: this should be properly synchronized with our generated clock, but CDC may need
    // either to be waived or resolved.
    buf_d[0] = tx_enable_i | sda_i;
  end

  // Data buffering.
  logic buf_rd_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dtype_q   <= I3CType_Address;
      buf_rd_q  <= 1'b0;
      sda_q     <= '1;
      buf_q     <= '0;
    end else if (sw_reset_i) begin
      buf_rd_q  <= 1'b0;
      sda_q     <= 1'b1;
    end else if (enable) begin
      // The MSB drives SDA directly and must have the appropriate state for START and STOP
      // signaling.
      if (&{state_q == State_HDRExRstLoD || (state_q == State_ClkLoHoldD && last_bit),
            phase_end, !leave_hdr_exit}) begin
        sda_q <= 1'b1;
      end else if (state_q == State_HDRExRstHiD && phase_end) sda_q <= 1'b0;
      else if (|{starting, stopping, pre_stop, stop}) sda_q <= stop;
      else if (tx_shift) begin
        sda_q <= (state_q == State_Start && phase_end) ? buf_rdata_sh[DataWidth-1]
                                                       : buf_q[DataWidth-1];
      end
      // Shift the transmit/receive buffer and sample the SDA input.
      // TODO: See whether we can read in the arbitrated address at the same time as writing out?
      if (buf_shift) buf_q <= buf_d;
      // Retain the type of data unit being transferred.
      if (buf_rd_q) dtype_q <= buf_rtype_i;
      // Read data is returned a cycle after the request.
      buf_rd_q  <= buf_rd_o;
    end else begin
      buf_rd_q <= 1'b0;
    end
  end

  assign buf_rd_o = &{tx_enable_i, state_q == State_PreStart, phase_end, !buf_empty_i};

  // Transceiver state.
  always_comb begin
    state_d = State_Idle;
    case (state_q)
      State_Idle:         state_d = starting ? State_PreStart : State_Idle;
      State_PreStart:     state_d = State_Start;
      State_Start:        state_d = tx_enable_i ?
        (buf_rtype_i == I3CType_Address ? State_AddrClkLoSetupD : State_ClkLoSetupD)
                                                : State_ClkLoAwaitD;
      // --- Address Arbitration ---
      State_AddrClkLoSetupD: state_d = State_AddrClkHiHoldD;
      State_AddrClkHiHoldD:  state_d = State_AddrClkHiCheckD;
      State_AddrClkHiCheckD: state_d = State_AddrClkLoIdle;
      State_AddrClkLoIdle:   state_d = last_bit ? State_PreStop : State_AddrClkLoSetupD;

// TODO: Word Tx/Rx probably to be combined?

      // --- Word Transmission ---
      State_ClkLoSetupD:  state_d = State_ClkHiHoldD;
      State_ClkHiHoldD:   state_d = State_ClkHiSetupD;
      State_ClkHiSetupD:  state_d = State_ClkLoHoldD;
      State_ClkLoHoldD:   state_d = last_bit ? (gen_hdr_patt ? State_HDRExRstHiD : State_PreStop)
                                             : State_ClkLoSetupD;
      // --- Word Reception ---
      State_ClkLoAwaitD:  state_d = State_ClkHiSampleD;
      State_ClkHiSampleD: state_d = State_ClkHiAwaitD;
      State_ClkHiAwaitD:  state_d = State_ClkLoSampleD;
      State_ClkLoSampleD: state_d = last_bit ? State_PreStop : State_ClkLoAwaitD;

      // --- HDR Exit/Restart ---
      // TODO: Restart handling incomplete.
      State_HDRExRstHiD:  state_d = leave_hdr_sr   ? State_Idle : State_HDRExRstLoD;
      State_HDRExRstLoD:  state_d = leave_hdr_exit ? State_Stop : State_HDRExRstHiD;
      // --- SDR STOP ---
      State_PreStop:      state_d = State_Stop;
      // The default case handles State_Stop as well as invalid states.
      default:            state_d = State_Idle;
    endcase
  end

  // Duration of the next state, in clock cycles.
  logic [CntWidth-1:0] tm_cycles;
  always_comb begin
    case (state_q)
      State_Idle:        tm_cycles = tcas_i;
      // HDR Exit/Restart signaling requires 2 cycles per state.
      State_ClkLoHoldD:  tm_cycles = last_bit ? (gen_hdr_patt ? 'b1 : tcbp_i) : clkdiv_i;
      State_HDRExRstHiD: tm_cycles = 'b1;
      State_HDRExRstLoD: tm_cycles = 'b1;
      default:           tm_cycles = clkdiv_i;
    endcase
  end

  logic rx_sample_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      crc5_q    <= '1;
      parity_q  <= 2'b01;
      cnt       <= '0;
      state_q   <= State_Idle;
      bits_left <= '1;
      buf_wr_o  <= 1'b0;
      hdr_patt_cnt  <= '1;
    end else if (sw_reset_i) begin
      crc5_q    <= '1;
      parity_q  <= 2'b01;
      cnt       <= '0;
      state_q   <= State_Idle;
      bits_left <= '1;
      buf_wr_o  <= 1'b0;
    end else if (enable) begin
      // Counting of bits within data unit.
      if (buf_rd_q) begin
        bits_left <= d_bits_m1;
      end else if (phase_end) begin
        bits_left <= bits_left - Log2DW'(buf_shift);
      end

      if (phase_end) begin
        // Update CRC-5 and parity on every other bit shifted out.
        if (buf_shift & bits_left[0]) begin
          crc5_q   <= crc5_d;
          parity_q <= parity_d;
        end
        // HDR pattern counting.
        case (state_q)
          State_ClkLoHoldD:  hdr_patt_cnt <= gen_hdr_exit_i ? 3'b100 : 3'b010;
          State_HDRExRstHiD: hdr_patt_cnt <= hdr_patt_cnt - 'b1;
          default: /* Do nothing */;
        endcase
        // Advance signaling state machine.
        state_q   <= state_d;
        cnt       <= tm_cycles;
      end else begin
        cnt <= cnt - 'b1;
      end
      buf_wr_o <= rx_shift & last_bit;
    end else begin
      buf_wr_o <= 1'b0;
    end
  end

  // Output Drivers.
  logic sda_pp_en_q;  // SDA Push-Pull driver enable.
  logic sda_od_en_q;  // SDA Open Drain driver enable.
  logic sda_pu_en_q;  // SDA Pullup enable.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // We must have no impact upon the bus post-reset until our role has been decided.
      // TODO: PrimaryController parameter?
      sda_pp_en_q   <= 1'b0;
      sda_od_en_q   <= 1'b0;
      sda_pu_en_q   <= 1'b0;
    end else if (enable) begin
      if (phase_end) begin
        case (state_q)
          State_Idle: if (starting) begin
            sda_pp_en_q <= 1'b0;
            sda_od_en_q <= 1'b1;
            sda_pu_en_q <= 1'b1;
          end

          // Address arbitration following a START employs open drain signaling, but the address
          // phase following a Repeated START in SDR mode employs push-pull signaling because
          // arbitration shall not occur.
          //
          // Note: Presently we do not try to optimize the driving of non-initial bits in SDR mode
          // address arbitration.
          State_AddrClkHiCheckD: begin
          end
          State_AddrClkLoIdle,
          State_ClkLoSampleD: begin
            if (phase_end & last_bit) begin
              // TODO: If at this point we were continuing to transmit then we'd of course enable pp
              sda_pu_en_q <= 1'b0;
              sda_od_en_q <= 1'b0;
            end
            // TODO: Loss of arbitration.
          end

          State_ClkLoHoldD: begin
            if (phase_end & last_bit & !gen_hdr_patt) begin
              // TODO: If at this point we were continuing to transmit then we'd of course enable pp
              sda_pu_en_q <= 1'b0;
              sda_od_en_q <= 1'b0;
            end
          end
          State_Stop: begin
            sda_pu_en_q <= 1'b1;
            sda_pp_en_q <= 1'b0;
            sda_od_en_q <= 1'b0;
          end

          State_HDRExRstLoD: if (leave_hdr_sr) begin
            sda_pu_en_q <= 1'b1;
            sda_pp_en_q <= 1'b0;
            sda_od_en_q <= 1'b0;
          end

          // These states do not change the state of the driver enables and must be listed
          // explicitly.
          State_PreStart,
          State_Start,
          State_PreStop,
          State_AddrClkLoSetupD,
          State_AddrClkHiHoldD,
          State_ClkLoSetupD,
          State_ClkHiHoldD,
          State_ClkHiSetupD,
          State_ClkLoHoldD,
          State_ClkLoAwaitD,
          State_ClkHiSampleD,
          State_ClkHiAwaitD,
          State_ClkLoSampleD,
          State_HDRExRstHiD: /* Do nothing */;

          // Handle invalid states; relinquish everything but the pullup.
          // TODO: And that too if we're not the active controller.
          default: begin
            sda_pu_en_q <= 1'b1;
            sda_pp_en_q <= 1'b0;
            sda_od_en_q <= 1'b0;
          end
        endcase
      end
    end else begin
      sda_pu_en_q <= 1'b1;
      sda_pp_en_q <= 1'b0;
      sda_od_en_q <= 1'b1;
    end
  end

  assign buf_wmask_o = '1;  // TODO
  assign buf_wdata_o = buf_q;

  assign scl_o       = state_q[StBit_SCL];
  assign sda_o       = sda_q;
  assign scl_pp_en_o = state_q[StBit_SCL_PP_En];
  assign scl_od_en_o = 1'b0;  // Not required presently because HDR-BT is not supported.
  assign sda_pp_en_o = sda_pp_en_q;
  assign sda_od_en_o = sda_od_en_q;

  // Drive pullup enables.
  // - These need to be driven when open drain signaling is employed.
  assign scl_pu_en_o = 1'b1;  //scl_pu_en_q;
  assign sda_pu_en_o = sda_pu_en_q;

endmodule
