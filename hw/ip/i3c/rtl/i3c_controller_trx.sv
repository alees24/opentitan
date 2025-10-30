// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C Controller transceiver.
//
// - SCL-driving I3C transmitter/receiver.
// - This is presently Controller-side only, since HDR-BT mode is not supported.

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

  input                         enable_i,

  // Request from controller logic.
  // TODO: Perhaps these want renaming from our perspective? f2t, t2f
  input                         trx_dvalid_i,
  output                        trx_dready_o,
  input  i3c_ctrl_trx_req_t     trx_dreq_i,

  // Response to controller logic.
  output                        trx_rvalid_o,
  output i3c_ctrl_trx_rsp_t     trx_rsp_o,

  // Timing parameters.
  input          [CntWidth-1:0] clkdiv_i,
  // TODO: Choose suitable dimensions.
  input          [CntWidth-1:0] tcas_i,
  input          [CntWidth-1:0] tcbp_i,

  // I3C I/O signaling.
  output                        scl_o,
  output                        scl_pp_en_o,
  output                        scl_od_en_o,
  input       [NumSDALanes-1:0] sda_i,
  output      [NumSDALanes-1:0] sda_o,
  output                        sda_pp_en_o,
  output                        sda_od_en_o,

  // Pullup enables.
  output                        scl_pu_en_o,
  output                        sda_pu_en_o
);

  import i3c_consts_pkg::*;

  localparam int unsigned Log2DW = $clog2(DataWidth);

  // Current SDA output, when driving.
  logic sda_q;
  // Data buffer.
  logic [DataWidth-1:0] buf_q;
  // Number of units read.
  logic [1:0] rlen;
  // TODO: Each data unit is transferred individually at present, but this is a rather wasteful use
  // of buf_q on the receive path.
  assign rlen = '0;

  // SCL/SCL_PP_En are driven directly from bits of the state variable to eliminate glitches that
  // could otherwise result from any combinational logic.
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
  // Final cycle of the current phase/state; potentially about to perform a state transition.
  wire phase_end = ~|cnt;
  logic phase_trans;
  // Index of bit within the current transmission unit, counting down to 0 because the MSB is sent
  // is first.
  logic [Log2DW-1:0] bit_idx;
  // Is the current output bit part of the payload, i.e. it contributes to parity and/or CRC-5?
  logic data_bit_q;
  // Captured request properties.
  typedef struct packed {
    i3c_xfer_mode_e mode;   // Signaling mode.
    i3c_dtype_e     dtype;  // Data type of current transfer.
    logic           rx;     // Reception, not transmission.
    logic           i2c;    // Interpret `mode` as I2C rather than I3C.
    logic [1:0]     len;    // Read/write length (bytes minus 1).
  } req_t;
  req_t req_q;

  function automatic logic ddr_request(i3c_xfer_mode_e mode, logic i2c);
    return (mode == XferMode_HDRDDR & !i2c);
  endfunction

  logic ddr_word;  // The _next_ unit is a DDR word.
  logic data_bit;  // The _next_ bit is a data bit within the word.

  // Starting a new request?
  wire req_starting = trx_dvalid_i & trx_dready_o;
  // TODO: Better name.
  wire req_repeating = &{phase_trans, ~|bit_idx, |req_q.len};
  // Starting an HDR-DDR request? (Command Word, Data Word or CRC Word.)
  wire ddr_starting = req_starting & ddr_word;
  // Start a Command Word?
  wire cmd_starting = req_starting & (trx_dreq_i.dtype == I3CDType_CommandWord);

  // Current output mode.
  logic ddr_mode, sdr_mode;
  logic ddr_crc;  // Transmitting CRC word.
  logic ddr_crcn; // Transmitting a non-CRC HDR-DDR word.

  always_comb begin
    ddr_word = ddr_request(req_starting ? trx_dreq_i.mode : req_q.mode,
                           req_starting ? trx_dreq_i.i2c  : req_q.i2c);
    // HDR-DDR CRC Word includes 12 data bits including the final setup bit.
    // All non-CRC words carry 16 data bits within a 20-bit word (including Preamble and Parity).
    ddr_crc  = (req_q.dtype == I3CDType_CRCWord);
    ddr_crcn = ddr_mode & !ddr_crc;
    // Decide whether the _next_ bit is a data bit, to be shifted out of the data buffer onto SDA.
    data_bit = |{ req_starting & !ddr_word,
                 !req_starting &  ddr_crcn & (bit_idx >= Log2DW'('h3) && bit_idx < Log2DW'('h13)),
                 !req_starting &  ddr_crc  & (bit_idx >= Log2DW'('h1) && bit_idx < Log2DW'('h0b)),
                 !req_starting & !ddr_mode & (bit_idx >= Log2DW'('h2))};
  end

  wire transmitting = |{state_q == State_ClkLoSetupD, state_q == State_ClkHiHoldD,
                        state_q == State_ClkHiSetupD, state_q == State_ClkLoHoldD};
  wire tx_advance = |{state_q == State_ClkHiHoldD && ddr_mode,
                      state_q == State_ClkLoHoldD,
                      state_q == State_AddrClkLoIdle} & phase_end;
  wire tx_load    = req_starting & !trx_dreq_i.rx;
  wire tx_shift   = tx_advance & data_bit;
  wire rx_advance = |{state_q == State_ClkHiSampleD,
                      state_q == State_ClkLoSampleD && ddr_mode} & phase_end;
  wire rx_shift   = rx_advance & data_bit;

  wire buf_shift  = tx_shift | rx_shift;

  logic starting, pre_stop, stop;
  logic last_bit;
  logic advance;
  assign advance  = enable_i & phase_end;
  // Indications of preantepenultimate, prepenultimate, penultimate and the last bit;
  // these are useful for both the addressing phase and the reception of HDR-DDR words.
  wire preant_bit = (bit_idx == Log2DW'('d3));
  wire prepen_bit = (bit_idx == Log2DW'('d2));
  wire penult_bit = (bit_idx == Log2DW'('d1));
  // Final bit of the final byte within this data unit.
  assign last_bit = ~|{bit_idx, req_q.len};

  // Preamble and parity bit indicators, in the appropriate modes.
  // Note that these signals are asserted in the preceding bit intervals, to ensure that 'sda_d'
  // is ready.
  //
  // We're starting a HDR-DDR word; we can ascertain this from the requested transfer mode.
  wire hdr_pre1_bit = (req_starting | req_repeating) & ddr_word;
  wire hdr_pre0_bit = (bit_idx == Log2DW'('h13));  // All other units contain fewer bits.
  wire hdr_para_bit = (req_q.dtype == I3CDType_CommandWord) & preant_bit;
  wire hdr_par1_bit = ddr_crcn & prepen_bit;
  wire hdr_par0_bit = ddr_crcn & penult_bit;
  wire sdr_par_bit  = sdr_mode & penult_bit;

  // TODO: Decide where the timing shall be done.
  assign starting = &{advance, state_q == State_Idle, trx_dvalid_i};
  // We must lower SDA before we can raise it for STOP signaling.
  assign pre_stop = &{advance, state_q == State_PreStop};
  // STOP signaling; SDA raised but SCL still driven low.
  assign stop     = &{advance, state_q == State_Stop};

  // Generation of HDR Exit/Restart signaling.
  // TODO: Suggest reusing the bit_idx counter?
  logic [2:0] hdr_patt_cnt;
  wire gen_hdr_patt   = (trx_dreq_i.dtype == I3CDType_HDRExit ||
                         trx_dreq_i.dtype == I3CDType_HDRRestart) & trx_dvalid_i;
  wire leave_hdr_exit = (req_q.dtype == I3CDType_HDRExit)    & ~|hdr_patt_cnt;
  wire leave_hdr_sr   = (req_q.dtype == I3CDType_HDRRestart) & ~|hdr_patt_cnt;

  // Single data bit used in parity and CRC-5 calculation.
  // - for SDR write traffic the first data bit contributing to the parity comes from the request.
  wire parcrc_bit = transmitting ? sda_q : sda_i;

  // Parity calculated on transmitted/received data.
  // - HDR-DDR collects two independent parity bits, one on the odd bits and one on the even bits.
  logic       parity_sdr;
  logic [1:0] parity_q;  // {PA1, PA0}
  logic [1:0] parity_d;
  logic [1:0] upd_parity;
  logic       init_parity;
  // - SDR employs odd parity across all 8 data bits, but must also include the current bit
  //   and `parity_q` has not yet been updated to reflect that; use `parity_d`.
  assign parity_sdr = ^parity_d;
  // Does the current bit contribute to the parity calculations?
  assign upd_parity[0] =  phase_trans & data_bit_q & !bit_idx[0];
  assign upd_parity[1] = (phase_trans & data_bit_q &  bit_idx[0]) |
                         (req_starting & !ddr_word);  // First SDR bit.
  // Parity must be reinitialized at the start of each transmitted data unit, as well as being
  assign init_parity = req_starting | req_repeating;
  wire  [1:0] parity_mod = {parity_q[1] & ~init_parity, parity_q[0] | init_parity};
  assign parity_d = parity_mod ^ ({2{parcrc_bit}} & upd_parity);  // Updating `q` is conditional.

  // CRC-5 calculated on transmitted/received data; operating on a single bit at a time.
  logic [4:0] crc5_q;
  logic [4:0] crc5_d;
  logic init_crc, upd_crc;
  wire next_crc0 = crc5_q[4] ^ parcrc_bit;
  assign crc5_d = init_crc ? '1 : {crc5_q[3:2], next_crc0 ^ crc5_q[1], crc5_q[0], next_crc0};
  // Update CRC-5 with this bit?
  // TODO: Could choose `dtype` encoding more appropriately?
  assign upd_crc = &{ddr_mode, phase_trans, data_bit_q, req_q.dtype != I3CDType_CRCWord};
  // Initialize the CRC-5 when we first encounter a Command Word; this cannot be coincident with
  // updating the CRC.
  assign init_crc = cmd_starting;
  // TODO: Check whether the CRC-5 that is sent with Read Data by a Target shall INCLUDE the
  // Command Word?

  // Does the current request demand Single Data Rate (SDR) signaling?
  // TODO: Decide whether this needs qualifying with the transfer direction.
  assign sdr_mode = (req_q.dtype == I3CDType_SDRBytes);
  // Does the current request demand Double Data Rate (DDR) signaling?
  // TODO: We have TWO ways to ascertain whether this is DDR transmission!
  always_comb begin
    case (req_q.dtype)
      I3CDType_CommandWord,
      I3CDType_DataWord,
      I3CDType_CRCWord: ddr_mode = 1'b1;
      default: ddr_mode = 1'b0;
    endcase
  end

  // Length of the current data unit, in bits.
  logic [Log2DW-1:0] d_bits_m1;
  assign d_bits_m1 = unitlen_bits(trx_dreq_i.dtype);

  // Transition at the end of a unit within a request.
  always_comb begin
    phase_trans = 1'b0;
    case (state_q)
      State_ClkHiHoldD,
      State_ClkHiSampleD: phase_trans = phase_end & ddr_mode;
      State_AddrClkLoIdle,
      State_ClkLoHoldD,
      State_ClkLoSampleD: phase_trans = phase_end;
      State_HDRExRstHiD:  phase_trans = phase_end & leave_hdr_sr;
    endcase
  end

  // Accept a new request upon completion of the current unit.
  assign trx_dready_o = |{state_q == State_Start & phase_end,
                          last_bit & phase_trans,
                          state_q == State_HDRExRstLoD & leave_hdr_sr};

  // Interpret the new command.
  i3c_ddr_cmd_word_t dreq_cmd;
  assign dreq_cmd = i3c_ddr_cmd_word_t'(trx_dreq_i.wdata);

  // TODO: Currently we retain the type of the data unit here....for bring up.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      req_q <= '0;
    end else if (enable_i) begin
      if (req_starting) begin
        // Retain the details of the current request.
        req_q.dtype <= trx_dreq_i.dtype;
        req_q.mode  <= trx_dreq_i.mode;
        req_q.i2c   <= trx_dreq_i.i2c;
        req_q.len   <= trx_dreq_i.wlen;
        req_q.rx    <= trx_dreq_i.rx;
      end else if (phase_trans & ~|bit_idx) begin
        // Repetition within the current request.
        req_q.len   <= req_q.len - 2'b01;
      end
    end
  end

  // SDA normally acquires the value of the data buffer MSB when it advances, but to keep the
  // line free of glitches it must be driven directly from a flop. It therefore assumes the
  // correct state for START and STOP signaling, as well as receiving injected preamble, parity
  // and CRC-5 bits.
  logic sda_d;
  always_comb begin
    sda_d = sda_q;
    // The MSB drives SDA directly and must have the appropriate state for START and STOP
    // signaling.
    if (&{state_q == State_HDRExRstLoD, phase_end, !leave_hdr_exit}) begin
      sda_d = 1'b1;
    end else if (state_q == State_HDRExRstHiD && phase_end) sda_d = 1'b0;
    else if (|{starting, pre_stop, stop}) sda_d = stop;
    else if (tx_advance) begin
      // TODO: Perhaps we just lose these clunky bit selectors because when we have to inject
      // the CRC-5 we're just selecting based on bit_idx anyway.

      // When emitting SDR write data or DDR-HDR words, we need to inject the calculated parity.
      // Preamble and Parity bits are not part of the data supplied by the controller.
      unique casez ({hdr_pre1_bit, hdr_pre0_bit, hdr_para_bit,
                     hdr_par1_bit, hdr_par0_bit | sdr_par_bit})
        5'b1????: sda_d = trx_dreq_i.dtype[1]; // PRE1 ... not yet captured.
        5'b01???: sda_d = req_q.dtype[0]; // PRE0
        5'b001??: sda_d = ~parity_q[0];   // PARA  ...TODO: check!
        5'b0001?: sda_d = parity_q[1];    // PAR1
        5'b00001: sda_d = parity_q[0];    // PAR0
        default: begin
          if (ddr_crc) begin
            case (bit_idx)
              // Inject the CRC-5 result into the transmitted CRC word at the appropriate position.
              'h6: sda_d = crc5_q[4];
              'h5: sda_d = crc5_q[3];
              'h4: sda_d = crc5_q[2];
              'h3: sda_d = crc5_q[1];             
              'h2: sda_d = crc5_q[0];
              default: sda_d = buf_q[DataWidth-1];
            endcase
          end else begin
            sda_d = tx_load ? trx_dreq_i.wdata[DataWidth-1] : buf_q[DataWidth-1];
          end
        end
      endcase
    end
  end

  // Data buffering.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sda_q     <= '1;
      buf_q     <= '0;
    end else if (sw_reset_i) begin
      sda_q     <= 1'b1;
    end else if (enable_i) begin
      // Update SDA output.
      sda_q     <= sda_d;
      // Shift the transmit/receive buffer and sample the SDA input.
      // - Starting an HDR-DDR word must capture all of the data bits.
      // - Starting a SDR byte transmission we must output the MSbit immediately.
      if (ddr_starting) buf_q <= trx_dreq_i.wdata;
      else if (tx_load | buf_shift) begin
        // TODO: See whether we can read in the arbitrated address at the same time as writing out?
        buf_q[DataWidth-1:1] <= req_starting ? trx_dreq_i.wdata[DataWidth-2:0]
                                             : buf_q[DataWidth-2:0];
        // SDA input value, for read data but also for address arbitration; otherwise '1'.
        // TODO: this should be properly synchronized with our generated clock, but CDC may need
        // either to be waived or resolved.
        buf_q[0] <= transmitting | sda_i;
      end
    end
  end

  // Next state transition upon completion of a data unit.
  state_e state_next_unit;
  always_comb begin
    state_next_unit = State_PreStop;
    if (trx_dvalid_i) begin
      case (trx_dreq_i.dtype)
        // SDR and HDR signaling.
        I3CDType_SDRBytes,
        I3CDType_CommandWord,
        I3CDType_DataWord,
        I3CDType_CRCWord:     state_next_unit = trx_dreq_i.rx ? State_ClkLoAwaitD   // Reception.
                                                              : State_ClkLoSetupD;  // Transmission.
        // HDR Exit and Restart signaling.
        I3CDType_HDRExit,
        I3CDType_HDRRestart:  state_next_unit = State_HDRExRstLoD;
        // Target Reset signaling.
//      I3CDType_TargetReset:  state_next_unit = ;
      endcase
    end
  end

  // Transceiver state.
  always_comb begin
    state_d = State_Idle;
    case (state_q)
      // --- SDR START ---
      State_Idle:             state_d = starting ? State_PreStart : State_Idle;
      State_PreStart:         state_d = State_Start;
      State_Start:            state_d = State_AddrClkLoSetupD;
      // --- Address Arbitration ---
      State_AddrClkLoSetupD:  state_d = State_AddrClkHiHoldD;
      State_AddrClkHiHoldD:   state_d = State_AddrClkHiCheckD;
      State_AddrClkHiCheckD:  state_d = State_AddrClkLoIdle;
      State_AddrClkLoIdle:    state_d = last_bit ? state_next_unit : State_AddrClkLoSetupD;
      // --- Word Transmission ---
      State_ClkLoSetupD:      state_d = State_ClkHiHoldD;
      State_ClkHiHoldD:       state_d = State_ClkHiSetupD;
      State_ClkHiSetupD:      state_d = State_ClkLoHoldD;
      State_ClkLoHoldD:       state_d = last_bit ? state_next_unit : State_ClkLoSetupD;
      // --- Word Reception ---
      State_ClkLoAwaitD:      state_d = State_ClkHiSampleD;
      State_ClkHiSampleD:     state_d = State_ClkHiAwaitD;
      State_ClkHiAwaitD:      state_d = State_ClkLoSampleD;
      State_ClkLoSampleD:     state_d = last_bit ? state_next_unit : State_ClkLoAwaitD;
      // --- HDR Exit/Restart ---
      // TODO: Restart handling incomplete.
      State_HDRExRstHiD:      state_d = leave_hdr_sr   ? State_Idle : State_HDRExRstLoD;
      State_HDRExRstLoD:      state_d = leave_hdr_exit ? State_Stop : State_HDRExRstHiD;
      // --- SDR STOP ---
      State_PreStop:          state_d = State_Stop;
      // The default case handles State_Stop as well as invalid states.
      default:                state_d = State_Idle;
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

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      crc5_q        <= '1;
      parity_q      <= 2'b01;
      cnt           <= '0;
      state_q       <= State_Idle;
      bit_idx       <= '1;
      data_bit_q    <= 1'b0;
      hdr_patt_cnt  <= '1;
    end else if (sw_reset_i) begin
      crc5_q        <= '1;
      parity_q      <= 2'b01;
      cnt           <= '0;
      state_q       <= State_Idle;
      bit_idx       <= '1;
      data_bit_q    <= 1'b0;
    end else if (enable_i) begin
      if (req_starting) begin
        bit_idx     <= d_bits_m1;
        // Behavior specific to the new data unit.
        case (trx_dreq_i.dtype)
          I3CDType_HDRRestart: hdr_patt_cnt <= 3'b010;
          I3CDType_HDRExit:    hdr_patt_cnt <= 3'b100;
        endcase
      end else if (phase_end) begin
        if (|bit_idx) begin
          // Counting of bits within data unit.
          bit_idx <= bit_idx - Log2DW'(tx_advance | rx_advance);
        end else if (phase_trans) begin
          // Repetition within this data unit.
          bit_idx <= unitlen_bits(req_q.dtype);
        end
        // HDR pattern counter.
        if (state_q == State_HDRExRstHiD) hdr_patt_cnt <= hdr_patt_cnt - 'b1;
      end

      // Decide whether the current output bit contributes to parity and CRC-5 calculations;
      // the first bit of the transfer unit contributes for SDR but not for HDR-DDR words.
      if (req_starting | req_repeating) data_bit_q <= !ddr_word;
      else if (phase_trans) data_bit_q <= data_bit;

      // Conditionally update CRC-5 and parity.
      if (init_crc | upd_crc) crc5_q <= crc5_d;
      if (init_parity | upd_parity[1]) parity_q[1] <= parity_d[1];
      if (init_parity | upd_parity[0]) parity_q[0] <= parity_d[0];

      if (phase_end) begin
        // Advance the signaling state machine.
        state_q   <= state_d;
        cnt       <= tm_cycles;
      end else if (state_q != State_Idle) begin
        cnt <= cnt - 'b1;
      end
// TODO: Buffer writing indirect now.
//    buf_wr_o <= rx_shift & last_bit;
    end else begin
//    buf_wr_o <= 1'b0;
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
    end else if (enable_i) begin
      if (phase_end) begin
        case (state_q)
          State_Idle: begin
            // Leave the bus inactive with pullup enabled until starting a transaction.
            sda_pp_en_q <= 1'b0;
            sda_od_en_q <= starting;
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
          State_AddrClkLoIdle: begin
            if (penult_bit) begin
              // ACK/NAK cycle of address arbitration
              // TODO: This presently does not handle the receipt of IBI etc.
              sda_od_en_q <= 1'b0;
            end else if (~|bit_idx) begin
              // Transitioning to SDR
              sda_pp_en_q <= 1'b1;
              sda_pu_en_q <= 1'b0;
              sda_od_en_q <= 1'b0;
            end
          end
          State_ClkLoSampleD: begin
            if (last_bit) begin
              sda_pp_en_q <= (state_next_unit != State_ClkLoAwaitD);
              sda_pu_en_q <= 1'b0;
              sda_od_en_q <= 1'b0;
            end else if (penult_bit) begin
            // TODO: Loss of arbitration.
            end
          end

          // End of Command Word transmission may signal the start of data reception.
          State_ClkLoHoldD: begin
            if (last_bit) begin
              sda_pp_en_q <= (state_next_unit != State_ClkLoAwaitD);
              sda_pu_en_q <= 1'b1;
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
          State_ClkLoAwaitD,
          State_ClkHiSampleD,
          State_ClkHiAwaitD,
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
      // Controller disabled; ensure that we have no impact upon the bus.
      sda_pu_en_q <= 1'b0;
      sda_pp_en_q <= 1'b0;
      sda_od_en_q <= 1'b0;
    end
  end

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

  // TODO: Collect data units (partial words) into `buf_q` until full or target returns no more
  // read data?
  assign trx_rvalid_o = &{state_q == State_ClkLoSampleD, phase_end,
                          ddr_mode ? prepen_bit : penult_bit};  // Final data bit.
  always_comb begin
    trx_rsp_o = '0;
    trx_rsp_o.rtype = I3CRType_OK;
    // TODO: Decide what should happen here! Probably just want to send a byte count really.
    trx_rsp_o.dtype = ddr_mode ? I3CDType_DataWord : I3CDType_SDRBytes;
    trx_rsp_o.rdata = buf_q;
    trx_rsp_o.rlen  = rlen;
  end

endmodule
