// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// SCK-driving I3C transmitter/receiver.
// - This is presently Controller only, since HDR-BT mode is not supported.

module i3c_tx #(
  // Number of SDA lanes; must be one presently since HDR-BT mode not supported.
  parameter int unsigned NumSDALanes = 1,
  parameter int unsigned DataWidth = 32,
  parameter int unsigned CntWidth = 16
) (
  input                         clk_i,
  input                         rst_ni,

  input                         tx_enable_i,
  input                         rx_enable_i,
  input                         ddr_enable_i,

  // Timing parameters.
  input          [CntWidth-1:0] clkdiv_i,
  // TODO: Choose suitable dimensions.
  input          [CntWidth-1:0] tcas_i,
  input          [CntWidth-1:0] tcbp_i,

  // Buffer reading.
  output logic                  buf_rd_o,
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
  output                        sda_od_en_o
);

  localparam int unsigned Log2DW = $clog2(DataWidth);

  // Data buffer.
  logic [DataWidth-1:0] buf_q;

  //  Enable for transmission/reception logic; for reduced power consumption.
  wire enable = tx_enable_i | rx_enable_i;

  // TODO: Perhaps we drive the sampling enable from a bit of the state too?

  // SCL and SCL_EN are driven directly from bits of the state variable to eliminate
  // glitches that could otherwise result from any combinational logic.
  localparam int unsigned StBit_SDA_En = 2;
  localparam int unsigned StBit_SCL_En = 1;
  localparam int unsigned StBit_SCL    = 0;

  typedef enum logic [4:0] {
    // <i>_`SDA-EN`_`SCL-EN`_`SCL`
    State_Idle          = 5'b00_0_0_1,
    State_PreStart      = 5'b00_1_1_1,
    State_Start         = 5'b00_1_1_0,
    State_Stop          = 5'b11_1_1_1,
    State_PreStop       = 5'b01_1_1_0,

    // --- Word Transmission ---

    // SCK low->high transition.
    State_ClkLoSetupD   = 5'b10_1_1_0,
    State_ClkHiHoldD    = 5'b01_1_1_1,
    // SCK high->low transition.
    State_ClkHiSetupD   = 5'b10_1_1_1,
    State_ClkLoHoldD    = 5'b11_1_1_0,

    // --- Word reception ---

    // SCK low->high transition
    State_ClkLoAwaitD   = 5'b00_0_1_0,
    State_ClkHiSampleD  = 5'b00_0_1_1,
    // SCK high->low transition
    State_ClkHiAwaitD   = 5'b01_0_1_1,
    State_ClkLoSampleD  = 5'b01_0_1_0

  } state_e;

  // Transceiver state.
  state_e state_q, state_d;
  logic [CntWidth-1:0] cnt;
  logic [Log2DW-1:0] bits_left;

// TODO:
wire scl_changed = 1'b1;  // TODO: this will need to be counter-dependent, not clock.

  // Shift buffered output data?
  wire tx_shift = |{state_q == State_ClkHiHoldD && ddr_enable_i,
                    state_q == State_ClkLoHoldD} & scl_changed;
  wire rx_shift = |{state_q == State_ClkHiSampleD,
                    state_q == State_ClkLoSampleD && ddr_enable_i} & scl_changed;
  wire buf_shift = tx_shift | rx_shift;

  logic starting, stopping, pre_stop, stop;
  logic last_bit;
  logic advance;
  assign advance  = enable & ~|cnt;
  assign last_bit = ~|bits_left;
  assign starting = &{advance, state_q == State_Idle, rx_enable_i | !buf_empty_i};
  // When the final bit has been stable long enough, move into the PreStop state.
  assign stopping = &{advance, state_q == State_ClkLoHoldD || state_q == State_ClkLoSampleD,
                      last_bit};
  // We must lower SDA before we can raise it for STOP signaling.
  assign pre_stop = &{advance, state_q == State_PreStop};
  // STOP signaling; SDA raised but SCL still driven low.
  assign stop     = &{advance, state_q == State_Stop};

  // Parity calculated on transmitted/received data.
  logic [1:0] parity_q;
  logic [1:0] parity_d;
  assign parity_d = {parity_q[1] ^ buf_q[DataWidth-2],
                     parity_q[1] ^ buf_q[DataWidth-1]};
  // CRC-5 calculated on transmitted/received data.
  logic [4:0] crc5_q;
  logic [4:0] crc5_d;
  assign crc5_d = {crc5_q[2],
                   crc5_q[1] ^ buf_q[DataWidth-2] ^ crc5_q[4],
                   crc5_q[0] ^ buf_q[DataWidth-1] ^ crc5_q[3],
                   crc5_q[4] ^ buf_q[DataWidth-2],
                   crc5_q[3] ^ buf_q[DataWidth-1]};

  // Shift the input data so that the first bit is ready for output.
  wire [4:0] sh_amount = 5'b0;  // TODO:
  wire [DataWidth-1:0] buf_rdata_sh = buf_rdata_i << sh_amount;

  // Data buffering.
  logic buf_rd_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      buf_rd_q  <= 1'b0;
      buf_q     <= '1;
    end else if (enable) begin
      // The MSB drives SDA directly and must have the appropriate state for START and STOP
      // signaling.
      if (|{starting, stopping, pre_stop, stop}) buf_q[DataWidth-1] <= stop;
      else if (buf_rd_q) buf_q[DataWidth-1] <= buf_rdata_sh[DataWidth-1];
      else if (buf_shift) buf_q[DataWidth-1] <= buf_q[DataWidth-2];

      // Collect the rest of the read data.
      if (buf_rd_q) buf_q[DataWidth-2:0] <= buf_rdata_sh[DataWidth-2:0];
      else if (buf_shift) begin
        // Sample the read data.
        // TODO: this should be properly synchronized with our generated clock, but CDC may need
        // either to be waived or resolved.
        buf_q[DataWidth-2:0] <= {buf_q[DataWidth-3:0], tx_shift | sda_i};
      end
      // Read data is returned a cycle after the request.
      buf_rd_q  <= buf_rd_o;
    end else begin
      buf_rd_q <= 1'b0;
    end
  end

  assign buf_rd_o = &{tx_enable_i, state_q == State_PreStart, ~|cnt, !buf_empty_i};

  // Transceiver state.
  always_comb begin
    state_d = State_Idle;
    case (state_q)
      State_Idle:         state_d = starting ? State_PreStart : State_Idle;
      State_PreStart:     state_d = State_Start;
      State_Start:        state_d = tx_enable_i ? State_ClkLoSetupD : State_ClkLoAwaitD;

      // --- Word Transmission ---
      State_ClkLoSetupD:  state_d = State_ClkHiHoldD;
      State_ClkHiHoldD:   state_d = State_ClkHiSetupD;
      State_ClkHiSetupD:  state_d = State_ClkLoHoldD;
      State_ClkLoHoldD:   state_d = last_bit ? State_PreStop : State_ClkLoSetupD;
      // --- Word Reception ---
      State_ClkLoAwaitD:  state_d = State_ClkHiSampleD;
      State_ClkHiSampleD: state_d = State_ClkHiAwaitD;
      State_ClkHiAwaitD:  state_d = State_ClkLoSampleD;
      State_ClkLoSampleD: state_d = last_bit ? State_PreStop : State_ClkLoAwaitD;

      State_PreStop:      state_d = State_Stop;
      // The default case handles State_Stop as well as invalid states.
      default:            state_d = State_Idle;
    endcase
  end

  // Duration of the next state, in clock cycles.
  logic [CntWidth-1:0] tm_cycles;
  always_comb begin
    case (state_q)
      State_Idle:       tm_cycles = tcas_i;
      State_ClkLoHoldD: tm_cycles = last_bit ? tcbp_i : clkdiv_i;
      default:          tm_cycles = clkdiv_i;
    endcase
  end

  // TODO: It seems likely that we will want this logic to advance two bits at a time in
  // HDR-DDR mode.
  logic rx_sample_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      crc5_q    <= '1;
      parity_q  <= 2'b01;
      cnt       <= '0;
      state_q   <= State_Idle;
      bits_left <= '1;
      buf_wr_o  <= 1'b0;
    end else if (enable) begin
      if (|cnt) cnt <= cnt - 'b1;
      else begin
        // Update CRC-5 and parity on every other bit shifted out.
        if (buf_rd_q) begin
          crc5_q   <= '1;
          parity_q <= 2'b01;
        end else if (buf_shift & bits_left[0]) begin
          crc5_q   <= crc5_d;
          parity_q <= parity_d;
        end
        state_q   <= state_d;
        bits_left <= bits_left - Log2DW'(buf_shift);
        cnt       <= tm_cycles;
      end
      buf_wr_o <= rx_shift & ~|bits_left;
    end else begin
      buf_wr_o <= 1'b0;
    end
  end

  assign buf_wmask_o = '1;  // TODO
  assign buf_wdata_o = buf_q;

  assign scl_o       = state_q[StBit_SCL];
  assign sda_o       = buf_q[DataWidth-1];
  assign scl_pp_en_o = state_q[StBit_SCL_En];
  assign scl_od_en_o = 1'b0;
  assign sda_pp_en_o = state_q[StBit_SDA_En];
  assign sda_od_en_o = 1'b0;

endmodule
