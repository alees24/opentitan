// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C receiver.
// - Target (including Secondary Controller) functionality.
// - This logic is reactive, responding to the SCK that it receives and the
//   word-level decoding is driven entirely by that clock before moving data
//   words and other signals across a CDC boundary into the main IP block.

module i3c_rx #(
  // Number of SDA lanes; must be one presently since HDR-BT mode not supported.
  parameter int unsigned NumSDALanes = 1,
  parameter int unsigned DataWidth = 32,
  parameter int unsigned CntWidth = 16
) (
  input                     clk_i,
  input                     rst_ni,

  input                     tx_enable_i,
  input                     rx_enable_i,
  input                     ddr_enable_i,

  // Buffer reading.
  output                    buf_rd_o,
  input     [DataWidth-1:0] buf_rdata_i,
  input                     buf_empty_i,

  // Buffer writing.
  output                    buf_wr_o,
  output  [DataWidth/8-1:0] buf_wmask_o,
  output    [DataWidth-1:0] buf_wdata_o,

  // I3C I/O signaling.
  input                     scl_i,
  input   [NumSDALanes-1:0] sda_i,
  output  [NumSDALanes-1:0] sda_o,
  output                    sda_pp_en_o,
  output                    sda_od_en_o,

  // DFT-related controls.
  input                     scanmode_i
);

  localparam int unsigned Log2DW = $clog2(DataWidth);

  // TODO: Consider sharing this with the i3c_patt_detector.
  // Clock buffering.
  logic scl_buf;
  prim_clock_buf #(
    .NoFpgaBuf(1'b1)
  ) u_scl_buf (
    .clk_i  (scl_i),
    .clk_o  (scl_buf)
  );
  // Inverted buffered clock.
  logic scl_buf_n;
  prim_clock_inv #(
    .NoFpgaBufG(1'b1)
  ) u_scl_buf_inv (
    .clk_i      (scl_i),
    .scanmode_i (scanmode_i),
    .clk_no     (scl_buf_n)
  );

  // Positive-edge data bit.
  logic scl_ph0;
  logic sda_q;
  always_ff @(posedge scl_buf or negedge rst_ni) begin
    if (!rst_ni) begin
      scl_ph0 <= 1'b1;
      sda_q   <= '1;
    end else begin
      scl_ph0 <= !scl_ph0;
      sda_q   <= sda_i;
    end
  end
  // Negative-edge data bit (DDR signaling).
  logic [2*NumSDALanes-1:0] sda_nq;
  logic [1:0] scl_phase;
  always_ff @(posedge scl_buf_n or negedge rst_ni) begin
    if (!rst_ni) begin
      scl_phase <= '1;
      sda_nq    <= '1;
    end else begin
      scl_phase <= {!scl_phase[1], scl_ph0};
      sda_nq    <= {sda_i, sda_q};
    end
  end

  // Synchronize the I3C input signals to the main clock.
  //
  // TODO: Perhaps here we use Gray's code on the phase and/or data and/or an
  // async FIFO rather than two banks of synchronizer flops?
  logic [2*NumSDALanes-1:0] sda_sync;
  logic [1:0] scl_sync;
  prim_flop_2sync #(
    .Width      (2+2*NumSDALanes),
    .ResetValue ('1)
  ) u_sync(
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .d_i      ({scl_phase, sda_nq}),
    .q_o      ({scl_sync, sda_sync})
  );

  // TODO:
  logic sda_d, scl_d;
  ///prim_flop_2sync #(.Width(2), .ResetValue('1)) u_sync_tmp(
  //  .clk_i  (clk_i),
  //  .rst_ni (rst_ni),
  //  .d_i    ({scl_i, sda_i}),
  //  .q_o    ({scl_d, sda_d})
  //);
  assign {scl_d, sda_d} = {scl_i, sda_i};

  // TODO: rework this.
  logic scl_q, sda_q2;

  // Data buffer.
  logic [DataWidth-1:0] buf_q;

  //  Enable for transmission/reception logic; for reduced power consumption.
  wire enable = tx_enable_i | rx_enable_i;

  // SDA_PP_EN and SDA_OD_EN are driven directly from bits of the state variable to eliminate
  // glithces that could otherwise result from combinational logic.
  localparam int unsigned StBit_SDA_OD_En = 1;
  localparam int unsigned StBit_SDA_PP_En = 0;

  typedef enum logic [4:0] {
    // <i>_`SDA-OD-EN`_`SDA-PP-EN`
    State_Idle         = 5'b000_0_0,
    State_Start        = 5'b101_0_0,
    // TODO: Temporary.
    State_PreStartTx   = 5'b111_0_0,
    State_StartTx      = 5'b101_0_1,

    State_PreStop      = 5'b110_0_0,

    // --- Word transmission ---

    // SCK low->high transition
    State_ClkLoSetupD  = 5'b000_0_1,
    State_ClkHiHoldD   = 5'b001_0_1,
    // SCK high->low transition
    State_ClkHiSetupD  = 5'b010_0_1,
    State_ClkLoHoldD   = 5'b011_0_1,

    // --- Word reception ---

    // SCK low->high transition
    State_ClkLoAwaitD  = 5'b001_0_0,
    State_ClkHiSampleD = 5'b010_0_0,
    // SCK high->low transition
    State_ClkHiAwaitD  = 5'b011_0_0,
    State_ClkLoSampleD = 5'b100_0_0
  } state_e;

  // Transceiver state.
  state_e state_q, state_d;
  logic [Log2DW-1:0] bits_left;
  wire last_bit = ~|bits_left;

  // TODO:
  logic advance, starting, transmitting;
  assign advance  = (sda_q2 != sda_d);
  assign starting = &{advance, scl_d, state_q == State_Idle};
  assign transmitting = (state_q == State_ClkLoSetupD) || (state_q == State_ClkHiHoldD) ||
                        (state_q == State_ClkHiSetupD) || (state_q == State_ClkLoHoldD);

  wire starting_tx = starting & tx_enable_i & !rx_enable_i;
  assign buf_rd_o = starting_tx & !buf_empty_i;

  // TODO: This will not be required when we're not oversampling
  logic scl_changed;
  assign scl_changed = scl_q ^ scl_d;

  always_comb begin
    state_d = state_q;
    // STOP handling.
    if (&{sda_d, !sda_q2, scl_q, !transmitting, !ddr_enable_i}) begin
      state_d = State_Idle;
    // START handling.
    end else if (starting_tx) begin
      state_d = State_PreStartTx;
    end else if (starting) begin
      state_d = State_Start;
    end else if (scl_changed) begin
      case (state_q)
        State_PreStartTx:   state_d = State_StartTx;
        State_StartTx:      state_d = State_ClkLoHoldD;
        State_Start:        state_d = State_ClkLoAwaitD;

        // --- Word Reception ---
        State_ClkLoAwaitD:  state_d = State_ClkHiAwaitD;
        State_ClkHiAwaitD:  state_d = last_bit ? State_PreStop : State_ClkLoAwaitD;

        // --- Word Transmission ---
        State_ClkLoHoldD:   state_d = State_ClkHiHoldD;
        State_ClkHiHoldD:   state_d = last_bit ? State_PreStop : State_ClkLoHoldD;

        default:            state_d = State_Idle;
      endcase
    end
  end

  // Reception logic.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      scl_q   <= 1'b1;
      sda_q2  <= 1'b1;
      state_q <= State_Idle;
    end else if (enable) begin
      scl_q   <= scl_d;
      sda_q2  <= sda_d;
      state_q <= state_d;
    end
  end

  // Supply new data on SDA?
  logic tx_supply;
  assign tx_supply = ((state_q == State_ClkLoHoldD) ||
                     (state_q == State_ClkHiHoldD && ddr_enable_i))
                     & scl_changed;

  // Sample the current state of SDA?
  logic rx_sample;
  assign rx_sample = ((state_q == State_ClkHiAwaitD) ||
                     (state_q == State_ClkLoAwaitD && ddr_enable_i))
                     & scl_changed;

  wire buf_shift = tx_supply | rx_sample;

  // Shift the input data so that the first bit is ready for output.
  wire [4:0] sh_amount = 5'b0;  // TODO:
  wire [DataWidth-1:0] buf_rdata_sh = buf_rdata_i << sh_amount;

  // Data reception/transmission.
  logic buf_rd_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      buf_rd_q  <= 1'b0;
      buf_q     <= '1;
    end else if (enable) begin
      // The MSB drives SDA directly, so it must be set to the appropriate start
      // during SDA signalling; TODO: This is temporary for bring up, until we properly observe the
      // protocol.
      if (buf_rd_q) buf_q[DataWidth-1] <= buf_rdata_sh[DataWidth-1];
      else if (starting_tx) buf_q[DataWidth-1] <= 1'b1;
      else if (buf_shift) buf_q[DataWidth-1] <= buf_q[DataWidth-2];

      if (buf_rd_q) buf_q[DataWidth-2:0] <= buf_rdata_sh[DataWidth-2:0];
      else if (buf_shift) begin
        buf_q[DataWidth-2:0] <= {buf_q[DataWidth-3:0], !rx_enable_i | sda_q2};
      end

      // Read data is returned a cycle after the request.
      // TODO: We need to address contention here, prefetching probably.
      buf_rd_q <= buf_rd_o;
    end else begin
      buf_rd_q <= 1'b0;
    end
  end
  // Parity calculated on received data.
  logic [1:0] parity_q;
  logic [1:0] parity_d;
  assign parity_d = {parity_q[1] ^ sda_q2,
                     parity_q[1] ^ buf_q[0]};
  // CRC-5 calculated on transmitted data.
  logic [4:0] crc5_q;
  logic [4:0] crc5_d;
  assign crc5_d = {crc5_q[2],
                   crc5_q[1] ^ sda_q2   ^ crc5_q[4],
                   crc5_q[0] ^ buf_q[0] ^ crc5_q[3],
                   crc5_q[4] ^ sda_q2,
                   crc5_q[3] ^ buf_q[0]};

  // TODO: It seems likely that we will want this logic to advance two bits at a time in
  // HDR-DDR mode.
  logic rx_sample_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      crc5_q      <= '1;
      parity_q    <= 2'b01;
      bits_left     <= '1;
      rx_sample_q <= 1'b0;
    end else if (enable) begin
      bits_left   <= bits_left - Log2DW'(buf_shift);
      // Update CRC-5 and parity on every other bit shifted in.
      if (rx_sample & !bits_left[0]) begin
        crc5_q    <= crc5_d;
        parity_q  <= parity_d;
      end
      rx_sample_q <= rx_sample & ~|bits_left;
    end else begin
      rx_sample_q <= 1'b0;
    end
  end

  assign buf_wr_o    = rx_sample_q;
  assign buf_wmask_o = '1;
  assign buf_wdata_o = buf_q;

  assign sda_o       = buf_q[DataWidth-1];
  assign sda_pp_en_o = state_q[StBit_SDA_PP_En];
  assign sda_od_en_o = state_q[StBit_SDA_OD_En];

endmodule
