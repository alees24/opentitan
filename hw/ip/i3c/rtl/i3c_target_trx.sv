// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C Target transceiver.
//
// - Target (including Secondary Controller) functionality.
// - This logic is reactive, responding to the SCK that it receives and the
//   word-level decoding is driven entirely by that clock before moving data
//   words and other signals across a CDC boundary into the main IP block.

module i3c_target_trx #(
  // Number of SDA lanes; must be one presently since HDR-BT mode not supported.
  parameter int unsigned NumSDALanes = 1,
  parameter int unsigned DataWidth = 32,
  parameter int unsigned CntWidth = 16
) (
  input                     clk_i,
  input                     rst_ni,

  input                     sw_reset_i,

  input                     tx_enable_i,
  input                     rx_enable_i,
  input                     ddr_enable_i,

  // Secondary Controller address and validity indicator.
  input               [6:0] ctrl_addr_i,
  input                     ctrl_addr_valid_i,
  // Target addresses and masks.
  input               [6:0] targ_addr0_mask_i,
  input               [6:0] targ_addr0_match_i,
  input               [6:0] targ_addr1_mask_i,
  input               [6:0] targ_addr1_match_i,

  // HDR pattern detection.
  input                     hdr_exit_det_i,
  input                     hdr_restart_det_i,

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

  // TODO: Consider sharing this with the i3c_patt_detector, by moving the buffers into i3c_core
  // perhaps? Then we can leave it as an implementation choice whether to move the AON/wake
  // logic outside of the core IP block.

  // Clock buffering.
  logic scl_buf;
  prim_clock_mux2 #(
    .NoFpgaBufG (1'b0)
  ) u_scl_buf (
    .clk0_i (1'b1),
    .clk1_i (scl_i),
    .sel_i  (rx_enable_i),
    .clk_o  (scl_buf)
  );
  // Inverted buffered clock.
  logic scl_buf_n;
  clock_inv_en #(
    .HasScanMode  (1'b1),
    .NoFpgaBufG   (1'b0)
  ) u_scl_buf_inv (
    .en_i       (rx_enable_i),
    .clk_i      (scl_i),
    .scanmode_i (scanmode_i),
    .clk_no     (scl_buf_n)
  );

  // Data buffering
  logic [NumSDALanes-1:0] sda_buf;
  wire sda0_buf = sda_buf[0];
  buf_en #(
    .Width        (NumSDALanes),
    .OutDisabled  ({NumSDALanes{1'b1}})
  ) u_sda_buf (
    .en_i       (rx_enable_i),
    .in_i       (sda_i),
    .out_o      (sda_buf)
  );
  // Inverted buffered data, used as a clock.
  logic sda0_clk_n;
  clock_inv_en #(
    .HasScanMode  (1'b1),
    .NoFpgaBufG   (1'b0)
  ) u_sda0_buf_n (
    .en_i       (rx_enable_i),
    .clk_i      (sda_i[0]),
    .scanmode_i (scanmode_i),
    .clk_no     (sda0_clk_n)
  );

  // Ensure that we do not respond to our own SDA signaling.
  wire sda_en = sda_pp_en_o | sda_od_en_o;
  logic start_det, stop_det;

  // SDR START detection.
  always_ff @(posedge sda0_clk_n or negedge rst_ni) begin
    if (!rst_ni) begin
      start_det <= 1'b0;
    end else if (!sda_en & !ddr_enable_i) begin
      start_det <= scl_buf;
    end
  end
  // SDR STOP detection.
  always_ff @(posedge sda0_buf or negedge rst_ni) begin
    if (!rst_ni) begin
      stop_det  <= 1'b0;
    end else if (start_det) begin
      stop_det  <= 1'b0;
    end else if (!sda_en & !ddr_enable_i) begin
      stop_det  <= scl_buf;
    end
  end

  // Positive-edge data.
  // - we have 10 data bits for each clock edge to support the collection of a 20-bit HDR-DDR word.
  logic [9:0] sda_pq[NumSDALanes];
  always_ff @(posedge scl_buf or negedge rst_ni) begin
    if (!rst_ni) begin
      sda_pq  <= '{'b0};
    end else begin : shift_pq
      for (int unsigned lane = 0; lane < NumSDALanes; lane++) begin
        sda_pq[lane]  <= {sda_pq[lane][8:0], sda_buf[lane]};
      end
   end
  end

  // Negative-edge data (DDR signaling).
  logic [9:0] sda_nq[NumSDALanes];
  always_ff @(posedge scl_buf_n or negedge rst_ni) begin
    if (!rst_ni) begin
      sda_nq  <= '{'b0};
    end else begin : shift_nq
      for (int unsigned lane = 0; lane < NumSDALanes; lane++) begin
        sda_nq[lane]  <= {sda_nq[lane][8:0], sda_buf[lane]};
      end
    end
  end

  // Check whether the arbitrated address matches against our configuration.
  wire [6:0] arb_addr = {sda_pq[0][5:0], sda_buf};
  wire ctrl_addr_matches =  (arb_addr == ctrl_addr_i) & ctrl_addr_valid_i;
  wire targ_addr_matches = ((arb_addr & targ_addr0_mask_i) == targ_addr0_match_i) ||
                           ((arb_addr & targ_addr1_mask_i) == targ_addr1_match_i);
  wire addr_matches = ctrl_addr_matches | targ_addr_matches;
  logic arb_won;

  // Reception state.
  typedef enum logic [2:0] {
    RxIdle = 0,
    RxIgnore,
    RxSDR,
    RxDDR,
    RxAckNak
  } rx_state_e;

  // TODO: Most of the parsing logic will need to exist here and be clocked by SCL;
  // then we post byte reads and other information across the CDC.
  rx_state_e rx_state;
  logic [9:0] rx_clks;
  always_ff @(posedge scl_buf or negedge rst_ni) begin
    if (!rst_ni) begin
      rx_state  <= RxIdle;
      rx_clks   <= 'b0;
      arb_won   <= 1'b0;
    end else begin
      case (rx_state)
        RxIdle: if (start_det) begin
          rx_state <= RxSDR;
          rx_clks  <= '0;
          arb_won  <= 1'b0;
        end

        RxSDR: begin
          if (stop_det) rx_state <= RxIdle;
          else begin
            case (rx_clks)
              'h6: begin
                // Did we win the arbitration?
                rx_state    <= addr_matches ? RxAckNak : RxIgnore;
                arb_won     <= addr_matches;
              end
            endcase
          end
          rx_clks <= rx_clks + 'b1;
        end

        RxAckNak: begin
          arb_won <= 1'b0;
          rx_state <= RxIgnore;
        end

        RxIgnore: if (hdr_exit_det_i | stop_det) rx_state <= RxIdle;

        RxDDR: begin
        end

        default: if (hdr_exit_det_i) rx_state <= RxIdle;
      endcase
    end
  end

  // TODO: Replace this logic; use the controller clock.
  logic sda_d, scl_d;
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
    // SCK high->low transition
    State_ClkHiAwaitD  = 5'b011_0_0
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
    end else begin
      if (enable) begin
        scl_q   <= scl_d;
        sda_q2  <= sda_d;
      end
      if (sw_reset_i) state_q <= State_Idle;
      else if (enable) state_q <= state_d;
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
    end else if (sw_reset_i) begin
      buf_rd_q  <= 1'b0;
      buf_q[DataWidth-1] <= 1'b1;
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
  assign parity_d = {parity_q[1] ^ buf_q[0],
                     parity_q[0] ^ sda_q2};
  // CRC-5 calculated on transmitted data.
  logic [4:0] crc5_q;
  logic [4:0] crc5_d;
  assign crc5_d = {crc5_q[2],
                   crc5_q[1] ^ buf_q[0] ^ crc5_q[4],
                   crc5_q[0] ^ sda_q2   ^ crc5_q[3],
                   crc5_q[4] ^ buf_q[0],
                   crc5_q[3] ^ sda_q2};

  // TODO: It seems likely that we will want this logic to advance two bits at a time in
  // HDR-DDR mode.
  logic rx_sample_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      crc5_q      <= '1;
      parity_q    <= 2'b01;
      bits_left   <= '1;
      rx_sample_q <= 1'b0;
    end else if (sw_reset_i) begin
      crc5_q      <= '1;
      parity_q    <= 2'b01;
      bits_left   <= '1;
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

  // Pull the SDA line low to ACK the arbitration if we won.
  // TODO:
  //sda_od_en_o <= addr_matches;
  //sda_o       <= !addr_matches;

  assign sda_o       = buf_q[DataWidth-1] & ~arb_won;
  assign sda_pp_en_o = state_q[StBit_SDA_PP_En];
  assign sda_od_en_o = state_q[StBit_SDA_OD_En] | arb_won;

endmodule
