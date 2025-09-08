// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C transmitter.

module i3c_tx #(
  parameter int unsigned DataWidth = 32,
  parameter int unsigned CntWidth = 16
) (
  input                         clk_i,
  input                         rst_ni,

  input                         tx_enable_i,
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

  // I3C I/O signaling.
  output                        scl_o,
  output                        scl_en_o,
  output                        sda_o,
  output                        sda_en_o,
  output                        sda_od_en_o
);

  localparam int unsigned Log2DW = $clog2(DataWidth);

  // Data buffer.
  logic [DataWidth-1:0] buf_q;

  // SCL and SDA are driven directly from bits of the state variable to eliminate
  // glitches that could otherwise result from any combinational logic.
  typedef enum logic [4:0] {
    // ... _`SCL-EN`_`SCL`
    TxState_Idle        = 5'b000_0_1,
    TxState_Start       = 5'b001_1_1,
    TxState_Stop        = 5'b101_1_1,
    // SCK low->high transition.
    TxState_ClkLoSetupD = 5'b010_1_0,
    TxState_ClkHiHoldD  = 5'b010_1_1,
    // SCK high->low transition.
    TxState_ClkHiSetupD = 5'b011_1_1,
    TxState_ClkLoHoldD  = 5'b011_1_0
  } tx_state_e;

  // Transmission state.
  tx_state_e tx_state_q, tx_state_d;
  logic [CntWidth-1:0] cnt;
  logic [Log2DW-1:0] tx_left;

  // Shift buffered output data?
  logic buf_shift;
  assign buf_shift = |{tx_state_q == TxState_ClkHiHoldD,
                       tx_state_q == TxState_ClkLoHoldD && ddr_enable_i};

  logic starting, stopping, stop;
  logic last_bit;
  logic advance;
  assign advance  = tx_enable_i & ~|cnt;
  assign last_bit = ~|tx_left;
  assign starting = &{advance, tx_state_q == TxState_Idle};
  assign stopping = &{advance, tx_state_q == TxState_ClkLoHoldD, last_bit};
  assign stop     = &{advance, tx_state_q == TxState_Stop};

  // Data buffering.
  logic buf_rd_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      buf_rd_q  <= 1'b0;
      buf_q     <= '1;
    end else begin
      buf_rd_q  <= buf_rd_o;
      // The MSB drives SDA directly and must have the appropriate state for START and STOP
      // signaling.
      if (starting | stopping | stop) buf_q[DataWidth-1] <= !stop;
      else if (buf_rd_q) buf_q[DataWidth-1] <= buf_rdata_i[DataWidth-1];
      else if (buf_shift) buf_q[DataWidth-1] <= buf_q[DataWidth-2];

      // Collect the rest of the read data.
      if (buf_rd_q) buf_q[DataWidth-2:0] <= buf_rdata_i[DataWidth-2:0];
      else if (buf_shift) buf_q[DataWidth-2:0] <= {buf_q[DataWidth-3:0], 1'b1};
    end
  end

  // TODO:
  assign buf_rd_o = (tx_state_q == TxState_Idle) & !buf_empty_i;

  // Transmission state.
  always_comb begin
    tx_state_d = TxState_Idle;
    case (tx_state_q)
      TxState_Idle:        tx_state_d = TxState_Start;
      TxState_Start:       tx_state_d = TxState_ClkLoSetupD;
      TxState_ClkLoSetupD: tx_state_d = TxState_ClkHiHoldD;
      TxState_ClkHiHoldD:  tx_state_d = TxState_ClkHiSetupD;
      TxState_ClkHiSetupD: tx_state_d = TxState_ClkLoHoldD;
      TxState_ClkLoHoldD:  tx_state_d = last_bit ? TxState_Stop : TxState_ClkLoSetupD;
      default:             tx_state_d = TxState_Idle;
    endcase
  end

  // Duration of the next transmission state, in clock cycles.
  logic [CntWidth-1:0] tx_cycles;
  always_comb begin
    case (tx_state_q)
      TxState_Idle:       tx_cycles = tcas_i;
      TxState_ClkLoHoldD: tx_cycles = last_bit ? tcbp_i : clkdiv_i;
      default:            tx_cycles = clkdiv_i;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cnt   <= '0;
      tx_state_q <= TxState_Idle;
      tx_left <= 5'h1f;
    end else if (tx_enable_i) begin
      if (|cnt) cnt <= cnt - 'b1;
      else begin
        tx_state_q <= tx_state_d;
        tx_left <= tx_left - Log2DW'(buf_shift);
        cnt <= tx_cycles;
      end
    end
  end

  assign scl_o    = tx_state_q[0];
  assign sda_o    = buf_q[DataWidth-1];
  assign scl_en_o = tx_state_q[1];
  assign sda_en_o = tx_state_q[1];  // TODO: Same as `scl_en_o` for now
  assign sda_od_en_o = 1'b0;

endmodule
