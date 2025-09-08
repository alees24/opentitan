// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C receiver.

module i3c_rx #(
  parameter int unsigned DataWidth = 32,
  parameter int unsigned CntWidth = 16
) (
  input                     clk_i,
  input                     rst_ni,

  // Clock and reset for `always on` bus monitoring.
  input                     clk_aon_i,
  input                     rst_aon_ni,

  input                     rx_enable_i,
  input                     ddr_enable_i,

  // Buffer writing.
  output                    buf_wr_o,
  output  [DataWidth/8-1:0] buf_wmask_o,
  output    [DataWidth-1:0] buf_wdata_o,

  // I3C I/O signaling.
  input                     scl_i,
  input                     sda_i
);

  // Synchronize the I3C input signals to the AON clock.
  logic sda_q, sda_d;
  logic scl_q, scl_d;
  prim_flop_2sync #(
    .Width      (2),
    .ResetValue ('1)
  ) u_sync(
    .clk_i    (clk_aon_i),
    .rst_ni   (rst_aon_ni),
    .d_i      ({sda_i, scl_i}),
    .q_o      ({sda_d, scl_d})
  );

  typedef enum logic [2:0] {
    RxState_Idle,
    RxState_ClkLoSetupD,
    RxState_ClkHiHoldD,
    RxState_ClkHiSetupD,
    RxState_ClkLoHoldD
  } rx_state_e;

  // Reception state.
  rx_state_e rx_state_d;
  always_comb begin
    rx_state_d = RxState_Idle;
    if (scl_q != scl_d) begin
      rx_state_d = scl_q ? RxState_ClkLoHoldD : RxState_ClkHiHoldD;
    end else begin
      case (rx_state_q)
        RxState_ClkLoSetupD: rx_state_d = RxState_ClkHiHoldD;
        RxState_ClkHiHoldD:  rx_state_d = RxState_ClkHiSetupD;
        RxState_ClkHiSetupD: rx_state_d = RxState_ClkLoHoldD;
        RxState_ClkLoHoldD:  rx_state_d = RxState_ClkLoSetupD;
        default:             rx_state_d = RxState_Idle;
      endcase
    end
  end

  // Receiver logic operates on the `always on` clock so that it can remain operational when the
  // rese of the IP block is in a sleep state.
  rx_state_e rx_state_q;
  always_ff @(posedge clk_aon_i or negedge rst_aon_ni) begin
    if (!rst_aon_ni) begin
      scl_q <= 1'b1;
      sda_q <= 1'b1;
      rx_state_q <= RxState_Idle;
    end else if (rx_enable_i) begin
      scl_q <= scl_d;
      sda_q <= sda_d;
      rx_state_q <= rx_state_d;
    end
  end

  // Sample the current state of SDA?
  logic rx_sample;
  assign rx_sample = (rx_state_q == RxState_ClkHiHoldD) ||
                     (rx_state_q == RxState_ClkLoHoldD && ddr_enable_i);

  // Collect the sampled data.
  logic [DataWidth-1:0] buf_q;
  always_ff @(posedge clk_aon_i or negedge rst_aon_ni) begin
    if (!rst_aon_ni) begin
      buf_q <= '1;
    end else if (rx_enable_i && rx_sample) begin
      buf_q <= {buf_q[DataWidth-2:0], sda_q};
    end
  end

  // TODO: I guess we need a CDC here from AON to main.

  // TODO: Count the bits and emit an appropriate `wmask`
  assign buf_wr_o    = 1'b0;
  assign buf_wmask_o = '1;
  assign buf_wdata_o = buf_q;

endmodule
