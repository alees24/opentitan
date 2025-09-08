// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_patt_detector #(
  parameter int unsigned Dummy = 0
) (
  input         rst_aon_ni,

  // Control inputs.
  input         enable_i,

  // Configuration.
  input   [7:0] rst_action_i,

  // I3C I/O signals being monitored.
  input         scl_i,
  input         sda_i,

  // State.
  input         activate_i,
  input         hdr_mode_i,
  output        active_o,

  // HDR pattern detection.
  output        hdr_exit_det_o,
  output logic  hdr_restart_det_o,

  // Control signals in response to Target Reset pattern.
  output        target_reset_o,  // I3C target only.
  output        chip_reset_o,    // Entire chip.

  // DFT-related controls.
  input         scanmode_i
);

  import i3c_consts_pkg::*;

  // TODO: Describe the HDR Exit and Restart patterns...
  //
  //
  //

  // Clock buffering.
  logic scl_buf;
  prim_clock_mux2 #(
    .NoFpgaBufG (1'b0)
  ) u_scl_buf (
    .clk0_i (1'b1),
    .clk1_i (scl_i),
    .sel_i  (enable_i),
    .clk_o  (scl_buf)
  );

  // Buffer SDA input and use it as a clock.
  logic sda_clk;
  prim_clock_mux2 #(
    .NoFpgaBufG (1'b0)
  ) u_sda_buf (
    .clk0_i (1'b1),
    .clk1_i (sda_i),
    .sel_i  (enable_i),
    .clk_o  (sda_clk)
  );

  // Invert SDA input and use it as a clock.
  logic sda_clk_n;
  clock_inv_en #(
    .HasScanMode  (1'b1),
    .NoFpgaBufG   (1'b0)
  ) u_sda_inv (
    .en_i       (enable_i),
    .clk_i      (sda_i),
    .scanmode_i (scanmode_i),
    .clk_no     (sda_clk_n)
  );

  // SCL low in HDR state enables operation of the Exit and Restart detector.
  wire scl_rst_n = !scl_buf & hdr_mode_i;
  logic [3:0] exit_det;
  always_ff @(posedge sda_clk_n or negedge scl_rst_n) begin
    if (!scl_rst_n) exit_det <= '0;
    else exit_det <= {exit_det[2:0], 1'b1};
  end

  // HDR Exit pattern detected.
  // TODO: Check any timing requirements on HDR Exit/Restart patterns; is the recipient going to
  // respond quickly enough to a notification even with CDC delays.
  assign hdr_exit_det_o = exit_det[3];

  // Possible Restart means exactly 2 falling edges of SDA and then rising edge of SDA.
  logic poss_restart;
  always_ff @(posedge sda_clk or negedge rst_aon_ni) begin
    if (!rst_aon_ni) poss_restart <= 1'b0;
    else poss_restart <= exit_det[1] & !exit_det[2];
  end

  // A possible restart is confirmed by the rising edge of SCL.
  always_ff @(posedge scl_buf or negedge rst_aon_ni) begin
    if (!rst_aon_ni) hdr_restart_det_o <= 1'b0;
    else hdr_restart_det_o <= poss_restart;
  end

  // Target Reset detection; operational in all modes.
  // - count 14 or 15 positive edges of SDA before SCL rises, followed by Sr and P.
  logic [3:0] rst_cnt;
  always_ff @(posedge sda_clk or negedge scl_buf) begin
    if (!scl_buf) rst_cnt <= '0;
    else rst_cnt <= rst_cnt + ~&rst_cnt;
  end

  // Following 14 transitions on SDA, a rising edge primes the detection of an ensuing Sr and P.
  logic poss_reset_pend;
  always_ff @(posedge scl_buf or negedge rst_aon_ni) begin
    if (!rst_aon_ni) poss_reset_pend <= 1'b0;
    else poss_reset_pend <= (rst_cnt >= 'hE) & scl_buf;
  end
  // Detection of Sr (restart).
  logic poss_reset_start;
  always_ff @(posedge sda_clk or negedge rst_aon_ni) begin
    if (!rst_aon_ni) poss_reset_start <= 1'b0;
    else poss_reset_start <= scl_buf & poss_reset_pend;
  end
  // Detection of P (stop).
  logic targ_reset_det;
  always_ff @(posedge sda_clk_n or negedge rst_aon_ni) begin
    if (!rst_aon_ni) targ_reset_det <= 1'b0;
    else targ_reset_det <= scl_buf & poss_reset_start;
  end

  // These are RISING EDGE signals which will be asserted for an indefinite period and the receiver
  // shall synchronize into the target clock domain and respond to the rising edge there.
  // This avoids the requirement for clocked logic in the AON domain.
  //
  // I3C Peripheral only.
  assign target_reset_o = targ_reset_det & (rst_action_i == RstAct_ResetPeripheral);
  // Entire chip.
  assign chip_reset_o   = targ_reset_det & (rst_action_i == RstAct_ResetTarget);

endmodule
