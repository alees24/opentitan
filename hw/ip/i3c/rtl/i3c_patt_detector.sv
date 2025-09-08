// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_patt_detector
  import prim_mubi_pkg::*;
#(
  parameter int unsigned Dummy = 0
) (
  input         rst_ni,

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
  output        target_reset_o,
  output        chip_reset_o,

  // DFT-related controls.
  input         mbist_en_i,
  input         scan_clk_i,
  input         scan_rst_ni,
  input mubi4_t scanmode_i
);

// TODO: Describe the HDR Exit and Restart patterns...
//
//
//

// Clock buffering.
logic scl_buf;
prim_clock_buf #(
  .RegionSel (1'b1)
) u_scl_buf (
  .clk_i  (scl_i),
  .clk_o  (scl_buf)
);

// Buffer SDA input and use it as a clock.
logic sda_clk;
prim_clock_buf #(
  .RegionSel (1'b1)
) u_sda_buf (
  .clk_i  (sda_i),
  .clk_o  (sda_clk)
);

// Invert SDA input and use it as a clock.
logic sda_clk_n;
prim_clock_inv #(
  .NoFpgaBufG(1'b1)
) u_sda_inv (
  .clk_i      (sda_i),
  .clk_no     (sda_clk_n),
  .scanmode_i (prim_mubi_pkg::mubi4_test_true_strict(scanmode_i))  // TODO: sync?
);

wire scl_rst_n = !scl_buf & hdr_mode_i;
logic [3:0] exit_det;
always_ff @(posedge sda_clk_n or negedge scl_rst_n) begin
  if (!scl_rst_n) exit_det <= '0;
  else exit_det <= {exit_det[2:0], 1'b1};
end

// HDR Exit pattern detected.
assign hdr_exit_det_o = exit_det[3];

// Possible Restart means exactly 2 falling edges of SDA and then rising edge of SDA.
logic poss_restart;
always_ff @(posedge sda_clk or negedge rst_ni) begin
  if (!rst_ni) poss_restart <= 1'b0;
  else poss_restart <= exit_det[1] & !exit_det[2];
end

// A possible restart is confirmed by the rising edge of SCL.
always_ff @(posedge scl_buf or negedge rst_ni) begin
  if (!rst_ni) hdr_restart_det_o <= 1'b0;
  else hdr_restart_det_o <= poss_restart;
end

// TODO:
assign target_reset_o = 1'b0;
assign chip_reset_o   = 1'b0;

endmodule
