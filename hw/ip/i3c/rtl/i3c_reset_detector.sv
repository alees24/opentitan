// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Reset detector for I3C target; this logic detects the Target Reset pattern and is expected
// to be situated in a low power 'Always On' domain. It does not have a free-running clock of its
// own, but responds directly to the Controlled-supplied SCL signal on the I3C bus.
//
// The action taken upon detection of the Target Reset pattern depends upon configuration supplied
// earlier:
//
// - Issue Peripheral Reset.
// - Issue Whole Target Reset.
//
// A secondary occurrence of the Target Reset pattern without an intervening acknowledgement
// is escalated to a Whole Target Reset.

module i3c_reset_detector
  import i3c_pkg::*;
  import prim_mubi_pkg::*;
(
  // No free-running clock from the IP block core; driven by controller-supplied SCL.
  // Asynchronous reset.
  input                   rst_aon_ni,

  // Request from the IP block.
  input  i3c_rstdet_req_t req_i, 
  // Reponse to the IP block.
  output i3c_rstdet_rsp_t rsp_o,

  // I3C I/O signals being monitored.
  input                   scl_i,
  input                   sda_i,

  // Control signals in response to Target Reset pattern.
  output                  target_reset_o,  // I3C target only.
  output                  chip_reset_o,    // Entire chip.

  // DFT-related signals.
  input                   scan_clk_i,
  input  mubi4_t          scanmode_i
);

  logic scanmode;
  assign scanmode = mubi4_test_true_strict(scanmode_i);

  // TODO: The detector does not become active immediately upon request?
  logic enable;
  assign enable = req_i.enable;
  // TODO:
  assign rsp_o.activating = 1'b0;
  assign rsp_o.active = req_i.enable;
  assign rsp_o.peri_rst_det = target_reset_o;
  assign rsp_o.targ_rst_det = chip_reset_o;
  logic active;
  assign active = rsp_o.active;

  // SCL and SDA inputs, and their negations, are used in as clocks in some parts of the IP block,
  // and the logic driven also required a DFT scan chain clock.
  //
  // Additionally the 'enable_i' signal can be used to protect the logic from disconnected SCL/SDA
  // inputs or other activity when the Target logic is not ready to process traffic.
  logic scl_buf;
  logic scl_buf_n;
  logic sda_clk;
  logic sda_clk_n;

  // Buffered SCL input.
  i3c_clock_buf_en #(
    .OutDisabled  (1'b1),
    .NoFpgaBufG   (1'b0)
  ) u_scl_buf (
    .en_i       (enable),
    .clk_i      (scl_i),
    .scanmode_i (scanmode),
    .scan_clk_i (scan_clk_i),
    .clk_o      (scl_buf)
  );
  // Inverted buffered SCL input.
  i3c_clock_inv_en #(
    .OutDisabled  (1'b0),
    .NoFpgaBufG   (1'b0)
  ) u_scl_inv (
    .en_i       (enable),
    .clk_i      (scl_i),
    .scanmode_i (scanmode),
    .scan_clk_i (scan_clk_i),
    .clk_no     (scl_buf_n)
  );

  // Buffer SDA input and use it as a clock.
  i3c_clock_buf_en #(
    .OutDisabled  (1'b1),
    .NoFpgaBufG   (1'b0)
  ) u_sda0_clk (
    .en_i       (enable),
    .clk_i      (sda_i),
    .scanmode_i (scanmode),
    .scan_clk_i (scan_clk_i),
    .clk_o      (sda_clk)
  );
  // Inverted buffered SDA input, used as a clock.
  i3c_clock_inv_en #(
    .OutDisabled  (1'b0),
    .NoFpgaBufG   (1'b0)
  ) u_sda0_clk_n (
    .en_i       (enable),
    .clk_i      (sda_i),
    .scanmode_i (scanmode),
    .scan_clk_i (scan_clk_i),
    .clk_no     (sda_clk_n)
  );

  // Target Reset detection; operational in all modes.
  // - count 7 or 8 positive edges of SDA before SCL rises, followed by Sr and P.
  logic [2:0] rst_cnt;
  always_ff @(posedge sda_clk or negedge scl_buf_n) begin
    if (!scl_buf_n) rst_cnt <= '0;
    else if (active) rst_cnt <= rst_cnt + ~&rst_cnt;
  end

  // Following 14 transitions on SDA, a rising edge primes the detection of an ensuing Sr and P.
  logic poss_reset_pend;
  always_ff @(posedge scl_buf or negedge rst_aon_ni) begin
    if (!rst_aon_ni) poss_reset_pend <= 1'b0;
    else if (active) poss_reset_pend <= (rst_cnt >= 'h7);
  end
  // Detection of Sr (restart).
  logic poss_reset_start;
  always_ff @(posedge sda_clk_n or negedge rst_aon_ni) begin
    if (!rst_aon_ni) poss_reset_start <= 1'b0;
    else if (active) poss_reset_start <= scl_buf & poss_reset_pend;
  end
  // Detection of P (stop).
  logic targ_reset_det;
  always_ff @(posedge sda_clk or negedge rst_aon_ni) begin
    if (!rst_aon_ni) targ_reset_det <= 1'b0;
    else if (active) targ_reset_det <= scl_buf & poss_reset_start;
  end

  // These are RISING EDGE signals which will be asserted for an indefinite period and the receiver
  // shall synchronize into the target clock domain and respond to the rising edge there.
  // This avoids the requirement for clocked logic in the AON domain.
  //
  // I3C Peripheral only.
  assign target_reset_o = targ_reset_det & req_i.rst_periph;
  // Entire chip.
  assign chip_reset_o   = targ_reset_det & req_i.rst_target;

endmodule
