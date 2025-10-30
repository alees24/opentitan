// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Tri-stated inverting combinational clock buffer. There is no suitable primitive at present.
//
// TODO:  decide whether this is required within the IP block to ensure that the pattern detectors
// do not respond to disconnected/undefined inputs. In an actual chip deployment, pinmux surely
// guarantees a defined state to a disconnected input. Parameterized inclusion may be an option.

module i3c_clock_buf_en #(
  parameter bit OutDisabled = 1'b0,  // output state when disabled.
  parameter bit NoFpgaBufG  = 1'b0   // only used in FPGA case
) (
  input   en_i,
  input   clk_i,
  input   scanmode_i,
  input   scan_clk_i,
  output  clk_o
);

  // Gated input clock.
  logic clk_gated;
  buf_en #(
    .Width        (1),
    .OutDisabled  (OutDisabled)
  ) u_buf (
    .en_i   (en_i),
    .in_i   (clk_i),
    .out_o  (clk_gated)
  );

  // Scan chain clocking.
  prim_clock_mux2 #(
    .NoFpgaBufG (NoFpgaBufG)
  ) u_mux (
    .clk0_i (clk_gated),
    .clk1_i (scan_clk_i),
    .sel_i  (scanmode_i),
    .clk_o  (clk_o)
  );

endmodule
