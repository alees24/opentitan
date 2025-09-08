// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Tri-stated inverting combinational clock buffer. There is no suitable primitive at present.
//
// TODO:  decide whether this is required within the IP block to ensure that the pattern detectors
// do not respond to disconnected/undefined inputs. In an actual chip deployment, pinmux surely
// guarantees a defined state to a disconnected input. Parameterized inclusion may be an option.

module clock_inv_en #(
  parameter bit HasScanMode = 1'b1,
  parameter bit NoFpgaBufG  = 1'b0  // only used in FPGA case
) (
  input   en_i,
  input   clk_i,
  input   scanmode_i,
  output  clk_no
);

  logic clk_n;
  prim_clock_inv #(
    .HasScanMode  (HasScanMode),
    .NoFpgaBufG   (NoFpgaBufG)
  ) u_inv (
    .clk_i      (clk_i),
    .scanmode_i (scanmode_i),
    .clk_no     (clk_n)
  );

  assign clk_no = en_i ? clk_n : 1'b0;

endmodule
