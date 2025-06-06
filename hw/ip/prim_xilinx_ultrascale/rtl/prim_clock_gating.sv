// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module prim_clock_gating #(
  parameter bit NoFpgaGate = 1'b0,
  parameter bit FpgaBufGlobal = 1'b1 // No function in Ultrascale
) (
  input        clk_i,
  input        en_i,
  input        test_en_i,
  output logic clk_o
);

  if (NoFpgaGate) begin : gen_no_gate
    assign clk_o = clk_i;
  end else begin : gen_gate
    BUFGCE #(
      .SIM_DEVICE("ULTRASCALE")
    ) u_bufgce (
      .I (clk_i),
      .CE(en_i | test_en_i),
      .O (clk_o)
    );
  end



endmodule
