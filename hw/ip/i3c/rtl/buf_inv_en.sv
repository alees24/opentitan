// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Tri-stated inverting combinational buffer. There is no suitable primitive at present.
//
// TODO:  decide whether this is required within the IP block to ensure that the pattern detectors
// do not respond to disconnected/undefined inputs. In an actual chip deployment, pinmux surely
// guarantees a defined state to a disconnected input. Parameterized inclusion may be an option.

module buf_inv_en #(
  parameter int Width = 1,
  parameter logic [Width-1:0] OutDisabled = {Width{1'b1}}
) (
  input              en_i,
  input  [Width-1:0] in_i,
  output [Width-1:0] out_o
);

  logic [Width-1:0] out_int;
  prim_buf #(.Width(Width)) u_buf (
    .in_i   (in_i),
    .out_o  (out_int)
  );

  assign out_o = en_i ? ~out_int : OutDisabled;

endmodule
