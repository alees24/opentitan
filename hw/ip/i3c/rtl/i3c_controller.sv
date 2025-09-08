// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_controller
#(
) (
  // Clock and reset for system interface.
  input                   clk_i,
  input                   rst_ni,

  // Clock and reset for `always on` bus monitoring.
  input                   clk_aon_i,
  input                   rst_aon_ni,

  input                   enable_i,

  // Pullup enables.
  output                  scl_pu_en_o,
  output                  sda_pu_en_o
);

// Drive pullup enables.
assign scl_pu_en_o = 1'b0;
assign sda_pu_en_o = 1'b0;

endmodule
