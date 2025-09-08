// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_target
  import i3c_pkg::*;
  import i3c_reg_pkg::*;
#(
  // TODO:
  parameter int unsigned DataWidth = 32
) (
  // Clock and reset for system interface.
  input                   clk_i,
  input                   rst_ni,

  // CLock and reset for `always on` bus monitoring.
  input                   clk_aon_i,
  input                   rst_aon_ni,

  // Control inputs.
  input                   enable_i,
  input                   sw_reset_i,

  // Configuration settings.
  input i3c_reg2hw_t      reg2hw_i
  // State information, presented via TTI.
);

endmodule
