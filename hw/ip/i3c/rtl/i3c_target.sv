// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_target
#(
) (
  // Clock and reset for system interface.
  input                   clk_i,
  input                   rst_ni,

  // CLock and reset for `always on` bus monitoring.
  input                   clk_aon_i,
  input                   rst_aon_ni,

  input                   enable_i
);

endmodule
