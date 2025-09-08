// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module i3c_phy #(
  parameter int unsigned NumSDALanes = 1
) (
  // Pullup enables.
  //
  // TODO: The pullup resistances need to be quite small so it may be more
  // appropriate to drive the enables off-chip such that the PHY is not
  // concerned with pullups.
  input                   scl_pu_en_i,
  input                   sda_pu_en_i,

  // SCL driver enables.
  input                   scl_pp_en_i,
  input                   scl_od_en_i,
  // SCL signal from IP block.
  input                   scl_i,

  // SDA driver enables.
  input                   sda_pp_en_i,
  input                   sda_od_en_i,
  // SDA signal from IP block.
  input [NumSDALanes-1:0] sda_i,

  // I3C I/O signals.
  inout                   scl,
  inout [NumSDALanes-1:0] sda
);

`ifdef VERILATOR
// TODO: Can we build a better model for use within Verilator simulation?
assign scl = scl_pp_en_i ? scl_i : ((scl_od_en_i & !scl_i) ? 1'b0 : (scl_pu_en_i ? 1'b1 : 1'bZ));
assign sda = sda_pp_en_i ? sda_i : ((sda_od_en_i & !sda_i) ?   '0 : (sda_pu_en_i ?   '1 :   'Z));
`else
// TODO: Some of this is not multi-lane aware.
// Push-pull drivers.
assign (strong0, strong1) scl = scl_pp_en_i ? scl_i : 1'bZ;
assign (strong0, strong1) sda = sda_pp_en_i ? sda_i :   'Z;
// Open drain drivers.
assign (strong0, weak1)   scl = (scl_od_en_i & !scl_i) ? 1'b0 : 1'bZ;
assign (strong0, weak1)   sda = (sda_od_en_i & !sda_i) ?   '0 :   'Z;
// Pullups for open drain operation.
assign (weak0, pull1) scl = scl_pu_en_i ? 1'b1 : 1'bZ;
assign (weak0, pull1) sda = sda_pu_en_i ?   '1 :   'Z;
`endif

endmodule
