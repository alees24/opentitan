// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Interrupt handling for a set of associated interrupts.
//
// The HCI defines four groups of interrupts:
// - Host Controller
// - PIO
// - Secondary Controller
// - Ring Header
//
// Each has an associated set of registers:
// - _INTR_STATUS        - Indicates the current interrupt status and permits interrupt clearing.
// - _INTR_STATUS_ENABLE - Enables/disables the setting of interrupt state by controller events.
// - _INTR_SIGNAL_ENABLE - Enables/disables the generation of interrupt signals from their status.
// - _INTR_FORCE         - Software-controlled forcing of the interrupt status for diagnostic use.

module i3c_intr
#(
  parameter int unsigned Width = 1
) (
  input               clk_i,
  input               rst_ni,

  // Interrupt event from internal logic.
  input   [Width-1:0] event_i,

  // Interrupt enable.
  input   [Width-1:0] status_en_i,
  // Set interrupt status bit in HCI register.
  output  [Width-1:0] set_status_o,
  // Interrupt force signal from HCI register.
  input   [Width-1:0] force_i,
  // Interrupt status bit from configuration registers.
  input   [Width-1:0] status_i,
  // Interrupt signal enable from HCI configuration.
  input   [Width-1:0] signal_en_i,

  // Interrupt signal to system.
  output logic        intr_o
);

  // Interrupt status bits are set by the occurrence of enabled events or software forcing for
  // diagnostic purposes.
  assign set_status_o = force_i | (event_i & status_en_i);

  // Re-time the interrupt output to aid timing and prevent glitches.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) intr_o <= '0;
    else intr_o <= |{signal_en_i & status_i};
  end

endmodule
