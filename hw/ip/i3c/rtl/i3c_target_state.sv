// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C Target state tracking.

module i3c_target_state
  import i3c_target_pkg::*;
(
  // Main IP clock and reset.
  input                    clk_i,
  input                    rst_ni,
  // Reset to the transceiver logic.
  input                    rst_trx_ni,

  // Control inputs from software.
  input                    enable_i,
  input                    sw_reset_i,

  // Notifications from Target core.
  input                    error_i,

  // Requests to the Target reset logic.
  output                   en_req_o,
  output                   rst_req_o,
  output                   rst_force_o,

  // Responses from the Target reset logic.
  input                    en_done_i,   // Expected response to `en_req`
  input                    rst_done_i,  // Expected response to `rst_req`

  // Current global state of the Target.
  output i3c_targ_gstate_e tgstate_o
);

  i3c_targ_gstate_e tgstate_q, tgstate_d;
  logic sw_rst_pend;

  // Next Target global state.
  // - note that a second 'sw_reset' request becomes a forced reset as a last resort recovery
  //   before an asynchronous reset of the entire IP block.
  always_comb begin
    unique case (tgstate_q)
      // Software resets are ignored when 'Disabled'
      TGState_Disabled:  tgstate_d = enable_i   ? TGState_Enabling  : TGState_Disabled;
      TGState_Disabling: tgstate_d = rst_done_i ? TGState_Disabled  : TGState_Disabling;
      TGState_Enabling:  tgstate_d = sw_reset_i ? TGState_EnReset   :
                                     en_done_i  ? TGState_Enabled   : TGState_Enabling;
      TGState_Enabled:   tgstate_d = sw_reset_i ? TGState_Disabling :
                                     enable_i   ? TGState_Enabled   : TGState_Disabling;
      TGState_EnReset:   tgstate_d = sw_reset_i ? TGState_Disabled  :  // Forced reset.
                                     en_done_i  ? TGState_Disabling : TGState_EnReset;
      // Awaiting a reset from software.
      TGState_Error:     tgstate_d = sw_reset_i ? TGState_Disabled : TGState_Error;
      // Invalid states; transition to Error state and await a software reset.
      default:           tgstate_d = sw_reset_i ? TGState_Disabled : TGState_Error;
    endcase
  end

  // Retain the global state of the Target.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) tgstate_q <= TGState_Disabled;
    else tgstate_q <= tgstate_d;
  end
  assign tgstate_o = tgstate_q;

  // Request to enable the transceiver logic an connect to the I3C bus.
  assign en_req_o    = enable_i   & (tgstate_q == TGState_Disabled);
  // Forced reset; this is a recovery mechanism in case the first request failed.
  assign rst_force_o = sw_reset_i & (tgstate_q == TGState_EnReset);
  // Request to disconnect from the I3C bus and reset the transceiver logic.
  assign rst_req_o   = |{(sw_reset_i | enable_i) & (tgstate_q == TGState_Enabled),
                         rst_force_o};

  // TODO(#31337): Under construction.
  logic unused_;
  assign unused_ = ^{enable_i, error_i, rst_trx_ni, sw_rst_pend};

endmodule
