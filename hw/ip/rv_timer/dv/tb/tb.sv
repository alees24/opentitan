// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
module tb;
  // dep packages
  import uvm_pkg::*;
  import dv_utils_pkg::*;
  import tl_agent_pkg::*;
  import rv_timer_env_pkg::*;
  import rv_timer_test_pkg::*;
  import top_racl_pkg::*;

  // macro includes
  `include "uvm_macros.svh"
  `include "dv_macros.svh"

  wire clk, rst_n;
  wire [NUM_HARTS-1:0][NUM_TIMERS-1:0] intr_timer_expired;
  wire [NUM_MAX_INTERRUPTS-1:0] interrupts;
  racl_policy_vec_t racl_policies;
  assign racl_policies = 0; // Not currently used

  // interfaces
  clk_rst_if clk_rst_if(.clk(clk), .rst_n(rst_n));
  pins_if #(NUM_MAX_INTERRUPTS) intr_if(.pins(interrupts));
  tl_if tl_if(.clk(clk), .rst_n(rst_n));

 `DV_ALERT_IF_CONNECT()

  // dut
  rv_timer dut (
    .clk_i                 (clk        ),
    .rst_ni                (rst_n      ),

    .tl_i                  (tl_if.h2d  ),
    .tl_o                  (tl_if.d2h  ),

    .alert_rx_i            (alert_rx   ),
    .alert_tx_o            (alert_tx   ),

    .racl_policies_i       (racl_policies),
    .racl_error_o          (),

    .intr_timer_expired_hart0_timer0_o(intr_timer_expired[0][0])
  );

  assign interrupts[0] = intr_timer_expired[0][0];

  initial begin
    // drive clk and rst_n from clk_if
    clk_rst_if.set_active();
    uvm_config_db#(virtual clk_rst_if)::set(null, "*.env", "clk_rst_vif", clk_rst_if);
    uvm_config_db#(intr_vif)::set(null, "*.env", "intr_vif", intr_if);
    uvm_config_db#(virtual tl_if)::set(null, "*.env.m_tl_agent*", "vif", tl_if);
    $timeformat(-12, 0, " ps", 12);
    run_test();
  end

endmodule
