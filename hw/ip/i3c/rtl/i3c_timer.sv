// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Timer module that reports on timed events such as bus activity and receiver responses.
module i3c_timer
#(
  parameter int unsigned ClkFreq = 50_000_000
) (
  input     clk_i,
  input     rst_ni,

  input     rst_bus_avail_i,
  input     rst_bus_idle_i,
  input     rst_te0_recov_i,

  // TODO: Parameterized number of client timers?
  output    bus_avail_o,
  output    bus_idle_o,
  output    te0_recovery_o
);

  // Clock cycles per 500ns.
  localparam int unsigned CntMax = (ClkFreq + 1_999_999) / 2_000_000 - 1;
  localparam int unsigned CntW   = $clog2(CntMax + 1);

  // Free-running timer on main IP clock to produce a single-cycle tick every 500ns.
  logic [CntW-1:0] cnt;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cnt <= CntMax;
    end else begin
      cnt <= |cnt ? (cnt - 'b1) : CntMax;
    end
  end
  wire tick_half_us = ~|cnt;

  // Bus Available condition is entered after 1us, which for a free-running timer means counting
  // down from 3 to 0.
  logic [1:0] cnt_avail;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) cnt_avail <= 'd3;
    else if (rst_bus_avail_i) cnt_avail <= 'd3;
    else cnt_avail <= cnt_avail - (tick_half_us & |cnt_avail);
  end
  assign bus_avail_o = ~|cnt_avail;

  // Bus Idle condition is entered after 200us, which for a free-running timer means counting
  // down from 401 to 0.
  logic [8:0] cnt_idle;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) cnt_idle <= 'd401;
    else if (rst_bus_idle_i) cnt_idle <= 'd401;
    else cnt_idle <= cnt_idle - (tick_half_us & |cnt_idle);
  end
  assign bus_idle_o = ~|cnt_idle;

  // TE0 Recovery occurs after 60us of bus inactivity.
  logic [6:0] cnt_recov;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) cnt_recov <= 'd121;
    else if (rst_te0_recov_i) cnt_recov <= 'd121;
    else cnt_recov <= cnt_recov - (tick_half_us & |cnt_recov);
  end
  assign te0_recovery_o = ~|cnt_recov;

endmodule
