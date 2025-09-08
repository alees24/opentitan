// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Simple Verilator testbench for the I3C core.
module tb(
  input clk_i,
  input rst_ni,

  input clk_aon_i,
  input rst_aon_ni
);

// Properties of TL-UL interface.
localparam int unsigned AW  = top_pkg::TL_AW;
localparam int unsigned DW  = top_pkg::TL_DW;
localparam int unsigned DBW = top_pkg::TL_DBW;

// Simulation lifetime.
initial begin
  $write("I3C simulation starting! Logging to sim.fst\n");
end
logic [18:0] cnt;
always_ff @(posedge clk_i) begin
  cnt <= cnt + 'b1; if (cnt[11]) $finish;
end

// TODO:
tlul_pkg::tl_h2d_t tl_h2d;
tlul_pkg::tl_d2h_t tl_d2h;

logic lsio_trigger;
logic intr_done;

// I3C bus.
logic scl, sda;

// I3C driver signals.
logic cio_scl;
logic cio_scl_en;
logic cio_sda;
logic cio_sda_en;
logic cio_sda_od_en;

prim_alert_pkg::alert_rx_t alert_rx;
prim_alert_pkg::alert_tx_t alert_tx;

i3c dut(
  .clk_i            (clk_i),
  .rst_ni           (rst_ni),

  .clk_aon_i        (clk_aon_i),
  .rst_aon_ni       (rst_aon_ni),

  .ram_cfg_i        ('0),
  .ram_cfg_rsp_o    (),

  // Register interface.
  .tl_i             (tl_h2d),
  .tl_o             (tl_d2h),

  // Alerts.
  .alert_rx_i       (alert_rx),
  .alert_tx_o       (alert_tx),

  // RACL interface.
  .racl_policies_i  (top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
  .racl_error_o     (),

  // I3C I/O signaling.
  .cio_scl_i        (scl),
  .cio_scl_o        (cio_scl),
  .cio_scl_en_o     (cio_scl_en),
  .cio_sda_i        (sda),
  .cio_sda_o        (cio_sda),
  .cio_sda_en_o     (cio_sda_en),
  .cio_sda_od_en_o  (cio_sda_od_en),

  // Interrupt-signaling to hardware
  .lsio_trigger_o   (lsio_trigger),

  // Interrupts.
  .intr_done_o      (intr_done),

  // DFT-related controls.
  .mbist_en_i       (1'b0),
  .scan_clk_i       (1'b0),
  .scan_rst_ni      (1'b1),
  .scanmode_i       (prim_mubi_pkg::MuBi4False)
);

// I3C driver.
//
// SCL is always push-pull driven.
assign scl = cio_scl_en ? cio_scl : 1'bZ;
// SDA may be either open drain or push-pull.
assign sda = (cio_sda_od_en & !cio_sda) ? 1'b0 : (cio_sda_en ? cio_sda : 1'bZ);

// TODO: Move the following into an interface/module?

// Host->device TL-UL with no integrity.
tlul_pkg::tl_h2d_t tl_h2d_int;
// Configuration file descriptor.
int cfg;
// Program the configuration registers.
logic               cfg_prog;
tlul_pkg::tl_a_op_e cfg_opcode;
logic [AW-1:0]      cfg_addr;
logic [DBW-1:0]     cfg_wmask;
logic [DW-1:0]      cfg_wdata;
always_comb begin
  tl_h2d_int = '0;
  // A channel carries write request.
  tl_h2d_int.a_valid   = cfg_prog;
  tl_h2d_int.a_opcode  = cfg_opcode;
  tl_h2d_int.a_param   = '0;
  tl_h2d_int.a_address = AW'(cfg_addr);
  tl_h2d_int.a_source  = '0;
  tl_h2d_int.a_size    = 'h2;
  tl_h2d_int.a_mask    = (cfg_opcode == tlul_pkg::Get) ? {DBW{1'b1}} : cfg_wmask;
  tl_h2d_int.a_data    = cfg_wdata;
  tl_h2d_int.a_user.instr_type = prim_mubi_pkg::MuBi4False;
  // D channel always accepts response immediately.
  tl_h2d_int.d_ready   = 1'b1;
end

tlul_cmd_intg_gen u_intg_gen(
  .tl_i (tl_h2d_int),
  .tl_o (tl_h2d)
);

// Open configuration file.
initial begin
  string cfg_filename;
  cfg_filename = "cfg_regs";
  cfg_prog = 1'b0;
  cfg = $fopen(cfg_filename, "r");
  if (~|cfg) $display("WARNING: Unable to open config file '%s'", cfg_filename);
end

// Advance the configuration logic; this is just a simple means of programming a
// set of configuration registers.
always_ff @(posedge clk_i or negedge rst_ni) begin
  if (!rst_ni) begin
    cfg_prog <= 1'b1;
  end else if (tl_h2d.a_valid & tl_d2h.a_ready) begin
    if ($feof(cfg)) begin
      cfg_prog <= 1'b0;
    end else begin
      $fscanf(cfg, "%x:%x:%x:%x\n", cfg_opcode, cfg_addr, cfg_wmask, cfg_wdata);
      case (cfg_opcode)
        tlul_pkg::PutFullData: $display("Write [%x]:%x", cfg_addr, cfg_wdata);
        tlul_pkg::PutPartialData: $display("Write [%x]:%x,%x", cfg_addr, cfg_wmask, cfg_wdata);
        default: $display("Read [%x]", cfg_addr);
      endcase
    end
  end
end

endmodule
