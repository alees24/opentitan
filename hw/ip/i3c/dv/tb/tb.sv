// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Simple Verilator testbench for the I3C core.

`ifndef VERILATOR
`define STANDALONE
`endif

module tb(
`ifndef STANDALONE
  input clk_i,
  input rst_ni,

  input clk_aon_i,
  input rst_aon_ni
`endif
);

`ifdef STANDALONE
logic clk_i;
logic rst_ni;

logic clk_aon_i;
logic rst_aon_ni;
`endif

// Behavioural utilities to help with initial bring up.
`include "../env/i3c_utils.svh"

// Connect the Controller and Target together on a single I3C bus?
localparam bit SingleBus = 1;

// Number of SDA lines; presently, because HDR-BT is not supported, this must be 1.
localparam int unsigned NumSDALanes = 1;

// Properties of TL-UL interface.
localparam int unsigned AW  = top_pkg::TL_AW;
localparam int unsigned DW  = top_pkg::TL_DW;
localparam int unsigned DBW = top_pkg::TL_DBW;

// Simulation lifetime.
logic [15:0] cnt;
initial begin
  $display("I3C simulation starting.\nLogging to sim.fst\n");
end
always_ff @(posedge clk_i or negedge rst_ni) begin
  if (!rst_ni) cnt <= '0;
  else begin
    cnt <= cnt + 'b1;
    if (cnt[11]) $finish;
  end
end

`ifdef STANDALONE
// Main clock to the IP block (50MHz).
localparam int unsigned rst_period = 100 * 1000;
localparam int unsigned clk_period = 20 * 1000;
initial begin
  clk_i = 1'b1;
  rst_ni = 1'b0;
  #rst_period rst_ni = 1'b1;
end
always #(clk_period/2) clk_i = !clk_i;

// Always On clock (100kHz).
localparam int unsigned aon_rst_period = 100 * 1000 * 1000;
localparam int unsigned aon_clk_period = 10 * 1000 * 1000;
initial begin
  clk_aon_i = 1'b1;
  rst_aon_ni = 1'b0;
  #aon_rst_period rst_aon_ni = 1'b1;
end
always #(aon_clk_period/2) clk_aon_i = !clk_aon_i;
`endif

// Message data to be sent over the I3C bus.
byte msg_data[$];
bit msg_bits[$];

// TODO:
tlul_pkg::tl_h2d_t tl_h2d;
tlul_pkg::tl_d2h_t tl_d2h;

logic lsio_trigger;
logic intr_done;
logic intr_error;

// I3C buses.
wire ctrl_scl, targ_scl;
wire [NumSDALanes-1:0] ctrl_sda, targ_sda;

// I3C Controller driver signals.
logic cio_ctrl_scl;
logic cio_ctrl_scl_pp_en;
logic cio_ctrl_scl_od_en;
logic cio_ctrl_sda_pp_en;
logic cio_ctrl_sda_od_en;
logic [NumSDALanes-1:0] cio_ctrl_sda;

// Pullup enables.
logic cio_ctrl_scl_pu_en;
logic cio_ctrl_sda_pu_en;

// High-keeper enables.
logic cio_scl_hk_en;
logic cio_sda_hk_en;

// I3C Target driver signals.
logic cio_targ_scl;
logic cio_targ_scl_pp_en;
logic cio_targ_scl_od_en;
logic cio_targ_sda_pp_en;
logic cio_targ_sda_od_en;
logic [NumSDALanes-1:0] cio_targ_sda;

// Chip reset request.
logic cio_chip_rst_req;

// Use a single PHY if the Controller and Target are on a single bus?
wire single_phy = SingleBus & 1'b1;

prim_alert_pkg::alert_rx_t alert_rx;
prim_alert_pkg::alert_tx_t alert_tx;

// Device Under Test.
i3c dut(
  .clk_i                (clk_i),
  .rst_ni               (rst_ni),

  .clk_aon_i            (clk_aon_i),
  .rst_aon_ni           (rst_aon_ni),

  .ram_cfg_i            ('0),
  .ram_cfg_rsp_o        (),

  // Register interface.
  .tl_i                 (tl_h2d),
  .tl_o                 (tl_d2h),

  // Alerts.
  .alert_rx_i           (alert_rx),
  .alert_tx_o           (alert_tx),

  // RACL interface.
  .racl_policies_i      (top_racl_pkg::RACL_POLICY_VEC_DEFAULT),
  .racl_error_o         (),
  // I3C Controller I/O signaling.
  .cio_ctrl_scl_i       (ctrl_scl),
  .cio_ctrl_scl_o       (cio_ctrl_scl),
  .cio_ctrl_scl_pp_en_o (cio_ctrl_scl_pp_en),
  .cio_ctrl_scl_od_en_o (cio_ctrl_scl_od_en),
  .cio_ctrl_sda_i       (ctrl_sda),
  .cio_ctrl_sda_o       (cio_ctrl_sda),
  .cio_ctrl_sda_pp_en_o (cio_ctrl_sda_pp_en),
  .cio_ctrl_sda_od_en_o (cio_ctrl_sda_od_en),

  // Pullup enables for open drain intervals.
  .cio_ctrl_scl_pu_en_o (cio_ctrl_scl_pu_en),
  .cio_ctrl_sda_pu_en_o (cio_ctrl_sda_pu_en),

  // High-keeper enables.
  .cio_scl_hk_en_o      (cio_scl_hk_en),
  .cio_sda_hk_en_o      (cio_sda_hk_en),

  // I3C Target I/O signaling.
  .cio_targ_scl_i       (targ_scl),
  .cio_targ_sda_i       (targ_sda),
  .cio_targ_sda_o       (cio_targ_sda),
  .cio_targ_sda_pp_en_o (cio_targ_sda_pp_en),
  .cio_targ_sda_od_en_o (cio_targ_sda_od_en),

  // Chip reset request.
  .cio_chip_rst_req_o   (cio_chip_rst_req),

  // Interrupt-signaling to hardware
  .lsio_trigger_o       (lsio_trigger),

  // Interrupts.
  .intr_done_o          (intr_done),
  .intr_error_o         (intr_error),

  // DFT-related controls.
  .mbist_en_i           (1'b0),
  .scan_clk_i           (1'b0),
  .scan_rst_ni          (1'b1),
  .scanmode_i           (prim_mubi_pkg::MuBi4False)
);

// Control signals to the I3C Controller PHY, which may be the only PHY.
// This supports two modes of operation:
// - Primary Controller (only; cannot become Secondary) with Target on the same bus.
// - Primary/Secondary Controller, capable of switching roles.
wire prim_phy_sda_pp_en = cio_ctrl_sda_pp_en | (single_phy & cio_targ_sda_pp_en);
wire prim_phy_sda_od_en = cio_ctrl_sda_od_en | (single_phy & cio_targ_sda_od_en);
// Each party has the capacity to drive the data line into the PHY low.
wire prim_phy_ctrl_sda  = cio_ctrl_sda_pp_en ? cio_ctrl_sda : !(cio_ctrl_sda_od_en & !cio_ctrl_sda);
wire prim_phy_targ_sda  = cio_targ_sda_pp_en ? cio_targ_sda : !(cio_targ_sda_od_en & !cio_targ_sda);
// Data line is high only if both parties agree that it shall be high.
wire prim_phy_sda       = prim_phy_ctrl_sda & (prim_phy_targ_sda | !single_phy);

// I3C Controller driver; anticipated to be outside of the i3c IP block itself.
i3c_phy #(
  .NumSDALanes  (NumSDALanes)
) u_ctrl_phy (
  // Pull-up enables.
  .scl_pu_en_i  (cio_ctrl_scl_pu_en),
  .sda_pu_en_i  (cio_ctrl_sda_pu_en),

  // SCL driver enables.
  .scl_pp_en_i  (cio_ctrl_scl_pp_en),
  .scl_od_en_i  (cio_ctrl_scl_od_en),
  // SCL signal from IP block.
  .scl_i        (cio_ctrl_scl),

  // SDA driver enables.
  .sda_pp_en_i  (prim_phy_sda_pp_en),
  .sda_od_en_i  (prim_phy_sda_od_en),
  // SDA signal from IP block.
  .sda_i        (prim_phy_sda),

  // I3C I/O signals.
  .scl          (ctrl_scl),
  .sda          (ctrl_sda)
);

// I3C Target driver; anticipated to be outside of the i3c IP block itself,
// if present.
// - This secondary PHY would be used in the event that the IP block is deployed as separated
//   controller and target on two different I3C buses. In this case the Controller must remain
//   as the Primary Controller and cannot hand over the bus to another controller.
wire sec_phy_sda_pp_en = cio_targ_sda_pp_en & !single_phy;
wire sec_phy_sda_od_en = cio_targ_sda_od_en & !single_phy;

i3c_phy #(
  .NumSDALanes  (NumSDALanes)
) u_targ_phy (
  // Target does not drive pull-ups.
  .scl_pu_en_i  (1'b0),
  .sda_pu_en_i  (1'b0),

  // Target does not drive SCL line (no support for HDR-BT mode).
  .scl_pp_en_i  (1'b0),
  .scl_od_en_i  (1'b0),
  .scl_i        (1'b1),

  // SDA driver enables.
  .sda_pp_en_i  (sec_phy_sda_pp_en),
  .sda_od_en_i  (sec_phy_sda_od_en),
  // SDA signal from IP block.
  .sda_i        (cio_targ_sda),

  // I3C I/O signals.
  .scl          (targ_scl),
  .sda          (targ_sda)
);

// Optionally tie the I3C Controller and Target together on a single bus.
assign targ_scl = ctrl_scl;
assign targ_sda = ctrl_sda;


// I3C high-keepers.
// - expected to be external to the device, but if not then there must be some means of disabling
//   them in the event that they are insufficient for the physical bus properties.
`ifdef VERILATOR
assign ctrl_scl = (cio_ctrl_scl_pp_en | cio_ctrl_scl_od_en) ? 1'bZ : cio_scl_hk_en;
assign ctrl_sda = (cio_ctrl_sda_pp_en | cio_ctrl_sda_od_en) ? 1'bZ : cio_sda_hk_en;
`else
assign (weak0, weak1) ctrl_scl = cio_scl_hk_en ? 1'b1 : 1'bZ;
assign (weak0, weak1) ctrl_sda = cio_sda_hk_en ? 1'b1 : 1'bZ;
`endif

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
  cfg = $fopen(cfg_filename, "r");
  if (~|cfg) $display("WARNING: Unable to open config file '%s'", cfg_filename);
end

// Advance the configuration logic; this is just a simple means of programming a
// set of configuration registers.
logic [31:0] delay;
logic proceed;
always_ff @(posedge clk_i or negedge rst_ni) begin
  if (!rst_ni) begin
    cfg_prog  <= 1'b0;
    delay     <= '0;
    proceed   <= 1'b1;
  end else if (proceed || (tl_h2d.a_valid & tl_d2h.a_ready)) begin
    if (|cfg && $feof(cfg)) begin
      cfg_prog <= 1'b0;
    end else begin : read_config
      if (~|delay) begin
        string str;
        if (1 < $fgets(str, cfg)) begin
          case (str[0])
            // Time delay
            35: begin : read_delay
              int unsigned cycles;
              void'($sscanf(str, "#%d", cycles));
              $display("Delaying for %d cycle(s)", cycles);
              delay <= cycles;
              proceed <= 1'b1;
              cfg_prog <= 1'b0;
            end
            // Comments
            47: begin
              proceed <= 1'b1;
              cfg_prog <= 1'b0;
            end
            // Register read/write
            default: begin : reg_op
              int unsigned opcode, addr, wmask, wdata;
              void'($sscanf(str, "%x:%x:%x:%x\n", opcode, addr, wmask, wdata));
              case (cfg_opcode)
                tlul_pkg::PutFullData: $display("Write [%x]:%x", addr, wdata);
                tlul_pkg::PutPartialData: $display("Write [%x]:%x,%x", addr, wmask, wdata);
                default: $display("Read [%x]", addr);
              endcase
              cfg_prog    <= 1'b1;
              cfg_addr    <= addr;
              cfg_opcode  <= tlul_pkg::tl_a_op_e'(opcode);
              cfg_wmask   <= wmask[top_pkg::TL_DBW-1:0];
              cfg_wdata   <= wdata;
              // We must wait until the transfer has completed.
              proceed     <= 1'b0;
            end
          endcase
        end else begin
          // All done.
          cfg_prog <= 1'b0;
          proceed  <= 1'b0;
        end
      end else delay <= delay - 1'b1;
    end
  end
end

// Load a message to be transmitted.
int msg;
initial begin
  static string msg_filename = "msg";
  localparam int EOF = -1;
  int ch;
  msg = $fopen(msg_filename, "r");
  if (~|msg) begin
    $display("WARNING: Unable to read message data file '%s'", msg_filename);
  end else begin
    static bit [1:0] parity = 2'b01;
    static bit [4:0] crc5 = 5'h1f;
    do begin
      ch = $fgetc(msg);
      if (ch != EOF) begin
        $display("%x", ch);
        msg_data.push_back(ch[7:0]);
        // The bits are checked in the order of transmission, which means that we
        // need to know whether the data will be sent in HDR or SDR mode!
        for (int i = 7; i >= 0; i--) msg_bits.push_back(ch[i]);  // SDR ordering.
      end
    end while (ch != EOF);
    // Update CRC-5 and Parity values.
    parity = update_parity(msg_bits, parity);
    crc5 = ~update_crc5(msg_bits, crc5);
    $display("CRC-5: 0x%x", crc5);
    $display("Parity: 0x%x", parity);
    $fclose(msg);
    msg = 0;
  end
end

endmodule
