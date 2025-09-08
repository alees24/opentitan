// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// I3C Controller logic.
module i3c_controller
  import i3c_pkg::*;
  import i3c_reg_pkg::*;
  import prim_ram_1p_pkg::*;
#(
  parameter int unsigned DataWidth = 32,
  parameter int unsigned NumDATEntries = i3c_pkg::NumDATEntries,
  parameter int unsigned NumDCTEntries = i3c_pkg::NumDCTEntries,

  // Derived parameters.
  localparam int unsigned DATAddrW = $clog2(NumDATEntries),
  localparam int unsigned DCTAddrW = $clog2(NumDCTEntries)
) (
  // Clock and reset for system interface.
  input                       clk_i,
  input                       rst_ni,

  // Clock and reset for `always on` bus monitoring.
  input                       clk_aon_i,
  input                       rst_aon_ni,

  // Control inputs.
  input                       enable_i,
  input                       sw_reset_i,

  // Configuration settings.
  input i3c_reg2hw_t          reg2hw_i,
  // State information, presented via HCI.
  output                      ac_current_own_o,

  // TODO:
  // Command Descriptor queue.
  output                      cmd_desc_full_o,
  input                       cmd_desc_write_i,
  input   [DataWidth-1:0] cmd_desc_wdata_i,
  // Response Descriptor queue.
  output                      rsp_desc_avail_o,
  input                       rsp_desc_read_i,
  output  [DataWidth-1:0] rsp_desc_rdata_o,
  // IBI Status/Data queue.
  output                      ibi_stat_avail_o,
  input                       ibi_stat_read_i,
  output  [DataWidth-1:0] ibi_stat_rdata_o,

  // HCI Device Address Table interface.
  input                       sw_dat_req_i,
  output                      sw_dat_gnt_o,
  input                       sw_dat_we_i,
  input          [DATAddrW:0] sw_dat_addr_i,
  input       [DataWidth-1:0] sw_dat_wdata_i,
  output                      sw_dat_rvalid_o,
  output      [DataWidth-1:0] sw_dat_rdata_o,

  // HCI Device Characteristics Table interface.
  input                       sw_dct_req_i,
  output                      sw_dct_gnt_o,
  input                       sw_dct_we_i,
  input        [DCTAddrW+1:0] sw_dct_addr_i,
  input       [DataWidth-1:0] sw_dct_wdata_i,
  output                      sw_dct_rvalid_o,
  output      [DataWidth-1:0] sw_dct_rdata_o,

  // Interrupt signals.
  output i3c_hc_intr_t        intr_hc_o,
  output i3c_pio_intr_t       intr_pio_o,
  output i3c_stby_cr_intr_t   intr_stby_cr_o,

  // DFT-related signals.
  input  ram_1p_cfg_t         dat_cfg_i,
  output ram_1p_cfg_rsp_t     dat_cfg_rsp_o,
  input  ram_1p_cfg_t         dct_cfg_i,
  output ram_1p_cfg_rsp_t     dct_cfg_rsp_o
);

  // TODO: Suspect that we'd want to widen to 64a, write alternating words and then
  // use the static FIFO outputs directly, reading only after the command transfer is complete?
  localparam int unsigned CmdDescW = 32;
  localparam int unsigned CmdDepth = i3c_pkg::CmdQueueEntries * 2;

  localparam int unsigned RspDescW = 32;
  localparam int unsigned RspDepth = i3c_pkg::RspQueueEntries;

  localparam int unsigned IBIStatW = 32;
  localparam int unsigned IBIDepth = i3c_pkg::IBIQueueDWORDs;

  // TODO: This is just some nonsense to keep the FIFOs in a trial synth.
  wire                 cmd_desc_rvalid;
  wire                 cmd_desc_rready;
  wire [DataWidth-1:0] cmd_desc_rdata;

  wire                 rsp_desc_wvalid = cmd_desc_rvalid & cmd_desc_rready;
  wire                 rsp_desc_wready;
  wire [DataWidth-1:0] rsp_desc_wdata  = cmd_desc_rdata;

  wire                 ibi_stat_wvalid = cmd_desc_rvalid & cmd_desc_rready;
  wire                 ibi_stat_wready;
  wire [DataWidth-1:0] ibi_stat_wdata  = cmd_desc_rdata;

  assign cmd_desc_rready = rsp_desc_wready & ibi_stat_wready;

  // TODO: These three queues are implemented as separate physical FIFOs presently, and it may be
  // preferable to carve these out of the message buffer to reduce area. They are small, low
  // bandwidth and not particularly latency-sensitive.

  // Command Descriptor queue.
  prim_fifo_sync #(
    .Width      (CmdDescW),
    .Pass       (1'b0),
    .Depth      (CmdDepth)
  ) u_cmd_queue (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    .clr_i      (sw_reset_i),

    .wvalid_i   (cmd_desc_write_i),
    .wready_o   (),
    .wdata_i    (cmd_desc_wdata_i),

    .rvalid_o   (cmd_desc_rvalid),
    .rready_i   (cmd_desc_rready),
    .rdata_o    (cmd_desc_rdata),

    .full_o     (cmd_desc_full_o),
    .depth_o    (),
    .err_o      ()
  );

  // Response Descriptor queue.
  prim_fifo_sync #(
    .Width      (RspDescW),
    .Pass       (1'b0),
    .Depth      (RspDepth)
  ) u_rsp_queue (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    .clr_i      (sw_reset_i),

    .wvalid_i   (rsp_desc_wvalid),
    .wready_o   (rsp_desc_wready),
    .wdata_i    (rsp_desc_wdata),

    .rvalid_o   (rsp_desc_avail_o),
    .rready_i   (rsp_desc_read_i),
    .rdata_o    (rsp_desc_rdata_o),

    .full_o     (),
    .depth_o    (),
    .err_o      ()
  );

  // IBI Status/Data queue.
  prim_fifo_sync #(
    .Width      (IBIStatW),
    .Pass       (1'b0),
    .Depth      (IBIDepth)
  ) u_ibi_queue (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    .clr_i      (sw_reset_i),

    .wvalid_i   (ibi_stat_wvalid),
    .wready_o   (ibi_stat_wready),
    .wdata_i    (ibi_stat_wdata),

    .rvalid_o   (ibi_stat_avail_o),
    .rready_i   (ibi_stat_read_i),
    .rdata_o    (ibi_stat_rdata_o),

    .full_o     (),
    .depth_o    (),
    .err_o      ()
  );

  // TODO: Maybe we move the DCT and DAT into submodules to hide some of this ugliness,
  // or use functions to perform the remapping?

  // Remap the read/write data from/to the packed memory entries.
  i3c_pkg::i3c_dat_mem_t sw_dat_wmask;
  i3c_pkg::i3c_dat_mem_t sw_dat_wdata_packed;
  i3c_pkg::i3c_dat_entry_t sw_dat_wdata_full;
  assign sw_dat_wdata_full = i3c_pkg::i3c_dat_entry_t'({2{sw_dat_wdata_i}});
  always_comb begin
    // Write strobes for software access to the DAT
    // TODO: Determine whether we can avoid bit-level write strobes!
    sw_dat_wmask = '0;
    if (sw_dat_addr_i[0]) begin
      sw_dat_wmask.autocmd_hdr_code   = '1;
      sw_dat_wmask.autocmd_mode       = '1;
      sw_dat_wmask.autocmd_value      = '1;
      sw_dat_wmask.autocmd_mask       = '1;
    end else begin
      sw_dat_wmask.device             = '1;
      sw_dat_wmask.dev_nack_retry_cnt = '1;
      sw_dat_wmask.ring_id            = '1;
      sw_dat_wmask.dynamic_address    = '1;
      sw_dat_wmask.ts                 = '1;
      sw_dat_wmask.crr_reject         = '1;
      sw_dat_wmask.ibi_reject         = '1;
      sw_dat_wmask.ibi_payload        = '1;
      sw_dat_wmask.static_address     = '1;
    end

    sw_dat_wdata_packed.autocmd_hdr_code   = sw_dat_wdata_full.autocmd_hdr_code;
    sw_dat_wdata_packed.autocmd_mode       = sw_dat_wdata_full.autocmd_mode;
    sw_dat_wdata_packed.autocmd_value      = sw_dat_wdata_full.autocmd_value;
    sw_dat_wdata_packed.autocmd_mask       = sw_dat_wdata_full.autocmd_mask;
    sw_dat_wdata_packed.device             = sw_dat_wdata_full.device;
    sw_dat_wdata_packed.dev_nack_retry_cnt = sw_dat_wdata_full.dev_nack_retry_cnt;
    sw_dat_wdata_packed.ring_id            = sw_dat_wdata_full.ring_id;
    sw_dat_wdata_packed.dynamic_address    = sw_dat_wdata_full.dynamic_address;
    sw_dat_wdata_packed.ts                 = sw_dat_wdata_full.ts;
    sw_dat_wdata_packed.crr_reject         = sw_dat_wdata_full.crr_reject;
    sw_dat_wdata_packed.ibi_reject         = sw_dat_wdata_full.ibi_reject;
    sw_dat_wdata_packed.ibi_payload        = sw_dat_wdata_full.ibi_payload;
    sw_dat_wdata_packed.static_address     = sw_dat_wdata_full.static_address;
  end

  i3c_pkg::i3c_dat_entry_t sw_dat_rdata_full;
  i3c_pkg::i3c_dat_mem_t sw_dat_rdata_packed;
  logic [$bits(i3c_pkg::i3c_dat_mem_t)-1:0] sw_dat_rdata_raw;
  assign sw_dat_rdata_packed = i3c_pkg::i3c_dat_mem_t'(sw_dat_rdata_raw);

  always_comb begin
    sw_dat_rdata_full = '0;  // Zero-initialize the reserved fields.
    sw_dat_rdata_full.autocmd_hdr_code   = sw_dat_rdata_packed.autocmd_hdr_code;
    sw_dat_rdata_full.autocmd_mode       = sw_dat_rdata_packed.autocmd_mode;
    sw_dat_rdata_full.autocmd_value      = sw_dat_rdata_packed.autocmd_value;
    sw_dat_rdata_full.autocmd_mask       = sw_dat_rdata_packed.autocmd_mask;
    sw_dat_rdata_full.device             = sw_dat_rdata_packed.device;
    sw_dat_rdata_full.dev_nack_retry_cnt = sw_dat_rdata_packed.dev_nack_retry_cnt;
    sw_dat_rdata_full.ring_id            = sw_dat_rdata_packed.ring_id;
    sw_dat_rdata_full.dynamic_address    = sw_dat_rdata_packed.dynamic_address;
    sw_dat_rdata_full.ts                 = sw_dat_rdata_packed.ts;
    sw_dat_rdata_full.crr_reject         = sw_dat_rdata_packed.crr_reject;
    sw_dat_rdata_full.ibi_reject         = sw_dat_rdata_packed.ibi_reject;
    sw_dat_rdata_full.ibi_payload        = sw_dat_rdata_packed.ibi_payload;
    sw_dat_rdata_full.static_address     = sw_dat_rdata_packed.static_address;
  end
  logic sw_dat_rdata_sel;
  assign sw_dat_rdata_o = sw_dat_rdata_sel ? sw_dat_rdata_full[63:32] : sw_dat_rdata_full[31:0];

  // TODO: Hardware use of DAT.
  logic dat_re, dat_we;
  logic [DATAddrW-1:0] dat_idx;
  i3c_pkg::i3c_dat_mem_t dat_wdata_packed;
  logic [$bits(i3c_pkg::i3c_dat_mem_t)-1:0] dat_rdata_raw;

  // Device Address Table.
  i3c_dxt #(
    .EntryWidth ($bits(i3c_pkg::i3c_dat_mem_t)),
    .NumEntries (NumDATEntries),
    .OOBWidth   (1)
  ) u_dat(
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    // Port A (hardware; highest priority).
    .a_re_i     (dat_re),
    .a_we_i     (dat_we),
    .a_idx_i    (dat_idx),
    .a_wdata_i  (dat_wdata_packed),
    .a_rdata_o  (dat_rdata_raw),

    // Port B (software; lowest priority).
    .b_req_i    (sw_dat_req_i),
    .b_gnt_o    (sw_dat_gnt_o),
    .b_we_i     (sw_dat_we_i),
    .b_idx_i    (sw_dat_addr_i[DATAddrW:1]),
    .b_wmask_i  (sw_dat_wmask),
    .b_wdata_i  (sw_dat_wdata_packed),
    .b_oob_i    (sw_dat_addr_i[0]),
    .b_rvalid_o (sw_dat_rvalid_o),
    .b_rdata_o  (sw_dat_rdata_raw),
    .b_oob_o    (sw_dat_rdata_sel),

    .cfg_i      (dat_cfg_i),
    .cfg_rsp_o  (dat_cfg_rsp_o)
  );

  // TODO: We can use a single write strobe per 8 bits for DCT.
  i3c_pkg::i3c_dct_mem_t sw_dct_wmask;
  i3c_pkg::i3c_dct_mem_t sw_dct_wdata_packed;
  i3c_pkg::i3c_dct_entry_t sw_dct_wdata_full;
  assign sw_dct_wdata_full = i3c_pkg::i3c_dct_entry_t'({4{sw_dct_wdata_i}});
  always_comb begin
    sw_dct_wmask = '0;
    case (sw_dct_addr_i[1:0])
      2'b00: sw_dct_wmask.pid_hi = '1;
      2'b01: sw_dct_wmask.pid_lo = '1;
      2'b10: begin
        sw_dct_wmask.dcr = '1;
        sw_dct_wmask.bcr = '1;
      end
      default: sw_dct_wmask.dynamic_address = '1;
    endcase

    sw_dct_wdata_packed.dynamic_address = sw_dct_wdata_full.dynamic_address;
    sw_dct_wdata_packed.bcr = sw_dct_wdata_full.bcr;
    sw_dct_wdata_packed.dcr = sw_dct_wdata_full.dcr;
    sw_dct_wdata_packed.pid_lo = sw_dct_wdata_full.pid_lo;
    sw_dct_wdata_packed.pid_hi = sw_dct_wdata_full.pid_hi;
  end

  i3c_pkg::i3c_dct_entry_t sw_dct_rdata_full;
  i3c_pkg::i3c_dct_mem_t sw_dct_rdata_packed;
  logic [$bits(i3c_pkg::i3c_dct_mem_t)-1:0] sw_dct_rdata_raw;
  assign sw_dct_rdata_packed = i3c_pkg::i3c_dct_mem_t'(sw_dct_rdata_raw);

  always_comb begin
    sw_dct_rdata_full = '0;  // Zero-initialize the reserved fields.
    sw_dct_rdata_full.dynamic_address = sw_dct_rdata_packed.dynamic_address;
    sw_dct_rdata_full.bcr             = sw_dct_rdata_packed.bcr;
    sw_dct_rdata_full.dcr             = sw_dct_rdata_packed.dcr;
    sw_dct_rdata_full.pid_lo          = sw_dct_rdata_packed.pid_lo;
    sw_dct_rdata_full.pid_hi          = sw_dct_rdata_packed.pid_hi;
  end

  logic [1:0] sw_dct_rdata_sel;
  assign sw_dct_rdata_o = DataWidth'(sw_dct_rdata_full >> {sw_dct_rdata_sel, 5'b0});

  // TODO: Hardware use of DCT.
  logic dct_re, dct_we;
  logic [DCTAddrW-1:0] dct_idx;
  i3c_pkg::i3c_dct_mem_t dct_wdata_packed;
  logic [$bits(i3c_pkg::i3c_dct_mem_t)-1:0] dct_rdata_raw;

  // Device Characteristics Table.
  i3c_dxt #(
    .EntryWidth ($bits(i3c_pkg::i3c_dct_mem_t)),
    .NumEntries (NumDCTEntries),
    .OOBWidth   (2)
  ) u_dct(
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    // Port A (hardware; highest priority).
    .a_re_i     (dct_re),
    .a_we_i     (dct_we),
    .a_idx_i    (dct_idx),
    .a_wdata_i  (dct_wdata_packed),
    .a_rdata_o  (dct_rdata_raw),

    // Port B (software; lowest priority).
    .b_req_i    (sw_dct_req_i),
    .b_gnt_o    (sw_dct_gnt_o),
    .b_we_i     (sw_dct_we_i),
    .b_idx_i    (sw_dct_addr_i[DCTAddrW+1:2]),
    .b_wmask_i  (sw_dct_wmask),
    .b_wdata_i  (sw_dct_wdata_packed),
    .b_oob_i    (sw_dct_addr_i[1:0]),
    .b_rvalid_o (sw_dct_rvalid_o),
    .b_rdata_o  (sw_dct_rdata_raw),
    .b_oob_o    (sw_dct_rdata_sel),

    .cfg_i      (dct_cfg_i),
    .cfg_rsp_o  (dct_cfg_rsp_o)
  );

  // TODO: nonsense for trial synth purposes.
  assign dct_re = sw_dat_req_i;
  assign dct_we = 1'b0;
  assign dat_re = sw_dct_req_i;
  assign dat_we = 1'b0;
  assign dct_idx = sw_dct_addr_i[DCTAddrW-1:0];
  assign dat_idx = sw_dat_addr_i[DATAddrW-1:0];  
  assign dct_wdata_packed = dct_rdata_raw;
  assign dat_wdata_packed = dat_rdata_raw;

  // TODO:
  assign ac_current_own_o = 1'b1;

  // Interrupt generation.
  always_comb begin
    intr_hc_o = '0;
    intr_pio_o = '0;
    intr_stby_cr_o = '0;
  end

endmodule
