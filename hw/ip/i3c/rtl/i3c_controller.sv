// Copyright lowRISC contributors (OpenTitan project).
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
  input  i3c_reg2hw_t         reg2hw_i,
  // State information, presented via HCI.
  output                      ac_current_own_o,

  // TODO:
  // Command Descriptor queue.
  output                      cmd_desc_full_o,
  input                       cmd_desc_write_i,
  input       [DataWidth-1:0] cmd_desc_wdata_i,
  // Response Descriptor queue.
  output                      rsp_desc_avail_o,
  input                       rsp_desc_read_i,
  output      [DataWidth-1:0] rsp_desc_rdata_o,
  // IBI Status/Data queue.
  output                      ibi_stat_avail_o,
  input                       ibi_stat_read_i,
  output      [DataWidth-1:0] ibi_stat_rdata_o,

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

  // Buffer reading.
  output logic                buf_rd_o,
  input       [DataWidth-1:0] buf_rdata_i,
  input                       buf_empty_i,

  // Buffer writing.
  output logic                buf_wr_o,
  output    [DataWidth/8-1:0] buf_wmask_o,
  output      [DataWidth-1:0] buf_wdata_o,

  // Request to transceiver logic.
  output                      trx_dvalid_o,
  input                       trx_dready_i,
  output i3c_ctrl_trx_req_t   trx_dreq_o,

  // Response from transceiver logic.
  input                       trx_rvalid_i,
  input  i3c_ctrl_trx_rsp_t   trx_rsp_i,

  // Debug status information.
  output                [4:0] ibi_status_cnt_o,
  output                [7:0] ibibuf_lvl_o,
  output                [7:0] rspbuf_lvl_o,
  output                [7:0] cmdq_free_lvl_o,
  output                [7:0] rxbuf_lvl_o,
  output                [7:0] txbuf_free_lvl_o,
  output                [3:0] cmd_tid_o,
  output                [5:0] bcl_tfr_ststat_o,
  output                [5:0] bfl_tfr_status_o,
  output                [7:0] ce2_error_cnt_o,

  // DFT-related signals.
  input  ram_1p_cfg_t         dat_cfg_i,
  output ram_1p_cfg_rsp_t     dat_cfg_rsp_o,
  input  ram_1p_cfg_t         dct_cfg_i,
  output ram_1p_cfg_rsp_t     dct_cfg_rsp_o
);

  import i3c_consts_pkg::*;

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

  // TODO: Presently the decision is to leave the IP block domain, not the transceiver
  // to do any required byte swizzling.
  function automatic bit [DataWidth-1:0] revb(bit [DataWidth-1:0] wdata);
    for (int unsigned b = 0; b < DataWidth; b += 8) begin
      revb[b +: 8] = wdata[DataWidth - b - 8 +: 8];
    end
  endfunction

  // Debug extended capability; although the FIFO implementations are stil unsettled, these could be
  // wired easily enough.
  // TODO
  assign ibi_status_cnt_o = '0;
  assign ibibuf_lvl_o = '0;
  assign rspbuf_lvl_o = '0;
  assign cmdq_free_lvl_o = '0;
  assign rxbuf_lvl_o = '0;
  assign txbuf_free_lvl_o = '0;
  assign cmd_tid_o = '0;
  assign bcl_tfr_ststat_o = '0;
  assign bfl_tfr_status_o = '0;
  assign ce2_error_cnt_o = '0;

  `define CMD_READ
 
  // TODO: This is just a simple state machine that runs the bus through its various modes,
  // generating I2C, SDR and HDR traffic.
  logic [3:0] ctrl_state;
  always_comb begin
    trx_dreq_o = '0;
    case (ctrl_state)
      // HDR-DDR Signalling.
      4'b0000:  begin trx_dreq_o.dtype = I3CDType_Address;
                      trx_dreq_o.mode  = i3c_xfer_mode_e'(XferMode_I2CFM);
                      trx_dreq_o.wdata = Addr_Broadcast << (DataWidth - 7);
                      trx_dreq_o.wlen  = 0;
                end
      4'b0001:  begin trx_dreq_o.dtype = I3CDType_SDRBytes;
                      trx_dreq_o.mode  = XferMode_SDR0;
                      trx_dreq_o.wdata = ENTHDR0 << (DataWidth - 8);
                      trx_dreq_o.wlen  = 0;
                end
      4'b0010:  begin trx_dreq_o.dtype = I3CDType_CommandWord;
                      trx_dreq_o.mode  = XferMode_HDRDDR;
`ifdef CMD_READ
                      trx_dreq_o.wdata = revb(32'h876531a1);  // Address 0x18
`else
                      trx_dreq_o.wdata = revb(32'h87653121);  // Address 0x18
`endif
                      // Send only a single Command Word.
                      trx_dreq_o.wlen  = 2'b00;
                end
`ifdef CMD_READ
      // Just collect a few words for now.
      4'b0011,
      4'b0100,
      4'b0101:  begin trx_dreq_o.dtype = I3CDType_DataWord;
                      trx_dreq_o.rx    = 1'b1;
                end
// TODO: I think this makes no sense...the state machine needs to be reactive in some sense.
      4'b0110:  begin //trx_dreq_o.dtype = I3CDType_CRCWord;
trx_dreq_o.dtype = I3CDType_DataWord;
                      trx_dreq_o.rx    = 1'b1;
                end
`else
      4'b0011:  begin trx_dreq_o.dtype = I3CDType_DataWord;
                      trx_dreq_o.mode  = XferMode_HDRDDR;
                      trx_dreq_o.wdata = revb(32'habcdefab);
                      trx_dreq_o.wlen  = 2'b01;
                end
      4'b0100:  begin trx_dreq_o.dtype = I3CDType_DataWord;
                      trx_dreq_o.mode  = XferMode_HDRDDR;
                      trx_dreq_o.wdata = revb(32'hfedcbafe);
                      trx_dreq_o.wlen  = 2'b01;
                end
      4'b0101:  begin trx_dreq_o.dtype = I3CDType_DataWord;
                      trx_dreq_o.mode  = XferMode_HDRDDR;
                      trx_dreq_o.wdata = revb(32'h12345678);
                      // Note: should send only the 16 MSBs here.
                      trx_dreq_o.wlen  = 2'b00;
                end
      4'b0110:  begin trx_dreq_o.dtype = I3CDType_CRCWord;
                      trx_dreq_o.mode  = XferMode_HDRDDR;
                      // 4'hC token indicating a valid CRC word, and the final bit of the 10-bit
                      // payload is set in preparation for HDR Restart/Exit.
                      trx_dreq_o.wdata = 32'hc040_0000;
                      trx_dreq_o.wlen  = 2'b00;
                end
`endif
      4'b0111:  begin trx_dreq_o.dtype = I3CDType_HDRExit;
                      trx_dreq_o.mode  = XferMode_HDRDDR;
                      trx_dreq_o.wdata = 0;
                      trx_dreq_o.wlen  = 0;
                end
      // SDR Signalling.
      4'b1000:  begin trx_dreq_o.dtype = I3CDType_Address;
                      trx_dreq_o.mode  = i3c_xfer_mode_e'(XferMode_I2CFM);
                      trx_dreq_o.wdata = 'h18;
                      trx_dreq_o.wlen  = 0;
                end
      4'b1001:  begin trx_dreq_o.dtype = I3CDType_SDRBytes;
                      trx_dreq_o.mode  = XferMode_SDR0;
                      // Note: this represents the raw data read from the message buffer, with
                      // the LS byte (0x01) being the first byte to be transmitted/received.
                      trx_dreq_o.wdata = revb(32'h04030201);
                      trx_dreq_o.wlen  = 2'b11;
                end
      4'b1010:  begin trx_dreq_o.dtype = I3CDType_SDRBytes;
                      trx_dreq_o.mode  = XferMode_SDR0;
                      trx_dreq_o.wdata = revb(32'h08070605);
                      trx_dreq_o.wlen  = 2'b11;
                end
      4'b1011:  begin trx_dreq_o.dtype = I3CDType_SDRBytes;
                      trx_dreq_o.mode  = XferMode_SDR0;
                      trx_dreq_o.wdata = revb(32'h0c0b0a09);
                      trx_dreq_o.wlen  = 2'b11;
                end
      // Finnish state.
      default:  begin
                end
    endcase
  end
  assign trx_dvalid_o = enable_i & (ctrl_state <= 4'b1011);
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_state <= '0;
    end else if (enable_i & trx_dready_i) begin
      ctrl_state <= ctrl_state + 'b1;
    end
  end

  // TODO: No direct message buffer access presently.
  assign buf_rd_o = 1'b0;

  // TODO: Temporarily dump everything in the buffer.
  logic        wvalid;
  logic  [3:0] wmask;
  logic [31:0] wdata;
  assign buf_wr_o    = wvalid;
  assign buf_wmask_o = wmask;
  assign buf_wdata_o = wdata;

  i3c_dword_buffer u_dword_buf (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    // Input SDR bytes/HDR-DDR Data Words.
    .valid_i    (trx_rvalid_i),
    .flush_i    (1'b0),
    .dtype_i    (trx_rsp_i.dtype),
    .data_i     (trx_rsp_i.rdata[15:0]),

    // Output DWORDs.  
    .valid_o    (wvalid),
    .mask_o     (wmask),
    .data_o     (wdata)
  );

endmodule
