// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C buffer
//
// The buffer storage has 5 clients, in order of descending priority:
//
// - Controller transmitter access (hard real time, must respond immediately).
// - Target transmitter access (hard real time, must respond immediately).
// - Controller receiver access (hard real time, but can defer the write operation).
// - Target receiver access (hard real time, but can defer the write operation).
// - software/DMA access via the system bus (readily delayed).

// TODO: Having the Controller and Target operating independently on separate buses at maximum
//       speed does break this model. We cannot guarantee to service both transmitters immediately!
//       Either we resort to prefetching within the buffer itself or - perhaps more sensible -
//       we modify the transmitter state machine to ask ahead of time so that it can tolerate a
//       two cycle latency.
//
// TODO: Ability to service a request depends upon availability of data/space.

module i3c_buffer
  import i3c_pkg::*;
  import prim_ram_1p_pkg::*;
#(
  parameter int unsigned BufAddrW = 9,
  parameter int unsigned DataWidth = 32,

  // Always seven clients at present.
  localparam int unsigned N = 7
) (
  input                     clk_i,
  input                     rst_ni,

  input  ram_1p_cfg_t       ram_cfg_i,
  output ram_1p_cfg_rsp_t   ram_cfg_rsp_o,

  // Buffer configuration.
  input  i3c_pkg::bufcfg_t  ctrl_txbuf_cfg_i,
  input  i3c_pkg::bufcfg_t  targ_txbuf_cfg_i,
  input  i3c_pkg::bufcfg_t  ctrl_rxbuf_cfg_i,
  input  i3c_pkg::bufcfg_t  targ_rxbuf_cfg_i,
  input  i3c_pkg::bufcfg_t  xfer_txbuf_cfg_i,
  input  i3c_pkg::bufcfg_t  xfer_rxbuf_cfg_i,
  input  i3c_pkg::bufcfg_t  swbuf_cfg_i,
  // Update buffer status.
  output i3c_pkg::bufupd_t  ctrl_txbuf_upd_o,
  output i3c_pkg::bufupd_t  targ_txbuf_upd_o,
  output i3c_pkg::bufupd_t  ctrl_rxbuf_upd_o,
  output i3c_pkg::bufupd_t  targ_rxbuf_upd_o,
  output i3c_pkg::bufupd_t  xfer_txbuf_upd_o,
  output i3c_pkg::bufupd_t  xfer_rxbuf_upd_o,

  // HCI XFER_DATA_PORT interface to buffer.
  input                     xfer_req_i,
  output                    xfer_gnt_o,
  input                     xfer_we_i,
  input   [DataWidth/8-1:0] xfer_wmask_i,
  input     [DataWidth-1:0] xfer_wdata_i,
  output                    xfer_rvalid_o,
  output    [DataWidth-1:0] xfer_rdata_o,
  output              [1:0] xfer_rerror_o,

  // Direct software interface to buffer.
  input                     swbuf_req_i,
  output                    swbuf_gnt_o,
  input                     swbuf_we_i,
  input      [BufAddrW-1:0] swbuf_addr_i,
  input   [DataWidth/8-1:0] swbuf_wmask_i,
  input     [DataWidth-1:0] swbuf_wdata_i,
  output                    swbuf_rvalid_o,
  output              [1:0] swbuf_rerror_o,
  output    [DataWidth-1:0] swbuf_rdata_o,

  // Controller TX interface to buffer.
  input                     ctrl_txbuf_rd_i,
  output    [DataWidth-1:0] ctrl_txbuf_rdata_o,
  output                    ctrl_txbuf_empty_o,

  // Controller RX interface to buffer.
  input                     ctrl_rxbuf_wr_i,
  input   [DataWidth/8-1:0] ctrl_rxbuf_wmask_i,
  input     [DataWidth-1:0] ctrl_rxbuf_wdata_i,

  // Target TX interface to buffer.
  input                     targ_txbuf_rd_i,
  output    [DataWidth-1:0] targ_txbuf_rdata_o,
  output                    targ_txbuf_empty_o,

  // Target RX interface to buffer.
  input                     targ_rxbuf_wr_i,
  input   [DataWidth/8-1:0] targ_rxbuf_wmask_i,
  input     [DataWidth-1:0] targ_rxbuf_wdata_i
);

  // Size of message buffer, in words.
  localparam int unsigned BufWords = 1 << BufAddrW;

  // Use the arbiter to steer the appropriate inputs.
  typedef struct packed {
    bit                    write;
    i3c_pkg::bufcfg_t      addr;
    bit  [DataWidth/8-1:0] wmask;
    bit    [DataWidth-1:0] wdata;
  } arb_data_t;
  localparam int unsigned ArbDataWidth = $bits(arb_data_t);

  // Simple indications of empty TX buffers.
  assign ctrl_txbuf_empty_o = (ctrl_txbuf_cfg_i.curr == ctrl_txbuf_cfg_i.limit);
  assign targ_txbuf_empty_o = (targ_txbuf_cfg_i.curr == targ_txbuf_cfg_i.limit);

  // TODO: Simple indications of full RX buffers.
  // assign ctrl_rxbuf_full_o = (ctrl_rxbuf_cfg_i.curr == ctrl_rxbuf_cfg_i.limit);
  // assign targ_rxbuf_full_o = (targ_rxbuf_cfg_i.curr == targ_rxbuf_cfg_i.limit);

  logic ctrl_txbuf_gnt, targ_txbuf_gnt, xfer_txbuf_gnt;
  logic ctrl_rxbuf_gnt, targ_rxbuf_gnt, xfer_rxbuf_gnt;

  // TODO: If this becomes any more involved then it may warrant the use of two submodule instances.

  // We may need to defer a RX write request, since the TX traffic has higher priority.
  // TODO: Remember that any notifications to software must occur only _after_ the final word is
  // available, even though in practice we'll only be delaying by a single cycle here.
  logic [DataWidth-1:0]   ctrl_rxbuf_wdata, ctrl_rxbuf_wdata_q;
  logic [DataWidth-1:0]   targ_rxbuf_wdata, targ_rxbuf_wdata_q;
  logic [DataWidth/8-1:0] ctrl_rxbuf_wmask, ctrl_rxbuf_wmask_q;
  logic [DataWidth/8-1:0] targ_rxbuf_wmask, targ_rxbuf_wmask_q;
  logic ctrl_rxbuf_wr, ctrl_rxbuf_wr_q;
  logic targ_rxbuf_wr, targ_rxbuf_wr_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_rxbuf_wr_q    <= 1'b0;
      ctrl_rxbuf_wmask_q <= '0;
      ctrl_rxbuf_wdata_q <= '0;
    end else begin
      // Capture properties of any ungranted request.
      if (ctrl_rxbuf_wr_i & ~ctrl_rxbuf_gnt) begin
        ctrl_rxbuf_wmask_q <= ctrl_rxbuf_wmask_i;
        ctrl_rxbuf_wdata_q <= ctrl_rxbuf_wdata_i;
      end
      // Sustain any request until it is granted.
      if (ctrl_rxbuf_wr) ctrl_rxbuf_wr_q <= !ctrl_rxbuf_gnt;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      targ_rxbuf_wr_q    <= 1'b0;
      targ_rxbuf_wmask_q <= '0;
      targ_rxbuf_wdata_q <= '0;
    end else begin
      // Capture properties of any ungranted request.
      if (targ_rxbuf_wr_i & ~targ_rxbuf_gnt) begin
        targ_rxbuf_wmask_q <= targ_rxbuf_wmask_i;
        targ_rxbuf_wdata_q <= targ_rxbuf_wdata_i;
      end
      // Sustain any request until it is granted.
      if (targ_rxbuf_wr) targ_rxbuf_wr_q <= !targ_rxbuf_gnt;
    end
  end

  assign ctrl_rxbuf_wr    = ctrl_rxbuf_wr_i | ctrl_rxbuf_wr_q;
  assign targ_rxbuf_wr    = targ_rxbuf_wr_i | targ_rxbuf_wr_q;
  assign ctrl_rxbuf_wmask = ctrl_rxbuf_wr_i ? ctrl_rxbuf_wmask_i : ctrl_rxbuf_wmask_q;
  assign targ_rxbuf_wmask = targ_rxbuf_wr_i ? targ_rxbuf_wmask_i : targ_rxbuf_wmask_q;
  assign ctrl_rxbuf_wdata = ctrl_rxbuf_wr_i ? ctrl_rxbuf_wdata_i : ctrl_rxbuf_wdata_q;
  assign targ_rxbuf_wdata = targ_rxbuf_wr_i ? targ_rxbuf_wdata_i : targ_rxbuf_wdata_q;

  // HCI XFER_DATA_PORT accesses may be either reads (RX_DATA) or writes (TX_DATA).
  logic xfer_txbuf_wr, xfer_rxbuf_rd;
  assign xfer_txbuf_wr = xfer_req_i &  xfer_we_i;
  assign xfer_rxbuf_rd = xfer_req_i & !xfer_we_i;
  assign xfer_gnt_o    = xfer_txbuf_gnt | xfer_rxbuf_gnt;

  arb_data_t arb_data_in[N];
  assign arb_data_in[6] = {swbuf_we_i,      swbuf_cfg_i,      swbuf_wmask_i,      swbuf_wdata_i};
  assign arb_data_in[5] = {1'b1,       xfer_txbuf_cfg_i,       xfer_wmask_i,       xfer_wdata_i};
  assign arb_data_in[4] = {1'b0,       xfer_rxbuf_cfg_i,   ctrl_rxbuf_wmask,   ctrl_rxbuf_wdata};
  assign arb_data_in[3] = {1'b1,       targ_rxbuf_cfg_i,   targ_rxbuf_wmask,   targ_rxbuf_wdata};
  assign arb_data_in[2] = {1'b1,       ctrl_rxbuf_cfg_i,   ctrl_rxbuf_wmask,   ctrl_rxbuf_wdata};
  // Unused wmask/wdata since the transmitter logic only reads. Use `rxbuf_` to simplify the MUXing.
  assign arb_data_in[1] = {1'b0,       targ_txbuf_cfg_i,   targ_rxbuf_wmask,   targ_rxbuf_wdata};
  assign arb_data_in[0] = {1'b0,       ctrl_txbuf_cfg_i,   ctrl_rxbuf_wmask,   ctrl_rxbuf_wdata};

  // After arbitration.
  logic                    mem_req;
  i3c_pkg::bufcfg_t        mem_cfg;
  logic                    mem_write;
  logic  [DataWidth/8-1:0] mem_wmask;
  logic    [DataWidth-1:0] mem_wdata;
  logic    [DataWidth-1:0] mem_rdata;

  // Absolute priority arbiter; index 0 has the highest priority.
  prim_arbiter_fixed #(
    .N    (N),
    .DW   (ArbDataWidth)
  ) u_arbiter (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    .req_i      ({swbuf_req_i,  // Lowest priority.
                  xfer_txbuf_wr,
                  xfer_rxbuf_rd,
                  targ_rxbuf_wr,
                  ctrl_rxbuf_wr,
                  targ_txbuf_rd_i,
                  ctrl_txbuf_rd_i}),  // Highest priority.                  
    .data_i     (arb_data_in),
    .gnt_o      ({swbuf_gnt_o,
                  xfer_txbuf_gnt,
                  xfer_rxbuf_gnt,
                  targ_rxbuf_gnt,
                  ctrl_rxbuf_gnt,
                  targ_txbuf_gnt,
                  ctrl_txbuf_gnt}),
    .idx_o      (),  // Not used.

    .valid_o    (mem_req),
    .data_o     ({mem_write, mem_cfg, mem_wmask, mem_wdata}),
    .ready_i    (1'b1)
  );

  // Address advancement.
  logic [BufAddrW-1:0] next_addr, mem_addr;
  assign mem_addr = mem_cfg.curr;
  assign next_addr = (mem_addr == mem_cfg.max) ? mem_cfg.min : mem_addr + 'b1;
  always_comb begin
    ctrl_txbuf_upd_o.valid = ctrl_txbuf_gnt;
    targ_txbuf_upd_o.valid = targ_txbuf_gnt; 
    ctrl_rxbuf_upd_o.valid = ctrl_rxbuf_gnt;
    targ_rxbuf_upd_o.valid = targ_rxbuf_gnt;
    xfer_rxbuf_upd_o.valid = xfer_rxbuf_gnt;
    xfer_txbuf_upd_o.valid = xfer_txbuf_gnt;
    // The address may be returned to all simultaneously; its use is qualified by `valid.`
    ctrl_txbuf_upd_o.next  = next_addr;
    targ_txbuf_upd_o.next  = next_addr;
    ctrl_rxbuf_upd_o.next  = next_addr;
    targ_rxbuf_upd_o.next  = next_addr;
    xfer_rxbuf_upd_o.next  = next_addr;
    xfer_txbuf_upd_o.next  = next_addr;
  end

  // SRAM wrapper requires bit-level write strobes.
  logic [DataWidth-1:0] mem_wmask_full;
  always_comb begin
    for (int unsigned b = 0; b < DataWidth / 8; b++) begin
      mem_wmask_full[b*8 +: 8] = {8{mem_wmask[b]}};
    end
  end

  // Return the read data.
  logic [1:0] mem_rerror;
  logic mem_rvalid;
  logic xfer_gnt_q, swbuf_gnt_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      xfer_gnt_q  <= 1'b0;
      swbuf_gnt_q <= 1'b0;
    end else begin
      xfer_gnt_q  <= xfer_gnt_o;
      swbuf_gnt_q <= swbuf_gnt_o;
    end
  end
  assign swbuf_rvalid_o = mem_rvalid & swbuf_gnt_q;
  assign swbuf_rerror_o = mem_rerror;
  assign xfer_rvalid_o  = mem_rvalid & xfer_gnt_q;
  assign xfer_rerror_o  = mem_rerror;
  // Read data may be returned to all without qualification.
  assign {swbuf_rdata_o, xfer_rdata_o, targ_txbuf_rdata_o, ctrl_txbuf_rdata_o} = {4{mem_rdata}};

  // SRAM Wrapper
  prim_ram_1p_adv #(
    .Depth                (BufWords),
    .Width                (DataWidth),
    .DataBitsPerMask      (8),
    .EnableECC            (0), // No Protection
    .EnableParity         (0),
    .EnableInputPipeline  (0),
    .EnableOutputPipeline (0)
  ) u_memory_1p (
    .clk_i,
    .rst_ni,

    .req_i      (mem_req),
    .write_i    (mem_write),
    .addr_i     (mem_addr),
    .wdata_i    (mem_wdata),
    .wmask_i    (mem_wmask_full),
    .rdata_o    (mem_rdata),
    .rvalid_o   (mem_rvalid),
    .rerror_o   (mem_rerror),
    .cfg_i      (ram_cfg_i),
    .cfg_rsp_o  (ram_cfg_rsp_o),
    .alert_o    ()
  );

endmodule
