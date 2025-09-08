// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// I3C buffer
//
// The buffer storage has 3 clients, in order of descending priority:
// - transmitter access (hard real time, must respond immediately).
// - receiver access (hard real time, but can defer the writing).
// - software/DMA access via the system bus (readily delayed).

module i3c_buffer
  import prim_ram_1p_pkg::*;
#(
  parameter int unsigned BufAddrWidth = 9,
  parameter int unsigned DataWidth = 32
) (
  input                     clk_i,
  input                     rst_ni,

  input  ram_1p_cfg_t       ram_cfg_i,
  output ram_1p_cfg_rsp_t   ram_cfg_rsp_o,

  // Buffer allocation.
  // TODO:

  // Software interface to buffer.
  input                     swbuf_req_i,
  output                    swbuf_gnt_o,
  input                     swbuf_we_i,
  input  [BufAddrWidth-1:0] swbuf_addr_i,
  input   [DataWidth/8-1:0] swbuf_wmask_i,
  input     [DataWidth-1:0] swbuf_wdata_i,
  output                    swbuf_rvalid_o,
  output              [1:0] swbuf_rerror_o,
  output    [DataWidth-1:0] swbuf_rdata_o,

  // Transmitter interface to buffer.
  input                     txbuf_rd_i,
  output    [DataWidth-1:0] txbuf_rdata_o,
  output                    txbuf_empty_o,

  // Receiver interface to buffer.
  input                     rxbuf_wr_i,
  input   [DataWidth/8-1:0] rxbuf_wmask_i,
  input     [DataWidth-1:0] rxbuf_wdata_i
);

  // Size of message buffer, in words.
  localparam int unsigned BufWords = 1 << BufAddrWidth;

  logic                    mem_req;
  logic                    mem_write;
  logic [BufAddrWidth-1:0] mem_addr;
  logic  [DataWidth/8-1:0] mem_wmask;
  logic    [DataWidth-1:0] mem_wdata;
  logic    [DataWidth-1:0] mem_rdata;

  // Use the arbiter to steer the appropriate inputs.
  typedef struct packed {
    bit                    write;
    bit [BufAddrWidth-1:0] addr;
    bit  [DataWidth/8-1:0] wmask;
    bit    [DataWidth-1:0] wdata;
  } arb_data_t;
  localparam int unsigned ArbDataWidth = $bits(arb_data_t);

  // TODO: Decide where this addressing logic needs to be; this is just a temporary sequential
  // transfer from the start of the message buffer.
  logic [BufAddrWidth-1:0] txbuf_addr;
  logic [BufAddrWidth-1:0] rxbuf_addr;
  logic txbuf_gnt, rxbuf_gnt;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      txbuf_addr <= '0;
    end else if (txbuf_rd_i) begin
      txbuf_addr <= txbuf_addr + 'b1;
    end
  end

  // We may need to defer a receiver write request.
  // TODO: Remember that any notifications to software must occur only _after_ the final word is
  // available, even though in practice we'll only be delaying by a single cycle here.
  logic [DataWidth/8-1:0] rxbuf_wmask_q;
  logic   [DataWidth-1:0] rxbuf_wdata_q;
  logic [DataWidth/8-1:0] rxbuf_wmask;
  logic   [DataWidth-1:0] rxbuf_wdata;
  logic rxbuf_wr_q;
  logic rxbuf_wr;

  assign rxbuf_wr    = rxbuf_wr_i | rxbuf_wr_q;
  assign rxbuf_wmask = rxbuf_wr_i ? rxbuf_wmask_i : rxbuf_wmask_q;
  assign rxbuf_wdata = rxbuf_wr_i ? rxbuf_wdata_i : rxbuf_wdata_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rxbuf_addr    <= '0;
      rxbuf_wmask_q <= '0;
      rxbuf_wdata_q <= '0;
    end else begin
      if (rxbuf_wr_i & ~rxbuf_gnt) begin
        rxbuf_wr_q    <= 1'b1;
        rxbuf_wmask_q <= rxbuf_wmask_i;
        rxbuf_wdata_q <= rxbuf_wdata_i;
      end
      if (rxbuf_wr & rxbuf_gnt) begin
        rxbuf_wr_q    <= 1'b0;
        rxbuf_addr    <= rxbuf_addr + 'b1;
      end
    end
  end

  arb_data_t arb_data_in[2:0];
  arb_data_t arb_data_out;
  assign arb_data_in[2] = {swbuf_we_i, swbuf_addr_i, swbuf_wmask_i, swbuf_wdata_i};
  assign arb_data_in[1] = {1'b1,       rxbuf_addr,   rxbuf_wmask,   rxbuf_wdata};
  // Unused wmask/wdata since the transmitter only reads. Use `rxbuf_` to simplify the MUXing.
  assign arb_data_in[0] = {1'b0,       txbuf_addr,   rxbuf_wmask,   rxbuf_wdata};

  // Absolute priority arbiter; index 0 has the highest priority.
  prim_arbiter_fixed #(
    .N    (3),
    .DW   (ArbDataWidth)
  ) u_arbiter (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),

    .req_i      ({txbuf_rd_i,  // Highest priority.
                  rxbuf_wr,
                  swbuf_req_i}),  // Lowest priority.
    .data_i     (arb_data_in),
    .gnt_o      ({rxbuf_gnt,
                  txbuf_gnt,
                  swbuf_gnt_o}),
    .idx_o      (),

    .valid_o    (mem_req),
    .data_o     ({mem_write, mem_addr, mem_wmask, mem_wdata}),
    .ready_i    (1'b1)
  );

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
  logic swbuf_gnt_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) swbuf_gnt_q <= 1'b0;
    else swbuf_gnt_q <= swbuf_gnt_o;
  end
  assign swbuf_rvalid_o = mem_rvalid & swbuf_gnt_q;
  assign swbuf_rerror_o = mem_rerror;
  assign {swbuf_rdata_o, txbuf_rdata_o} = {2{mem_rdata}};

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
