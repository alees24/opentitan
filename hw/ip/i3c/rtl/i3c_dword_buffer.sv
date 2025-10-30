// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Simple module to collects SDR bytes and HDR-DDR Data Words into 32-bit DWORDs for writing
// into the HCI Rx Data Queue.
//
// TODO: Note that presently there is a combinational feedthrough to the mask/data bits when the
// DWORD is completed, so storage may be required elsewhere.
module i3c_dword_buffer
  import i3c_pkg::*;
(
  input             clk_i,
  input             rst_ni,

  // Input SDR bytes/HDR-DDR Data Words.
  input             valid_i,
  input             flush_i,
  input i3c_dtype_e dtype_i,
  input      [15:0] data_i,

  // Output DWORDs.
  output            valid_o,
  output      [3:0] mask_o,
  output     [31:0] data_o
);

  // Collect data into DWORDs.
  logic [23:0] wdata_q;
  logic  [2:0] wmask_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wdata_q <= '0;
      wmask_q <= '0;
    end else if (valid_i) begin
      case (dtype_i)
        I3CDType_DataWord: begin
          wdata_q <= {8'b0, data_i[7:0], data_i[15:8]};
          wmask_q <= {1'b0, ~wmask_q[1:0]};
        end
        I3CDType_SDRBytes: begin
          wdata_q[23:16] <= (wmask_q == 3'b011) ? data_i[7:0] : 8'b0;
          wdata_q[15:8]  <= (wmask_q == 3'b001) ? data_i[7:0] : wdata_q[15:8];
          wdata_q[7:0]   <= |wmask_q ? wdata_q[7:0] : data_i[7:0];
          wmask_q[0]     <= wmask_q[2] | !wmask_q[0];
          wmask_q[1]     <= wmask_q[0] & ~wmask_q[2];
          wmask_q[2]     <= wmask_q[1] & ~wmask_q[2];
        end
        default: begin
          wdata_q <= '0;
          wmask_q <= '0;
        end
      endcase
    end else if (flush_i) wmask_q <= '0;  // Explicit flush operation.
  end

  logic        wflush;
  logic [31:0] wdata;
  logic  [3:0] wmask;
  always_comb begin
    if (valid_i) begin
      case (dtype_i)
        I3CDType_DataWord: wflush = wmask_q[1];
        I3CDType_SDRBytes: wflush = wmask_q[2];
      endcase
      case (dtype_i)
        I3CDType_DataWord: wmask = '1;
        I3CDType_SDRBytes: wmask = '1;
        default:           wmask = {1'b0, wmask_q};
      endcase
      case (dtype_i)
        I3CDType_DataWord: wdata = {data_i[7:0], data_i[15:8], wdata_q[15:0]};
        I3CDType_SDRBytes: wdata = {data_i[7:0], wdata_q[23:0]};
        default:           wdata = {8'b0, wdata_q};
      endcase
    end else begin
      // Set up outputs in case explicit flushing is performed by assertion of `flush_i`.
      wflush = |wmask_q;
      wmask = {1'b0, wmask_q};
      wdata = {8'b0, wdata_q};
    end
  end

  // Send all data into the message buffer for now.
  assign valid_o = (valid_i & wflush) | (flush_i & |wmask_q);
  assign mask_o  = wmask;
  assign data_o  = wdata;

endmodule
