// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

package i3c_pkg;
  import i3c_consts_pkg::*;

  // Size of Command Queue, in entries.
  localparam int unsigned CmdQueueEntries = 8;
  
  // Size of Response Queue, in entries.
  localparam int unsigned RspQueueEntries = 8;

  // Size of IBI Queue, in DWORDs.
  localparam int unsigned IBIQueueDWORDs = 8;

  // Number of entries in the Device Address Table
  // (This determines the maximum number of targets that may be addressed simultaneously using the
  //  Host Controller Interface.)
  localparam int unsigned NumDATEntries = 32;

  // Number of entries in the Device Characteristics Table
  // (The maximum number of devices that may be allocated dynamic addresses in a single allocation
  //  sequence.)
  localparam int unsigned NumDCTEntries = 32;

  localparam int unsigned BufAddrW = 9;

  typedef struct packed {
    logic [BufAddrW-1:0] min;
    logic [BufAddrW-1:0] max;
    logic [BufAddrW-1:0] curr;
    logic [BufAddrW-1:0] limit;
  } bufcfg_t;

  typedef struct packed {
    logic                valid;
    logic [BufAddrW-1:0] next;
  } bufupd_t;

  // Type of data unit transferred.
  typedef enum logic [2:0] {
    // HDR-DDR Words; bits [1:0] are the Preamble bits.
    I3CType_CommandWord  = 3'b001,
    I3CType_DataWord     = 3'b011,
    I3CType_ChecksumWord = 3'b101,
    // SDR signaling.
    I3CType_Address      = 3'b100,    
    I3CType_SDRByte      = 3'b110,
    // TODO: Temporary development aid.
    I3CType_DWORD        = 3'b111
  } i3c_dtype_e;

  // Full DAT Entry as per the HCI Specification.
  typedef struct packed {
    logic [63:59] reserved2;
    logic [58:51] autocmd_hdr_code;
    logic [50:48] autocmd_mode;
    logic [47:40] autocmd_value;
    logic [39:32] autocmd_mask;
    logic         device;
    logic [30:29] dev_nack_retry_cnt;
    logic [28:26] ring_id;
    logic [25:24] reserved1;
    logic [23:16] dynamic_address;
    logic         ts;
    logic         crr_reject;
    logic         ibi_reject;
    logic         ibi_payload;
    logic [11:7]  reserved0;
    logic [6:0]   static_address;
  } i3c_dat_entry_t;

  // Packed DAT Entry as stored internally; as above but without the reserved fields.
  typedef struct packed {
    logic [58:51] autocmd_hdr_code;
    logic [50:48] autocmd_mode;
    logic [47:40] autocmd_value;
    logic [39:32] autocmd_mask;
    logic         device;
    logic [30:29] dev_nack_retry_cnt;
    logic [28:26] ring_id;
    logic [23:16] dynamic_address;
    logic         ts;
    logic         crr_reject;
    logic         ibi_reject;
    logic         ibi_payload;
    logic [6:0]   static_address;
  } i3c_dat_mem_t;

  // Full DCT Entry as per the HCI Specification.
  typedef struct packed {
    logic [127:104] reserved2;
    logic [103:96]  dynamic_address;
    logic [95:80]   reserved1;
    logic [79:72]   bcr;
    logic [71:64]   dcr;
    logic [63:48]   reserved0;
    logic [47:32]   pid_lo;
    logic [31:0]    pid_hi;
  } i3c_dct_entry_t;

  // Packed DCT Entry as stored internally; as above but without the reserved fields.
  typedef struct packed {
    logic [103:96]  dynamic_address;
    logic [79:72]   bcr;
    logic [71:64]   dcr;
    logic [47:32]   pid_lo;
    logic [31:0]    pid_hi;
  } i3c_dct_mem_t;

  // General Host Controller interrupts.
  typedef struct packed {
    // The order of these fields is important in connecting to the `i3c_intr` instance.
    logic sched_cmd_missed_tick;
    logic hc_err_cmd_seq_timeout;
    logic hc_warn_cmd_seq_stall;
    logic hc_seq_cancel;
    logic hc_internal_err;
  } i3c_hc_intr_t;

  // PIO (Programmed Input/Output) Interrupts.
  typedef struct packed {
    // The order of these fields is important in connecting to the `i3c_intr` instance.
    logic transfer_err;
    logic transfer_abort;
    logic resp_ready;
    logic cmd_queue_ready;
    logic ibi_status_thld;
    logic rx_thld;
    logic tx_thld;
  } i3c_pio_intr_t;

  // Secondary Controller Interrupts.
  typedef struct packed {
    // The order of these fields is important in connecting to the `i3c_intr` instance.
    logic ccc_fatal_rstdaa_err;
    logic ccc_unhandled_nack;
    logic ccc_param_modified;
    logic stby_cr_op_rstact;
    logic stby_cr_accept_err;
    logic stby_cr_accept_ok;
    logic stby_cr_accept_nacked;
    logic stby_cr_dyn_addr;
    logic crr_response;
    logic acr_handoff_err_m3;
    logic acr_handoff_err_fail;
    logic acr_handoff_ok_primed;
    logic acr_handoff_ok_remain;
  } i3c_stby_cr_intr_t;

  // Immediate Data Transfer Command.
  // Format 1.
  typedef struct packed {
    logic     [7:0] data_byte_4;
    logic     [7:0] data_byte_3;
    logic     [7:0] data_byte_2;
    logic     [7:0] data_byte_1;
    logic           toc;
    logic           wroc;
    logic           rnw;
    i3c_xfer_mode_e mode;
    logic     [2:0] dtt;
    logic     [1:0] reserved;
    logic     [4:0] dev_index;
    logic           cp;
    i3c_ccc_e       cmd;
    logic     [3:0] tid;
    i3c_cmd_attr_e  cmd_attr;
  } i3c_xfer_cmd_imm_t;

  // Regular Transfer Command.
  // Format 1.
  typedef struct packed {
    logic    [15:0] data_length;
    logic     [7:0] reserved1;
    logic     [7:0] def_byte;
    logic           toc;
    logic           wroc;
    logic           rnw;
    i3c_xfer_mode_e mode;
    logic           dbp;
    logic           sre;
    logic     [2:0] reserved;
    logic     [4:0] dev_index;
    logic           cp;
    i3c_ccc_e       cmd;
    logic     [2:0] tid;
    i3c_cmd_attr_e  cmd_attr;
  } i3c_xfer_cmd_reg_t;

  // Combo Transfer Command.
  // Format 1.
  typedef struct packed {
    logic           toc;
    logic           wroc;
    logic           rnw;
    i3c_xfer_mode_e mode;
    logic           h_b;  // TODO:
    logic           fpm;
    logic     [2:0] dlp;
    logic           reserved;
    logic     [4:0] dev_index;
    logic           cp;
    i3c_ccc_e       cmd;
    logic     [2:0] tid;
    i3c_cmd_attr_e  cmd_attr;
  } i3c_xfer_cmd_combo_t;

  // Response Descriptor.
  typedef struct packed {
    i3c_err_status_e err_status;
    logic      [3:0] tid;
    logic      [7:0] reserved;
    logic     [15:0] data_length;
  } i3c_xfer_rsp_t;

endpackage
