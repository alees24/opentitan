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

  // Number of target(s) or target group(s) presented simultaneously on the I3C bus, including the
  // Secondary Controller Role.
  localparam int unsigned NumTargets = 2;
  localparam int unsigned Log2NT = $clog2(NumTargets);

  // Logical width of the data path; in practice this is fixed by the HCI specification.
  localparam int unsigned DW = 32;
  localparam int unsigned Log2DW = $clog2(DW);

  // Maximum of data received by the Target-side logic.
  localparam int unsigned TargDW  = 16;
  localparam int unsigned Log2TDW = $clog2(TargDW);

  // --- HCI/TCRI structure descriptions ---

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

  // --- End of HCI/TCRI structure descriptions ---

  // --- HDR-DDR structure descriptions ---

  typedef struct packed {
    logic        rnw;        // 1: Read, 0: Write.
    logic [14:8] cmd_code;   // Command code.
    logic  [7:1] targ_addr;  // Dynamic address of target.
    logic        para;       // Parity Adjustment.
  } i3c_ddr_cmd_word_t;

  // --- End of HDR-DDR structure descriptions ---

  // --- Interface to/from the Target Reset detector ---
  typedef struct packed {
    logic     enable;       // Enable the detector.
    logic     rst_periph;   // Reset Peripheral if Target Reset signaling detected.
    logic     rst_target;   // Reset Whole Target if Target Reset signaling detected.
  } i3c_rstdet_req_t;

  typedef struct packed {
    logic     activating;     // Waiting to become active.
    logic     active;         // Detector is active.
    logic     peri_rst_det;   // Peripheral Reset detected.
    logic     targ_rst_det;   // Whole Target Reset detected.
  } i3c_rstdet_rsp_t;

  // --- Interface to the internal message buffer ---

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

  // --- Requests to the transceiver logic ---

  // TODO: Perhaps this is becoming confused; signaling mode, direction and data unit all
  // conflated together.
  // Type of data unit transferred.
  typedef enum logic [3:0] {
    // HDR-DDR Words; bits [1:0] are the Preamble bits.
    I3CDType_CommandWord  = 4'b0001,
    I3CDType_DataWord     = 4'b0011,
    I3CDType_CRCWord      = 4'b0101,
    // SDR signaling.
    I3CDType_Address      = 4'b0100,
    I3CDType_SDRBytes     = 4'b1010,
    // HDR Restart/Exit signaling.
    I3CDType_HDRRestart   = 4'b1000,
    I3CDType_HDRExit      = 4'b1011,
    // Target Reset signaling.
    I3CDType_TargetReset  = 4'b1111
  } i3c_dtype_e;

  // --- Responses from the transceiver logic ---
  typedef enum logic [1:0] {
    I3CRType_OK = 2'b00
  } i3c_rtype_e;

  // Description of a Controller transceiver request.
  typedef struct packed {
    // Data type.
    i3c_dtype_e        dtype;
    // Signaling mode.
    i3c_xfer_mode_e    mode;
    // Reception, not transmission.
    logic              rx;
    // Specifies whether `mode` shall be interpreted as I3C or I2C.
    logic              i2c;
    // Write data and length.
    logic     [DW-1:0] wdata;
    logic [Log2DW-4:0] wlen;
  } i3c_ctrl_trx_req_t;

  // Description of a Controller transceiver response.
  typedef struct packed {
    // Response type.
    i3c_rtype_e        rtype;
    // TODO: Fold these into the response type?
    logic              parity_err;
    logic              crc5_err;
    // TODO: Perhaps not the most appropriate interface?
    i3c_dtype_e        dtype;
    // Response data.
    logic     [DW-1:0] rdata;
    logic [Log2DW-4:0] rlen;
  } i3c_ctrl_trx_rsp_t;

  // Target device description.
  // - these signals may be treated as quasi-static and must be set up before target mode is
  //   enabled.
  // TODO: Decide upon multi-cycle path/CDC/waiver here.
  typedef struct packed {
    logic  [6:0] mask;
    logic  [6:0] addr;

    logic [47:0] pid;  // Provisional ID.
    logic  [7:0] dcr;  // Device Characteristics Register.
    logic  [7:0] bcr;  // Bus Characteristics Register.
    logic  [7:0] lvr;  // Legacy I2C Virtual Register.

    // Data Transfer Early Termination Configuration.
    // TODO: These may be changed during operation.
    logic        en_write_nack;  // May NACK Write commands?
    logic        en_write_term;  // May perform Early Termination of Write transfers.
    logic        en_crc_drop;    // Suppress CRC Word after Early Termination of Read transfers.
  } i3c_targ_info_t;

  // Description of a Target transceiver request/state information.
  // - this must be presented up front for each supported target, allowing the transceiver to
  //   respond with sufficient speed to a Read command.
  typedef struct packed {
    // Target indicator.
    logic [Log2NT-1:0] targ_id;
    // Read data (from the perspective of the Controller).
    logic        [9:0] rdata_nq;
    logic        [9:0] rdata_pq;
  } i3c_targ_trx_req_t;

  // Description of a Target transceiver prefetch request.
  typedef struct packed {
    // Command code, including RnW as its MS bit.
    logic       [7:0] cmd;
    // Signaling mode.
    i3c_xfer_mode_e   mode;
  } i3c_targ_trx_pre_t;

  // Description of a Target transceiver response.
  typedef struct packed {
    // Response type.
    i3c_rtype_e         rtype;
    // TODO: Fold these into the response type?
    logic              parity_err;
    logic              crc5_err;
    // Data type.
    i3c_dtype_e         dtype;
    // Write data (from the perspective of the Controller).
    logic  [TargDW-1:0] wdata;
    logic [Log2TDW-4:0] wlen;
  } i3c_targ_trx_rsp_t;

  // Return the length in bits of the given data type, minus 1 for counting down.
  function automatic bit [Log2DW-1:0] unitlen_bits(i3c_dtype_e dtype);
    case (dtype)
      I3CDType_CommandWord,
      I3CDType_DataWord:  return Log2DW'(19);
      // CRC Words are 12 bits including the final setup bit ('1') for HDR Restart/Exit.
      I3CDType_CRCWord:   return Log2DW'(11);
      default:            return Log2DW'(8);  // includes I3CDType_SDRBytes/Address.
    endcase
  endfunction

endpackage
