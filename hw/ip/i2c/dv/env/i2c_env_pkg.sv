// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

package i2c_env_pkg;
  // dep packages
  import uvm_pkg::*;
  import top_pkg::*;
  import dv_utils_pkg::*;
  import csr_utils_pkg::*;
  import dv_base_reg_pkg::*;
  import tl_agent_pkg::*;
  import i2c_agent_pkg::*;
  import dv_lib_pkg::*;
  import cip_base_pkg::*;
  import i2c_reg_pkg::*;
  import i2c_ral_pkg::*;
  import i2c_pkg::*;

  // macro includes
  `include "dv_macros.svh"

  // parameters
  typedef enum int {
    FmtThreshold   = 0,
    RxThreshold    = 1,
    AcqThreshold   = 2,
    RxOverflow     = 3,
    ControllerHalt = 4,
    SclInference   = 5,
    SdaInference   = 6,
    StretchTimeout = 7,
    SdaUnstable    = 8,
    CmdComplete    = 9,
    TxStretch      = 10,
    TxThreshold    = 11,
    AcqStretch     = 12,
    UnexpStop      = 13,
    HostTimeout    = 14,
    NumI2cIntr     = 15
  } i2c_intr_e;

  typedef enum int {
    ReadOnly  = 0,
    WriteOnly = 1,
    ReadWrite = 2
  } tran_type_e;

  // Possible Start/Stop conditions that can be issued during an active transmission
  //
  // DUT Mode | SCL driver | Operation  |   Stage      | SDA Driver | Start/Stop condition
  // -------- | ---------- | ---------- | ------------ | ---------- | --------------------
  // Target   |    TB      | Write/Read | Address Byte |  TB        |  Start/Stop
  // Target   |    TB      | Write      | Data Byte    |  TB        |  Start/Stop
  // Target   |    TB      | Read       | Data byte    |  DUT       |  Start (with rdata 'hFF)
  // Target   |    TB      | Read       | Data Ack     |  TB        |  Start/Stop
  typedef enum int{
    AddressByteStart   = 0,
    AddressByteStop    = 1,
    WriteDataByteStart = 2,
    WriteDataByteStop  = 3,
    ReadDataByteStart  = 4,
    ReadDataAckStart   = 5,
    ReadDataAckStop    = 6
  } glitch_e;

  typedef enum int {
    Standard,
    Fast,
    FastPlus
  } speed_mode_e;

  typedef uvm_tlm_analysis_fifo #(i2c_item) i2c_analysis_fifo;

  typedef i2c_item i2c_transfer_q[$];
  typedef i2c_transfer_q i2c_transaction;

  parameter uint I2C_FMT_FIFO_DEPTH = i2c_reg_pkg::FifoDepth;
  parameter uint I2C_RX_FIFO_DEPTH  = i2c_reg_pkg::FifoDepth;
  parameter uint I2C_TX_FIFO_DEPTH  = i2c_reg_pkg::FifoDepth;
  parameter uint I2C_ACQ_FIFO_DEPTH = i2c_reg_pkg::AcqFifoDepth;

  // alerts
  parameter uint NUM_ALERTS = i2c_reg_pkg::NumAlerts;
  parameter string LIST_OF_ALERTS[NUM_ALERTS] = {"fatal_fault"};

  function automatic i2c_acqdata_item acq2item(bit [bus_params_pkg::BUS_DW-1:0] data);
    i2c_acqdata_item item;
    `uvm_create_obj(i2c_acqdata_item, item);

    // abyte+signal use the lower 11 bits of the ACQDATA register
    // Check unused upper bits are zero
    `DV_CHECK_EQ(data >> 11, '0, , , $sformatf("%m"))

    // Decode ACQFIFO item
    item.abyte = data[7:0];
    item.signal = i2c_acq_byte_id_e'(data[10:8]);

    return item;
  endfunction: acq2item

  // Print write data with 16 byte aligned.
  function automatic void print_host_wr_data(bit [7:0] data[$]);
    int idx = 1;
    string str;
    foreach (data[i]) begin
      if (i % 16 == 0) begin
        str = $sformatf("wrdata: %4d:", i  / 16);
      end
      str = {str, $sformatf(" %2x", data[i])};
      if ((i+1) % 16 == 0) begin
        `uvm_info("print_host_wr_data", str, UVM_MEDIUM)
        str = "";
      end
    end
    if (str != "") begin
      `uvm_info("print_host_wr_data", str, UVM_MEDIUM)
    end
    `uvm_info("print_host_wr_data", $sformatf("wrdata: size: %0d", data.size()), UVM_MEDIUM)
  endfunction

  // package sources
  `include "i2c_seq_cfg.sv"
  `include "i2c_env_cfg.sv"
  `include "i2c_env_cov.sv"
  `include "i2c_virtual_sequencer.sv"
  `include "i2c_reference_model.sv"
  `include "i2c_scoreboard.sv"
  `include "i2c_env.sv"
  `include "i2c_vseq_list.sv"

endpackage : i2c_env_pkg
