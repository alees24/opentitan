// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

class usbdev_scoreboard extends cip_base_scoreboard #(
  .CFG_T(usbdev_env_cfg),
  .RAL_T(usbdev_reg_block),
  .COV_T(usbdev_env_cov)
);
  `uvm_component_utils(usbdev_scoreboard)

  // Model of USBDEV.
  usbdev_bfm bfm;

  // TLM agent fifos
  uvm_tlm_analysis_fifo #(usb20_item) req_usb20_fifo;
  uvm_tlm_analysis_fifo #(usb20_item) rsp_usb20_fifo;

  // Intr checks
  local bit [NumUsbdevInterrupts-1:0] intr_exp;
  local bit [NumUsbdevInterrupts-1:0] intr_exp_at_addr_phase;

  // Local queue of expected responses from the DUT.
  local usb20_item expected_rsp_q[$];

  `uvm_component_new

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    bfm = new();
    req_usb20_fifo = new("req_usb20_fifo", this);
    rsp_usb20_fifo = new("rsp_usb20_fifo", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
  endfunction

  task run_phase(uvm_phase phase);
    super.run_phase(phase);
    fork
      process_usb20_req();
      process_usb20_rsp();
    join_none
  endtask

  // Receive and model the request from the host/driver to the DUT.
  virtual task process_usb20_req();
    usb20_item item;
    forever begin
      req_usb20_fifo.get(item);
      case (item.m_ev_type)
        EvBusReset:   `uvm_info(`gfn, "Bus Reset received from monitor", UVM_MEDIUM)
        EvSuspend:    `uvm_info(`gfn, "Suspend Signaling received from monitor", UVM_MEDIUM)
        EvResume:     `uvm_info(`gfn, "Resume Signaling received from monitor", UVM_MEDIUM)
        EvDisconnect: `uvm_info(`gfn, "VBUS Disconnection received from monitor", UVM_MEDIUM)
        EvConnect:    `uvm_info(`gfn, "VBUS Connection received from monitor", UVM_MEDIUM)
        EvPacket: begin
          // Response to received packet, if any.
          usb20_item rsp;
          // Driver -> DUT traffic is just passed to the BFM to update its internal state.
          case (item.m_pkt_type)
            PktTypeSoF: begin
              sof_pkt sof;
              `downcast(sof, item);
              bfm.sof_packet(sof);
            end
            PktTypeToken: begin
              token_pkt token;
              `downcast(token, item);
              if (bfm.token_packet(token, rsp)) expected_rsp_q.push_back(rsp);
            end
            PktTypeData: begin
              data_pkt data;
              `downcast(data, item);
              if (bfm.data_packet(data, rsp)) expected_rsp_q.push_back(rsp);
            end
            PktTypeHandshake: begin
              handshake_pkt handshake;
              `downcast(handshake, item);
              bfm.handshake_packet(handshake);
            end
            // The BFM does not need to know about Special PIDs since the DUT shall ignore them.
            default: `uvm_info(`gfn, "Special PID packet received from monitor", UVM_DEBUG)
          endcase
        end
        default: `uvm_fatal(`gfn, $sformatf("Invalid/unexpected event type %p", item.m_ev_type))
      endcase
    end
  endtask

  // -------------------------------
  virtual task process_usb20_rsp();
    usb20_item act_item;
    usb20_item exp_item;
    forever begin
      // Collect an actual response from the DUT.
      rsp_usb20_fifo.get(act_item);
      `uvm_info(`gfn, "Comparing DUT response against the BFM exected response:", UVM_MEDIUM)
      `uvm_info(`gfn, $sformatf(" - Actual:\n%0s", act_item.sprint()), UVM_MEDIUM)
      `DV_CHECK_GT(expected_rsp_q.size(), 0, "Unexpected response from DUT; no expectation ready")
      // Compare it against the expected response.
      exp_item = expected_rsp_q.pop_front();
      `uvm_info(`gfn, $sformatf(" - Expected:\n%0s", exp_item.sprint()), UVM_MEDIUM)
      `DV_CHECK_EQ(act_item.compare(exp_item), 1, "DUT response does not match expectation")
    end
  endtask

  // These two tasks are overridden because the implementation in the base class performs checks
  // against predictions, whereas the USBDEV packet buffer memory may change essentially at any
  // time because packets are written into it automatically by the DUT.
  //
  // We could perhaps at some point be a little smarter and expect that only those buffers that
  // have been made available for DUT use may change, since we _can_ track which buffers have been
  // placed in the Av OUT/SETUP FIFOs and which have subsequently been removed from the RX FIFO.
  virtual task process_mem_write(tl_seq_item item, string ral_name);
    // Update the BFM's model of the DUT packet buffer memory.
// TODO:
//    bfm.write_buffer(item.a_data);
  endtask
  virtual task process_mem_read(tl_seq_item item, string ral_name);
    // Check that the word read from the DUT packet buffer matches that within the BFM.
  endtask

  // Return the index that a register name refers to e.g. "configin_1" yields 1
  function int unsigned get_index_from_reg_name(string reg_name);
    int str_len = reg_name.len();
    // Note: this extracts the final two characters which are either '_y' or 'xy',
    //       and because '_' is permitted in (System)Verilog numbers, it works for 0-99
    string index_str = reg_name.substr(str_len-2, str_len-1);
    return index_str.atoi();
  endfunction

  // Look up the address of a TL access and return information about the target CSR (handle, name,
  // and index if it's within a bank of similar CSRs) or memory (name, word index within the
  // memory).
  function string lookup_tl_addr(ref tl_seq_item item, input string ral_name,
                                 output uvm_reg csr, output int unsigned index);
    // Attempt to locate a CSR at this address.
    uvm_reg_addr_t csr_addr = cfg.ral_models[ral_name].get_word_aligned_addr(item.a_addr);
    if (csr_addr inside {cfg.ral_models[ral_name].csr_addrs}) begin
      string csr_name;
      csr = cfg.ral_models[ral_name].default_map.get_reg_by_offset(csr_addr);
      `DV_CHECK_NE_FATAL(csr, null)
      csr_name = csr.get_name();
      index = get_index_from_reg_name(csr_name);
      return csr_name;
    end else if (is_mem_addr(item, ral_name)) begin
      // There is only a single memory window; it provides access to the packet buffer memory.
      uvm_mem mem = ral.default_map.get_mem_by_offset(item.a_addr);
      uvm_reg_addr_t masked_addr = item.a_addr & ral.get_addr_mask();
      uvm_reg_addr_t base;
      `DV_CHECK_NE(mem, null)
      base = mem.get_offset(0, ral.default_map);
      index = (masked_addr - base) >> $clog2(TL_DW / 8);
      // Caller shall use the name to differentiate from the CSRs.
      csr = null;
      return "buffer";
    end else begin
      `uvm_fatal(`gfn, $sformatf("Access to unexpected addr 0x%0h", csr_addr))
    end
  endfunction

  // TL Address Channel transaction.
  function void process_tl_addr(ref tl_seq_item item, input string ral_name);
    logic [TL_DW-1:0] wdata = item.a_data;
    int unsigned index;
    uvm_reg csr;
    string csr_name = lookup_tl_addr(item, ral_name, csr, index);

    // TODO: Nothing to do here for reads?
    if (!item.is_write()) return;

    `uvm_info(`gfn, $sformatf("Writing 0x%0x to %s (index %0h)", wdata, csr_name, index),
              UVM_MEDIUM)
    if (csr_name != "buffer") void'(csr.predict(.value(wdata), .kind(UVM_PREDICT_WRITE)));

    // Update the state of the model according to this write operation.
    case (1)
      // Interrupt handling.
      (!uvm_re_match("intr_test", csr_name)): begin
        bit [TL_DW-1:0] intr_en = ral.intr_enable.get_mirrored_value();
        intr_exp |= item.a_data;
        if (cfg.en_cov) begin
          foreach (intr_exp[i]) begin
            cov.intr_test_cg.sample(i, item.a_data[i], intr_en[i], intr_exp[i]);
          end
        end
        // TODO: Is this required; does not the RAL predictor understand the register field type?
        // this field is WO - always returns 0
        // void'(csr.predict(.value(0), .kind(UVM_PREDICT_WRITE)));
      end
      (!uvm_re_match("intr_state", csr_name)): bfm.intr_state &= ~wdata;
      (!uvm_re_match("intr_enable", csr_name)): bfm.intr_enable = wdata;

      // Control register.
      (!uvm_re_match("usbctrl", csr_name)): begin
        bfm.dev_address = get_field_val(ral.usbctrl.device_address, wdata);
        if (get_field_val(ral.usbctrl.resume_link_active, wdata)) bfm.resume_link_active();
        bfm.set_enable(get_field_val(ral.usbctrl.enable, wdata));
      end

      // Simple register configuration writes.
      (!uvm_re_match("ep_out_enable", csr_name)):  bfm.ep_out_enable  = NEndpoints'(wdata);
      (!uvm_re_match("ep_in_enable", csr_name)):   bfm.ep_in_enable   = NEndpoints'(wdata);
      (!uvm_re_match("rxenable_setup", csr_name)): bfm.rxenable_setup = NEndpoints'(wdata);
      (!uvm_re_match("rxenable_out", csr_name)):   bfm.rxenable_out   = NEndpoints'(wdata);
      (!uvm_re_match("set_nak_out", csr_name)):    bfm.set_nak_out    = NEndpoints'(wdata);
      (!uvm_re_match("out_stall", csr_name)):      bfm.out_stall      = NEndpoints'(wdata);
      (!uvm_re_match("in_stall", csr_name)):       bfm.in_stall       = NEndpoints'(wdata);
      (!uvm_re_match("out_iso", csr_name)):        bfm.out_iso        = NEndpoints'(wdata);
      (!uvm_re_match("in_iso", csr_name)):         bfm.in_iso         = NEndpoints'(wdata);

      // configin_ registers (of which are there many), specifying IN packets for collection, are
      // more involved.
      (!uvm_re_match("configin_*", csr_name)): begin
        int unsigned ep = index;  // One register per endpoint.
        // Write 1 to clear fields.
        if (get_field_val(ral.configin[ep].sending, wdata)) bfm.config_in[ep].sending = 1'b0;
        if (get_field_val(ral.configin[ep].pend, wdata))    bfm.config_in[ep].pend    = 1'b0;
        bfm.config_in[ep].buffer = get_field_val(ral.configin[ep].buffer, wdata);
        bfm.config_in[ep].size   = get_field_val(ral.configin[ep].size,   wdata);
        bfm.config_in[ep].rdy    = get_field_val(ral.configin[ep].rdy,    wdata);
      end

      // in_sent register, clears interrupts for sent packet.
      (!uvm_re_match("in_sent", csr_name)): bfm.in_sent &= ~wdata;

      // Available Buffer FIFOs.
      (!uvm_re_match("avoutbuffer", csr_name)): begin
        bfm.avout_fifo_add(get_field_val(ral.avoutbuffer.buffer, wdata));
      end
      (!uvm_re_match("avsetupbuffer", csr_name)): begin
        bfm.avsetup_fifo_add(get_field_val(ral.avsetupbuffer.buffer, wdata));
      end

      // Read Only registers.
      (!uvm_re_match("usbstat", csr_name)),
      (!uvm_re_match("rxfifo", csr_name)),
      (!uvm_re_match("phy_pins_sense", csr_name)): begin
        `uvm_info(`gfn, $sformatf("Write to RO register '%0s'", csr_name), UVM_LOW)
      end

      // Write Only registers that cannot be modified by the DUT hardware.
      (!uvm_re_match("wake_control", csr_name)),
      (!uvm_re_match("phy_config", csr_name)),
      (!uvm_re_match("phy_pins_drive", csr_name)): begin
      end

      // Packet buffer access.
      (!uvm_re_match("buffer", csr_name)): bfm.write_buffer(index, wdata);

      default: `uvm_fatal(`gfn, $sformatf("TL address access to '%0s' not handled", csr_name))
    endcase
  endfunction

  // TL Data Channel transaction.
  function void process_tl_data(ref tl_seq_item item, input string ral_name);
    logic [TL_DW-1:0] rdata = item.d_data;
    int unsigned index;
    uvm_reg csr;
    string csr_name = lookup_tl_addr(item, ral_name, csr, index);
    bit do_read_check = 1'b1;

    // TODO: Nothing to do here for writes?
    if (item.is_write()) return;

    `uvm_info(`gfn, $sformatf("Reading from %0s (index 0x%0h)", csr_name, index), UVM_MEDIUM)
    if (csr_name != "buffer") void'(csr.predict(.value(item.d_data), .kind(UVM_PREDICT_READ)));

    case (1)
      // Interrupt handling.
      (!uvm_re_match("intr_test", csr_name)): begin
      end
      (!uvm_re_match("intr_state", csr_name)): begin
        // TODO:
      end
      "alert_test": begin
        // TODO
      end

      // Simple register configuration reads; these registers are unmodified by the DUT hardware.
      (!uvm_re_match("ep_out_enable", csr_name)),
      (!uvm_re_match("ep_in_enable", csr_name)),
      (!uvm_re_match("rxenable_setup", csr_name)),
      (!uvm_re_match("rxenable_out", csr_name)),
      (!uvm_re_match("set_nak_out", csr_name)),
      (!uvm_re_match("out_stall", csr_name)),
      (!uvm_re_match("in_stall", csr_name)),
      (!uvm_re_match("out_iso", csr_name)),
      (!uvm_re_match("in_iso", csr_name)): do_read_check = 1'b1;

      (!uvm_re_match("usbstat", csr_name)): begin
        int unsigned avsetup_lvl = bfm.avsetup_fifo.size();
        int unsigned avout_lvl = bfm.avout_fifo.size();
        int unsigned rx_lvl = bfm.rx_fifo.size();
        bit avsetup_full = (avsetup_lvl >= AvSetupFIFODepth);
        bit avout_full = (avout_lvl >= AvOutFIFODepth);
        // Form a prediction from the current BFM state; this is mostly the status information on
        // the FIFOs.
        uvm_reg_data_t usbstat = get_csr_val_with_updated_field(ral.usbstat.frame, 0, bfm.frame);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.host_lost, usbstat, bfm.host_lost);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.link_state, usbstat, bfm.link_state);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.sense, usbstat, bfm.sense);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.av_out_depth, usbstat, avout_lvl);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.av_setup_depth, usbstat, avsetup_lvl);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.av_out_full, usbstat, avout_full);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.rx_depth, usbstat, rx_lvl);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.av_setup_full, usbstat, avsetup_full);
        usbstat = get_csr_val_with_updated_field(ral.usbstat.rx_empty, usbstat, !rx_lvl);
        `DV_CHECK_EQ(rdata, usbstat)
        void'(csr.predict(.value(usbstat), .kind(UVM_PREDICT_READ)));
      end
      (!uvm_re_match("avoutbuffer", csr_name)),
      (!uvm_re_match("avsetupbuffer", csr_name)): begin
        // TODO: DV should never be reading from these.
        do_read_check = 1'b0;
      end

      // TODO:
      (!uvm_re_match("rxfifo", csr_name)): begin
        // Collect a RX FIFO entry from the BFM, if there is one.
        usbdev_bfm::rxfifo_entry entry;
        if (bfm.rx_fifo_read(entry)) begin
          // Check the fields of the RX FIFO match the expectations produced by the BFM.
          `DV_CHECK_EQ(get_field_val(ral.rxfifo.buffer, rdata), entry.buffer, "RX buffer mismatch")
          `DV_CHECK_EQ(get_field_val(ral.rxfifo.ep,     rdata), entry.ep,     "RX ep mismatch")
          `DV_CHECK_EQ(get_field_val(ral.rxfifo.setup,  rdata), entry.setup,  "RX setup mismatch")
          `DV_CHECK_EQ(get_field_val(ral.rxfifo.size,   rdata), entry.size,   "RX size mismatch")
        end else begin
          // TODO: DV should never really be doing this; but the DUT won't fault it.
        end
        do_read_check = 1'b0;
      end

      // configin_ registers (of which are there many), specifying IN packets for collection.
      // TODO: the BFM has its own idea of these registers.
      (!uvm_re_match("configin_*", csr_name)): do_read_check = 1'b0;

      (!uvm_re_match("in_sent", csr_name)): begin
        do_read_check = 1'b0;
      end

      // The BFM has internal knowledge of the Data Toggle bits because packet transactions
      // and link resets modify them.
      (!uvm_re_match("out_data_toggle", csr_name)): begin
         void'(csr.predict(.value(bfm.out_toggles), .kind(UVM_PREDICT_READ)));
      end
      (!uvm_re_match("in_data_toggle", csr_name)): begin
         void'(csr.predict(.value(bfm.out_toggles), .kind(UVM_PREDICT_READ)));
      end

      (!uvm_re_match("phy_pins_sense", csr_name)): do_read_check = 1'b0;

      (!uvm_re_match("wake_events", csr_name)): begin
        do_read_check = 1'b0;
      end
      (!uvm_re_match("fifo_ctrl", csr_name)): begin
        // TODO
        do_read_check = 1'b0;
      end
      (!uvm_re_match("count_out", csr_name)): begin
        do_read_check = 1'b0;
      end
      (!uvm_re_match("count_in", csr_name)): begin
        do_read_check = 1'b0;
      end
      (!uvm_re_match("count_nodata_in", csr_name)): begin
        do_read_check = 1'b0;
      end
      (!uvm_re_match("count_errors", csr_name)): begin
        do_read_check = 1'b0;
      end

      // Write Only registers that cannot be altered by the DUT hardware.
      (!uvm_re_match("intr_enable", csr_name)),
      (!uvm_re_match("wake_control", csr_name)),
      (!uvm_re_match("phy_config", csr_name)),
      (!uvm_re_match("phy_pins_drive", csr_name)): begin
      end

      // Packet buffer access.
      (!uvm_re_match("buffer", csr_name)): begin
        logic [TL_DW-1:0] exp_data = bfm.read_buffer(index);
        `DV_CHECK_EQ(item.d_data, exp_data, "Unexpected read data from packet buffer memory")
        do_read_check = 1'b0;  // Do not attempt to access/check 'csr' below.
      end

      default: `uvm_fatal(`gfn, $sformatf("TL data access to '%0s' not handled", csr_name))
    endcase
    // On reads, if do_read_check is set, check the mirrored value against `item.d_data`
    if (do_read_check) begin
      `DV_CHECK_EQ(csr.get_mirrored_value(), item.d_data,
                   $sformatf("reg name: %0s", csr.get_full_name()))
    end
    if (csr_name != "buffer") void'(csr.predict(.value(item.d_data), .kind(UVM_PREDICT_READ)));
  endfunction

  virtual task process_tl_access(tl_seq_item item, tl_channels_e channel, string ral_name);
    case (channel)
      AddrChannel: process_tl_addr(item, ral_name);
      DataChannel: process_tl_data(item, ral_name);
      default: `uvm_fatal(`gfn, $sformatf("Invalid channel: %0h", channel))
    endcase
  endtask

  virtual function void reset(string kind = "HARD");
    super.reset(kind);
    // Reset local fifos queues and variables
    req_usb20_fifo.flush();
    rsp_usb20_fifo.flush();
    expected_rsp_q.delete();
    intr_exp = 0;
    intr_exp_at_addr_phase = 0;
  endfunction

  function void check_phase(uvm_phase phase);
    super.check_phase(phase);
    // Post test checks to ensure that all local fifos and queues are empty
    `DV_EOT_PRINT_TLM_FIFO_CONTENTS(usb20_item, req_usb20_fifo)
    `DV_EOT_PRINT_TLM_FIFO_CONTENTS(usb20_item, rsp_usb20_fifo)
  endfunction

endclass
