// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Bus Functional Model of USBDEV.
class usbdev_bfm extends uvm_component;
  `uvm_component_utils(usbdev_bfm)

  // Shift used on address offset to identify the buffer being adressed.
  // TODO: not presently required.
  localparam int unsigned BufShift = $clog2(MaxPktSizeByte);

  // Number of bits required to represent the number of bytes within a buffer.
  localparam int unsigned BufSizeW = $clog2(MaxPktSizeByte + 1);

  //------------------------------------------------------------------------------------------------
  // Type definitions for use via API functions.
  //------------------------------------------------------------------------------------------------
  typedef bit [5:0] buf_num;

  // Details of RX FIFO entry.
  typedef struct
  {
    buf_num            buffer;
    bit          [3:0] ep;
    bit                setup;
    bit [BufSizeW-1:0] size;
  } rxfifo_entry;

  //------------------------------------------------------------------------------------------------
  // Internal type definitions.
  //------------------------------------------------------------------------------------------------
  // State of IN endpoint and any packet awaiting collection.
  typedef struct {
    buf_num            buffer;
    bit [BufSizeW-1:0] size;
    bit                sending;
    bit                pend;
    bit                rdy;
  } config_in_t;

  // Current interrupt state; exact CSR mimics, may be updated directly.
  bit [31:0] intr_test;
  bit [31:0] intr_state;
  bit [31:0] intr_enable;

  // Control and status.
  // TODO: enable required?
  bit [6:0] dev_address;

  // Most recent frame number.
  bit [10:0] frame;
  // Connection state.
  bit host_lost;  // Non-idle but no SOF detected for > 4ms.
  usbdev_link_state_e link_state;  // Current state of link.
  bit sense;  // VBUS/SENSE signal from host/hub.

  // Current endpoint configuration; exact CSR mimics, may be updated directly.
  bit [NEndpoints-1:0] ep_out_enable;
  bit [NEndpoints-1:0] ep_in_enable;
  bit [NEndpoints-1:0] rxenable_setup;
  bit [NEndpoints-1:0] rxenable_out;
  bit [NEndpoints-1:0] set_nak_out;
  bit [NEndpoints-1:0] out_stall;
  bit [NEndpoints-1:0] in_stall;
  bit [NEndpoints-1:0] out_iso;
  bit [NEndpoints-1:0] in_iso;

  // Current reception (SETUP and OUT) state information.
  token_pkt rx_token;  // Most recent OUT-side token packet.
  // Current transmission (IN) state information.
  bit [3:0] tx_ep;

  // Sent IN packets; exact CSR mimic, may be updated directly.
  bit [NEndpoints-1:0] in_sent;

  // Current IN configuration.
  config_in_t config_in[NEndpoints];

  // Data Toggle bits; current state of toggles may be _read_ directly.
  // Use 'write_in|out_toggles' functions to modify them.
  bit [NEndpoints-1:0] in_toggles;
  bit [NEndpoints-1:0] out_toggles;

  // Available OUT and SETUP Buffer FIFOs.
  buf_num avout_fifo[$];
  buf_num avsetup_fifo[$];

  // RX FIFO.
  rxfifo_entry rx_fifo[$];

  // Internal packet buffer memory; this is a 32-bit addressable read/write memory accessed from
  // both the CSR and USB sides. It is, however, decomposed into 'NumBuffers' packet-sized buffers
  // of 'MaxPktSizeByte' bytes each, with each packet buffer serving only as a undirectional channel
  // (IN packet or OUT packet), and is not required to support simultaneous reads and writes to any
  // given packet buffer.
  logic [31:0] buffer[NumBuffers * MaxPktSizeByte / 4];

  `uvm_component_new

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    // TODO: set up any required initial state.
  endfunction

  //------------------------------------------------------------------------------------------------
  // Interrupt-related operations.
  //------------------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------------------
  // CSR-side endpoint operations.
  //------------------------------------------------------------------------------------------------
  // TODO: anything to be done here?

  //------------------------------------------------------------------------------------------------
  // CSR-side Data Toggle operations.
  //------------------------------------------------------------------------------------------------
  function void write_out_toggles(bit [NEndpoints-1:0] mask, bit [NEndpoints-1:0] status);
    out_toggles = (out_toggles & ~mask) | (status & mask);
  endfunction

  function void write_in_toggles(bit [NEndpoints-1:0] mask, bit [NEndpoints-1:0] status);
    in_toggles = (in_toggles & ~mask) | (status & mask);
  endfunction

  //------------------------------------------------------------------------------------------------
  // CSR-side FIFO operations.
  //------------------------------------------------------------------------------------------------
  function void avsetup_fifo_add(buf_num b);
    if (avsetup_fifo.size() < AvSetupFIFODepth) begin
      avsetup_fifo.push_back(b);
      intr_state[IntrAvSetupEmpty] = 1'b0;  // Deassert Status interrupt.
    end else intr_state[IntrAvOverflow] = 1'b1;
  endfunction

  function void avout_fifo_add(buf_num b);
    if (avout_fifo.size() < AvOutFIFODepth) begin
      avout_fifo.push_back(b);
      intr_state[IntrAvOutEmpty] = 1'b0;  // Deassert Status interrupt.
    end else intr_state[IntrAvOverflow] = 1'b1;
  endfunction

  function bit rx_fifo_read(output rxfifo_entry entry);
    if (rx_fifo.size() > 0) begin
      entry = rx_fifo.pop_front();
      return 1;
    end
    // No FIFO entry available.
    return  0;
  endfunction

  //------------------------------------------------------------------------------------------------
  // CSR-side packet buffer operations.
  //------------------------------------------------------------------------------------------------
  // Write a 32-bit word into the packet buffer memory; address offset is in 32-bit words.
  function void write_buffer(int unsigned offset, logic [31:0] d);
    buffer[offset] = d;
  endfunction

  // Read a 32-bit word from the packet buffer memory; address offset is in 32-bit words.
  function logic [31:0] read_buffer(int unsigned offset);
    return buffer[offset];
  endfunction

  //------------------------------------------------------------------------------------------------
  // Link state changes.
  //------------------------------------------------------------------------------------------------
  function void set_enable(bit en);
    // TODO: do we need to retain this? I think it can certainly change the link_state.
    // enable = en;
  endfunction

  function void resume_link_active();
    `uvm_fatal(`gfn, "Not yet implemented")
  endfunction

  //------------------------------------------------------------------------------------------------
  // USB-side bus events.
  //------------------------------------------------------------------------------------------------
  function void bus_reset();
    `uvm_fatal(`gfn, "Not yet implemented")
  endfunction

  function void bus_suspend();
    `uvm_fatal(`gfn, "Not yet implemented")
  endfunction

  function void bus_resume();
    `uvm_fatal(`gfn, "Not yet implemented")
  endfunction

  function void bus_connect();
    `uvm_fatal(`gfn, "Not yet implemented")
  endfunction

  function void bus_disconnect();
    `uvm_fatal(`gfn, "Not yet implemented")
  endfunction

  //------------------------------------------------------------------------------------------------
  // USB-side packet operations.
  //------------------------------------------------------------------------------------------------
  function void sof_packet(ref sof_pkt sof);
    if (sof.valid_crc()) intr_state[IntrRxCrcErr] = 1'b1;
    if (sof.valid_pid()) begin
      // The frame number itself shall be used only if the CRC matches.
      if (sof.valid_crc()) frame = sof.framenum;
      // TODO: the SOF packet, however, may be used to update other state even if the CRC is
      // incorrect....host lost, link state.
    end else intr_state[IntrRxPidErr] = 1'b1;
  endfunction

  // Process a token packet from the USB host controller.
  function bit token_packet(ref token_pkt token, output usb20_item rsp);
    if (!token.valid_crc()) begin
      intr_state[IntrRxCrcErr] = 1'b1;
    end else begin
      case (token.m_pid_type)
        // SETUP token has immediate consequence for the DUT state; both SETUP and OUT token packets
        // cause some state information to be retained so that the ensuing DATA packet may be
        // processed.
        PidTypeSetupToken: begin
          // TODO:
          `downcast(rx_token, token.clone())
        end
        PidTypeOutToken: begin
          `downcast(rx_token, token.clone())
        end
        PidTypeInToken: begin
          return in_packet(token, rsp);
        end
        default: begin
          // DUT shall ignore all other token packets; not relevant to Full Speed devices.
          `uvm_info(`gfn, $sformatf("Ignoring unexpected token 0%0x", token.m_pid_type), UVM_LOW)
          if (!token.valid_pid()) intr_state[IntrRxPidErr] = 1'b1;
        end
      endcase
    end
    return 0;
  endfunction

  // Process receipt of a DATA packet from the USB host controller; returns an indication of
  // whether a response packet is expected, and the response itself via `rsp`.
  function bit data_packet(ref data_pkt data, output usb20_item rsp);
    if (!data.valid_crc()) begin
      intr_state[IntrLinkOutErr] = 1'b1;
      intr_state[IntrRxCrcErr] = 1'b1;
      return  0;
    end
    // Consult the most recent token packet to decide how to handle the DATA packet.
    if (rx_token == null || (rx_token.m_pid_type != PidTypeSetupToken &&
                             rx_token.m_pid_type != PidTypeOutToken)) begin
      // DUT shall ignore any DATA packet without a preceding SETUP/OUT packet; likely data loss.
      `uvm_info(`gfn, $sformatf("Ignoring orphaned DATA packet 0x%0x", data.m_pid_type), UVM_LOW)
      return 0;
    end
    return out_packet(rx_token, data, rsp);
  endfunction

  // Process an IN packet request from the USB host controller; returns an indication of whether
  // a response packet is expected, and the response itself via `rsp`.
  function bit in_packet(ref token_pkt token, output usb20_item rsp);
    bit [3:0] ep;

    // Validate inputs.
    assert(token.m_ev_type  == EvPacket);
    assert(token.m_pkt_type == PktTypeToken);

    // Basic check of whether we should do anything at all for this transaction.
    ep = token.endpoint;
    if (token.address != dev_address || ep >= NEndpoints || !ep_in_enable[ep]) return  0;

    if (in_stall[ep]) begin
      // TODO: Construct a STALL handshake packet.
      rsp = handshake_pkt::type_id::create("stall", this, PidTypeStall);
    end else begin
      // What about the IN configuration for this endpoint?
      if (config_in[ep].rdy) begin
        int unsigned buf_start = (config_in[ep].buffer * MaxPktSizeByte) >> 2;
        int unsigned pkt_len = config_in[ep].size;
        byte unsigned buf_data[$];
        data_pkt data;
        config_in[ep].sending = 1'b1;

        data = data_pkt::type_id::create("data", this, in_toggles[ep] ? PidTypeData1
                                                                      : PidTypeData0);
        // TODO: constructor is defeating us because we have no data; sort out the constructor mess.
        data.m_pid_type = in_toggles[ep] ? PidTypeData1 : PidTypeData0;

        // Collect the data bytes from the packet buffer memory.
        for (int unsigned offset = 0; offset < pkt_len; offset += 4) begin
          logic [31:0] d = read_buffer(buf_start + (offset >> 2));
          buf_data.push_back(d[7:0]);
          if (pkt_len - offset > 1) buf_data.push_back(d[15:8]);
          if (pkt_len - offset > 2) buf_data.push_back(d[23:16]);
          if (pkt_len - offset > 3) buf_data.push_back(d[31:24]);
        end
        data.data = buf_data;
        data.crc16 = data.exp_crc();
        rsp = data;
      end else begin
        rsp = handshake_pkt::type_id::create("nak", this, PidTypeNak);
      end
    end
    return 1;
  endfunction

  // Process a handshake packet, in response to an attempted transaction to the given endpoint.
  function void handshake_packet(ref handshake_pkt handshake);
    if (!handshake.valid_pid()) intr_state[IntrRxPidErr] = 1'b1;
    if (tx_ep < NEndpoints) begin
      case (handshake.m_pid_type)
        PidTypeAck: begin
          // Update the Data Toggle bit.
          in_toggles[tx_ep] ^= 1'b1;
          // Update the IN configuration.
          config_in[tx_ep].sending = 1'b0;
          config_in[tx_ep].rdy = 1'b0;
          in_sent[tx_ep] = 1'b1;
          intr_state[IntrPktSent] = 1'b1;
        end
        PidTypeNak: begin
          config_in[tx_ep].sending = 1'b0;
        end
      endcase
    end
  endfunction

  // Process the receipt of an OUT DATA or SETUP DATA packet.
  function bit out_packet(ref token_pkt token, ref data_pkt data, output usb20_item rsp);
    rxfifo_entry e;
    bit bad_toggle;
    bit [3:0] ep;

    // Validate inputs.
    assert(token.m_ev_type  == EvPacket);
    assert(data.m_ev_type   == EvPacket);
    assert(token.m_pkt_type == PktTypeToken);
    assert(data.m_pkt_type  == PktTypeData);

    // Basic check of whether we should do anything at all for this transaction.
    ep = token.endpoint;
    if (token.address != dev_address || ep >= NEndpoints) return  0;

    // First we concern ourselves with the packet type-dependent impact upon the hardware state.
    case (token.m_pid_type)
      PidTypeSetupToken: begin
        // Detection of a SETUP, whether or not it's stored clears the Data Toggles bits, iff
        // the endpoints are enabled.
        if (ep_out_enable[ep]) out_toggles[ep] = 1'b0;
        if (ep_in_enable[ep])  in_toggles[ep]  = 1'b1;

        if (!ep_out_enable[ep] || !avsetup_fifo.size() || rx_fifo.size() >= RxFIFODepth ||
            !rxenable_setup[ep]) begin
          // SETUP packets that cannot be accepted are just ignored.
          return  0;
        end

        bad_toggle = out_toggles[ep] ^ (data.m_pid_type != PidTypeData0);
        // TODO: bad toggle should never occur here.
        if (!bad_toggle) begin
          e.buffer = avsetup_fifo.pop_front();
          e.setup = 1;
          if (!avsetup_fifo.size()) intr_state[IntrAvSetupEmpty] = 1;
          // A SETUP packet successfully received also clears the stall conditions.
          out_stall[ep]   = 1'b0;
          in_stall[ep]    = 1'b0;
        end
      end
      PidTypeOutToken: begin
        // Note: the final entry of the RX FIFO is reserved solely for SETUP packets.
        if (!ep_out_enable[ep] || !avout_fifo.size() || rx_fifo.size() >= RxFIFODepth - 1 ||
            !rxenable_out[ep]) begin
          // Construct a NAK response indicating that we're busy, unless Isochronous.
          if (out_iso) return  0;
          rsp = handshake_pkt::type_id::create("nak", this, PidTypeNak);
          return 1;
        end
        // Is the OUT endpoint stalled?
        // TODO: resolve priority of this check against FIFO-related causes of NAK.
        if (out_stall[ep]) begin
          rsp = handshake_pkt::type_id::create("stall", this, PidTypeStall);
          return 1;
        end

        // A packet shall be ACKnowledged even if the Data Toggle bit is not as expected, because
        // that is taken to imply the loss of our previous ACKnowledgement and the host is
        // resending.
        //
        // We store and record the packet only if the toggle bit is as expected, however.
        bad_toggle = out_toggles[ep] ^ (data.m_pid_type != PidTypeData0);
        if (!bad_toggle) begin
          e.buffer = avout_fifo.pop_front();
          e.setup = 0;
          if (!avout_fifo.size()) intr_state[IntrAvOutEmpty] = 1'b1;
        end
      end
      default: `uvm_fatal(`gfn, $sformatf("Unexpected PID type 0x%x", token.m_pid_type))
    endcase

    if (bad_toggle) begin
      intr_state[IntrLinkOutErr] = 1'b1;
    end else begin
      // Now add the packet content to the buffer and its description to the RX FIFO.
      int unsigned buf_start = (e.buffer * MaxPktSizeByte) >> 2;
      // Update the Data Toggle bit.
      out_toggles[ep] ^= 1'b1;
      // Complete the packet description.
      e.ep = ep;
      e.size = data.data.size();
      for (int unsigned offset = 0; offset < e.size; offset += 4) begin
        logic [31:0] d;
        // Collect bytes, remembering that the final word may be incomplete; so leave as 'X'
        d[7:0] = data.data[offset];
        if (e.size - offset > 1) d[15:8]  = data.data[offset + 1];
        if (e.size - offset > 2) d[23:16] = data.data[offset + 2];
        if (e.size - offset > 3) d[31:24] = data.data[offset + 3];
        write_buffer(buf_start + (offset >> 2), d);
      end
      // Packet received.
      rx_fifo.push_back(e);
      intr_state[IntrRxFull] = (rx_fifo.size() >= RxFIFODepth);
      intr_state[IntrPktReceived] = 1'b1;
    end
    rsp = handshake_pkt::type_id::create("ack", this, PidTypeAck);
    return 1;
  endfunction

  // TODO: Other things that we may wish to model at some point.
  // - event counters.
  // - FIFO reset behavior.
  // Open questions:
  // - what do we do about phy sw pin drive functionality?

endclass : usbdev_bfm
