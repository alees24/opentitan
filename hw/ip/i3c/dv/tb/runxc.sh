RTL=../../rtl
TB=.
PRIM=../../../prim/rtl
PRIMGEN=../../../prim_generic/rtl
RACL=../../../../top_earlgrey/rtl/autogen
TOP=../../../../top_earlgrey/rtl
TLUL=../../../tlul/rtl
xrun -input waves.tcl -access r -incdir $PRIM $PRIM/prim_alert_pkg.sv $PRIM/prim_mubi_pkg.sv $PRIM/prim_util_pkg.sv $PRIMGEN/prim_ram_1p_pkg.sv $PRIM/prim_secded_pkg.sv $PRIM/prim_subreg_pkg.sv $TOP/top_pkg.sv $TLUL/tlul_pkg.sv $RACL/top_racl_pkg.sv $PRIMGEN/prim_buf.sv $PRIMGEN/prim_clock_mux2.sv $PRIMGEN/prim_clock_buf.sv $PRIMGEN/prim_clock_inv.sv $PRIMGEN/prim_flop.sv $PRIMGEN/prim_flop_2sync.sv $PRIM/prim_onehot_check.sv $PRIM/prim_arbiter_fixed.sv $PRIM/prim_secded_inv_39_32_dec.sv $PRIM/prim_secded_inv_39_32_enc.sv $PRIM/prim_secded_inv_64_57_dec.sv $PRIM/prim_secded_inv_64_57_enc.sv $PRIM/prim_subreg.sv $PRIM/prim_subreg_arb.sv $PRIM/prim_subreg_ext.sv $PRIM/prim_reg_we_check.sv $PRIM/prim_fifo_sync.sv $RTL/i3c_consts_pkg.sv $RTL/i3c_pkg.sv $RTL/i3c_reg_pkg.sv $RTL/i3c_reg_top.sv $RTL/i3c_controller.sv $RTL/i3c_target.sv $RTL/i3c_buffer.sv $RTL/i3c_tx.sv $RTL/i3c_rx.sv $RTL/i3c_phy.sv $TB/tb.sv $PRIMGEN/prim_ram_1p.sv $PRIM/prim_ram_1p_adv.sv $PRIM/prim_sec_anchor_buf.sv $PRIM/prim_sec_anchor_flop.sv $PRIM/prim_diff_decode.sv $PRIM/prim_alert_sender.sv $PRIM/prim_intr_hw.sv $TLUL/tlul_cmd_intg_gen.sv $TLUL/tlul_rsp_intg_gen.sv $TLUL/tlul_cmd_intg_chk.sv $TLUL/tlul_data_integ_dec.sv $TLUL/tlul_data_integ_enc.sv $TLUL/tlul_err.sv $TLUL/tlul_fifo_sync.sv $TLUL/tlul_adapter_reg.sv $TLUL/tlul_sram_byte.sv $TLUL/tlul_adapter_sram.sv $TLUL/tlul_socket_1n.sv $RTL/i3c_patt_detector.sv $RTL/i3c_core.sv $RTL/i3c.sv

