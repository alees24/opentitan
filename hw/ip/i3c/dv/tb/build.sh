echo Bulding I3C simulation environment with verilator...
export DVUTILS=../../../dv/sv/dv_utils
export RTL=../../rtl
export PRIMGEN=../../../prim_generic/rtl
export PRIM=../../../prim/rtl
export TOP=../../../../top_earlgrey/rtl/
export TLUL=../../../tlul/rtl
export UVM=/home/adrianl/uvm-1.2
export I3C_PKGS="$RTL/i3c_consts_pkg.sv $RTL/i3c_pkg.sv $RTL/i3c_reg_pkg.sv"
export I3C_RTL="$RTL/i3c.sv"
verilator --cc --exe --build --threads 1 --trace-fst -j 10 -I$RTL -I$PRIMGEN -I$PRIM -I$TLUL -I$RACL -I$UVM main.cpp $TOP/top_pkg.sv $PRIM/prim_secded_pkg.sv $PRIM/prim_subreg_pkg.sv $PRIM/prim_mubi_pkg.sv $TLUL/tlul_pkg.sv $PRIM/prim_alert_pkg.sv $PRIM/prim_count_pkg.sv $PRIMGEN/prim_ram_1p_pkg.sv $PRIM/prim_util_pkg.sv $TOP/autogen/top_racl_pkg.sv $I3C_PKGS tb.sv $PRIMGEN/prim_ram_1p.sv $PRIMGEN/prim_buf.sv $PRIMGEN/prim_clock_inv.sv $PRIMGEN/prim_flop.sv $PRIMGEN/prim_flop_2sync.sv $I3C_RTL ../../lint/i3c.vlt
echo Use 'obj_dir/Vtop_pkg' to run...
echo ... and then 'gtkwave Vtop_pkg.fst' to view traces/waves...
