# waves.tcl
#
# SystemVerilog source file `expr.sv` contains top-level module `tb`
#
#  For simple manual invocation of xcelium:
#  xrun -input waves.tcl -access r <expr.sv>
#
database -open waves.shm -default -shm
probe tb -all -depth all -memories -shm
run
finish 2

