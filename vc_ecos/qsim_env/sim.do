#cd G:/zho/dma_/VC/ecos_master/qsim_env
#cd E:/proj/proj_ax_b/VERSION_VC/ecos_master/qsim_env
vlib work
vlog -work work ps_master.sv
vlog -work work tb.sv


vsim -t 1ps -novopt work.tb_env
log tb_env/* -r    
do wave.do
run 1ms

