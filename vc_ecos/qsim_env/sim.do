#cd G:/zho/dma_/VC/ecos_master/qsim_env
vlib work
vlog -work work tb_env.sv


vsim -t 1ps -novopt work.tb_env
log tb_env/* -r    
do wave.do
run 10us

