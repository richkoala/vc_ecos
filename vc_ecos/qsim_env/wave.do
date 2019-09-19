onerror {resume}
radix define float32 {
    -default symbolic
}
radix define float64 {
    -default symbolic
}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb_env/ps2pl_cnt
add wave -noupdate /tb_env/ps2pl_sop
add wave -noupdate -radix unsigned /tb_env/frame_id
add wave -noupdate -radix decimal /tb_env/frame_len
add wave -noupdate -radix decimal /tb_env/frame_cnt
add wave -noupdate -radix decimal /tb_env/frame_iter_cnt
add wave -noupdate /tb_env/ps2pl_start
add wave -noupdate /tb_env/pl_clk
add wave -noupdate /tb_env/pl_resetn
add wave -noupdate /tb_env/buf_w32
add wave -noupdate /tb_env/m_axis_mm2s_tdata
add wave -noupdate /tb_env/m_axis_mm2s_tkeep
add wave -noupdate /tb_env/m_axis_mm2s_tlast
add wave -noupdate /tb_env/m_axis_mm2s_tvalid
add wave -noupdate -radix unsigned /tb_env/trans_send/i
add wave -noupdate -radix unsigned /tb_env/ps2pl_trans_cnt
add wave -noupdate -radix unsigned /tb_env/data_last_num
add wave -noupdate -radix decimal /tb_env/row_0
add wave -noupdate -radix decimal /tb_env/row_1
add wave -noupdate -radix decimal /tb_env/row_2
add wave -noupdate -radix decimal /tb_env/row_3
add wave -noupdate /tb_env/trans_dat32_flag
add wave -noupdate /tb_env/trans_dat64_flag
add wave -noupdate /tb_env/trans_dat128_flag
add wave -noupdate /tb_env/row_idx
add wave -noupdate /tb_env/col_idx
add wave -noupdate /tb_env/dat_val
add wave -noupdate /tb_env/b_val0
add wave -noupdate /tb_env/b_val1
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {3005000 ps} 0} {{Cursor 2} {4829737 ps} 0}
quietly wave cursor active 2
configure wave -namecolwidth 203
configure wave -valuecolwidth 247
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {194370064 ps}
