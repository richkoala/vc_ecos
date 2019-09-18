onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb_env/ps2pl_cnt
add wave -noupdate /tb_env/ps2pl_sop
add wave -noupdate /tb_env/frame_len
add wave -noupdate /tb_env/ps2pl_start
add wave -noupdate /tb_env/pl_clk
add wave -noupdate /tb_env/pl_resetn
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {3109139 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 164
configure wave -valuecolwidth 40
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
WaveRestoreZoom {2853392 ps} {3099667 ps}
