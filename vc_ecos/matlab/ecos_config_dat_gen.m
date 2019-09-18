clc
clear all
close all

file_mat = '../data/db/fpga/PS2PL_send0004_MatA_FRAME_INIT00.txt';
file_vec = '../data/db/fpga/PS2PL_send0001_SIGN.txt';

file_config = '../data/db/fpga/config.dat';
fid_config = fopen(file_config,'ab');

len_1 = func_decode_matrix2file(file_mat,fid_config)
len_2 = func_decode_vector2file(file_vec,fid_config,'%f')

fclose all