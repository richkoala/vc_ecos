clc
clear all
close all

path_data_db = '../data/db/fpga/';
mat_l = func_decode_matrix(strcat(path_data_db,'MatL_PS2PL.txt'));
mat_lt = func_decode_matrix(strcat(path_data_db,'MatL_T_PS2PL.txt'));

max(max(abs(mat_l - mat_lt')))

% mesh(abs(mat_l))
dem = size(mat_lt);

col_nz = zeros(dem(1),0);

for col_idx=1:dem(1)
    
    col_nz(col_idx) = sum(mat_lt(:,col_idx)~=0);
    
end

csc_col = cumsum(col_nz);

file_name = '../data/db/fpga/MatLt_ROW_PS2PL.txt';
fid = fopen(file_name,'r');
sop = fscanf(fid,'%d',4);
dat = fscanf(fid,'%f',inf);
csc_col_ps2pl = (dat(1:end-3))';

cmp = [csc_col,csc_col_ps2pl];
