clc
clear all
close all
fclose all

dem = 757;

path_data_db = '../data/db/fpga';
file_start_idx = 1;
files          = dir(path_data_db);

file_config = '../data/dout/ps_result.dat';
if (exist(file_config,'file'))
    system('del ..\data\dout\ps_result.dat');
end

fid_config = fopen(file_config,'a');
len_all = [];
for i = file_start_idx :length(files)
    filename = files(i).name;
    if (length(filename )>=7)
        if (strcmp(filename(1:8),'PS_SOLVE'))
                                    
            filename_ps_result_path = strcat(path_data_db,'/',filename);
            fid_ps_result = fopen(filename_ps_result_path);
            disp(filename);
                len = func_decode_vector2txtfile(fid_ps_result,fid_config,'%f');
                len_all = [len_all,len];
                if (len ~= dem*8)
                    disp(sprintf('%f len is not equal %f',filename_ps_result_path,Dem));
                end
%             dec2hex(sum(len_all))        
        end
    end
end
 fclose all
dec2hex(sum(len_all))