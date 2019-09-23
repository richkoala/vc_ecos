clc
clear all
close all
fclose all

dem_all = [727,757]
mode_all = ['p','s']

for dem_cnt = 1:length(dem_all)
    for mode_cnt = 1:length(mode_all)
        dem = dem_all(dem_cnt)
        mode = mode_all(mode_cnt)
        path_data_db = strcat('../data_',mode,num2str(dem),'/db/fpga');
        file_start_idx = 1;
        files          = dir(path_data_db);

        file_config = strcat('../data_',mode,num2str(dem),'/dout/ps_result_',mode,num2str(dem),'.txt');
        sys_cmd1 = strcat('del ..\data_',mode,num2str(dem),'\dout\ps_result_',mode,num2str(dem),'.txt');
        sys_cmd2 = strcat('copy ..\data_',mode,num2str(dem),'\dout\ps_result_',mode,num2str(dem),'.txt '  , ' ..\inout\ps_result_',mode,num2str(dem),'.txt');
        if (exist(file_config,'file'))
            system(sys_cmd1);
        end

        fid_config = fopen(file_config,'w');
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

        system(sys_cmd2)
end
end