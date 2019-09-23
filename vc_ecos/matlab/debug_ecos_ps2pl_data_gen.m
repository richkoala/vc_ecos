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


        file_config = strcat('../data_',mode,num2str(dem),'/din/config_',mode,num2str(dem),'.dat');
        sys_cmd1 = strcat('del ..\data_',mode,num2str(dem),'\din\config_',mode,num2str(dem),'.dat');
        sys_cmd2 = strcat('copy ..\data_',mode,num2str(dem),'\din\config_',mode,num2str(dem),'.dat '  , ' ..\inout\config_',mode,num2str(dem),'.dat');
        if (exist(file_config,'file'))
            system(sys_cmd1);
        end



        if (exist(file_config,'file'))
            system('del ..\data\db\fpga\config.dat');
        end

        fid_config = fopen(file_config,'ab');
        len_all = [];
        for i = file_start_idx :length(files)
            filename = files(i).name;
            if (length(filename )>=9)
                if (strcmp(filename(1:10),'PS2PL_send'))
                    filename_ps2pl_path = strcat(path_data_db,'/',filename);
                    fid_ps2pl = fopen(filename_ps2pl_path);
                    disp(filename);
                    if (    isempty(strfind(filename,'SIGN'))==0 ) 
                        len = func_decode_vector2file(fid_ps2pl,fid_config,'%d');
                        len_all = [len_all,len];
                        if (len ~= dem*4+1*16)
                            disp(sprintf('%f len is not equal 2924',filename_ps2pl_path));
                        end
                    else if (isempty(strfind(filename,'MatL_COL'))==0 || ...
                             isempty(strfind(filename,'MatL_ROW'))==0 )
                        len = func_decode_vector2file(fid_ps2pl,fid_config,'%d');
                        len_all = [len_all,len];
                        if (len ~= (dem+1)*4+1*16)
                            disp(sprintf('%f len is not equal 2928',filename_ps2pl_path));
                        end
                    else if(isempty(strfind(filename,'Vecb'))==0)
                        len = func_decode_vector2file(fid_ps2pl,fid_config,'%f');
                        len_all = [len_all,len];
                        if (len ~= dem*8+1*16)
                            disp(sprintf('%f len is not equal 5832',filename_ps2pl_path));
                        end
                    else if(isempty(strfind(filename,'VecPb'))==0 )
                        len = func_decode_vector2file(fid_ps2pl,fid_config,'%f');
                        len_all = [len_all,len];
                        if (len ~= dem*8*2+1*16)
                            disp(sprintf('%f len is not equal 11648',filename_ps2pl_path));
                        end
                    else if (isempty(strfind(filename,'MatA'))==0)
                            len = func_decode_matrix2file(fid_ps2pl,fid_config);
                            len_all = [len_all,len];
                    else
                            disp(sprintf('%f error ps2pl',filename_ps2pl_path));
                        end
                        end 
                        end
                        end
                    end    
                end
            end
        end
        fclose all
        dec2hex(sum(len_all))
        system(sys_cmd2);
    end
end