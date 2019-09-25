clc
clear all
close all
fclose all

mod_all = ['s','p'];
dem_all = [757,727];

for mod_cnt=1:2
    for dem_cnt = 1:2
     mode = mod_all(mod_cnt);
     dem  = dem_all(dem_cnt);   
        
    if (mode=='s' && dem == 727)
        ps2pl_num = 82;
    else if (mode=='p' && dem == 727)
        ps2pl_num = 61;
    else if (mode=='s' && dem == 757)
        ps2pl_num = 91;
    else if (mode=='p' && dem == 757)
        ps2pl_num = 67;
        end
        end
        end 
    end

    din_path = strcat('../data_',mode,num2str(dem),'/din/');
    dout_path =strcat('../data_',mode,num2str(dem),'/dout/');

    fid_config_dat = fopen(strcat(din_path,'config_',mode,num2str(dem),'.dat'),'rb');
    fid_result_dat = fopen(strcat(dout_path,'ps_result_',mode,num2str(dem),'.txt'),'r');
    solve_cnt = 0;


    MatA_tmp = zeros(dem,dem);
    Vect_b   = zeros(dem,1);
    Vect_b2  = zeros(dem,1);
    Vect_x   = zeros(dem,1);
    Vect_x2  = zeros(dem,1);

    CMDT_LDL_SIGN		  	=hex2dec('A8');
    CMDT_INFO_MatL_COLNUM 	=hex2dec('C0');
    CMDT_INFO_MatL_T_COLNUM =hex2dec('C1');
    CMDT_LDL_MatA_INIT    	=hex2dec('A0');
    CMDT_LDL_MatA_T_INIT  	=hex2dec('A1');
    CMDT_LDL_MatA_ITER    	=hex2dec('A2');
    CMDT_LDL_MatA_T_ITER  	=hex2dec('A3');
    CMDTR_LDL_MatA_INIT    	=hex2dec('A0');
    CMDTR_LDL_MatA_T_INIT  	=hex2dec('A1');
    CMDTR_LDL_MatA_ITER    	=hex2dec('A2');
    CMDTR_LDL_MatA_T_ITER  	=hex2dec('A3');
    CMDT_CAL_Vecb_INIT1   	=hex2dec('B0');
    CMDT_CAL_Vecb_INIT2   	=hex2dec('B1');
    CMDT_CAL_Vecb_INIT12   	=hex2dec('B2');
    CMDTR_CAL_Vecb_INIT1   	=hex2dec('B0');
    CMDTR_CAL_Vecb_INIT2   	=hex2dec('B1');
    CMDT_CAL_Vecb_ITER1   	=hex2dec('B8');
    CMDT_CAL_Vecb_ITER2   	=hex2dec('B9');
    CMDTR_CAL_Vecb_ITER1   	=hex2dec('B8');
    CMDTR_CAL_Vecb_ITER2   	=hex2dec('B9');
    CMDT_CAL_Vecb_ITER12  	=hex2dec('BA');
    CMDT_CAL_Vecb_ITER3   	=hex2dec('BB');
    CMDTR_CAL_Vecb_ITER3   	=hex2dec('BB');
    CMDT_MatLD_INIT 		=hex2dec('C2');
    CMDT_MatLD_T_INIT 		=hex2dec('C3');
    CMDT_MatLD_ITER 		=hex2dec('C4');
    CMDT_MatLD_T_ITER 		=hex2dec('C5');


    err = []

    for ps2pl_cnt = 1:ps2pl_num
        ps2pl_cmd = fread(fid_config_dat,4,'int32');
        data_cmd = ps2pl_cmd(1);disp(dec2hex(data_cmd));
        frame_len = ps2pl_cmd(2);

        switch ps2pl_cmd(1)
            case CMDT_LDL_SIGN
                data_len = (frame_len-16)/4;
                sign_dat = fread(fid_config_dat,data_len,'int32');
            case {CMDT_INFO_MatL_COLNUM,CMDT_INFO_MatL_T_COLNUM}
                data_len = (frame_len-16)/4;
                cunsum_info = fread(fid_config_dat,data_len,'int32');
            case {CMDT_LDL_MatA_INIT,CMDT_LDL_MatA_T_INIT  	,CMDT_LDL_MatA_ITER    	,CMDT_LDL_MatA_T_ITER  	,CMDTR_LDL_MatA_INIT    	,CMDTR_LDL_MatA_T_INIT  	,CMDTR_LDL_MatA_ITER    	,CMDTR_LDL_MatA_T_ITER  	}
                eps = fread(fid_config_dat,1,'double');
                delta = fread(fid_config_dat,1,'double');
                data_len = (frame_len-16-16)/16;
                for i=1:data_len
                    row_idx = fread(fid_config_dat,1,'int32');
                    col_idx = fread(fid_config_dat,1,'int32');
                    val     = fread(fid_config_dat,1,'double');
                    MatA_tmp(row_idx,col_idx)=val;
                end

                MatA = MatA_tmp + MatA_tmp'-diag(diag(MatA_tmp));
                [L,D] = func_ldl(MatA);
                for j = 1:length(sign_dat)
                    if (D(j,j)*sign_dat(j)<=eps)
                        D(j,j) = sign_dat(j)*delta;
                    end
                end

            case {CMDT_CAL_Vecb_INIT1 ,CMDT_CAL_Vecb_INIT2 ,CMDTR_CAL_Vecb_INIT1,CMDTR_CAL_Vecb_INIT2,CMDT_CAL_Vecb_ITER1 ,CMDT_CAL_Vecb_ITER2 ,CMDTR_CAL_Vecb_ITER1,CMDTR_CAL_Vecb_ITER2,CMDT_CAL_Vecb_ITER3 ,CMDTR_CAL_Vecb_ITER3}
                data_len = (frame_len-16)/8;
                Vect_b   = fread(fid_config_dat,data_len,'double');
                Vec_x    = fscanf(fid_result_dat,'%g',data_len);
                err = [err;Vect_b-L*D*L'*Vec_x];
                solve_cnt = solve_cnt+1;
            case {CMDT_CAL_Vecb_INIT12,CMDT_CAL_Vecb_ITER12}  
                data_len = (frame_len-16)/8/2;
                for i=1:data_len
                    Vect_b(i)  = fread(fid_config_dat,1,'double');
                    Vect_b2(i) = fread(fid_config_dat,1,'double');
                end
                Vec_x   = fscanf(fid_result_dat,'%g',data_len);
                Vec_x2  = fscanf(fid_result_dat,'%g',data_len);
                err = [err;Vect_b-L*D*L'*Vec_x];
                err = [err;Vect_b2-L*D*L'*Vec_x2];
                solve_cnt = solve_cnt+1;
            otherwise
                disp('ps2pl_cmd decode error');
        end

    end

    disp(strcat('solve cnt is ','/ ',num2str(solve_cnt)));
    substr = strcat('41',num2str((dem_cnt-1)*2+mod_cnt))
    subplot(substr);plot(err)
    end
end

