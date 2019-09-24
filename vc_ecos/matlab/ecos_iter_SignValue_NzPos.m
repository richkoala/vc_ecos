%% 测试后续迭代处理过程中 k 参数 与 KKT矩阵非零参数位置 是否一致
clc
clear all
close all

dem = 757;
iter_num = 11;
sign_num = iter_num+2;
matL_num = iter_num+2;
path_sign_data  = '../data/db/fpga/';
path_kkt_data   = '../data/db/fpga/';
path_col_data   = '../data/db/fpga/';

%% sign参数数值
    sign_data_all = zeros(dem,sign_num);
    sign_cmp_flag = zeros(dem,sign_num-1);
        
    for i = 1:sign_num
        if (i==1)
            sign_data_idx = strcat(path_sign_data,'HW_SIGN_PS2PL.txt');
        else if (i==2)
            sign_data_idx = strcat(path_sign_data,sprintf('HW_SIGN_FRAME_INIT%s.txt',num2str(i-2,'%02d')));
        else
            sign_data_idx = strcat(path_sign_data,sprintf('HW_SIGN_FRAME_ITER%s.txt',num2str(i-3,'%02d')));
            end
        end

            sign_struct = func_decode_ps2pl_vector_sign(sign_data_idx);   %sop-type data-type
            sign_data_all(:,i) = sign_struct.vector;
   
    end

    for i = 1:sign_num-1
        sign_cmp_flag(:,i)=sign_data_all(:,1)-sign_data_all(:,i+1);
    end

    if max(max(abs(sign_cmp_flag))) == 0
        disp('sign data do not changed ');
    else
        disp('sign data change ');
    end

%% kkt 矩阵非零位置参数数值
    kkt_row_col_all      = zeros(dem*2,iter_num);
    kkt_row_col_cmp_flag = zeros(dem*2,iter_num-1);
    for i = 1:iter_num
        if (i==1)
            row_col_idx = strcat(path_kkt_data,sprintf('HW_MatA_FRAME_INIT%s.txt',num2str(i-1,'%02d')));
        else
            row_col_idx = strcat(path_kkt_data,sprintf('HW_MatA_FRAME_ITER%s.txt',num2str(i-2,'%02d')));
        end
        kkt_struct = func_decode_row_col(row_col_idx,1);   %sop-type data-type
        kkt_row_col_all(:,i) = kkt_struct.vector;
    end

    for i = 1:iter_num-1
        kkt_row_col_cmp_flag(:,i)=kkt_row_col_all(:,end)-kkt_row_col_all(:,i);
    end

    if max(max(abs(kkt_row_col_cmp_flag))) == 0
        disp('kkt row_col_idx do not changed ');
    else
        disp('sign row_col_idx  changed ');
    end
    

 %% MAT_L 矩阵非零位置参数数值
    matL_col_all      = zeros(dem+1,matL_num);
    matL_col_cmp_flag = zeros(dem+1,matL_num-1);
    for i = 1:matL_num
        if (i==1)
            matl_col_idx = strcat(path_col_data,'MatL_COL_PS2PL.txt');
        else if  (i==2)
            matl_col_idx = strcat(path_col_data,sprintf('MatL_col_cumsum_init%s.txt',num2str(i-2,'%02d')));
        else
            matl_col_idx = strcat(path_col_data,sprintf('MatL_col_cumsum_iter%s.txt',num2str(i-3,'%02d')));
            end
        end
        
        if (i==1)
            col_struct = func_decode_ps2pl_vector_colcumsum(matl_col_idx);   %sop-type data-type
            matL_col_all(:,i) = col_struct.vector;
        else
            col_vector = func_decode_vector_colcumsum(matl_col_idx);   %sop-type data-type
            matL_col_all(:,i)  = col_vector;
        end
    end

    for i = 1:matL_num-1
        matL_col_cmp_flag(:,i)=matL_col_all(:,1)-matL_col_all(:,i+1);
    end

    if max(max(abs(matL_col_cmp_flag))) == 0
        disp('matL col cumsum do not changed ');
    else
        disp('matL col cumsum changed ');
    end

   fclose all;
















