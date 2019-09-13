%% 测试后续迭代处理过程中 k 参数 与 KKT矩阵非零参数位置 是否一致
clc
clear all
close all

dem = 727;
iter_num = 11;
path_sign_data  = '../data/db/fpga/';
path_kkt_data   = '../data/db/fpga/';

%% sign参数数值
    sign_data_all = zeros(dem,iter_num);
    sign_cmp_flag = zeros(dem,iter_num-1);
    for i = 1:iter_num
        if (i==1)
            sign_data_idx = strcat(path_sign_data,sprintf('HW_SIGN_FRAME_INIT%s.txt',num2str(i-1,'%02d')));
        else
            sign_data_idx = strcat(path_sign_data,sprintf('HW_SIGN_FRAME_ITER%s.txt',num2str(i-2,'%02d')));
        end
        sign_struct = func_decode_vector_1(sign_data_idx,'%d','%f');   %sop-type data-type
        sign_data_all(:,i) = sign_struct.vector;
    end

    for i = 1:iter_num-1
        sign_cmp_flag(:,i)=sign_data_all(:,end)-sign_data_all(:,i);
    end

    if max(max(abs(sign_cmp_flag))) == 0
        disp('every sign data is equal ');
    else
        disp('every sign data is not equal ');
    end

%% kkt 矩阵非零位置参数数值
    kkt_row_col_all      = zeros(dem*2,iter_num);
    kkt_row_col_cmp_flag = zeros(dem*2,iter_num-1);
    for i = 1:iter_num
        if (i==1)
            sign_data_idx = strcat(path_kkt_data,sprintf('HW_MatA_FRAME_INIT%s.txt',num2str(i-1,'%02d')));
        else
            sign_data_idx = strcat(path_kkt_data,sprintf('HW_MatA_FRAME_ITER%s.txt',num2str(i-2,'%02d')));
        end
        kkt_struct = func_decode_row_col(sign_data_idx,1);   %sop-type data-type
        kkt_row_col_all(:,i) = kkt_struct.vector;
    end

    for i = 1:iter_num-1
        kkt_row_col_cmp_flag(:,i)=kkt_row_col_all(:,end)-kkt_row_col_all(:,i);
    end

    if max(max(abs(kkt_row_col_cmp_flag))) == 0
        disp('every kkt row_col_idx is equal ');
    else
        disp('every sign row_col_idx is not equal ');
    end

















