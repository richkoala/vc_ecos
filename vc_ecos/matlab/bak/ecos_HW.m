clc
clear all
close all


path_din  = '../vstudio/run_ecos_v0/run_ecos_v0/data/din/';
path_dout = '../vstudio/run_ecos_v0/run_ecos_v0/data/dout/';
path_db   = '../vstudio/run_ecos_v0/run_ecos_v0/data/db/fpga/';

%%������ȡ�ļ�����
files          = dir(path_db);
file_start_idx = 3;
kkt_factor_num = 0;
kkt_solve_num  = 0;

%% ͼ����ʾʹ�ܱ���
CALC_LDL_ERR_SHOWEN   = 'OF';        %'ON'/'OF'
CALC_SOLVE_ERR_SHOWEN = 'ON';        %'ON'/'OF'

%%factor �������ͳ��
for i = file_start_idx :length(files)
    file = files(i).name;
    if (strcmp(file(1:9),'MatA_init'))
        kkt_factor_num = kkt_factor_num+1;
    else if (strcmp(file(1:9),'MatA_iter'))
        kkt_factor_num = kkt_factor_num+1;
        end
    end
%     disp(file)
end

%%solve �������ͳ�� �����ÿ�μ���
kkt_solve_refine = zeros(1*2 + (kkt_factor_num-1)*3,1);
kkt_solve_refine_idx =0;
for i = file_start_idx :length(files)
    file = files(i).name;
    if (strcmp(file(1:12),'solve_b_init'))
        kkt_solve_num = kkt_solve_num+1;
    else if (strcmp(file(1:12),'solve_b_iter'))
        kkt_solve_num = kkt_solve_num+1;
        end
    end
    
    %%kkt_solve ���x��ʼ���ǣ����̰���refine��
    if ( (strcmp(file(1:12),'solve_b_init')) || (strcmp(file(1:12),'solve_b_iter')) )
        if (strcmp(file(end-10:end),'_refine.txt'))
        else
            kkt_solve_refine_idx = kkt_solve_refine_idx+1;
            kkt_solve_refine(kkt_solve_refine_idx) = kkt_solve_num;
        end
    end
end


solve_all_cnt = 0;
initial_flag = 1;
for kkt_factor_cnt = 1:kkt_factor_num
    
    if (kkt_factor_cnt == 1)
        file_name_MatA = strcat('MatA_init',num2str(kkt_factor_cnt-1,'%02d'),'.txt');
        file_name_MatL = strcat('MatL_init',num2str(kkt_factor_cnt-1,'%02d'),'.txt');
        file_name_VecD = strcat('MatD_init',num2str(kkt_factor_cnt-1,'%02d'),'.txt');
    else
        file_name_MatA = strcat('MatA_iter',num2str(kkt_factor_cnt-2,'%02d'),'.txt');
        file_name_MatL = strcat('MatL_iter',num2str(kkt_factor_cnt-2,'%02d'),'.txt');
        file_name_VecD = strcat('MatD_iter',num2str(kkt_factor_cnt-2,'%02d'),'.txt');
    end
    
%     MatA = A������������Ϣ
    [MatA_L,MatA_nnz] = func_decode_matrix_0(strcat(path_db,file_name_MatA),1);
%     MatL_L ɾ���Խ���Ԫ�ص�L������Ϣ
    [MatL_L,MatL_nnz] = func_decode_matrix_0(strcat(path_db,file_name_MatL),0);
    VecD = func_decode_vector(strcat(path_db,file_name_VecD),'%f');

% %     MatA �� A������Ϣ
    MatA = MatA_L + MatA_L'-diag(diag(MatA_L));
    dem = size(MatA);
    dem_h = dem(1);
% %     MatL �� L������Ϣ
    MatL = MatL_L+diag(ones(1,dem_h));

%     %KKT_factor ���鿴
    if (CALC_LDL_ERR_SHOWEN == 'ON')
        figure
        mesh(MatA - MatL*diag(VecD)*MatL');
        str  = sprintf('kkt_factor cnt is %d',kkt_factor_cnt);
        title(str)
    end
    
    if (initial_flag ==1)
        solve_b_num = 2;
        initial_flag = 0;
    else
        solve_b_num = 3;
    end
    
    %% ����n��b���� ��ʼ��Ϊb0��b1,��������Ϊb0,b1,b2
    for solve_b_cnt = 1:solve_b_num  
        solve_all_cnt = solve_all_cnt + 1;
        solve_idx_start = kkt_solve_refine(solve_all_cnt);
        if (solve_all_cnt+1)> length(kkt_solve_refine)
            solve_idx_end   = kkt_solve_num;
        else
            solve_idx_end   = kkt_solve_refine(solve_all_cnt+1)-1;
        end

        subplot_num = solve_idx_end - solve_idx_start + 1;
        subplot_cnt = 0;
        
        if (CALC_SOLVE_ERR_SHOWEN == 'ON')
            figure;
        end
        
        %%���ÿ�μ���b�Ĵ��������������д���refinement,����subplot������1�������������̣�����Ϊrefine����
        for solve_idx = solve_idx_start:solve_idx_end

            kkt_solve_b_idx  = dir([path_db sprintf('solve_b_*cnt%02d*' ,solve_idx)]);
            kkt_solve_fx_idx = dir([path_db sprintf('solve_fx_*cnt%02d*',solve_idx)]);
            kkt_solve_dx_idx = dir([path_db sprintf('solve_fx_*cnt%02d*',solve_idx)]);
            kkt_solve_bx_idx = dir([path_db sprintf('solve_bx_*cnt%02d*',solve_idx)]);    
            b  = func_decode_vector(strcat(kkt_solve_b_idx.folder,'\',kkt_solve_b_idx.name),'%f');
            fx = func_decode_vector(strcat(kkt_solve_fx_idx.folder,'\',kkt_solve_fx_idx.name),'%f');
            dx = func_decode_vector(strcat(kkt_solve_dx_idx.folder,'\',kkt_solve_dx_idx.name),'%f');
            bx = func_decode_vector(strcat(kkt_solve_bx_idx.folder,'\',kkt_solve_bx_idx.name),'%f');

            vc_err = MatL*diag(VecD)*MatL'*bx - b;
            
            if (CALC_SOLVE_ERR_SHOWEN == 'ON')
                subplot_cnt = subplot_cnt +1;
                subplot_idx = sprintf('%d1%d',subplot_num,subplot_cnt);
                subplot(subplot_idx);plot(vc_err);title(sprintf('AX=b%01d solve\\_cnt%02d',solve_b_cnt,solve_idx))
            end
            
        end
    end
end

















