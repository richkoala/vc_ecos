clc
clear all
close all

row_len = 727;
col_len = 727;

path = '../vstudio/run_ecos_v0/run_ecos_v0/data/db/';


%%��ʼKU����
%       0 A' G'
%  KU =[A 0  0]
%       G 0  W
[mat_KU,] = func_decode_matrix_simple(strcat(path,'Mat_KU_setup.txt'));
[mat_PKPt_setup,] = func_decode_matrix_simple(strcat(path,'PKPt_setup.txt'));
[mat_PKPt_solve_init,] = func_decode_matrix_simple(strcat(path,'PKPt_solve_init.txt'));

%% permute process
vector_P = func_decode_vector(strcat(path,'P.txt'),'%d');
mat_P = zeros(row_len,col_len);
for i = 1:length(vector_P)
    mat_P(i,vector_P(i)+1)=1;
end
mat_PKPt_tmp = mat_P*mat_KU*mat_P';
mat_PKPt_theory = triu(mat_PKPt_tmp + mat_PKPt_tmp' - diag(diag(mat_PKPt_tmp)));


figure
subplot(221);imshow(abs(mat_KU));title('��ʼKU����');
subplot(222);imshow(abs(mat_PKPt_setup));title('ECOS\_setup���̽������PkPt����');
subplot(223);imshow(abs(mat_PKPt_solve_init));title('ECOS\_solve����init�����ڵ�PkPt����');
subplot(224);imshow(abs(mat_PKPt_theory));title('ECOS\_solve����init');
% subplot(224);imshow(abs(mat_PKP_setup-mat_PKPt_solve_init));title('ECOS\_solve����init');