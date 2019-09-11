clc
clear all
close all

A_nnz_len =      840;
G_nnz_len =      352;
n         =      255;
p         =      210;
m         =      262;
l         =      202;
ncones    =       15;
q         =   [ones(1,ncones)*4]';

path_din  = '../vstudio/run_ecos_v0/run_ecos_v0/data/din/';
path_db   = '../vstudio/run_ecos_v0/run_ecos_v0/data/db/';
path_dout = '../vstudio/run_ecos_v0/run_ecos_v0/data/dout/';

%% K æÿ’Û
[mat_K,mat_K_nnz] = matrix_decode_simple_func(strcat(path_db,'Mat_K.txt'));
imshow(abs(mat_K));title('Create KKT');
idx = sum(q);
W2 = mat_K(end-idx+1:end,end-idx+1:end);


path_db

[mat_PKPt,mat_PKPt_nnz] = matrix_decode_simple_func(strcat(path,'PKPt_earth.txt'));

idx = find(mat_ku_nnz(:,1)<mat_ku_nnz(:,2))
% »°æÿ’Û…œ»˝Ω«‘™Àÿ
triu(mat_ku,0)


p = vector_decode_func(strcat(path,'P.txt'),'%d');
pinv = vector_decode_func(strcat(path,'PINV.txt'),'%d');
permute_idx = vector_decode_func(strcat(path,'KKT_PK_s0.txt'),'%d');

%%÷√ªªæÿ’Û
mat_p = zeros(length(p),length(p));
for i = 1:length(p)
    mat_p(i,p(i)+1)=1;
end

%%÷√ªªæÿ’ÛµƒƒÊæÿ’Û
for j = 1:length(pinv)
    mat_pinv(p(j)+1,j)=1;
end

mat_PKPt_theory = (mat_p * mat_ku * mat_pinv)';

% figure
% subplot(121);imshow(mat_ku)
% subplot(122);imshow(mat_PKPt)
% 
% figure
% subplot(121);imshow(mat_p)
% subplot(122);imshow(mat_pinv)
% 
% figure
% subplot(131);imshow(mat_PKPt_theory)
% subplot(132);imshow(mat_PKPt)
% subplot(133);imshow(mat_PKPt_theory-mat_PKPt')
% 
% mat_PKPt_theory_flatten = reshape(mat_PKPt_theory,1,[]);
% mat_PKPt_flatten = reshape(mat_PKPt,1,[]);

% [mat_PKPt_theory_sort,idx_theory] = sort(mat_PKPt_theory_flatten);
% [mat_PKPt_sort,idx] = sort(mat_PKPt_flatten);
% 
% 
% plot(mat_PKPt_theory_sort-mat_PKPt_sort);

%%Õ≥º∆œ‘ æ
% data_statistics = tabulate(reshape(mat_ku,1,[]));
% 
% 
% [row_o,col_o]=find(abs(mat_ku-simple)<1e-8)
% pos_o = [row_o,col_o]
% [row,col]=find(abs(mat_ku-simple)<1e-8)
% pos = [row,col]
% 
% p(row_o)
% pinv(col_o)