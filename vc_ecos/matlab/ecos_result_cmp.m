clc
clear all
close all

path_result  = '../data/dout/';

n         =      255;
p         =      210;
m         =      262;
double_byte_len = 8;
result_len = n+p+m+m+m+1+1;

fid_arm_result 	= fopen(strcat(path_result,'arm_result_normal_1.bin'),'rb');
fid_arm_x       = fopen(strcat(path_result,'arm_x_normal_1.bin'),'rb');
fid_arm_y       = fopen(strcat(path_result,'arm_y_normal_1.bin'),'rb');
fid_arm_z       = fopen(strcat(path_result,'arm_z_normal_1.bin'),'rb');
fid_arm_s       = fopen(strcat(path_result,'arm_s_normal_1.bin'),'rb');
fid_arm_lambda 	= fopen(strcat(path_result,'arm_lambda_normal_1.bin'),'rb');
fid_arm_tau 	= fopen(strcat(path_result,'arm_tau_normal_1.bin'),'rb');
fid_arm_kap 	= fopen(strcat(path_result,'arm_kap_normal_1.bin'),'rb');
fid_vc_x 		= fopen(strcat(path_result,'result_x.txt')			 ,'r');                  
fid_vc_y 		= fopen(strcat(path_result,'result_y.txt')			 ,'r');                  
fid_vc_z 		= fopen(strcat(path_result,'result_z.txt')			 ,'r');                  
fid_vc_s 		= fopen(strcat(path_result,'result_s.txt')			 ,'r');                  
fid_vc_lambda 	= fopen(strcat(path_result,'result_lambda.txt')		 ,'r');        
fid_vc_tau 		= fopen(strcat(path_result,'result_tau.txt')		 ,'r'); 
fid_vc_kap 		= fopen(strcat(path_result,'result_kap.txt')		 ,'r');


result_len*double_byte_len;
idx_tmp = [n,p,m,m,m,1,1];
idx = cumsum(idx_tmp);

arm_result = fread(fid_arm_result,result_len,'double');
arm_x 		= arm_result(1			:	idx(1));
arm_y 		= arm_result(idx(1)+1	:	idx(2));
arm_z 		= arm_result(idx(2)+1	:	idx(3));
arm_s 		= arm_result(idx(3)+1	:	idx(4));
arm_lambda 	= arm_result(idx(4)+1	:	idx(5));
arm_kap 	= arm_result(idx(5)+1	:	idx(6));
arm_tau 	= arm_result(idx(6)+1	:	idx(7));

arm_x = fread(fid_arm_x,m,'double','ieee-le.l64');
arm_y = fread(fid_arm_y,m,'double','ieee-le.l64');
arm_z = fread(fid_arm_z,m,'double','ieee-le.l64');
arm_s = fread(fid_arm_s,m,'double','ieee-le.l64');
arm_lambda = fread(fid_arm_lambda,m,'double','ieee-le.l64');
arm_kap = fread(fid_arm_kap,1,'double','ieee-le.l64');
arm_tau = fread(fid_arm_tau,1,'double','ieee-le.l64');


vc_x 		= fscanf(fid_vc_x		,'%e',inf);
vc_y 		= fscanf(fid_vc_y 		,'%e',inf);  
vc_z 		= fscanf(fid_vc_z 		,'%e',inf);  
vc_s 		= fscanf(fid_vc_s 		,'%e',inf);  
vc_lambda 	= fscanf(fid_vc_lambda 	,'%e',inf);  
vc_tau 		= fscanf(fid_vc_tau 	,'%e',inf);  
vc_kap 		= fscanf(fid_vc_kap 	,'%e',inf);   

% vc_result = [vc_x;vc_y;vc_z;vc_s;vc_lambda;vc_tau;vc_kap];
% plot(arm_result-vc_result)
disp(sprintf('err_x 		 : %e',max(arm_x 		    - vc_x 		)))
disp(sprintf('err_y 		 : %e',max(arm_y 		    - vc_y 		)))
disp(sprintf('err_z 		 : %e',max(arm_z 		    - vc_z 		)))
disp(sprintf('err_s 		 : %e',max(arm_s 		    - vc_s 		)))
disp(sprintf('err_lambda 	 : %e',max(arm_lambda 	    - vc_lambda )))
disp(sprintf('err_tau 		 : %e',max(arm_tau 			- vc_tau 	)))
disp(sprintf('err_kap 		 : %e',max(arm_kap 			- vc_kap 	)))			

figure
subplot(511);plot(arm_x 		    - vc_x 		)
subplot(512);plot(arm_y 		    - vc_y 		)
subplot(513);plot(arm_z 		    - vc_z 		)
subplot(514);plot(arm_s 		    - vc_s 		)
subplot(515);plot(arm_lambda 		- vc_lambda )




