clc
clear all
close all

iter_num  =      10;

path = '../data/db/Con_exit/'

[A,] = func_decode_matrix(strcat(path,'origin_A.txt'));
[G,] = func_decode_matrix(strcat(path,'origin_G.txt'));
[c,] = func_decode_vector(strcat(path,'origin_c.txt'),'%f');
[b,] = func_decode_vector(strcat(path,'origin_b.txt'),'%f');
[h,] = func_decode_vector(strcat(path,'origin_h.txt'),'%f');

rx =max(1,norm(c,2));
ry =max(1,norm(b,2));
rz =max(1,norm(h,2));
for iter_cnt = 1:iter_num

    x   = func_decode_vector(strcat(path,sprintf('x_iter%02d.txt'  ,iter_cnt)),'%f');
    y   = func_decode_vector(strcat(path,sprintf('y_iter%02d.txt'  ,iter_cnt)),'%f');
    z   = func_decode_vector(strcat(path,sprintf('z_iter%02d.txt'  ,iter_cnt)),'%f');
    s   = func_decode_vector(strcat(path,sprintf('s_iter%02d.txt'  ,iter_cnt)),'%f');
    tau = func_decode_vector(strcat(path,sprintf('tau_iter%02d.txt',iter_cnt)),'%f');
    kap = func_decode_vector(strcat(path,sprintf('kap_iter%02d.txt',iter_cnt)),'%f');

    con1 = norm((A'*y+G'*z+tau.*c),2)/rx;
    con2 = norm((A*x-tau.*b),2)/ry;
    con3 = norm((G*x+s-tau.*h),2)/rz;
    
    ro  = max(-c'*x,-b'*y-h'*z);
    con4 = s'*z;
    con5 = s'*z/ro;
    
    disp(sprintf('iter_num: %5d\t tau: %e\t ro: %e\t, kap:%e\t con1: %e\t con2: %e\t con3: %e\t con4: %e\t con5: %e\t',...
                    iter_cnt,      tau,      ro,       kap,     con1,con2,con3,con4,con5 )...
                  );
end