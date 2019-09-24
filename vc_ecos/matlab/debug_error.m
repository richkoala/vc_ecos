clc
clear all
close all

dem = 757;

A = func_decode_matrix_l('../data/db/fpga/MatA_T_iter08.txt');
l = func_decode_matrix_l('../data/db/fpga/MatL_iter09.txt');
d = func_decode_vector_d('../data/db/fpga/MatD_iter09.txt');
b = func_decode_vector_b('../data/db/fpga/solve_b1_iter09.txt');

fx = func_decode_vector_b('../data/db/fpga/solve_fx1_iter09.txt');
dx = func_decode_vector_b('../data/db/fpga/solve_dx1_iter09.txt');
bx = func_decode_vector_b('../data/db/fpga/solve_bx1_iter09.txt');

l=l+(diag(ones(1,dem)));
plot(l*diag(d)*l' * bx -b );


fn_A1 ='../data/db/fpga/PS2PL_send0065_MatA_FRAME_ITER08.txt';
fn_A2 ='../data/db/fpga/PS2PL_send0074_MatA_FRAME_ITER09.txt';
fn_A3 ='../data/db/fpga/PS2PL_send0083_MatA_FRAME_ITER10.txt';

fn_hw_b1  = '../data/db/fpga/PS2PL_send0073_Vecb3_FRAME_ITER08_refine00.txt';
fn_hw_b2  = '../data/db/fpga/PS2PL_send0078_Vecb1_FRAME_ITER09_refine02.txt';
fn_hw_b3  = '../data/db/fpga/PS2PL_send0087_Vecb1_FRAME_ITER10_refine02.txt';

fn_x1    = '../data/db/fpga/PS_SOLVE_BX0073_Vecb3_FRAME_ITER08_refine00.txt';
fn_x2    = '../data/db/fpga/PS_SOLVE_BX0078_Vecb1_FRAME_ITER09_refine02.txt';
fn_x3    = '../data/db/fpga/PS_SOLVE_BX0087_Vecb1_FRAME_ITER10_refine02.txt';

ps2pl_A1 = func_decode_ps2pl_matrix(fn_A1);
ps2pl_A2 = func_decode_ps2pl_matrix(fn_A2);
ps2pl_A3 = func_decode_ps2pl_matrix(fn_A3);

ps2pl_b1 = func_decode_ps2pl_vector(fn_hw_b1,'%d','%f');
ps2pl_b2 = func_decode_ps2pl_vector(fn_hw_b2,'%d','%f');
ps2pl_b3 = func_decode_ps2pl_vector(fn_hw_b3,'%d','%f');

x1 = func_decode_ps_solve_vector(fn_x1,'%f');
x2 = func_decode_ps_solve_vector(fn_x2,'%f');
x3 = func_decode_ps_solve_vector(fn_x3,'%f');

subplot(311);plot((ps2pl_A1+ps2pl_A1'-diag(diag(ps2pl_A1)))*x1-ps2pl_b1)
subplot(312);plot((ps2pl_A2+ps2pl_A2'-diag(diag(ps2pl_A2)))*x2-ps2pl_b2)
subplot(313);plot((ps2pl_A3+ps2pl_A3'-diag(diag(ps2pl_A3)))*x3-ps2pl_b3)

max(max(A - ps2pl_A2))
max(max(b - ps2pl_b2))
max(max(bx - x2))

