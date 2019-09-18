#ifndef __COMM_PS_PL_H__
#define __COMM_PS_PL_H__

#include "glblopts.h"
#include "spla.h"

#ifdef ZCU102_HW_IMP
	#include "dma_intr.h"
	#define COMM_LOOP_TEST
	#define COMM_LOOP_NUM 1000
	extern  XScuGic system_interrupt_control; //GIC
	extern  XAxiDma AxiDma;
	extern volatile u32 success;
#endif

	extern idxint kkt_flag;

typedef struct ps2pl_sop{
   int frame_id;
   int iter_num;
   int cnt;
   int frame_len;
} ps2pl_sop;

typedef struct pl2ps_sop{
   int frame_id;
   int iter_num;
   int cnt;
   int frame_len;
} pl2ps_sop;

int DMA_COMM_TEST();

int kkt_sign_fpga(
		int* Vec_Sign,
		int* Sign,
		int  Sign_len,
		int* PS2PL_trans_cnt,
		int* PL2PS_trans_cnt
		);

int kkt_col_cumsum_fpga(
		int* Vec_Col_cumsum,
		int* Col_cumsum,
		int  Col_cumsum_len,
		int* PS2PL_trans_cnt,
		int* PL2PS_trans_cnt
		);

int kkt_row_cumsum_fpga(
		int* Vec_Row_cumsum,
		int* Row_cumsum,
		int  Row_cumsum_len,
		int* PS2PL_trans_cnt,
		int* PL2PS_trans_cnt
		);

int kkt_factor_fpga(
		ps2pl_sop Sop,	
		double dat_eps,
		double dat_delta,
		demat_struct* DeM_A,
		int* Vec_Sign,
		int* Sign,
		int  Sign_len,
		demat_struct* Dma_LD_buffer,
		int LD_nz,
		int* PS2PL_trans_cnt,
		int* PL2PS_trans_cnt
		);

int kkt_solve_fpga(
		double* Pb,
		devec_struct* b,
		int b_len,
		ps2pl_sop Sop,
		devec_struct* x,
		int* PS2PL_trans_cnt,
		int* PL2PS_trans_cnt
		);

int kkt_solve_fpga_p(
		double* Pb1,
		double* Pb2,
		devec_struct* Pb,		//Ö¡¸ñÊ½sop1+b1/b2
		int b_len,
		ps2pl_sop Sop,
		devec_struct* Px,
		int* PS2PL_trans_cnt,
		int* PL2PS_trans_cnt
		);

//protocol

#define CMD_FACTOR_SOF_LEN		2		//start of the frame kkt factor ps -> pl
#define MAT_LD_SOF_LEN			2		//start of the fram  mat_LD     pl -> ps

#define CMD_SOLVE_SOF_LEN		1		//start of the frame kkt factor ps -> pl
#define VEC_BX_SOF_LEN			1		//start of the fram  mat_LD     pl -> ps

#define CMDT_LDL_MatA_INIT    	0x00A0
#define CMDT_LDL_MatA_T_INIT  	0x00A1
#define CMDT_LDL_MatA_ITER    	0x00A2
#define CMDT_LDL_MatA_T_ITER  	0x00A3
#define CMDTR_LDL_MatA_INIT    	0x01A0
#define CMDTR_LDL_MatA_T_INIT  	0x01A1
#define CMDTR_LDL_MatA_ITER    	0x01A2
#define CMDTR_LDL_MatA_T_ITER  	0x01A3
#define CMDT_LDL_SIGN		  	0x00A8

#define CMDT_CAL_Vecb_INIT1   	0x00B0
#define CMDT_CAL_Vecb_INIT2   	0x00B1
#define CMDT_CAL_Vecb_INIT12   	0x00B2
#define CMDTR_CAL_Vecb_INIT1   	0x01B0
#define CMDTR_CAL_Vecb_INIT2   	0x01B1
#define CMDT_CAL_Vecb_ITER1   	0x00B8
#define CMDT_CAL_Vecb_ITER2   	0x00B9
#define CMDTR_CAL_Vecb_ITER1   	0x01B8
#define CMDTR_CAL_Vecb_ITER2   	0x01B9
#define CMDT_CAL_Vecb_ITER12  	0x00BA
#define CMDT_CAL_Vecb_ITER3   	0x00BB
#define CMDTR_CAL_Vecb_ITER3   	0x01BB
 
#define CMDT_INFO_MatL_COLNUM 	0x00C0
#define CMDT_INFO_MatL_T_COLNUM 0x00C1
#define CMDT_MatLD_INIT 		0x00C2
#define CMDT_MatLD_T_INIT 		0x00C3
#define CMDT_MatLD_ITER 		0x00C4
#define CMDT_MatLD_T_ITER 		0x00C5

#define CMDR_DB_MatLD		    0x00D0
#define CMDR_DB_FBX  			0x00D1

#define CMDR_DOUTX_INIT1    	0x00F0
#define CMDR_DOUTX_INIT2	  	0x00F1
#define CMDR_DOUTX_INIT12	  	0x00F2
#define CMDR_DOUTX_ITER1    	0x00F8
#define CMDR_DOUTX_ITER2	  	0x00F9
#define CMDR_DOUTX_ITER12	  	0x00FA
#define CMDR_DOUTX_ITER3	  	0x00FB


#endif
