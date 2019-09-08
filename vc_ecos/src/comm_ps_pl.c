#include "glblopts.h"
#include "comm_ps_pl.h"

#ifdef ZCU102_HW_IMP
XScuGic system_interrupt_control; //GIC
XAxiDma AxiDma;
volatile u32 success;
#endif

//malloc堆地址空间 为 外部DDR存储器  ========注意下========

int DMA_COMM_TEST()
{
#ifdef ZCU102_HW_IMP
	int comm_loop_cnt;
	int comm_loop_err_flag;
	int comm_loop_err_cnt=0;
	u32 Dma_rx_status;
	u32 Dma_tx_status;
	int Dma_rx_len=8192;
	int Dma_tx_len=8192;
	float *Dma_tx_buffer;
	float *Dma_rx_buffer;
	int	index;
	int success=0;

	Dma_tx_buffer = (float *)MALLOC((Dma_tx_len)*sizeof(float));
	Dma_rx_buffer = (float *)MALLOC((Dma_rx_len)*sizeof(float));

	for(index = 0; index < Dma_rx_len; index ++) {
		Dma_tx_buffer[index] = index;
	}
	Xil_DCacheFlushRange((u32)Dma_tx_buffer, Dma_rx_len);		//需要考虑数据格式位宽增加需要进行修改-sizeof(u8)

	for (comm_loop_cnt = 0; comm_loop_cnt<COMM_LOOP_NUM;comm_loop_cnt++)
	{

		//PS.DDR->PL 并查看DMA启动状态
		Xil_DCacheInvalidateRange((u32)Dma_rx_buffer, (Dma_rx_len/64)*64);
		Dma_rx_status = XAxiDma_SimpleTransfer(&AxiDma, (u32) Dma_rx_buffer	,	Dma_rx_len	, XAXIDMA_DEVICE_TO_DMA);

		//PS.DDR->PL 并查看DMA启动状态
		Dma_tx_status = XAxiDma_SimpleTransfer(&AxiDma,(u32) Dma_tx_buffer,Dma_tx_len, XAXIDMA_DMA_TO_DEVICE);

		if (Dma_rx_status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		if (Dma_tx_status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		while (!Dma_rx_done ||!Dma_tx_done) {
		}
		Dma_tx_done = 0;
		Dma_rx_done = 0;

		//data_check
		comm_loop_err_flag = 0;
		for (index=0;index<Dma_rx_len/16;index++)
		{
			if (Dma_rx_buffer[index] != Dma_tx_buffer[index])
				comm_loop_err_flag = 1;
		}

			if (comm_loop_err_flag==1)
				comm_loop_err_cnt++;

		memset(Dma_rx_buffer, 0, Dma_rx_len);
		Xil_DCacheFlushRange((u32)Dma_rx_buffer, Dma_rx_len);
		success++;
	}
	return comm_loop_err_cnt;
#else
	return 0;
#endif
}

int kkt_factor_fpga(
		ps2pl_sop Sop,	
		double dat_eps,
		double dat_delta,
		demat_struct* DeM_A,
		int* Vec_Sign,
		int* Sign,
		int  Sign_len,
		demat_struct* Dma_LD_buffer,
		int LD_nz
		)
{

	int 		 debug_info;
//====================帧协议=======================//
	//MatA帧头参数
	DeM_A[0].frame_id_or_row  = Sop.frame_id;
	DeM_A[0].frame_len_or_col = Sop.frame_len;
	DeM_A[0].cnt			  = Sop.cnt;
	DeM_A[0].iter_num		  = Sop.iter_num;
	//LDL分解所需参数
	DeM_A[1].double_data1 = dat_eps;
	DeM_A[1].double_data2 = dat_delta;

	//Sign帧头参数
	Vec_Sign[0] = CMDT_LDL_SIGN;
	Vec_Sign[1] = (Sign_len+4)*4;
	Vec_Sign[2] = Sop.cnt;
	Vec_Sign[3]	= Sop.iter_num;
	memcpy(&Vec_Sign[4],Sign,sizeof(int)*Sign_len);

#ifndef ZCU102_HW_IMP 
	return 2;
#elif KKT_FACTOR_PL_PROCESS == 0
	return 3;
#else
	u32 Dma_rx_status;
	u32 Dma_tx_status;
	int Dma_rx_len;
	int Dma_tx_len;
	int i;

	if (Sop.frame_id == CMDTR_LDL_MatA_INIT   ||
		Sop.frame_id == CMDTR_LDL_MatA_T_INIT ||
		Sop.frame_id == CMDTR_LDL_MatA_ITER   ||
		Sop.frame_id == CMDTR_LDL_MatA_T_ITER 
		) {
		Dma_rx_len = (LD_nz*16 + 16);			//调试回读LD矩阵信息
		Xil_DCacheInvalidateRange((u32)Dma_LD_buffer, (Dma_rx_len/64+1)*64);
		Dma_rx_status = XAxiDma_SimpleTransfer(&AxiDma, (u32) Dma_LD_buffer	,	Dma_rx_len	, XAXIDMA_DEVICE_TO_DMA);
		if (Dma_rx_status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	}
	//==========================================//
	//==========KKT send KKT_sign info==========//
	//==========================================//

	Dma_tx_len = Vec_Sign[1];	//int32 4B
	Xil_DCacheFlushRange((u32)Vec_Sign, Dma_tx_len);
	Dma_tx_status = XAxiDma_SimpleTransfer(&AxiDma, (u32)Vec_Sign, Dma_tx_len , XAXIDMA_DMA_TO_DEVICE);

	if (Dma_tx_status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	while (!Dma_tx_done)
	{
	}
	Dma_tx_done = 0;

	//==========================================//
	//==========KKT send KKT_sign info==========//
	//==========================================//

	Dma_tx_len = Sop.frame_len;	//128 - 16B
	Xil_DCacheFlushRange((u32)DeM_A, Dma_tx_len);
	Dma_tx_status = XAxiDma_SimpleTransfer(&AxiDma, (u32)DeM_A, Dma_tx_len, XAXIDMA_DMA_TO_DEVICE);

	if (Dma_tx_status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	if (debug_info){
		if (Dma_rx_status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	}

	if (Sop.frame_id == CMDTR_LDL_MatA_INIT   ||
		Sop.frame_id == CMDTR_LDL_MatA_T_INIT ||
		Sop.frame_id == CMDTR_LDL_MatA_ITER   ||
		Sop.frame_id == CMDTR_LDL_MatA_T_ITER 
		){
		while (!Dma_rx_done ||!Dma_tx_done) {
		}

	}
	else{
		while (!Dma_tx_done)
		{

		}
	}

	Dma_rx_done = 0;
	Dma_tx_done = 0;
	return XST_SUCCESS;

#endif
}

int kkt_solve_fpga(
		double* Pb,
		devec_struct* b,
		int b_len,
		ps2pl_sop Sop,
		devec_struct* x)

{
	//帧头参数
	b[0].frame_id_or_cnt 		= Sop.frame_id;
	b[0].frame_len_or_iter_num  = Sop.frame_len;
	b[1].frame_id_or_cnt 		= Sop.cnt;
	b[1].frame_len_or_iter_num  = Sop.iter_num;
	memcpy(&b[2],Pb,sizeof(devec_struct)*b_len);

#ifndef ZCU102_HW_IMP
	return 2;		
#elif KKT_SOLVE_PL_PROCESS == 0
	return 3;
#else
	u32 Dma_rx_status;
	u32 Dma_tx_status;
	int Dma_rx_len;
	int Dma_tx_len;
	int i;

	//读取PL逻辑处理结果
	//1）debug模式下读取前代fx和回代bx处理结果
	if (Sop.frame_id == CMDTR_CAL_Vecb_INIT1   ||
		Sop.frame_id == CMDTR_CAL_Vecb_INIT2   ||
		Sop.frame_id == CMDTR_CAL_Vecb_ITER1   ||
		Sop.frame_id == CMDTR_CAL_Vecb_ITER2   ||
		Sop.frame_id == CMDTR_CAL_Vecb_ITER3
			)
		Dma_rx_len = 2*b_len*8+16;
	//2）正常模式下读取回代x处理结果
	else
		Dma_rx_len = b_len*8+16;

	Xil_DCacheInvalidateRange((u32)x, (Dma_rx_len/64+1)*64);
	Dma_rx_status = XAxiDma_SimpleTransfer(&AxiDma,(u32) x,Dma_rx_len, XAXIDMA_DEVICE_TO_DMA);
	if (Dma_rx_status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	//将sop+b参数由DDR发送至PL逻辑

	Dma_tx_len = Sop.frame_len;
	Xil_DCacheFlushRange((u32)b, b[0].frame_len_or_iter_num);
	Dma_tx_status = XAxiDma_SimpleTransfer(&AxiDma, (u32) b, Dma_tx_len , XAXIDMA_DMA_TO_DEVICE);

	if (Dma_tx_status != XST_SUCCESS)
		return XST_FAILURE;

	if (Dma_rx_status != XST_SUCCESS)
		return XST_FAILURE;

	while (!Dma_rx_done ||!Dma_tx_done) {
			/* NOP */
	}

	Dma_rx_done = 0;
	Dma_tx_done = 0;

	return XST_SUCCESS;

#endif

}

int kkt_solve_fpga_p(
		double* Pb1,
		double* Pb2,
		devec_struct* Pb,
		int b_len,
		ps2pl_sop Sop,
		devec_struct* Px)

{
	//帧头参数
	int i;
	Pb[0].frame_id_or_cnt 		= Sop.frame_id;
	Pb[0].frame_len_or_iter_num = Sop.frame_len;
	Pb[1].frame_id_or_cnt 		= Sop.cnt;
	Pb[1].frame_len_or_iter_num = Sop.iter_num;
	for (i=0;i<(b_len/2);i++){
		Pb[2*i+2].double_data1 = Pb2[i];		//高位地址为Pb2
		Pb[2*i+3].double_data1 = Pb1[i];		//高位地址为Pb1
	}
	
#ifndef ZCU102_HW_IMP
	return 2;		
#elif KKT_SOLVE_PL_PROCESS == 0
	return 3;
#else
	u32 Dma_rx_status;
	u32 Dma_tx_status;
	int Dma_rx_len;
	int Dma_tx_len;
	int i;

	//读取PL逻辑处理结果
	//2）正常模式下读取2次x处理结果
	Dma_rx_len = Sop.frame_len;

	Xil_DCacheInvalidateRange((u32)x, (Dma_rx_len/64+1)*64);
	Dma_rx_status = XAxiDma_SimpleTransfer(&AxiDma,(u32) Px,Dma_rx_len, XAXIDMA_DEVICE_TO_DMA);
	if (Dma_rx_status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	//将sop+b参数由DDR发送至PL逻辑

	Dma_tx_len = Sop.frame_len;
	Xil_DCacheFlushRange((u32)b, Sop.frame_len);
	Dma_tx_status = XAxiDma_SimpleTransfer(&AxiDma, (u32) Pb, Dma_tx_len , XAXIDMA_DMA_TO_DEVICE);

	if (Dma_tx_status != XST_SUCCESS)
		return XST_FAILURE;

	if (Dma_rx_status != XST_SUCCESS)
		return XST_FAILURE;

	while (!Dma_rx_done ||!Dma_tx_done) {
			/* NOP */
	}

	Dma_rx_done = 0;
	Dma_tx_done = 0;

	return XST_SUCCESS;

#endif

}



