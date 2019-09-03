#include "dma_intr.h"

volatile int Dma_tx_done;
volatile int Dma_rx_done;
volatile int Dma_error;

/*****************************************************************************/
//DMA单元配置初始化
int DMA_Init(XAxiDma *DMAPtr,u32 DeviceId)
{
	int Status;
	XAxiDma_Config *Config=NULL;

	Config = XAxiDma_LookupConfig(DeviceId);
	if (!Config) {
		xil_printf("No config found for %d\r\n", DeviceId);
		return XST_FAILURE;
	}
	//初始化配置
	Status = XAxiDma_CfgInitialize(DMAPtr, Config);

	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}

	if(XAxiDma_HasSg(DMAPtr)){
		xil_printf("Device configured as SG mode \r\n");
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*****************************************************************************/
//DMA TX 中断处理函数.
//响应硬件发送完毕中断信息，当DMA单元出现故障是重新复位DMA，完成的DMA发送传输DDR->PL输出tx_done高信号
//参数Callback: is a pointer to TX channel of the DMA engine.
static void DMA_TxIntrHandler(void *Callback)
{
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	// 获取DMA中断状态寄存器
	IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DMA_TO_DEVICE);
	// 应答DMA中断响应？？为什么需要进行状态字回写操作？
	XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DMA_TO_DEVICE);
	//当没有以下三种情况出现时（DMA发送传输完毕、DMA发送超时、DMA发送错误）直接返回不做出任何处理响应
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}
	//当中断出现异常错误是，标注Dma_error信号，复位DMA单元，当超时或者DMA复位操作结束后，退出当前中断处理程序
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {
		Dma_error = 1;
		XAxiDma_Reset(AxiDmaInst);
		TimeOut = DMA_RESET_TIME_NUM;			//？？？需要确定DMA模块复位需要的时间？？？
		while (TimeOut) {
			if (XAxiDma_ResetIsDone(AxiDmaInst)) {
				break;
			}
			TimeOut -= 1;
		}
		return;
	}
	// 当传输中断为DMA传输结束中断时，标注Dma_tx_done信号
	if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) {
		Dma_tx_done = 1;
	}
	//？？？缺少一种DMA发送超时中断？？？
}

/*****************************************************************************/
//DMA RX 中断处理函数.
//响应硬件接收完毕中断信息，当DMA单元出现故障是重新复位DMA，完成的DMA发送传输PL-DDR输出rx_done高信号
//参数Callback: is a pointer to RX channel of the DMA engine. ？？？callback不太容易理解按照回调考虑？
static void DMA_RxIntrHandler(void *Callback)
{
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	// 获取DMA中断状态寄存器
	IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);
	// 应答DMA中断响应？？为什么需要进行状态字回写操作？
	XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);
	//当没有以下三种情况出现时（DMA发送传输完毕、DMA发送超时、DMA发送错误）直接返回不做出任何处理响应
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}
	//当中断出现异常错误是，标注Dma_error信号，复位DMA单元，当超时或者DMA复位操作结束后，退出当前中断处理程序
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {
		Dma_error = 1;
		XAxiDma_Reset(AxiDmaInst);
		TimeOut = DMA_RESET_TIME_NUM;
		while (TimeOut) {
			if(XAxiDma_ResetIsDone(AxiDmaInst)) {
				break;
			}
			TimeOut -= 1;
		}

		return;
	}

	// 当传输中断为DMA传输结束中断时，标注Dma_rx_done信号
	if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) {

		Dma_rx_done = 1;
	}
}

/*****************************************************************************/
//函数功能：设置DMA中断，假定系统中存在SCUGIC模块
//1) 中断系统管理单元INTC指针.
//2) DMA单元指针
//3) TX通道中断ID
//4) RX通道中断ID
int DMA_Interrupt_Setup(XScuGic * IntcInstancePtr,XAxiDma * AxiDmaPtr, u16 TxIntrId, u16 RxIntrId)
{
	int DMA_TX_Status;
	int DMA_RX_Status;

	//设置中断优先级及中断触发方式
	//3th 0 is highest priority, 0xF8 (248) is lowest
	//4th b01 Active HIGH level sensitive b11 Rising edge sensitive
	XScuGic_SetPriorityTriggerType(IntcInstancePtr, TxIntrId, 0xA0, 0x3);
	XScuGic_SetPriorityTriggerType(IntcInstancePtr, RxIntrId, 0xA0, 0x3);

	//设置中断服务程序入口地址
	DMA_TX_Status = XScuGic_Connect(	IntcInstancePtr,
										TxIntrId,
										(Xil_InterruptHandler)DMA_TxIntrHandler,
										AxiDmaPtr
									);
	DMA_RX_Status = XScuGic_Connect(	IntcInstancePtr,
										RxIntrId,
										(Xil_InterruptHandler)DMA_RxIntrHandler,
										AxiDmaPtr
									);
	if (DMA_TX_Status != XST_SUCCESS)
		return DMA_TX_Status;
	else if (DMA_RX_Status != XST_SUCCESS)
		return DMA_RX_Status;

	//GIC允许
	XScuGic_Enable(IntcInstancePtr, TxIntrId);
	XScuGic_Enable(IntcInstancePtr, RxIntrId);
	return XST_SUCCESS;
}

/*****************************************************************************/
//函数功能：DMA中断使能函数先关后开
int DMA_Interrupt_Enable(XScuGic * IntcInstancePtr,XAxiDma *DMAPtr)
{
	//关闭DMA中断
	XAxiDma_IntrDisable(DMAPtr, XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrDisable(DMAPtr, XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DEVICE_TO_DMA);
	//使能DMA中断
	XAxiDma_IntrEnable(DMAPtr, XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrEnable(DMAPtr, XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DEVICE_TO_DMA);
	return XST_SUCCESS;
}


/*****************************************************************************/
/*
*
* This function checks data buffer after the DMA transfer is finished.
*
* We use the static tx/rx buffers.
*
* @param	Length is the length to check
* @param	StartValue is the starting value of the first byte
*
* @return
*		- XST_SUCCESS if validation is successful
*		- XST_FAILURE if validation is failure.
*
* @note		None.
*
******************************************************************************/
 int DMA_CheckData(int Length, u8 StartValue)
{
	u8 *RxPacket;
	int Index = 0;
	u8 Value;

	RxPacket = (u8 *) RX_BUFFER_BASE;
	Value = StartValue;

	/* Invalidate the DestBuffer before receiving the data, in case the
	 * Data Cache is enabled
	 */
#ifndef __aarch64__
	Xil_DCacheInvalidateRange((u32)RxPacket, Length);
#endif

	for(Index = 0; Index < Length; Index++) {
		if (RxPacket[Index] != Value) {
			xil_printf("Data error %d: %x/%x\r\n",
			    Index, RxPacket[Index], Value);

			return XST_FAILURE;
		}
		Value = (Value + 1) & 0xFF;
	}

	return XST_SUCCESS;
}
/*****************************************************************************/
/**
*
* This function disables the interrupts for DMA engine.
*
* @param	IntcInstancePtr is the pointer to the INTC component instance
* @param	TxIntrId is interrupt ID associated w/ DMA TX channel
* @param	RxIntrId is interrupt ID associated w/ DMA RX channel
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
 void DMA_DisableIntrSystem(XScuGic * IntcInstancePtr,
					u16 TxIntrId, u16 RxIntrId)
{
#ifdef XPAR_INTC_0_DEVICE_ID
	/* Disconnect the interrupts for the DMA TX and RX channels */
	XIntc_Disconnect(IntcInstancePtr, TxIntrId);
	XIntc_Disconnect(IntcInstancePtr, RxIntrId);
#else
	XScuGic_Disconnect(IntcInstancePtr, TxIntrId);
	XScuGic_Disconnect(IntcInstancePtr, RxIntrId);
#endif
}

