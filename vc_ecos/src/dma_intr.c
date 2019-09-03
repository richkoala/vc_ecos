#include "dma_intr.h"

volatile int Dma_tx_done;
volatile int Dma_rx_done;
volatile int Dma_error;

/*****************************************************************************/
//DMA��Ԫ���ó�ʼ��
int DMA_Init(XAxiDma *DMAPtr,u32 DeviceId)
{
	int Status;
	XAxiDma_Config *Config=NULL;

	Config = XAxiDma_LookupConfig(DeviceId);
	if (!Config) {
		xil_printf("No config found for %d\r\n", DeviceId);
		return XST_FAILURE;
	}
	//��ʼ������
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
//DMA TX �жϴ�����.
//��ӦӲ����������ж���Ϣ����DMA��Ԫ���ֹ��������¸�λDMA����ɵ�DMA���ʹ���DDR->PL���tx_done���ź�
//����Callback: is a pointer to TX channel of the DMA engine.
static void DMA_TxIntrHandler(void *Callback)
{
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	// ��ȡDMA�ж�״̬�Ĵ���
	IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DMA_TO_DEVICE);
	// Ӧ��DMA�ж���Ӧ����Ϊʲô��Ҫ����״̬�ֻ�д������
	XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DMA_TO_DEVICE);
	//��û�����������������ʱ��DMA���ʹ�����ϡ�DMA���ͳ�ʱ��DMA���ʹ���ֱ�ӷ��ز������κδ�����Ӧ
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}
	//���жϳ����쳣�����ǣ���עDma_error�źţ���λDMA��Ԫ������ʱ����DMA��λ�����������˳���ǰ�жϴ������
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {
		Dma_error = 1;
		XAxiDma_Reset(AxiDmaInst);
		TimeOut = DMA_RESET_TIME_NUM;			//��������Ҫȷ��DMAģ�鸴λ��Ҫ��ʱ�䣿����
		while (TimeOut) {
			if (XAxiDma_ResetIsDone(AxiDmaInst)) {
				break;
			}
			TimeOut -= 1;
		}
		return;
	}
	// �������ж�ΪDMA��������ж�ʱ����עDma_tx_done�ź�
	if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) {
		Dma_tx_done = 1;
	}
	//������ȱ��һ��DMA���ͳ�ʱ�жϣ�����
}

/*****************************************************************************/
//DMA RX �жϴ�����.
//��ӦӲ����������ж���Ϣ����DMA��Ԫ���ֹ��������¸�λDMA����ɵ�DMA���ʹ���PL-DDR���rx_done���ź�
//����Callback: is a pointer to RX channel of the DMA engine. ������callback��̫������ⰴ�ջص����ǣ�
static void DMA_RxIntrHandler(void *Callback)
{
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	// ��ȡDMA�ж�״̬�Ĵ���
	IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);
	// Ӧ��DMA�ж���Ӧ����Ϊʲô��Ҫ����״̬�ֻ�д������
	XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);
	//��û�����������������ʱ��DMA���ʹ�����ϡ�DMA���ͳ�ʱ��DMA���ʹ���ֱ�ӷ��ز������κδ�����Ӧ
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}
	//���жϳ����쳣�����ǣ���עDma_error�źţ���λDMA��Ԫ������ʱ����DMA��λ�����������˳���ǰ�жϴ������
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

	// �������ж�ΪDMA��������ж�ʱ����עDma_rx_done�ź�
	if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) {

		Dma_rx_done = 1;
	}
}

/*****************************************************************************/
//�������ܣ�����DMA�жϣ��ٶ�ϵͳ�д���SCUGICģ��
//1) �ж�ϵͳ����ԪINTCָ��.
//2) DMA��Ԫָ��
//3) TXͨ���ж�ID
//4) RXͨ���ж�ID
int DMA_Interrupt_Setup(XScuGic * IntcInstancePtr,XAxiDma * AxiDmaPtr, u16 TxIntrId, u16 RxIntrId)
{
	int DMA_TX_Status;
	int DMA_RX_Status;

	//�����ж����ȼ����жϴ�����ʽ
	//3th 0 is highest priority, 0xF8 (248) is lowest
	//4th b01 Active HIGH level sensitive b11 Rising edge sensitive
	XScuGic_SetPriorityTriggerType(IntcInstancePtr, TxIntrId, 0xA0, 0x3);
	XScuGic_SetPriorityTriggerType(IntcInstancePtr, RxIntrId, 0xA0, 0x3);

	//�����жϷ��������ڵ�ַ
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

	//GIC����
	XScuGic_Enable(IntcInstancePtr, TxIntrId);
	XScuGic_Enable(IntcInstancePtr, RxIntrId);
	return XST_SUCCESS;
}

/*****************************************************************************/
//�������ܣ�DMA�ж�ʹ�ܺ����ȹغ�
int DMA_Interrupt_Enable(XScuGic * IntcInstancePtr,XAxiDma *DMAPtr)
{
	//�ر�DMA�ж�
	XAxiDma_IntrDisable(DMAPtr, XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrDisable(DMAPtr, XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DEVICE_TO_DMA);
	//ʹ��DMA�ж�
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

