/*
 *
 * www.osrc.cn
 * www.milinker.com
 * copyright by nan jin mi lian dian zi www.osrc.cn
*/
#ifndef DMA_INTR_H
#define DMA_INTR_H
#include "xaxidma.h"
#include "xparameters.h"
#include "xil_exception.h"
#include "xdebug.h"
#include "xscugic.h"

/************************** Constant Definitions *****************************/
/*
 * Device hardware build related constants.
 */
#define DMA_DEV_ID			XPAR_AXIDMA_0_DEVICE_ID
#define MEM_BASE_ADDR		0x01000000
#define DMA_RX_INTR_ID		XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR
#define DMA_TX_INTR_ID		XPAR_FABRIC_AXI_DMA_0_MM2S_INTROUT_INTR
#define TX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00100000)
#define RX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00300000)
#define RX_BUFFER_HIGH		(MEM_BASE_ADDR + 0x004FFFFF)


#define DMA_RESET_TIME_NUM	10000		//DMA传输超时时间（单位）
/* test start value
 */
#define TEST_START_VALUE	0x0
/*
 * Buffer and Buffer Descriptor related constant definition
 */
#define MAX_PKT_LEN			32176	//DMA包内数据大小
#define NUMBER_OF_TRANSFERS	3		//传输测试次数

extern volatile int 		Dma_tx_done;
extern volatile int 		Dma_rx_done;
extern volatile int 		Dma_error;

int  DMA_CheckData(int Length, u8 StartValue);
int  DMA_Interrupt_Setup(XScuGic * IntcInstancePtr,XAxiDma * AxiDmaPtr, u16 TxIntrId, u16 RxIntrId);
int  DMA_Interrupt_Enable(XScuGic * IntcInstancePtr,XAxiDma *DMAPtr);
int  DMA_Intr_Init(XAxiDma *DMAPtr,u32 DeviceId);

#endif
