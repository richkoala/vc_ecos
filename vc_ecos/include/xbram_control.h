#ifndef __XBRAM_CONTROL_H__
#define __XBRAM_CONTROL__

#include "xparameters.h"
#include "xbram.h"
#include "xstatus.h"
#include <stdio.h>

extern  XBram Bram;

#define BRAM_DEVICE_ID		XPAR_BRAM_0_DEVICE_ID


int XBram_CfgInitialize(XBram *InstancePtr,
			XBram_Config *Config,
			UINTPTR EffectiveAddr);

void InitializeECC(XBram_Config *ConfigPtr, u32 EffectiveAddr);

int BramControl_init(u16 DeviceId);

#endif
