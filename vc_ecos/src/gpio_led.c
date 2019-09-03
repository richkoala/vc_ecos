#include "gpio_led.h"

void gpio_conf()
{
	int Status;
	XGpioPs_Config *ConfigPtr;

	//根据ID，查找配置
	//参数来源于xparameters.h
	ConfigPtr = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);
    if (ConfigPtr == NULL)
	{
		return XST_FAILURE;
	}

	//初始化
	//根据配置初始化gpio对象
	Status = XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	//根据ug1182，按钮位于MIO22，LED位于MIO23

	//设置按钮MIO22的gpio方向
	//22表示MIO22，0表示方向为输入
	//XGpioPs_SetDirectionPin(&Gpio, 22, 0);

	//设置LED MIO23的gpio方向
	//23表示MIO23，1表示方向为输出
	XGpioPs_SetDirectionPin(&Gpio, 23, 1);
	//使用MIO23的输出（默认为非使能）
	XGpioPs_SetOutputEnablePin(&Gpio, 23, 1);
}
