#include "gpio_led.h"

void gpio_conf()
{
	int Status;
	XGpioPs_Config *ConfigPtr;

	//����ID����������
	//������Դ��xparameters.h
	ConfigPtr = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);
    if (ConfigPtr == NULL)
	{
		return XST_FAILURE;
	}

	//��ʼ��
	//�������ó�ʼ��gpio����
	Status = XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	//����ug1182����ťλ��MIO22��LEDλ��MIO23

	//���ð�ťMIO22��gpio����
	//22��ʾMIO22��0��ʾ����Ϊ����
	//XGpioPs_SetDirectionPin(&Gpio, 22, 0);

	//����LED MIO23��gpio����
	//23��ʾMIO23��1��ʾ����Ϊ���
	XGpioPs_SetDirectionPin(&Gpio, 23, 1);
	//ʹ��MIO23�������Ĭ��Ϊ��ʹ�ܣ�
	XGpioPs_SetOutputEnablePin(&Gpio, 23, 1);
}
