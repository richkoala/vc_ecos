#ifndef __GPIO_LED_H__
#define __GPIO_LED_H__

#include "xgpiops.h"

XGpioPs Gpio;
#define LED_ON   1
#define LED_OFF  0

void gpio_conf();

#endif
