#include "main.h"
#include "pid.h"

#define L3 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)
#define L2 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)
#define L1 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)
#define M  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)
#define R1 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)
#define R2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)
#define R3 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)
#define GREY ((0<<7)|(L3<<6)|(L2<<5)|(L1<<4)|(M<<3)|(R1<<2)|(R2<<1)|R3)

void PID_xunji_Init(void);
float xunji(void);
