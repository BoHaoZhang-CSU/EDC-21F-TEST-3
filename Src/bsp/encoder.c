#include "encoder.h"

//tim3£¬tim4±àÂëÆ÷Ä£Ê½
//tim3-->right
//tim4-->left
int Read_Speed(TIM_HandleTypeDef *htim)
{
	int temp;
	temp = (short) __HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
	return temp;
}
