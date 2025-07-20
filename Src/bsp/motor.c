#include "motor.h"

#define PWM_ARR  99 //PWM频率10kHz
#define PWM_MAX 100
#define PWM_MIN -100

extern TIM_HandleTypeDef htim2;//tim2

/*motorA :left
 *motorB :right
 */
void Motor_Load(int motorA,int motorB)
{
	if(motorA < 0)
	{
		MA_IN1_1;
		MA_IN2_0;
		motorA = 0 - motorA;
	}
	else if(motorA > 0)
	{
		MA_IN1_0;
		MA_IN2_1;
	}
	else{
		MA_IN1_0;
		MA_IN2_0;
	}
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,motorA);//left
	
	if(motorB < 0)
	{
		MB_IN1_0;
		MB_IN2_1;
		motorB = 0 - motorB;
	}
	else if(motorB > 0)
	{
		MB_IN1_1;
		MB_IN2_0;
	}
	else{
		MB_IN1_0;
		MB_IN2_0;
	}
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,motorB);//right
}

/********************************************
函数功能：限幅
参数：
返回值：
********************************************/
void LIMIT(int *a,int *b)
{
	if(*a > PWM_MAX)	*a=PWM_MAX;
	if(*a < PWM_MIN)    *a=PWM_MIN;
	if(*b > PWM_MAX)	*b=PWM_MAX;
	if(*b < PWM_MIN)	*b=PWM_MIN;
}