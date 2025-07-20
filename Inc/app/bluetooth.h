#include "main.h"
#include "pid.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
typedef struct
{
    float ch; 
    float target0;
    float target1;
    float target2;   
    float kp0;      // 比例系数0
    float ki0;      // 积分系数0
    float kd0;      // 微分系数0    
    float kp1;      // 比例系数1
    float ki1;      // 积分系数1
    float kd1;      // 微分系数1
    float kp2;      // 比例系数2
    float ki2;      // 积分系数2    
    float kd2;      // 微分系数2
}PID_Parameters;

void parse_coding_data(char* coding_data,
                        PID_Parameters* parameters);


#if 0
void parse_coding_data(char* coding_data,
                       float* ch,
                       float* value_1,
                       float* value_2,
                       float* value_3,
                       int* target1,
						   
                       float* value_11,
                       float* value_21,
                       float* value_31,
                       int* target2);
#endif
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);				   