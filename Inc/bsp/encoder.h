#include "main.h"

#define RESOLUTION 13
#define RESOLUTION_TOTAL 13*4
#define REDUCTION_RATIO 20
#define ENCODER_PPR 13*4*20
#define WHEEL_R 2.4 //cm
extern long g_total_encoder_cnt;
int Read_Speed(TIM_HandleTypeDef *htim);
