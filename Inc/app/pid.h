#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "xunji.h"
#include "control.h"
#include "motor.h"
#include "encoder.h"

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	
	float err;        // 当前误差
	float err_total;
	float err_last;
	float err_last2;
	float target;
	float output;
}PID_Para;

extern PID_Para PID_L;
extern PID_Para PID_R;
extern PID_Para PID_LOCATION;

void Control_test0(void);
float Turn(void);
float PID_A(float Encoder,float Target);
float PID_B(float Encoder,float Target);
int pid_control(float actual,PID_Para* pid);
void pid_set_target(PID_Para* pid, float target);



#endif
