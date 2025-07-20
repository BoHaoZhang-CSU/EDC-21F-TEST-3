/*mpu6050 control*/


#include "control.h"
#include "main.h"
#include "pid.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h"

extern float yaw;
PID_Para PID_mpu6050;

void PID_mpu6050_Init(void)
{
	PID_mpu6050.Kp=0;
	PID_mpu6050.Ki=0;
	PID_mpu6050.Kd=0;
	
	PID_mpu6050.err=0;
	PID_mpu6050.err_total=0;
	PID_mpu6050.err_last=0;
	PID_mpu6050.target=0;
}

float angle_diff(float angle1,float angle2)
{
	float temp;
	temp = angle1 - angle2;
	if(temp > 180)
		temp = temp - 360;
	if(temp < -180)
		temp = temp + 360;
	return temp;
}

float PID_angle_ctrl(void)
{
	float Expect_Pwm;
	float actual_val;
	
	//mpu_dmp_get_data(NULL,NULL,&yaw);
	actual_val = yaw;
	PID_mpu6050.err= angle_diff(PID_mpu6050.target,actual_val);
	
	PID_mpu6050.err_total+=PID_mpu6050.err;
	Expect_Pwm = PID_mpu6050.Kp * PID_mpu6050.err
			   + PID_mpu6050.Ki * PID_mpu6050.err_total
			   + PID_mpu6050.Kd * (PID_mpu6050.err - PID_mpu6050.err_last);
	PID_mpu6050.err_last = PID_mpu6050.err;
	
	return Expect_Pwm;
}