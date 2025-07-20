/********************************************
 *外环1:循迹，用于走直线
 *外环2:mpu6050，用于转向
 *内环：左右两个轮子的速度环
********************************************/
#include "pid.h"
#define Turn_Tolerance 2    //误差2°以内算转向结束
extern PID_Para PID_mpu6050;
extern PID_Para PID_xunji;
PID_Para PID_L;
PID_Para PID_R;
PID_Para PID_LOCATION;
extern int g_speed_L,g_speed_R;//速度值

_Bool is_crossing = 0;//是路口
_Bool is_turning = 0;//正在转向
_Bool Left=0,Right=0;

extern float yaw;
float yaw_now;//开始转向时的角度

int target_speed_L = 20;
int target_speed_R = 20;
int target_speed = 0;
int turn_out = 0;
int motorL,motorR;

/********************************************
函数功能：第一问，近端病房
参数：
返回值：
********************************************/
void Control_test1(void)
{
	is_turning = 0;
	/*省略：识别到1号病房，Left置1*/
	Left = 1;
	/*省略：药品装上*/
	
	
	if(L3 && M && R3)//是路口
	{
		is_turning = 1;
		yaw_now = yaw;
		target_speed = 0;
	}
	
	if(angle_diff(yaw,yaw_now)<1.5f)//转向完成
	{
		is_turning = 0;
		target_speed = 20;
	}

	
	turn_out = (int) Turn();
	target_speed_L = target_speed + turn_out;
	target_speed_R = target_speed - turn_out;
	motorL = PID_A(g_speed_L,target_speed_L);
	motorR = PID_B(g_speed_R,target_speed_R);
	
	LIMIT(&motorL,&motorR);
	Motor_Load(motorL,motorR);
}

/********************************************
函数功能：走直线test
参数：
返回值：
********************************************/
void Control_test0(void)
{
	
	turn_out = (int) Turn();//循迹辅助走直线
	motorL = PID_A(g_speed_L, target_speed + turn_out);
	motorR = PID_B(g_speed_R, target_speed - turn_out);
	if(L3 && M && R3)//是路口
	{
		target_speed = 0;
	}
	LIMIT(&motorL,&motorR);
	Motor_Load(motorL,motorR);
}

void Car_go(int32_t location_cm)
{
	float Car_location_target;
	Car_location_target = location_cm /(3.142 *2 *WHEEL_R) * ENCODER_PPR;//编码器脉冲值
	pid_set_target(&PID_LOCATION,Car_location_target);
	
	
}
/********************************************
函数功能：位置环pid
参数：
返回值：目标速度
********************************************/
float location_pid(float actual,PID_Para *pid)  //位置环光个Kp好像也可以
{
	float speed;

    pid->err=pid->target-actual;
    pid->err_total += pid->err;

    speed = pid->Kp*pid->err
		    +pid->Ki*pid->err_total
		    +pid->Kd*(pid->err-pid->err_last);
  
    pid->err_last=pid->err;
    
	return speed;
}

/********************************************
函数功能：角度环pid
参数：
返回值：PWM占空比 差值
********************************************/
float Turn(void)
{
	float Turn_Out;
	if(is_turning)//mpu6050
	{
		if(Left)//左+
		{
			PID_mpu6050.target = yaw_now+90;
		}
		else if(Right)//右-
		{
			PID_mpu6050.target = yaw_now-90;
		}
		Turn_Out = PID_angle_ctrl();
	}
	else//xunji
	{
		Turn_Out = xunji();
	}
	return Turn_Out;
}

/********************************************
函数功能：左/A轮速度环PID
参数：  Encoder：当前值,满转速数量级296
        Target：目标值，0-100
返回值：   PWM :占空比，0-100
备注：k的数量级：1，只用到了PI
********************************************/
float PID_A(float Encoder,float Target)
{
	float Kp0=PID_L.Kp,Ki0=PID_L.Ki,Kd0=PID_L.Kd;
	static float Bias, Last_bias, Last2_bias, Pwm;
	Bias = Target - Encoder*100/296;              	                                                //计算偏差
	Pwm += Kp0 * (Bias - Last_bias) + Ki0 * Bias + Kd0 * (Bias - 2 * Last_bias + Last2_bias);   	//增量式PI控制器
	Last2_bias = Last_bias;
	Last_bias = Bias;	                   															//保存上一次偏差
	return Pwm;                        				                                        		//返回增量值
}

/********************************************
函数功能：右/B轮速度环PID
参数：  Encoder：当前值,满转速数量级274
        Target：目标值
返回值：   PWM :占空比，0-100
********************************************/

float PID_B(float Encoder,float Target)
{
	float Kp1=PID_R.Kp,Ki1=PID_R.Ki,Kd1=PID_R.Kd;
	static float Bias, Last_bias, Last2_bias, Pwm;
	Bias = Target - Encoder*100/274;               													//计算偏差
	Pwm += Kp1 * (Bias - Last_bias) + Ki1 * Bias + Kd1 * (Bias - 2 * Last_bias + Last2_bias);   	//增量式PI控制器
	Last2_bias = Last_bias;
	Last_bias = Bias;	                   															//保存上一次偏差
	return Pwm;
}
/********************************************
函数功能：最原始的pid函数，调参用
参数：
返回值：
********************************************/
int pid_control(float actual,PID_Para* pid)
{
	
	float target = pid->target;
	float err;
	float Last_err = pid->err_last;
	float Last2_err = pid->err_last2;   
                       
	err = target - actual;

	pid->output += pid->Kp * (err - Last_err) + pid->Ki * err + pid->Kd * (err - 2 * Last_err + Last2_err);
	pid->err_last2 = Last_err;
	pid->err_last = err;
	return (int)pid->output;
}

//增量式pid


void pid_set_target(PID_Para* pid, float target)
{
    pid->target = target;

}


