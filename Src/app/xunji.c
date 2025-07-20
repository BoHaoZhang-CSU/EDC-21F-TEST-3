#include "xunji.h"

PID_Para PID_xunji;

void PID_xunji_Init(void)
{
	PID_xunji.Kp=0;
	PID_xunji.Ki=0;
	PID_xunji.Kd=0;
	
	PID_xunji.err=0;
	PID_xunji.err_total=0;
	PID_xunji.err_last=0;
}

float xunji(void)
{
	static float output;
	//pid
    if (L3 && !L2 && !L1 && !M && !R1 && !R2 && !R3) {
        PID_xunji.err = -3;
    } else if (L3 && L2 && !L1 && !M && !R1 && !R2 && !R3) {
        PID_xunji.err = -2.5;
    } else if (!L3 && L2 && !L1 && !M && !R1 && !R2 && !R3) {
        PID_xunji.err = -2;
    } else if (!L3 && L2 && L1 && !M && !R1 && !R2 && !R3) {
        PID_xunji.err = -1.5;
    } else if (!L3 && !L2 && L1 && !M && !R1 && !R2 && !R3) {
        PID_xunji.err = -1;
    } else if (!L3 && !L2 && L1 && M && !R1 && !R2 && !R3) {
        PID_xunji.err = -0.5;
    } else if (!L3 && !L2 && !L1 && M && !R1 && !R2 && !R3) {
        PID_xunji.err = 0;
    } else if (!L3 && !L2 && !L1 && M && R1 && !R2 && !R3) {
        PID_xunji.err = 0.5;
    } else if (!L3 && !L2 && !L1 && !M && R1 && !R2 && !R3) {
        PID_xunji.err = 1;
    } else if (!L3 && !L2 && !L1 && !M && R1 && R2 && !R3) {
        PID_xunji.err = 1.5;
    } else if (!L3 && !L2 && !L1 && !M && !R1 && R2 && !R3) {
        PID_xunji.err = 2;
    } else if (!L3 && !L2 && !L1 && !M && !R1 && R2 && R3) {
        PID_xunji.err = 2.5;
    } else if (!L3 && !L2 && !L1 && !M && !R1 && !R2 && R3) {
        PID_xunji.err = 3;
    } else {
        PID_xunji.err = 0; // 没检测到，保守处理
    }
	
    PID_xunji.err_total += PID_xunji.err;

	output = PID_xunji.Kp * PID_xunji.err
                     + PID_xunji.Ki * PID_xunji.err_total
                     + PID_xunji.Kd * (PID_xunji.err - PID_xunji.err_last);

    PID_xunji.err_last = PID_xunji.err;

    return output;
}
