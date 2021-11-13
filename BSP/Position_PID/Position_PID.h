#ifndef __POSITION_PID_H_
#define __POSITION_PID_H_

//#include "main.h"

typedef struct{
		float SetValue;
		float ActualValue;
		float err;
		float err_last;
		float Kp,Ki,Kd;
		float result;
		float voltage;
		float integral;
		float maxintegral;
}pid_t; 
typedef struct
{
		signed short 	ActualValue;
		float	 				a;
		signed short  Result_Last;
		signed short  Result;
}lowpass_t; 

extern float target;


void 	Position_PID_Init(float Kp,float Ki,float Kd,float maxintegral);
float Position_PID_Realize(float v,float v_r);
	
void Position_LWF_Init(void);
signed short Position_LWF_Realize(signed short Value,float a);

#endif
