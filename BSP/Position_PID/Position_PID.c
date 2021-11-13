#include "Position_PID.h"
#include "stdio.h"

pid_t 		PID_Motor[2];
lowpass_t	LWF_Motor;
float ControlSpeed = 0;
float target=2400;
float Pwm;
float fspeed;

void Position_PID_Init(float Kp,float Ki,float Kd,float maxintegral){
		PID_Motor->SetValue= 0.0;
		PID_Motor->ActualValue= 0.0;
		PID_Motor->err= 0.0;
		PID_Motor->err_last=0.0;
		PID_Motor->integral= 0.0;
		PID_Motor->Kp= Kp;
		PID_Motor->Ki= Ki;
		PID_Motor->Kd= Kd;
		PID_Motor->maxintegral=maxintegral;
	}
	

float Position_PID_Realize( float Target, float Input){
			PID_Motor->SetValue = Target;
			PID_Motor->ActualValue = Input;
			PID_Motor->err = PID_Motor->SetValue - PID_Motor->ActualValue;
			PID_Motor->integral += PID_Motor->err;
			//if(PID_Motor->integral>=PID_Motor->maxintegral) PID_Motor->integral = PID_Motor->maxintegral;
			PID_Motor->result = PID_Motor->Kp * PID_Motor->err + PID_Motor->Ki * PID_Motor->integral + PID_Motor->Kd * ( PID_Motor->err - PID_Motor->err_last);
			PID_Motor->err_last = PID_Motor->err;
			return PID_Motor->result;
	}

void Position_LWF_Init(void){
		LWF_Motor.Result_Last = 0;
}


signed short Position_LWF_Realize(signed short Value,float a){
		LWF_Motor.Result= (1-a)*Value + a*LWF_Motor.Result_Last;
		LWF_Motor.Result_Last = LWF_Motor.Result;
		return LWF_Motor.Result;
	}

//float GetSpeed()
//{
//	float speed_t;
//	speed_t = (short)(__HAL_TIM_GET_COUNTER(&htim2)*10);
//	__HAL_TIM_SET_COUNTER(&htim2,0);
//	return speed_t;
//}

//void WritePwm(float speed)
//{
//	if(speed>=0)
//	{
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(int)speed);
//	}
//	else
//	{
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(int)speed);
//	}
//}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	static uint8_t i=0;
//	if(htim->Instance == TIM4)
//	{
//		fspeed=GetSpeed();
//		Pwm=Position_PID_Realize(target,fspeed);
//		if(Pwm>1000) Pwm=1000;
//		WritePwm(Pwm);
//	}
//	i++;
//	if(i>=100)
//	{
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//		i=0;
//	}
//	if(htim->Instance == TIM2)
//	{
//		
//	}
//}

