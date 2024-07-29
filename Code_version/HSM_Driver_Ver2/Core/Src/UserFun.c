#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

extern UART uart_var;
extern CONTROL_OBJECT control_var;
extern ADC_OBJECT adc_var;
extern FLAG flag_var;

/*__________________________________________________________________________________________________________________________*/
	/*Delay function with counter timer of htim5 or htim3*/
	void delay_us(uint16_t us){			// Delay micro seconds
			__HAL_TIM_SetCounter(&htim5,0);
			while(__HAL_TIM_GetCounter(&htim5) < us);
		}
	void delay_ms(uint16_t ms){			// Delay milliseconds
		for(uint16_t i = 0; i<ms; i++){
			delay_us(1000);
		}
	}
	/*Blink Leds********************************************/
		void nhayled(uint16_t ms){
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			delay_ms(ms);
		}
/*__________________________________________________________________________________________________________________________*/
	/*Initiate the flag variables*/
	void FLAG_Init(FLAG *pFL){
		pFL->flag_25us = 0;
		pFL->flag_25us_50us = 0;
		pFL->flag_25us_20ms = 0;
	}
/*__________________________________________________________________________________________________________________________*/
	/*Initiate the low pass filter parameters*/ 
	void LPF_Init(LPF *pLPF, uint16_t Fcut, float Ts){
		float temp1, temp2;
			
		pLPF->Fc = Fcut;
		pLPF->Ts = Ts;
		pLPF->alpha = 1.0/(2*3.14*pLPF->Ts*pLPF->Fc);
		
		temp1 = (1+pLPF->alpha); 
		temp2 = temp1*temp1;
		pLPF->a1 = (1+2*pLPF->alpha)/temp2; 
		pLPF->a2 = -2.0*pLPF->alpha/temp2;
		pLPF->b1 = -2.0*pLPF->alpha/temp1; 
		pLPF->b2 = pLPF->alpha*pLPF->alpha/temp2;
		
		pLPF->yk = 0;
		pLPF->yk_1 = 0; 
		pLPF->yk_2 = 0;
		pLPF->uk = 0;
		pLPF->uk_1 = 0;
	}
/*__________________________________________________________________________________________________________________________*/
	/*Execute the low pass filter */ 
	float LPF_Run(LPF *pLPF, float input){
		//Read LPF input
		pLPF->uk = input;	
		//Compute LPF output
		pLPF->yk = -pLPF->b1*pLPF->yk_1 - pLPF->b2*pLPF->yk_2 + pLPF->a1*pLPF->uk + pLPF->a2*pLPF->uk_1; 
		//Save LPF past data
		pLPF->yk_2 = pLPF->yk_1; 
		pLPF->yk_1 = pLPF->yk; 
		pLPF->uk_1 = pLPF->uk;
		//Return LPF output
		return pLPF->yk;
	}
/*__________________________________________________________________________________________________________________________*/
void RAMP_Init(RAMP *pRamp, uint16_t Tacc_Sec, uint16_t Tdec_Sec, float inputMin, float inputMax, float Ts)
{
	pRamp->inputMax = inputMax;
	pRamp->inputMin = inputMin;
	pRamp->T_Acc = Tacc_Sec;
	pRamp->T_Dec = Tdec_Sec;
	
	//Compute the set count - 10ms resolution
	pRamp->set_AccCnt = pRamp->T_Acc/Ts;	
	pRamp->set_DecCnt = pRamp->T_Dec/Ts;
	//Compute the step size
	pRamp->delta_Acc = (float)(pRamp->inputMax - pRamp->inputMin)/pRamp->set_AccCnt;
	pRamp->delta_Dec = (float)(pRamp->inputMax - pRamp->inputMin)/pRamp->set_DecCnt;
	
	pRamp->SPk = 0;
	pRamp->Output = 0;
}
/*__________________________________________________________________________________________________________________________*/
	float RAMP_RUN(float SP, RAMP *pRamp)
{
	pRamp->SPk = SP;
	//Detect the state
	if(pRamp->SPk > pRamp->Output + pRamp->delta_Acc)
	{
		pRamp->State = ACC;
	}
	else if (pRamp->SPk < pRamp->Output - pRamp->delta_Dec)
	{
		pRamp->State = DEC;
	}
	else
	{
		pRamp->State = RUN;
	}
	//Generate output
	if(pRamp->State == ACC)
	{
		pRamp->Output = pRamp->Output + pRamp->delta_Acc;
		if(pRamp->Output >= pRamp->SPk)	//End the acceleration
		{
			pRamp->Output = pRamp->SPk;
			pRamp->State = RUN; 					
		}
	}
	else if(pRamp->State == DEC)
	{
		pRamp->Output = pRamp->Output - pRamp->delta_Dec;
		if(pRamp->Output <= pRamp->SPk)	//End the deceleration
		{
			pRamp->Output = pRamp->SPk;
			pRamp->State = RUN; 					
		}
	}
	else if(pRamp->State == RUN)
	{
		pRamp->Output = pRamp->SPk;
	}
	return pRamp->Output;
}
