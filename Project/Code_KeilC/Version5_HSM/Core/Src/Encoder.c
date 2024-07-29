#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;

extern CONTROL_OBJECT control_var;
extern ENCODER_OBJECT theta_var;
extern HSM hsm_var;
extern LPF LPF_EMFa, LPF_EMFb, LPF_speed_var;

/*_________________________________________________________________________________________________________________________*/
	/*Initiate the parameters used for cal speed from encoder*/
	void ENCODER_Init(ENCODER_OBJECT *pEnc, int ppr, int mode, double Ts){
		pEnc->ppr = ppr*mode;
		pEnc->Ts = Ts; 	
		pEnc->Direction = 0;
		pEnc->Overflow = 0;
		pEnc->pulse = 0;
		pEnc->pulse_pre = 0;
		pEnc->rpm_fb = 0; pEnc->rpm_fb_fil = 0; 
		pEnc->w_fb = 0;
		pEnc->delta_pul = 0;
		pEnc->first_cnt = 0;
		pEnc->gain = 60.0/((double)pEnc->ppr*pEnc->Ts);			// 1 delta_pulse -> 1v/p
		pEnc->gain_pul_the = 2*PI/80;					// 1 delta_pulse -> 1 rad (mechanic theta)
		pEnc->flag_get_pulse = 0; 
	}
/*_________________________________________________________________________________________________________________________*/
	/*Calculate speed from received pulse*/ 
	void CAL_SPEED(ENCODER_OBJECT *pEnc){
		//pEnc->Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
		pEnc->Direction = 1;
		pEnc->pulse = __HAL_TIM_GetCounter(&htim2);
		if(pEnc->first_cnt == 1)
		{
			if(pEnc->Direction == 0)		//  count up
			{
				if(pEnc->pulse >= pEnc->pulse_pre)	
					pEnc->delta_pul = pEnc->pulse - pEnc->pulse_pre;
				else 
					pEnc->delta_pul = pEnc->pulse + pEnc->ppr - pEnc->pulse_pre;		// overflow
			}
			if(pEnc->Direction == 1)		//  count down
			{
				if(pEnc->pulse <= pEnc->pulse_pre)	
					pEnc->delta_pul = pEnc->pulse_pre - pEnc->pulse;
				else 
					pEnc->delta_pul = pEnc->pulse_pre + pEnc->ppr - pEnc->pulse;		// overflow
			}
		}
		else
		{
			pEnc->delta_pul = 0;
			pEnc->first_cnt = 1;
		}
		if(pEnc->delta_pul*pEnc->gain <= 160 && pEnc->delta_pul*pEnc->gain >=-160)
			pEnc->rpm_fb_fil = LPF_Run(&LPF_speed_var, (int)(pEnc->delta_pul*pEnc->gain));
		//pEnc->rpm_fb = pEnc->rpm_fb_fil;
		if(pEnc->Direction == 1)
			pEnc->rpm_fb = pEnc->rpm_fb_fil; 	
		else 
			pEnc->rpm_fb = -pEnc->rpm_fb_fil;
		//pEnc->w_fb = pEnc->rpm_fb/9.5493f;
		pEnc->pulse_pre = pEnc->pulse;
	}
/*_________________________________________________________________________________________________________________________*/
	/*Calculate the theta_mechanic*/
	int16_t CAL_DELTA_PULSE(ENCODER_OBJECT *pEnc){
		if(pEnc->flag_get_pulse == 0)		
		{
			//pEnc->pulse_pre = __HAL_TIM_GetCounter(&htim2);				// Set the start point of pulse 
			__HAL_TIM_SetCounter(&htim2, 0);
			pEnc->flag_get_pulse = 1;
			return 0; 
		}
		else
		{
			//pEnc->Direction = 1;
			pEnc->Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
			pEnc->pulse = __HAL_TIM_GetCounter(&htim2);
			//-------------------------
			/*Counter up*/  
			if(pEnc->Direction == 0)
			{				
					pEnc->delta_pul  = pEnc->pulse;
			}
			/*Counter down*/
			if(pEnc->Direction == 1)
			{		
				pEnc->delta_pul = pEnc->ppr - pEnc->pulse;		// overflow, when pulse_k > pulse_k_1
			}
			
			//---------------------------
			return pEnc->delta_pul; 		// Return delta_pul
		}
	}
	/*_________________________________________________________________________________________________________________________*/
	float CAL_THETA(ENCODER_OBJECT *pEnc, uint8_t Dir){
		
		pEnc->Direction = 1;
		if(pEnc->flag_get_pulse == 0)				 
		{
			pEnc->pulse_pre = __HAL_TIM_GetCounter(&htim2);			
			pEnc->flag_get_pulse = 1;
			return 0; // theta mechanic  = 0
		}
		else
		{
			//pEnc->Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
			pEnc->pulse = __HAL_TIM_GetCounter(&htim2);
			//-------------------------
			/*Counter up*/  
			if(pEnc->Direction == 0 )
			{		
				if(pEnc->pulse >= pEnc->pulse_pre)		
					pEnc->delta_pul  = pEnc->pulse - pEnc->pulse_pre;
				else pEnc->delta_pul = pEnc->pulse + pEnc->ppr - pEnc->pulse_pre;		// overflow, when pulse_k < pulse_k_1
			}
			/*Counter down*/
			if(pEnc->Direction == 1 )
			{		
				if(pEnc->pulse <= pEnc->pulse_pre)	 
					pEnc->delta_pul = pEnc->pulse_pre - pEnc->pulse;
				else pEnc->delta_pul = pEnc->pulse_pre + pEnc->ppr - pEnc->pulse;		// overflow, when pulse_k > pulse_k_1
			}
			
			while(pEnc->delta_pul >= 80)
				pEnc->delta_pul -= 80;
			
			if(pEnc->Direction == 1)
				return pEnc->delta_pul*pEnc->gain_pul_the; // Return the mechanical theta 
			else 
				return -pEnc->delta_pul*pEnc->gain_pul_the; // Return the mechanical theta
		}
	}
	


