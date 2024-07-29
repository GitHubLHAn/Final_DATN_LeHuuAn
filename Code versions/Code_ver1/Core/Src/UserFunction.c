#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

extern UART uart_var;
extern CONTROL_OBJECT control_var;
extern ADC_OBJECT adc_var;
extern FLAG flag_var;

//========================================================================================================================
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
//========================================================================================================================
	/*Initiate the flag variables*/
	void FLAG_Init(FLAG *pFL){
		pFL->flag_25us = 0;
		pFL->flag_25us_50us = 0;
		pFL->flag_25us_20ms = 0;
		pFL->flag_cnt_sin = 0;
		pFL->flag_read = 0;
		pFL->flag_send = 0;
		pFL->flag_getdata = 0;
	}
//========================================================================================================================
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
//========================================================================================================================
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
//========================================================================================================================
	/*Data Handle*/
	void UART_Init(UART *pUART){
		pUART->data_RX = 0;
		for(int i=0; i<2001; i++){			
			pUART->data_1[i] = 0;
			pUART->data_2[i] = 0;
			pUART->data_3[i] = 0;
			pUART->data_4[i] = 0;
		}
	}	
//	void SAVE_DATA(void){
//		if (flag_var.flag_getdata < 40000) flag_var.flag_getdata++;		// if getdata=1 -> start save data into array
//		if (flag_var.flag_getdata==40000) 
//		{
//			flag_var.flag_read = 1;							// set only 1 time after delta_t (1s)  
//			flag_var.flag_getdata=40001;
//		}
//		
//		if(flag_var.flag_read ==1 ){				// start read and save data
//			if(flag_var.flag_cnt_sin < 2000){
//				uart_var.data_Ia[flag_var.flag_cnt_sin] = (int)(1000*adc_var.Ia_fb);
//				uart_var.data_Ib[flag_var.flag_cnt_sin] = (int)(1000*adc_var.Ib_fb);
//				uart_var.data_theta[flag_var.flag_cnt_sin] = (int)(1000*control_var.theta_e);
//				flag_var.flag_cnt_sin++;			// next sample
//			}
//			else{
//				flag_var.flag_send = 1;		// start sending
//				flag_var.flag_read = 0;			// stop reading
//			}
//		}	
//	}
	
//			if(uart_var.cnt < 50){
//				sprintf((char*)MSG,"%d,%d,%d;",1*uart_var.cnt,10*uart_var.cnt,100*uart_var.cnt);
//				HAL_UART_Transmit(&huart3, (uint8_t *)MSG, sizeof(MSG), 30);
//				uart_var.cnt++;
//			}
//			else if(uart_var.cnt == 50){
//				HAL_UART_Transmit(&huart3, (uint8_t *)"E0", sizeof("E0"), 10);
//				uart_var.cnt++;
//			}
//			
//			HAL_Delay(100) ;


			
//		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)os, 2);
//		HAL_Delay(100);
//		for(int i = 0; i < 2;i++){				
//			offset[i]=0;
//			os[i]=0;
//		}
//		for(uint8_t cnt_os = 0; cnt_os < 50; cnt_os++){
//			
//			offset[0] += os[0];
//			offset[1] += os[1];
//		}
//		offset[0] = offset[0]/50;
//		offset[1] = offset[1]/50;
//		HAL_ADC_Stop_DMA(&hadc1);	

