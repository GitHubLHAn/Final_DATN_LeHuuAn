#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern LPF LPF_current_var, LPF_voltage_var;

/*_________________________________________________________________________________________________________________________*/
	/*Initiate the ADC parameters*/
	void ADC_Init(ADC_OBJECT *pADC)	{		
		for(int i = 0; i < 3;i++){												// data[0]: current phase A (adc1 channel 9)
			pADC->data[i] = 0;															// data[1]: current phase B (adc1 channel 8)
		}																									// data[2]: voltage (adc1 channel 15)
		pADC->Vref_MCU = 3.275; // 3.275;
		pADC->gain_I = (double)pADC->Vref_MCU/4096.0;			// 12 bit: 2^12=4096
		
		/*Get offset of IC*/
		for(uint8_t cnt_os = 0; cnt_os < 100; cnt_os++){
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pADC->data, 2);
			pADC->Vref_I_C1 += pADC->gain_I*pADC->data[0];
			pADC->Vref_I_C2 += pADC->gain_I*pADC->data[1];
		}
		pADC->Vref_I_C1 = (double)pADC->Vref_I_C1/100.0;		//2.46
		pADC->Vref_I_C2 = (double)pADC->Vref_I_C2/100.0;		//2.33
		HAL_ADC_Stop_DMA(&hadc1);
		
		pADC->rawV_Ia = 0; 			pADC->rawV_Ib = 0;
		pADC->filterV_Ia = 0;		pADC->filterV_Ib = 0;	
		pADC->Ia_fb = 0; 				pADC->Ib_fb = 0; 	
		pADC->sensitivity = 0.185;
			
		pADC->rawV_Vs = 0;
		pADC->rawV_ref = 24;
		pADC->Vs_fb = 0;
		pADC->gain_U = (double)pADC->rawV_ref/4096.0;	
	}
/*_________________________________________________________________________________________________________________________*/
	/*Calculate the current feedback from ACS712*/ 
	void CAL_CURRENT(ADC_OBJECT *pADC){
		/*Receive analog by DMA*/
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pADC->data, 2);
	  
		/*Transer with gain*/
		pADC->rawV_Ia = pADC->gain_I*pADC->data[0]; //adc channel 9 Ia
		pADC->rawV_Ib = pADC->gain_I*pADC->data[1]; //adc channel 8 Ib 
				
		/*Pass the Filter*/
		pADC->filterV_Ia = LPF_Run(&LPF_current_var, pADC->rawV_Ia);		
		pADC->filterV_Ib = LPF_Run(&LPF_current_var, pADC->rawV_Ib);
		
		/*Real current feedback*/
		pADC->Ia_fb = (pADC->Vref_I_C1 - pADC->filterV_Ia)/pADC->sensitivity;
		pADC->Ib_fb = (pADC->Vref_I_C2 - pADC->filterV_Ib)/pADC->sensitivity;
		
//		if(pADC->Ia_fb >=5 || pADC->Ia_fb >= 5){
//			Motor_stop();
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//		}
		
		/*Voltage feedback*/
//		pADC->rawV_Vs = pADC->data[2];
//		pADC->Vs_fb = pADC->rawV_Vs*pADC->gain_U;
//		pADC->Vs_fb = LPF_Run(&LPF_voltage_var, pADC->Vs_fb);
	}
	

	
	
