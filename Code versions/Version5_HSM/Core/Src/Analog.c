#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern LPF LPF_current_var, LPF_voltage_var;
extern LPF LPF_Ia, LPF_Ib;
extern StateMachine mState_var;

/*_________________________________________________________________________________________________________________________*/
	/*Initiate the ADC parameters*/
	void ADC_Init(ADC_OBJECT *pADC)	{		
		for(int i = 0; i < 4;i++){												// data[0]: current phase A (adc1 channel 8)
			pADC->data[i] = 0;															// data[1]: current phase B (adc1 channel 9)
		}																									// data[2]: voltage (adc1 channel 15)
		pADC->Vref_MCU = 3.22; // 3.275;
		pADC->gain_I = pADC->Vref_MCU/4096.0;			// 12 bit: 2^12=4096
		
		/*Get offset of IC*/
//		for(uint8_t cnt_os = 0; cnt_os < 50; cnt_os++){
//			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pADC->data, 2);
//			pADC->Vref_I_C1 += pADC->gain_I*pADC->data[0];
//			pADC->Vref_I_C2 += pADC->gain_I*pADC->data[1];
//		}
//		pADC->Vref_I_C1 = (double)pADC->Vref_I_C1/50.0f;		
//		pADC->Vref_I_C2 = (double)pADC->Vref_I_C2/50.0f;	
//		HAL_ADC_Stop_DMA(&hadc1);
		
		pADC->Vref_I_C1 = 2.40;
		pADC->Vref_I_C2 = 2.42;
		
		pADC->rawV_Ia = 0; 			pADC->rawV_Ib = 0;
		pADC->filterV_Ia = 0;		pADC->filterV_Ib = 0;	
		pADC->Ia_fb = 0; 				pADC->Ib_fb = 0; 	
		pADC->sensitivity = 0.1;
			
		pADC->rawV_Vs = 0;
		pADC->rawV_ref = 26;
		pADC->Vs_fb = 0;
		pADC->gain_U = (double)pADC->rawV_ref/4096.0;	
		
		pADC->osA_max = 0; 	pADC->osA_min = 0; 
		pADC->osB_max = 0; 	pADC->osB_min = 0;;
		pADC->Iak = 0; 			pADC->Ibk = 0; 
		pADC->Iak_1 = 0; 		pADC->Ibk_1 = 0;
		pADC->os_flA = 0;		pADC->os_flB = 0;
		pADC->osA = 0;			pADC->osB = 0;
	}
/*_________________________________________________________________________________________________________________________*/
	/*Calculate the current feedback from ACS712*/ 
	void CAL_CURRENT(ADC_OBJECT *pADC){
		/*Receive analog by DMA*/
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pADC->data, 2);
	  
		/*Transer with gain*/
		pADC->rawV_Ia = pADC->gain_I*pADC->data[0]; //adc channel 8 Ia
		pADC->rawV_Ib = pADC->gain_I*pADC->data[1]; //adc channel 9 Ib 
			
		/*Pass the Filter*/
		pADC->filterV_Ia = LPF_Run(&LPF_Ia, pADC->rawV_Ia);		
		pADC->filterV_Ib = LPF_Run(&LPF_Ib, pADC->rawV_Ib);
			
		/*Real current feedback*/
		pADC->Ia_fb = (pADC->Vref_I_C1 - pADC->filterV_Ia)/pADC->sensitivity;
		pADC->Ib_fb = (pADC->Vref_I_C2 - pADC->filterV_Ib)/pADC->sensitivity;
			
		if(pADC->Ia_fb > 12.0f || pADC->Ia_fb < -12.0f || pADC->Ib_fb > 12.0f || pADC->Ib_fb < -12.0f)		// when current over rate
		{
			if(mState_var.mSTATE == mRUN){
				Motor_Disable();
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			}
		}
		
		/*Voltage feedback*/
		//pADC->rawV_Vs = pADC->data[2];
		//pADC->Vs_fb = pADC->data[3];
		//pADC->Vs_fb = pADC->rawV_Vs*pADC->gain_U;
		//pADC->Vs_fb = LPF_Run(&LPF_voltage_var, pADC->Vs_fb);
	}
	

//-----------------------------------------------
//	for(uint8_t cnt_os = 0; cnt_os < 100; cnt_os++){
//			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pADC->data, 2);
//			pADC->Vref_I_C1 += pADC->gain_I*pADC->data[0];
//			pADC->Vref_I_C2 += pADC->gain_I*pADC->data[1];
//		}
//		pADC->Vref_I_C1 = (double)pADC->Vref_I_C1/100.0;		
//		pADC->Vref_I_C2 = (double)pADC->Vref_I_C2/100.0;	
//		HAL_ADC_Stop_DMA(&hadc1);
	
//-----------------------------------------------	
//		float V_offset1 = 0, V_offset2 = 0;
//		for(uint8_t cnt_os = 0; cnt_os < 100; cnt_os++){
//			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pADC->data, 2);
//			V_offset1 += pADC->gain_I*pADC->data[0];
//			V_offset2 += pADC->gain_I*pADC->data[1];
//		}
//		V_offset1 = (double)V_offset1/100.0;		
//		V_offset2 = (double)V_offset2/100.0;	
//		HAL_ADC_Stop_DMA(&hadc1);
//		pADC->offset1 = 2.5 - V_offset1;
//		pADC->offset2 = 2.5 - V_offset2;		
//		pADC->Vref_I_C1 = 2.5 + pADC->offset1;
//		pADC->Vref_I_C2 = 2.5 + pADC->offset2;


//		if(pADC->filterV_Ia <= pADC->Vref_I_C1)
//			pADC->Ia_fb = ((pADC->Vref_I_C1 - pADC->offset1) - pADC->filterV_Ia)/pADC->sensitivity;
//		else
//			pADC->Ia_fb = ((pADC->Vref_I_C1 + pADC->offset1) - pADC->filterV_Ia)/pADC->sensitivity; 
//		
//		if(pADC->filterV_Ib <= pADC->Vref_I_C2)
//			pADC->Ib_fb = ((pADC->Vref_I_C2 - pADC->offset2) - pADC->filterV_Ib)/pADC->sensitivity;
//		else
//			pADC->Ib_fb = ((pADC->Vref_I_C2 + pADC->offset2) - pADC->filterV_Ib)/pADC->sensitivity; 
//		
//-----------------------------------------------		
//		pADC->Iak = (pADC->Vref_I_C1 - pADC->filterV_Ia)/pADC->sensitivity;
//		if(pADC->Iak_1 <= 0 && pADC->Iak > 0)
//		{
//			pADC->os_flA = 1;
//		}
//		if(pADC->Iak_1 >= 0 && pADC->Iak < 0)
//		{
//			pADC->os_flA = 2;
//		}
//		if(pADC->os_flA == 1){	// get offset A
//			pADC->osA += pADC->Iak;
//		}
//		else if(pADC->os_flA == 2)
//		{
//			pADC->osB_max = pADC->osA;
//			pADC->osA = 0;
//			pADC->os_flA = 0;
//		}
//		pADC->Iak_1 = pADC->Iak;
//		pADC->Ia_fb = pADC->Iak - pADC->osB/2;		