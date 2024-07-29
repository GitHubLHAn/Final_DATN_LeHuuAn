#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

//========================================================================================================================
/*Initiate the hybrid stepper motor parameters*/
	void MOTOR_Init(HSM *pHSM){
		pHSM->Vdc = 24*sqrt(2);
		pHSM->V_max = 24.0;
		pHSM->I_max = 6.0;
		pHSM->Speed_max = 130;	// round per minute
		pHSM->w_max = (double)pHSM->Speed_max/9.5493;
		pHSM->La = 0.0054;
		pHSM->Ra = 0.53;
		pHSM->flux = 4.25e-3; // 0.00425;   ???????
		pHSM->J = 1.2e-7;				// ???????
		pHSM->Pp = 50;			// Number of dual pole
		pHSM->Km = pHSM->Pp*pHSM->flux;		// Flex total
		pHSM->Torque_max = 8.5;
		pHSM->step_angle = 1.8;
	}	
//========================================================================================================================
	/*Stop motor*/
	void Motor_stop(void){			// Disnable all H bridge
		HAL_GPIO_WritePin(PWM_EN1_GPIO_Port, PWM_EN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWM_EN2_GPIO_Port, PWM_EN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	}
//========================================================================================================================
	/*Enable Motor */
	void Motor_Enable(void){			// Enable all H bridge
		HAL_GPIO_WritePin(PWM_EN1_GPIO_Port, PWM_EN1_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PWM_EN2_GPIO_Port, PWM_EN2_Pin, GPIO_PIN_SET);
	}

//========================================================================================================================
//		void fullstep(uint16_t ms, uint16_t duty){
//			htim1.Instance->CCR1 = (2100+duty)-1;
//			htim1.Instance->CCR2 = (2100+duty)-1;
//			HAL_Delay(ms);
//			
//			htim1.Instance->CCR1 = (2100+duty)-1;
//			htim1.Instance->CCR2 = (2100)-1;
//			HAL_Delay(ms);
//			
//			htim1.Instance->CCR1 = (2100+duty)-1;
//			htim1.Instance->CCR2 = (2100-duty)-1;
//			HAL_Delay(ms);
//			
//			htim1.Instance->CCR1 = (2100)-1;
//			htim1.Instance->CCR2 = (2100-duty)-1;
//			HAL_Delay(ms);
//			
//			htim1.Instance->CCR1 = (2100-duty)-1;
//			htim1.Instance->CCR2 = (2100-duty)-1;
//			HAL_Delay(ms);
//			
//			htim1.Instance->CCR1 = (2100-duty)-1;
//			htim1.Instance->CCR2 = (2100)-1;
//			HAL_Delay(ms);
//			
//			htim1.Instance->CCR1 = (2100-duty)-1;
//			htim1.Instance->CCR2 = (2100+duty)-1;
//			HAL_Delay(ms);
//			
//			htim1.Instance->CCR1 = (2100)-1;
//			htim1.Instance->CCR2 = (2100+duty)-1;
//			HAL_Delay(ms);
//		}

