#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

extern StateMachine mState_var;		// state machine variable
extern CONTROL_OBJECT control_var;
extern PI_OBJECT PI_Id_var, PI_Iq_var, PI_speed_var, P_position_var;
extern UART uart_var;
extern SMC SMC_var;
extern FLAG flag_var;

/*_________________________________________________________________________________________________________________________*/
	/*Initiate the hybrid stepper motor parameters*/
	void MOTOR_Init(HSM *pHSM){
		pHSM->Vdc = 24;
		pHSM->V_max = 24.0;
		pHSM->I_max = 6.0;
		pHSM->Speed_max = 120;	// round per minute
		pHSM->w_max = (double)pHSM->Speed_max/9.5493;
		pHSM->La = 0.00416;
		pHSM->Ra = 1.6;
		pHSM->flux = 4.25e-3; // 0.00425;   ???????
		pHSM->J = 1.2e-7;				// ???????
		pHSM->Pp = 50;			// Number of dual pole
		pHSM->Km = pHSM->Pp*pHSM->flux;		// Flex total
		pHSM->Torque_max = 8.5;
		pHSM->step_angle = 1.8;
	}	
/*_________________________________________________________________________________________________________________________*/
	/*Stop motor*/
	void Motor_Disable(void){			// Disnable all H bridge
		HAL_GPIO_WritePin(PWM_EN1_GPIO_Port, PWM_EN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWM_EN2_GPIO_Port, PWM_EN2_Pin, GPIO_PIN_RESET);
	}
/*_________________________________________________________________________________________________________________________*/
	/*Enable Motor */
	void Motor_Enable(void){			// Enable all H bridge
		HAL_GPIO_WritePin(PWM_EN1_GPIO_Port, PWM_EN1_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PWM_EN2_GPIO_Port, PWM_EN2_Pin, GPIO_PIN_SET);
	}
/*_________________________________________________________________________________________________________________________*/
	/*Initiate state machine*/
	void STATE_MACHINE_Init(mCONTROL_MODE control_mode){
		mState_var.mSTATE = mSTANDSTILL;
		mState_var.mCONTROL_MODE = control_mode;
		mState_var.cnt_delay = 0;
		mState_var.flag_set_rpm = 0;
	}
/*_________________________________________________________________________________________________________________________*/
	/*STATE_MACHINE_HANDLE*/
	void STATE_MACHINE_HANDLE(void){
		//
		// here: write a receive function to set the setpoint
		//
	/*===============STATE STANDSTILL=======================================*/
		if(mState_var.mSTATE == mSTANDSTILL){
			Motor_Disable();
						
			if(mState_var.cnt_delay < 20000){		// wait 1 second
				mState_var.cnt_delay++;
			}
			else{
				mState_var.cnt_delay = 0;
				if(control_var.cnt_k == 0){
					mState_var.mSTATE = mHOME;
					uart_var.saving = 1;
				}
			}		
		}
		
	/*===============STATE HOME=============================================*/
		if(mState_var.mSTATE == mHOME){
			Motor_Enable();
			
			if(mState_var.mCONTROL_MODE == OPENLOOP){
				htim1.Instance->CCR1 = 4500;		
				htim1.Instance->CCR2 = 3800;
			}

			htim1.Instance->CCR1 = (int)(8400*0.52f);		
			htim1.Instance->CCR2 = (int)(8400*(1.0f - 0.52f));
			htim1.Instance->CCR3 = (int)(8400*0.5f);		
			htim1.Instance->CCR4 = (int)(8400*(1.0f - 0.5f));
			/*----------------------------------*/
			if(mState_var.cnt_delay < 8000){		// time out 100ms
					mState_var.cnt_delay++;
			}
			else{
				mState_var.cnt_delay = 0;
				mState_var.mSTATE = mRUN;
			}
		}		
		
	/*=================STATE RUN============================================*/
		if(mState_var.mSTATE == mRUN){
			if(mState_var.mCONTROL_MODE == OPENLOOP){
				OPEN_LOOP_Fcn(&control_var, 20, 100);     // voltage and speed
			}

			else if(mState_var.mCONTROL_MODE == SMC_MODE){
				SMC_Fcn(&SMC_var, 120);
				
				flag_var.cnt_data++;
				if(flag_var.cnt_data >= 80){
					SAVE_DATA(GET_C_DATA);
					flag_var.cnt_data = 0;
				}			
				

			}
		}
	/*=================STATE STOP============================================*/
	if(mState_var.mSTATE == mSTOP){
		
	}		
}
