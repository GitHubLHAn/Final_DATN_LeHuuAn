#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

extern StateMachine mState_var;		// state machine variable
extern CONTROL_OBJECT control_var;
extern PI_OBJECT PI_Id_var, PI_Iq_var, PI_speed_var, P_position_var;
extern UART uart_var;

/*_________________________________________________________________________________________________________________________*/
	/*Initiate the hybrid stepper motor parameters*/
	void MOTOR_Init(HSM *pHSM){
		pHSM->Vdc = 24;
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
				htim1.Instance->CCR1 = 2200;		
				htim1.Instance->CCR2 = 2100;
			}
//			else if(mState_var.mCONTROL_MODE == CLOSE_LOOP_I){
//				mState_var.cnt_delay = 2000;		// skip the time out
//			}
			else{			// Close loop
				control_var.Id_sp = 1;
				control_var.Iq_sp = 0;
				CURRENT_CLOSELOOP(&control_var, &PI_Id_var, &PI_Iq_var);
			}
			/*----------------------------------*/
			if(mState_var.cnt_delay < 2000){		// time out 100ms
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
				OPEN_LOOP(&control_var, 24, 180);     // voltage and speed
			}
			/*-----------------------------------------------------------*/
			else if(mState_var.mCONTROL_MODE == CLOSE_LOOP_I){
				control_var.Id_sp = 0;			// optional
				control_var.Iq_sp = 1;
				CURRENT_CLOSELOOP(&control_var, &PI_Id_var, &PI_Iq_var);
			}
			/*-----------------------------------------------------------*/
			else if(mState_var.mCONTROL_MODE == CLOSE_LOOP_SPD){
					CURRENT_CLOSELOOP(&control_var, &PI_Id_var, &PI_Iq_var);
			}
			/*-----------------------------------------------------------*/
			else if(mState_var.mCONTROL_MODE == CLOSE_LOOP_POS){
					// not use yet
			}
		}
	/*=================STATE STOP============================================*/
	if(mState_var.mSTATE == mSTOP){
		
	}


			
		
	}


