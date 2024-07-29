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
		//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	}
/*_________________________________________________________________________________________________________________________*/
	/*Enable Motor */
	void Motor_Enable(void){			// Enable all H bridge
		HAL_GPIO_WritePin(PWM_EN1_GPIO_Port, PWM_EN1_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(PWM_EN2_GPIO_Port, PWM_EN2_Pin, GPIO_PIN_SET);
	}
/*_________________________________________________________________________________________________________________________*/
	/*Initiate state machine*/
	void STATE_MACHINE_Init(mRUN_MODE run_mode){
		mState_var.mSTATE = mSTAND;
		mState_var.mRUN_MODE = run_mode;
		mState_var.cnt_delay = 0;
		mState_var.start_set_rpm = 0;
	}
/*_________________________________________________________________________________________________________________________*/
	/*STATE_MACHINE_HANDLE*/
	void STATE_MACHINE_HANDLE(void){
		if(mState_var.mSTATE == mSTAND){
			Motor_Disable();
			// if have an start event
			mState_var.mSTATE = mREADY;
		}
	/*=================================*/
		if(mState_var.mSTATE == mREADY){			// ready to run motor
			Motor_Enable();
			if(mState_var.mRUN_MODE == OPENLOOP || mState_var.mRUN_MODE == CLOSE_C_LOOP){				// set the rotor on phase A
				htim1.Instance->CCR1 = 2500;		
				htim1.Instance->CCR2 = 2100;
			}
			if(mState_var.mRUN_MODE == CLOSE_CS_LOOP){		// set and hold the rotor on alpha
//				control_var.Id_sp = 2;
//				control_var.Iq_sp = 0;
//				CURRENT_CLOSELOOP(&control_var, &PI_Id_var, &PI_Iq_var);		// theta = 0;
			}
			if(mState_var.cnt_delay < 20000){		// wait 1 second
				mState_var.cnt_delay++;
			}
			else{
				mState_var.cnt_delay = 0;
				if(control_var.cnt_k == 0){
					mState_var.mSTATE = mRUN;
					uart_var.saving = 1;
				}
			}
		}		
	/*======================================================*/
		if(mState_var.mSTATE == mRUN){
			if(mState_var.mRUN_MODE == OPENLOOP)
				OPEN_LOOP(&control_var, 10, 60);
			/*--------------------------------------*/
			if(mState_var.mRUN_MODE == CLOSE_C_LOOP){
				control_var.Id_sp = 4;			// optional
				control_var.Iq_sp = 4;			// optional
				CURRENT_CLOSELOOP(&control_var, &PI_Id_var, &PI_Iq_var);		// only current loop, I setpoint random
			}
			/*------------------------------------*/
			if(mState_var.mRUN_MODE == CLOSE_CS_LOOP){
				if(mState_var.start_set_rpm == 0){
					control_var.Id_sp = 4;
					control_var.Iq_sp = 0;
					CURRENT_CLOSELOOP(&control_var, &PI_Id_var, &PI_Iq_var);
				}
				
				if(mState_var.cnt_delay < 2000){		// wait 1 second
					mState_var.cnt_delay++;
				}
				else{
					if(mState_var.start_set_rpm == 1)
						CURRENT_CLOSELOOP(&control_var, &PI_Id_var, &PI_Iq_var);		//  I setpoint is output of speed loop
				}
			}
		}
	/*======================================================*/
		if(mState_var.mSTATE == mERROR){
		
		}
		
	}

