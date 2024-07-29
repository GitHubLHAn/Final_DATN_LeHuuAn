/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
	typedef enum {
			mSTANDSTILL,			
			mHOME, 			
			mREADY, 		// put the rotor on alpha axis
			mRUN,				// run motor
			mSTOP,
			mFAULT			
		}mSTATE;
		/*--------------*/
		typedef enum {
			OPENLOOP,	
			CLOSE_LOOP_I, CLOSE_LOOP_SPD, CLOSE_LOOP_POS,			//  current OR current+speed OR current+speed+position CLOSELOOP
			SMC_MODE,
		}mCONTROL_MODE;
		/*--------------*/
		typedef enum {
			GET_C_DATA, GET_S_DATA, GET_P_DATA, NON_GET_DATA
		}mDATA;
	/*______________________________________________________________________________________*/
		typedef struct{
			mSTATE mSTATE;
			mCONTROL_MODE mCONTROL_MODE;
			unsigned int cnt_delay;
			uint8_t flag_set_rpm;
		}StateMachine;
	/*______________________________________________________________________________________*/
		/*Ramp Fcn generator*/
		typedef enum{
			ACC, 
			RUN, 
			DEC
		}RAMP_STATE;
		typedef struct
		{
			float SPk;												//Current and past value of setpoint
			float inputMax, inputMin;
			uint16_t T_Acc, T_Dec;						//Ramp up and down time (s)
			float delta_Acc, delta_Dec;				//step size in ACC and DEC mode
			uint16_t set_AccCnt, set_DecCnt;	//Set count - 10ms Resolution
			RAMP_STATE State;
			float Output;
		}RAMP;
	/*______________________________________________________________________________________*/
		/*HYBRID STEPPER MOTOR (HSM) PARAMETER*/ 
		typedef struct{
			float Vdc;
			float I_max, V_max, Speed_max, w_max;		// Current, Voltage, Speed (rpm) rated
			float La, Ra;
			float Torque_max;
			float Pp; 	// num of pole
			float flux;
			float Km; 	// total flex
			float J;
			float TL;		// momen load
			float step_angle;
		}HSM;
	/*______________________________________________________________________________________*/		
		typedef struct{
			volatile unsigned int flag_50us;		// set on every 25us
			volatile uint32_t flag_50us_1ms;		// set on every 50us
			volatile uint32_t flag_50us_5ms;		//set on every 500us
			volatile uint16_t flag_1ms;
			volatile uint16_t flag_1ms_20ms;
			volatile uint16_t cnt_s, second, cnt_data;
		}FLAG;
	/*______________________________________________________________________________________*/
		/*ENCODER*/ 
		typedef struct{		
			int32_t ppr;									//Pulse per round of the Encoder
			double Ts;										//Sampling time (sec)
			int32_t pulse, pulse_pre;			//Value of counter at time instance k and k_1
			volatile uint32_t Overflow; 	//Overflow counter
			volatile int16_t rpm_fb, rpm_fb_fil;				//Round per minute feedback
			double w_fb;									//Omega feedback
			int32_t Direction;						//Direction of the motor	
			float gain;
			float delta_pul; 						//pulse_k - pulse_k_1	
			double gain_pul_the;      // gain of 1 pulse with thera(rad)
			volatile uint8_t first_cnt;
			volatile uint8_t flag_get_pulse;
			
		}ENCODER_OBJECT;
	/*______________________________________________________________________________________*/
		/*PI CONTROLLER FOR PHASE A OR B*/ 
		typedef struct{		
			float Kp, Ti;								//Controller parameters	
			float Ka;										//Ka: Anti-windup Gain
			double Ek, Ek_1, Esk, Esk_1;			//Error
			double Umax, Umin, Usk, Usk_1, Uk, Uk_1;	//Control signal for PI controller of current
			double Imax, Imin, Isk, Isk_1, Ik, Ik_1;	//Control signal for PI controller of speed
			double Ts;												//Sampling time
		}PI_OBJECT;			
	/*______________________________________________________________________________________*/
		/*ADC*/ 
		typedef struct{					 
			volatile uint16_t data[4]; 		// ADC value cache byte
			volatile float rawV_Ia, rawV_Ib, rawV_Vs, rawV_ref;  //ADC value raw Voltage
			float Vref_MCU, Vref_I_C1, Vref_I_C2;     // ref voltage from MCU and sensor
			float offset1, offset2;
			volatile float filterV_Ia, filterV_Ib, filterV_Vs;  // Value after filter
			volatile float Ia_fb, Ib_fb, Vs_fb; //	Value after calculate
			float sensitivity;			//The sensitivity of sensor
			double gain_I, gain_U;
			
			volatile float osA_max, osA_min, osB_max, osB_min, osA, osB;
			volatile float Iak, Ibk, Iak_1, Ibk_1;
			volatile uint8_t os_flA, os_flB;
		}ADC_OBJECT;
	/*______________________________________________________________________________________*/
		/*BIEN DIEU KHIEN*/ 
		typedef struct{
			volatile float Id_sp, Id_fb, Iq_sp, Iq_fb;
			volatile float EBF_d, EBF_q;		// estimate back flux
			
			volatile float Ud, Uq, Ua, Ub;

			volatile double theta_e, theta_m; 	// theta electric and mechanics
			volatile float cos_theta, sin_theta;
			volatile uint16_t pul_theta_e, ptr_theta; 	//theta pointer
			volatile double Ts, T_sin, f_sin;
			volatile uint32_t k_max, cnt_k;
			
			volatile int16_t rpm_sp;
			volatile double w_sp;
			
			volatile int motor_dir;
			volatile float V_ref;
			uint16_t duty_max; // AutoReload 
			volatile float dutyA, dutyB;
		}CONTROL_OBJECT;

	/*______________________________________________________________________________________*/
		typedef struct{
			volatile int16_t data_1[2001];			// buffer array
			volatile int16_t data_2[2001];
			volatile int16_t data_3[2001];	
			volatile int16_t data_4[2001];	
			volatile uint8_t saving, save_done, sending;
			volatile uint16_t cnt;
			volatile uint16_t num_data;  // number of data
			volatile uint8_t start;
		}UART;
	/*______________________________________________________________________________________*/
		/*BO LOC THONG THAP (LPF)*/ 
		typedef struct{
			float Ts;							//Sampling time
			float Fc; 						//Conrner frequency (Hz)
			float a1, a2, b1, b2;	//Parameters of the filter
			float alpha;
			float yk, yk_1, yk_2, uk, uk_1;
		}LPF;
		
		typedef struct
		{
			float Ra, La, RPM_max;	//Parameters of the source inductor
			int16_t RPM_sp;
			float Iak, Iak_1, Ibk, Ibk_1;
			float Vak, Vbk, Vak_1, Vbk_1;
			float EMFak_1, EMFbk_1;
			double Ts, t;
			float EMFa_est, EMFb_est;
			float theta_set, theta_est, theta_real, theta_pll, theta_atan2;	
			float delta_theta, rpm_fb_est, rpm_fb_est_fil;  // feedback, estimate, filter

			float lamda;
			float erIak, erIbk;

			float Iak_ref, Ibk_ref;
			unsigned int flag_PIs, Ts_s_max;
			
			float Kps;
			float Tis;
			float isk_1, ik_1, ik, isk;
			float ek, esk_1, ek_1;
			float imax, imin;
			float Iq_ref, Iq_cal;
			
			float Va_ref, Vb_ref;
			volatile float dutyA, dutyB;
			uint16_t k_cnt;
			
			double pk, pk_1, theta_k, theta_k_1;
			double psk, psk_1, Wk, Wk_1;
			double Kp_pll, Ti_pll;
			
			volatile uint8_t Direction;
			uint16_t flag_speed;
			uint16_t flag_trans;
			uint16_t flag_fc;
			
		}SMC;
		
		
		typedef struct
		{
			float Vdc;
			uint8_t sector;
			float angle;
			double t1,t2,t5, Ts;
			float m1, m2, m3, m4;
			
		}SVPWM;
		
		
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
	/*_________________________________________________________________________________________________________________________*/
	/*USER FUNCTION.C*/
		void FLAG_Init(FLAG *pFL);	/*KHAI BAO CAC CO NGAT*/
		void delay_us(uint16_t us);
		void delay_ms(uint16_t ms);
		void LPF_Init(LPF *pLPF, uint16_t Fcut, float Ts);	/*KHAI BAO THONG SO BO LOC THONG THAP*/
		float LPF_Run(LPF *pLPF, float input);	/*RUN BO LOC THONG THAP*/
		void RAMP_Init(RAMP *pRamp, uint16_t Tacc_Sec, uint16_t Tdec_Sec, float inputMin, float inputMax, float Ts);
		float RAMP_RUN(float SP, RAMP *pRamp);
		void fullstep(float Ua, float Ub);
		void run_fullstep(float Ua, float Ub, uint32_t t);
		void fullstep(float Ua, float Ub);
		float ApproxAtan(float z);
		float ApproxAtan2(float y, float x);
		double cal_sin(float radian);
		double cal_cos(float radian);
		
	/*_________________________________________________________________________________________________________________________*/	
	/*UART.C*/
		void UART_Init(UART *pUART);
		void SAVE_DATA(mDATA get_mode);
		void SEND_DATA(void);
	/*_________________________________________________________________________________________________________________________*/	
	/*HSM.C*/
		void MOTOR_Init(HSM *pHSM);	/*KHAI BAO THONG SO DONG CO*/
		void Motor_Disable(void);	/*STOP MOTOR*/
		void Motor_Enable(void);	/*ENABLE MOTOR*/
		void STATE_MACHINE_Init(mCONTROL_MODE control_mode);
		void STATE_MACHINE_HANDLE(void);
	/*_________________________________________________________________________________________________________________________*/	
	/*ENCODER.C*/
		void ENCODER_Init(ENCODER_OBJECT *pEnc, int ppr, int mode, double Ts);	/*KHAI BAO THONG SO TINH TOC DO TU ENCODER*/
		void CAL_SPEED(ENCODER_OBJECT *pEnc);	/*TINH TOC DO TU ENCODER THU VE*/
		int16_t CAL_DELTA_PULSE(ENCODER_OBJECT *pEnc);
		//float Est_THETA_E(SMC *pSMC, float Ia_fb, float Ib_fb, float Uak, float Ubk);
		//void Est_Init(SMC *pSMC);
		float CAL_THETA(ENCODER_OBJECT *pEnc, uint8_t Dir);
	/*_________________________________________________________________________________________________________________________*/	
	/*ADC.C*/
		void ADC_Init(ADC_OBJECT *pADC);	/*KHAI BAO THONG SO TINH DONG DIEN*/
		void CAL_CURRENT(ADC_OBJECT *pADC);	/*TINH DONG DIEN THU VE TU ACS712*/ 
	/*_________________________________________________________________________________________________________________________*/	
	/*CONTROLLER.C*/
		void CONTROLLER_Init(CONTROL_OBJECT *pCon, double Ts);	/*KHAI BAO THONG SO BO DIEN KHIEN*/

		void OPEN_LOOP_Fcn(CONTROL_OBJECT *pCon, float Uq, float rpm_sp);
		
		void SMC_Init(SMC *pSMC);
		void SMC_Fcn(SMC *pSMC, int16_t RPM_sp);
		float Speed_SMC_Loop(SMC *pSMC, int16_t RPM_sp);
		void Est_Speed(SMC *pSMC, int32_t RPM_sp);
		
		void SVPWM_Init(SVPWM *pSV);
		void SVPWM_RUN(SVPWM *pSV,float Vak, float Vbk);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADDR_0_Pin GPIO_PIN_13
#define ADDR_0_GPIO_Port GPIOC
#define ADDR_1_Pin GPIO_PIN_14
#define ADDR_1_GPIO_Port GPIOC
#define ADDR_2_Pin GPIO_PIN_15
#define ADDR_2_GPIO_Port GPIOC
#define LPTIM1_T_PULSE_Cmd_Pin GPIO_PIN_0
#define LPTIM1_T_PULSE_Cmd_GPIO_Port GPIOC
#define DIRECTION_Pin GPIO_PIN_1
#define DIRECTION_GPIO_Port GPIOC
#define Servo_Lock_Pin GPIO_PIN_2
#define Servo_Lock_GPIO_Port GPIOC
#define RS485_nW_R2_Pin GPIO_PIN_3
#define RS485_nW_R2_GPIO_Port GPIOC
#define ADC2_14_VBusN_Pin GPIO_PIN_4
#define ADC2_14_VBusN_GPIO_Port GPIOC
#define ADC1_15_VBusP_Pin GPIO_PIN_5
#define ADC1_15_VBusP_GPIO_Port GPIOC
#define ADC_8_Ia_Pin GPIO_PIN_0
#define ADC_8_Ia_GPIO_Port GPIOB
#define ADC_9_Ib_Pin GPIO_PIN_1
#define ADC_9_Ib_GPIO_Port GPIOB
#define DI_0_Pin GPIO_PIN_12
#define DI_0_GPIO_Port GPIOB
#define PWM_EN2_Pin GPIO_PIN_15
#define PWM_EN2_GPIO_Port GPIOB
#define DI_1_Pin GPIO_PIN_6
#define DI_1_GPIO_Port GPIOC
#define DI_2_Pin GPIO_PIN_7
#define DI_2_GPIO_Port GPIOC
#define DI_3_Pin GPIO_PIN_8
#define DI_3_GPIO_Port GPIOC
#define PWM_EN1_Pin GPIO_PIN_9
#define PWM_EN1_GPIO_Port GPIOC
#define RS485_EN3_Pin GPIO_PIN_12
#define RS485_EN3_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define ADDR_3_Pin GPIO_PIN_5
#define ADDR_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PI 3.141593
#define GAIN_W	9.5473   // transfer rpm to rad/s
#define CCW 0    //counter-clock-wise
#define CW 1		// clock-wise
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
