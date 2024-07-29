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
		typedef enum{
			SPEED,
			CURRENT
		}TYPE;
		
		typedef enum{
			RL,		// Quay tu phai sang trai
			LR		// Quay tu trai sang phai
		}DIR;
		
		typedef struct{
			volatile unsigned int flag_25us;		// set on every 25us
			volatile uint32_t flag_25us_50us;		// set on every 50us
			volatile uint32_t flag_25us_20ms;		//set on every 500us
			volatile uint16_t flag_cnt_sin;			// count number of sin which wanna get data
			volatile uint8_t flag_read;		// set on when read data and save it into array of UART
			volatile uint8_t flag_send;		// set on when already read data
			volatile uint16_t flag_getdata;		// set on when start get data to save
		}FLAG;
		
		/*ENCODER*/ 
		typedef struct{		
			int32_t ppr;									//Pulse per round of the Encoder
			double Ts;										//Sampling time (sec)
			int32_t pulse, pulse_pre;			//Value of counter at time instance k and k_1
			volatile uint32_t Overflow; 	//Overflow counter
			volatile float rpm_fb;				//Round per minute feedback
			double w_fb;									//Omega feedback
			int32_t Direction;						//Direction of the motor	
			float gain;
			float delta_pul; 						//pulse_k - pulse_k_1	
			double gain_pul_the;      // gain of 1 pulse with thera(rad)
		}ENCODER_OBJECT;
		
		/*PI CONTROLLER FOR PHASE A OR B*/ 
		typedef struct{		
			float Kp, Ti;								//Controller parameters	
			float Ka, D;										//Ka: Anti-windup Gain; D = 1-Ts/TI
			double Ek, Ek_1, Esk, Esk_1;			//Error
			double Umax, Umin, Usk, Usk_1, Uk, Uk_1;	//Control signal for PI controller of current
			double Imax, Imin, Isk, Isk_1, Ik, Ik_1;	//Control signal for PI controller of speed
			double Ts;												//Sampling time
		}PI_OBJECT;			
		
		/*ADC*/ 
		typedef struct{					 
			volatile uint16_t data[3]; 		// ADC value cache byte
			volatile float rawV_Ia, rawV_Ib, rawV_Vs, rawV_ref;  //ADC value Voltage
			float Vref_MCU, Vref_I_C1, Vref_I_C2;     // ref voltage from MCU and sensor
			volatile float filterV_Ia, filterV_Ib, filterV_Vs;  // Value after filter
			volatile float Ia_fb, Ib_fb, Vs_fb; //	Value after calculate
			float sensitivity;			//The sensitivity of sensor
			double gain_I, gain_U;
			volatile uint16_t test_Ia, test_Ib;
			//volatile float offset_C1[2][50];
		}ADC_OBJECT;
		
		/*BIEN DIEU KHIEN*/ 
		typedef struct{
			volatile float Id_sp, Id_fb, Iq_sp, Iq_fb;
			volatile float EBF_d, EBF_q;		// estimate back flux
			
			volatile float Ud, Uq, Ua, Ub;

			volatile double theta_e, theta_m; 	// theta electric and mechanics
			volatile double Ts, T_sin, f_sin;
			volatile uint32_t k_max, cnt_k;
			
			volatile float rpm_sp;
			volatile double w_sp;
			
			volatile DIR motor_dir;
			volatile float V_ref;
			uint16_t duty_max; // AutoReload 
			volatile uint16_t dutyA, dutyB;
		}CONTROL_OBJECT;
		
		/*HYBRID STEPPER MOTOR (HSM) PARAMETER*/ 
		typedef struct{
			float Vdc;
			float I_max, V_max, Speed_max, w_max;		// Dong dien, dien ap, Toc do (rpm) DINH MUC
			float La, Ra;
			float Torque_max;
			float Pp; 	// so doi cuc
			float flux;
			float Km; 	// tu thong
			float J;
			float TL;		// momen tai
			float step_angle;
		}HSM;
		
		/*SVPWM*/
		typedef struct{
			volatile float angle_sector, V_star, V_phay;
			double Ts_half, t_A, t_B;
			volatile uint8_t sector;
			volatile double A, B, C, D;
			volatile double t10, t20, t11, t21;	//Thoi gian hoat dong cua tung vecto dien ap		
		}SVPWM;
		
		typedef struct{
			uint8_t data_RX;
			volatile int16_t data_1[2001];			// buffer array
			volatile int16_t data_2[2001];
			volatile int16_t data_3[2001];	
			volatile int16_t data_4[2001];	
			//uint8_t MSG[100];
			volatile uint16_t cnt;
		}UART;
		
		/*BO LOC THONG THAP (LPF)*/ 
		typedef struct{
			float Ts;							//Sampling time
			float Fc; 						//Conrner frequency (Hz)
			float a1, a2, b1, b2;	//Parameters of the filter
			float alpha;
			float yk, yk_1, yk_2, uk, uk_1;
		}LPF;
		
		enum{
			WAIT_TIME,
			OPENLOOP,
			CURRENT_CLOSELOOP,
			SPEED_CLOSELOOP
		}MODE_RUN;
		
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
	/*USER FUNCTION.C*/
		void FLAG_Init(FLAG *pFL);	/*KHAI BAO CAC CO NGAT*/
		void delay_us(uint16_t us);
		void delay_ms(uint16_t ms);
		void LPF_Init(LPF *pLPF, uint16_t Fcut, float Ts);	/*KHAI BAO THONG SO BO LOC THONG THAP*/
		float LPF_Run(LPF *pLPF, float input);	/*RUN BO LOC THONG THAP*/
		void UART_Init(UART *pUART);
		//void SAVE_DATA(void);
		
	/*HSM.C*/
		void MOTOR_Init(HSM *pHSM);	/*KHAI BAO THONG SO DONG CO*/
		void Motor_stop(void);	/*STOP MOTOR*/
		void Motor_Enable(void);	/*ENABLE MOTOR*/
	/*ENCODER.C*/
		void ENCODER_Init(ENCODER_OBJECT *pEnc, int ppr, int mode, double Ts);	/*KHAI BAO THONG SO TINH TOC DO TU ENCODER*/
		void CAL_SPEED(ENCODER_OBJECT *pEnc);	/*TINH TOC DO TU ENCODER THU VE*/
		float CAL_THETA(ENCODER_OBJECT *pEnc, uint32_t cnt_k);
	/*ADC.C*/
		void ADC_Init(ADC_OBJECT *pADC);
		//void ADC_Init(ADC_OBJECT *pADC);	/*KHAI BAO THONG SO TINH DONG DIEN*/
		void CAL_CURRENT(ADC_OBJECT *pADC);	/*TINH DONG DIEN THU VE TU ACS712*/ 
	/*CONTROLLER.C*/
		void CONTROLLER_Init(CONTROL_OBJECT *pCon, double Ts);	/*KHAI BAO THONG SO BO DIEN KHIEN*/
		void PI_SPEED_Init(PI_OBJECT *pPI, double Ts);	/*KHAI BAO THONG SO PI TOC DO*/ 
		void PI_CURRENT_Init(PI_OBJECT *pPI, double Ts);	/*KHAI BAO THONG SO PI DONG DIEN*/ 
		float PI_Run(PI_OBJECT *pPI, double SP, double FB);	/*RUN PI CONTROLLER*/ 
		float IP_Run(PI_OBJECT *pPI, double SP, double FB);
		
		void SPEED_LOOP(CONTROL_OBJECT *pCon, PI_OBJECT *pPI, float rpm_sp);	/*MACH VONG TOC DO*/
		void CURRENT_OPENLOOP(CONTROL_OBJECT *pCon, ADC_OBJECT *pADC, float Uq);
		void CURRENT_LOOP(CONTROL_OBJECT *pCon, PI_OBJECT *pPI_Id, PI_OBJECT *pPI_Iq); /*MACH VONG DONG DIEN*/
		
		void SVPWM_Init(SVPWM *pSV);
		void SVPWM_ComputeTime(SVPWM *pSV);
		void SVPWM_RUN(CONTROL_OBJECT *pCon, SVPWM *pSV, float Vdc);
		
		
	/*OLD TEST*/
//		void OPEN_LOOP(CONTROL_OBJECT *pCon, uint16_t rpm_sp, DIR dir, uint16_t delta_duty_hold); /*MACH VONG HO*/
//		void OPEN_LOOP_U(CONTROL_OBJECT *pCon, uint16_t rpm_sp, DIR dir, float U_hold);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADDR_0_Pin GPIO_PIN_13
#define ADDR_0_GPIO_Port GPIOC
#define ADDR_1_Pin GPIO_PIN_14
#define ADDR_1_GPIO_Port GPIOC
#define ADDR_2_Pin GPIO_PIN_15
#define ADDR_2_GPIO_Port GPIOC
#define ADC_15_Vbus_Pin GPIO_PIN_5
#define ADC_15_Vbus_GPIO_Port GPIOC
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
#define PI 3.14159
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
