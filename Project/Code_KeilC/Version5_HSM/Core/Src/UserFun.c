#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;

extern UART uart_var;
extern CONTROL_OBJECT control_var;
extern ADC_OBJECT adc_var;
extern FLAG flag_var;

double sin_degree[361] = {0.000000,0.017452,0.034899,0.052336,0.069756,0.087156,0.104528,0.121869,0.139173,0.156434,0.173648,0.190809,
													0.207912,0.224951,0.241922,0.258819,0.275637,0.292372,0.309017,0.325568,0.342020,0.358368,0.374607,0.390731,
													0.406737,0.422618,0.438371,0.453990,0.469472,0.484810,0.500000,0.515038,0.529919,0.544639,0.559193,0.573576,
													0.587785,0.601815,0.615661,0.629320,0.642788,0.656059,0.669131,0.681998,0.694658,0.707107,0.719340,0.731354,
													0.743145,0.754710,0.766044,0.777146,0.788011,0.798636,0.809017,0.819152,0.829038,0.838671,0.848048,0.857167,
													0.866025,0.874620,0.882948,0.891007,0.898794,0.906308,0.913545,0.920505,0.927184,0.933580,0.939693,0.945519,
													0.951057,0.956305,0.961262,0.965926,0.970296,0.974370,0.978148,0.981627,0.984808,0.987688,0.990268,0.992546,
													0.994522,0.996195,0.997564,0.998630,0.999391,0.999848,1.000000,0.999848,0.999391,0.998630,0.997564,0.996195,
													0.994522,0.992546,0.990268,0.987688,0.984808,0.981627,0.978148,0.974370,0.970296,0.965926,0.961262,0.956305,
													0.951057,0.945519,0.939693,0.933580,0.927184,0.920505,0.913545,0.906308,0.898794,0.891007,0.882948,0.874620,
													0.866025,0.857167,0.848048,0.838671,0.829038,0.819152,0.809017,0.798636,0.788011,0.777146,0.766044,0.754710,
													0.743145,0.731354,0.719340,0.707107,0.694658,0.681998,0.669131,0.656059,0.642788,0.629320,0.615661,0.601815,
													0.587785,0.573576,0.559193,0.544639,0.529919,0.515038,0.500000,0.484810,0.469472,0.453990,0.438371,0.422618,
													0.406737,0.390731,0.374607,0.358368,0.342020,0.325568,0.309017,0.292372,0.275637,0.258819,0.241922,0.224951,
													0.207912,0.190809,0.173648,0.156434,0.139173,0.121869,0.104528,0.087156,0.069756,0.052336,0.034899,0.017452,
													0.000000,-0.017452,-0.034899,-0.052336,-0.069756,-0.087156,-0.104528,-0.121869,-0.139173,-0.156434,-0.173648,
													-0.190809,-0.207912,-0.224951,-0.241922,-0.258819,-0.275637,-0.292372,-0.309017,-0.325568,-0.342020,-0.358368,
													-0.374607,-0.390731,-0.406737,-0.422618,-0.438371,-0.453990,-0.469472,-0.484810,-0.500000,-0.515038,-0.529919,
													-0.544639,-0.559193,-0.573576,-0.587785,-0.601815,-0.615661,-0.629320,-0.642788,-0.656059,-0.669131,-0.681998,
													-0.694658,-0.707107,-0.719340,-0.731354,-0.743145,-0.754710,-0.766044,-0.777146,-0.788011,-0.798636,-0.809017,
													-0.819152,-0.829038,-0.838671,-0.848048,-0.857167,-0.866025,-0.874620,-0.882948,-0.891007,-0.898794,-0.906308,
													-0.913545,-0.920505,-0.927184,-0.933580,-0.939693,-0.945519,-0.951057,-0.956305,-0.961262,-0.965926,-0.970296,
													-0.974370,-0.978148,-0.981627,-0.984808,-0.987688,-0.990268,-0.992546,-0.994522,-0.996195,-0.997564,-0.998630,
													-0.999391,-0.999848,-1.000000,-0.999848,-0.999391,-0.998630,-0.997564,-0.996195,-0.994522,-0.992546,-0.990268,
													-0.987688,-0.984808,-0.981627,-0.978148,-0.974370,-0.970296,-0.965926,-0.961262,-0.956305,-0.951057,-0.945519,
													-0.939693,-0.933580,-0.927184,-0.920505,-0.913545,-0.906308,-0.898794,-0.891007,-0.882948,-0.874620,-0.866025,
													-0.857167,-0.848048,-0.838671,-0.829038,-0.819152,-0.809017,-0.798636,-0.788011,-0.777146,-0.766044,-0.754710,
													-0.743145,-0.731354,-0.719340,-0.707107,-0.694658,-0.681998,-0.669131,-0.656059,-0.642788,-0.629320,-0.615661,
													-0.601815,-0.587785,-0.573576,-0.559193,-0.544639,-0.529919,-0.515038,-0.500000,-0.484810,-0.469472,-0.453990,
													-0.438371,-0.422618,-0.406737,-0.390731,-0.374607,-0.358368,-0.342020,-0.325568,-0.309017,-0.292372,-0.275637,
													-0.258819,-0.241922,-0.224951,-0.207912,-0.190809,-0.173648,-0.156434,-0.139173,-0.121869,-0.104528,-0.087156,
													-0.069756,-0.052336,-0.034899,-0.017452,-0.000000};

/*__________________________________________________________________________________________________________________________*/
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
	/*Blink Leds**************use for test******************************/
		void nhayled(uint16_t ms)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			delay_ms(ms);
		}
/*__________________________________________________________________________________________________________________________*/
	/*Initiate the flag variables*/
	void FLAG_Init(FLAG *pFL){
		pFL->flag_50us = 0;
		pFL->flag_50us_1ms = 0;
		pFL->flag_50us_5ms = 99;
		pFL->flag_1ms = 0;
		pFL->flag_1ms_20ms = 0;
		pFL->cnt_data = 0;
		pFL->cnt_s = 0;
		pFL->second = 0;
	}
/*__________________________________________________________________________________________________________________________*/
	/*Initiate the low pass filter parameters*/ 
	void LPF_Init(LPF *pLPF, uint16_t Fcut, float Ts)
	{
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
/*__________________________________________________________________________________________________________________________*/
	/*Execute the low pass filter */ 
	float LPF_Run(LPF *pLPF, float input)
	{
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
/*__________________________________________________________________________________________________________________________*/
void RAMP_Init(RAMP *pRamp, uint16_t Tacc_Sec, uint16_t Tdec_Sec, float inputMin, float inputMax, float Ts)
	{
		pRamp->inputMax = inputMax;
		pRamp->inputMin = inputMin;
		pRamp->T_Acc = Tacc_Sec;
		pRamp->T_Dec = Tdec_Sec;
		
		//Compute the set count - 10ms resolution
		pRamp->set_AccCnt = pRamp->T_Acc/Ts;	
		pRamp->set_DecCnt = pRamp->T_Dec/Ts;
		//Compute the step size
		pRamp->delta_Acc = (float)(pRamp->inputMax - pRamp->inputMin)/pRamp->set_AccCnt;
		pRamp->delta_Dec = (float)(pRamp->inputMax - pRamp->inputMin)/pRamp->set_DecCnt;
		
		pRamp->SPk = 0;
		pRamp->Output = 60;		// dat gia tri ban dau cho ham ramp
}
/*__________________________________________________________________________________________________________________________*/
	float RAMP_RUN(float SP, RAMP *pRamp)					// ham ramp gia tri dat 
	{
		pRamp->SPk = SP;
		//Detect the state
		if(pRamp->SPk > pRamp->Output + pRamp->delta_Acc)
		{
			pRamp->State = ACC;
		}
		else if (pRamp->SPk < pRamp->Output - pRamp->delta_Dec)
		{
			pRamp->State = DEC;
		}
		else
		{
			pRamp->State = RUN;
		}
		//Generate output
		if(pRamp->State == ACC)
		{
			pRamp->Output = pRamp->Output + pRamp->delta_Acc;
			if(pRamp->Output >= pRamp->SPk)	//End the acceleration
			{
				pRamp->Output = pRamp->SPk;
				pRamp->State = RUN; 					
			}
		}
		else if(pRamp->State == DEC)
		{
			pRamp->Output = pRamp->Output - pRamp->delta_Dec;
			if(pRamp->Output <= pRamp->SPk)	//End the deceleration
			{
				pRamp->Output = pRamp->SPk;
				pRamp->State = RUN; 					
			}
		}
		else if(pRamp->State == RUN)
		{
			pRamp->Output = pRamp->SPk;
		}
		return pRamp->Output;
	}
/*__________________________________________________________________________________________________________________________*/

	void fullstep(float Ua, float Ub){
		float Da = Ua/24.0f;
		float Db = Ub/24.0f;
		
		float duty_a = (1.0f + Da)/2.0f;
		float duty_b = (1.0f + Db)/2.0f;

		htim1.Instance->CCR1 = (int)(8400.0f*duty_a);
		htim1.Instance->CCR2 = (int)(8400.0f*(1.0f - duty_a));
		htim1.Instance->CCR3 = (int)(8400.0f*duty_b);
		htim1.Instance->CCR4 = (int)(8400.0f*(1.0f - duty_b));
	}
	
	void run_fullstep(float Ua, float Ub, uint32_t t)			// chay che do full step (use for test)
	{		
		fullstep(Ua, Ub);
		HAL_Delay(t);
		
		fullstep(Ua, -Ub);
		HAL_Delay(t);
		
		fullstep(-Ua, -Ub);
		HAL_Delay(t);
		
		fullstep(-Ua, Ub);
		HAL_Delay(t);
	}

/*_________________________________________________________________________________________________________________________*/
	float ApproxAtan(float z){					// xap xi ham arctan
			const float n1 = 0.97239411f;
			const float n2 = -0.19194795f;
			return (n1 + n2 * z * z) * z;
	}
	float ApproxAtan2(float y, float x){	// xap xi ham atan2
			if (x != 0.0f){
					if (fabsf(x) > fabsf(y)){
							const float z = y / x;
							if (x > 0.0f){
									return ApproxAtan(z);		// atan2(y,x) = atan(y/x) if x > 0
							}
							else if (y >= 0.0f){		
									return ApproxAtan(z) + PI;		// atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0
							}
							else{
									return ApproxAtan(z) - PI;		// atan2(y,x) = atan(y/x) - PI if x < 0, y < 0
							}
					}
					else{ 							// Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1.
							const float z = x / y;
							if (y > 0.0f){		
									return -ApproxAtan(z) + PI/2;		// atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0
							}
							else{
									return -ApproxAtan(z) - PI/2;		// atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0
							}
					}
			}
			else{
					if (y > 0.0f){ 					// x = 0, y > 0
							return PI/2;
					}
					else if (y < 0.0f){		 // x = 0, y < 0
							return -PI/2;
					}
			}
			return 0.0f; // x,y = 0. Could return NaN instead.
	}
/*_________________________________________________________________________________________________________________________*/
	double cal_sin(float radian){					// luong giac: sin x
    float rad = radian;
    if(rad>2*PI || rad<-2*PI){
    	while(rad>2*PI)
    		rad = rad - 2*PI;
    	while(rad<-2*PI)
    		rad = rad + 2*PI;
		}
		int degree = (int)((rad*180)/PI);
		
		if(radian >= 0){
			return sin_degree[degree];
		}
		else{
			return -sin_degree[-degree]; 
		}
	}
//----------------------------------------------------------------
	double cal_cos(float radian){     // cos x = sin ( Pi/2 - x)
		float rad = PI/2 - radian;
		return cal_sin(rad);
	}
	