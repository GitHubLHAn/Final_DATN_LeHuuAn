#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

extern ENCODER_OBJECT encoder_var, theta_var;
extern ADC_OBJECT adc_var;
extern CONTROL_OBJECT control_var;
extern HSM hsm_var;
extern SVPWM svpwm_var;
extern FLAG flag_var;
extern StateMachine mState_var;
extern RAMP ramp_rpm_var;

/*_________________________________________________________________________________________________________________________*/
	/*Initiate the controller parameters*/
	void CONTROLLER_Init(CONTROL_OBJECT *pCon, double Ts){
		pCon->Id_sp = 4;		pCon->Id_fb = 0;
		pCon->Iq_sp = 0;		pCon->Iq_fb = 0;
		pCon->EBF_d = 0,		pCon->EBF_q = 0;

		pCon->Ud = 0; 			pCon->Uq = 0; 
		pCon->Ua = 0; 			pCon->Ub = 0;

		pCon->theta_e = 0;	pCon->theta_m = 0;
		pCon->Ts = Ts;// 50us
		pCon->T_sin = 0;		pCon->f_sin = 0;
		pCon->k_max = 0; 		pCon->cnt_k = 0;

		pCon->rpm_sp = 0;		pCon->w_sp = 0;

		pCon->motor_dir = CCW;
		pCon->V_ref = 0;
		pCon->duty_max = htim1.Instance->ARR; // AutoReload 
		pCon->dutyA = pCon->duty_max/2; 		// put 0V to phase A
		pCon->dutyB = pCon->duty_max/2;		// put 0V to phase B
	}
/*_________________________________________________________________________________________________________________________*/
	/*Initiate the speed PI controller*/ 
	void PI_SPEED_Init(PI_OBJECT *pPI, double Ts){
		pPI->Ek = 0; pPI->Ek_1 = 0; pPI->Esk = 0; pPI->Esk_1 = 0;			// error
		pPI->Ik = 0; pPI->Ik_1 = 0; pPI->Isk = 0; pPI->Isk_1 = 0;		// current output
		pPI->Imax = 4;
		pPI->Imin = -4;
		pPI->Ts = Ts; 			//20ms
		pPI->Kp = 0.03; 					//0.1*pPI->Imax/3.14;   // 0.5*Imax/Emax 
		pPI->Ti = 0.5; 
		pPI->Ka = pPI->Kp;
		pPI->Uk = NULL; pPI->Uk_1 = NULL; pPI->Usk = NULL; pPI->Usk_1 = NULL; pPI->Umax = NULL; pPI->Umin = NULL; // not use
	}
/*_________________________________________________________________________________________________________________________*/
	/*Initiate the current PI controller*/ 
	void PI_CURRENT_Init(PI_OBJECT *pPI, double Ts){		
		pPI->Ek = 0; pPI->Ek_1 = 0; pPI->Esk = 0; pPI->Esk_1 = 0;
		pPI->Uk = 0; pPI->Uk_1 = 0; pPI->Usk = 0; pPI->Usk_1 = 0;
		pPI->Umax = 24;
		pPI->Umin = -24; 
		pPI->Ts = Ts; 
		pPI->Kp = 0.4*pPI->Umax/hsm_var.I_max;		// 0.4*Umax/Emax as Emax = Imax		
		pPI->Ti = 0.003;			 // tuned
		pPI->Ka = pPI->Kp;
		pPI->Ik = NULL; pPI->Ik_1 = NULL; pPI->Isk = NULL; pPI->Isk_1 = NULL; pPI->Imax = NULL; pPI->Imin = NULL;  // not use
	}
/*_________________________________________________________________________________________________________________________*/
	/*RUN PI CONTROLLER*/ 
	float PI_Run(PI_OBJECT *pPI, double SP, double FB){				// for current loop
		
		pPI->Ek = SP - FB;
		/*Error bo chong bao hoa tich phan*/ 
		pPI->Esk = pPI->Ek_1 + pPI->Ka*(pPI->Usk_1 - pPI->Uk_1);		
		/*Phuong phap gian doan hoa (xap xi khau tich bang pp hinh thang), su dung toan tu lui ZOH*/
		pPI->Uk = pPI->Usk_1 + pPI->Kp*(pPI->Ek - pPI->Esk_1+(pPI->Ts/(2*pPI->Ti))*(pPI->Ek + pPI->Esk_1));

		/*Limit output*/ 
		if(pPI->Uk > pPI->Umax)
				pPI->Usk = pPI->Umax;
		else if(pPI->Uk < pPI->Umin)
				pPI->Usk = pPI->Umin;
		else
				pPI->Usk = pPI->Uk;

		/*Update data*/
		pPI->Ek_1 = pPI->Ek;
		pPI->Esk_1 = pPI->Esk;
		pPI->Uk_1 = pPI->Uk;
		pPI->Usk_1 = pPI->Usk;

		return pPI->Usk;
	}
	/*RUN IP CONTROLLER for speed loop*/
	float IP_Run(PI_OBJECT *pPI, double SP, double FB){						// for speed loop
		
//		pPI->Ek = SP - FB;
//		pPI->Esk = pPI->Ek_1 + pPI->Ka*(pPI->Isk_1 - pPI->Ik_1);		
//		/*Phuong phap gian doan hoa (xap xi khau tich bang pp hinh thang), su dung toan tu lui ZOH*/
//		pPI->Ik = pPI->Isk_1 + pPI->Kp*(pPI->Ek - pPI->Esk_1+(pPI->Ts/(2*pPI->Ti))*(pPI->Ek + pPI->Esk_1));
//		/*Limit output*/ 
//		if(pPI->Ik > pPI->Imax)
//				pPI->Isk = pPI->Imax;
//		else if(pPI->Ik < pPI->Imin)
//				pPI->Isk = pPI->Imin;
//		else
//				pPI->Isk = pPI->Ik;
//		/*Update data*/
//		pPI->Ek_1 = pPI->Ek;
//		pPI->Esk_1 = pPI->Esk;
//		pPI->Ik_1 = pPI->Ik;
//		pPI->Isk_1 = pPI->Isk;
//		return pPI->Isk;
		
		
		pPI->Ek = SP - FB;
		pPI->Ik = pPI->Kp*pPI->Ek;
		if(pPI->Ik > pPI->Imax)
				pPI->Isk = pPI->Imax;
		else if(pPI->Ik < pPI->Imin)
				pPI->Isk = pPI->Imin;
		else
				pPI->Isk = pPI->Ik;
		return pPI->Isk;
	}
/*_________________________________________________________________________________________________________________________*/
	/*Closed Speed Loop*/ 
	void SPEED_LOOP(CONTROL_OBJECT *pCon, PI_OBJECT *pPI, float rpm_sp)	{
		//pCon->rpm_sp = rpm_sp;
		pCon->rpm_sp = RAMP_RUN(rpm_sp, &ramp_rpm_var);
		pCon->Id_sp = 0;
		pCon->Iq_sp = IP_Run(&*pPI, pCon->rpm_sp, encoder_var.rpm_fb);
		mState_var.start_set_rpm = 1;
	}

/*_________________________________________________________________________________________________________________________*/
	/*-------------THE CURRENT CLOSED LOOP-------------*/ 
	void CURRENT_CLOSELOOP(CONTROL_OBJECT *pCon, PI_OBJECT *pPI_Id, PI_OBJECT *pPI_Iq){		
		if(mState_var.start_set_rpm == 1){
			pCon->theta_e = 50*CAL_THETA(&theta_var, pCon->cnt_k);  // theta_electric = 50*theta_mechanical 
		}
		else pCon->theta_e = 0;				// when speedloop has not operated yet 
		
		/*Park transfer*/
		pCon->Id_fb = adc_var.Ia_fb*cos(pCon->theta_e) + adc_var.Ib_fb*sin(pCon->theta_e);
		pCon->Iq_fb = -adc_var.Ia_fb*sin(pCon->theta_e) + adc_var.Ib_fb*cos(pCon->theta_e);
		/*PI Controller*/
		pCon->Ud = PI_Run(&*pPI_Id, pCon->Id_sp, pCon->Id_fb);			// Id_sp = const = 0
		pCon->Uq = PI_Run(&*pPI_Iq, pCon->Iq_sp, pCon->Iq_fb);		
		/*Clark transfer*/
		pCon->Ua = pCon->Ud*cos(pCon->theta_e) - pCon->Uq*sin(pCon->theta_e);
		pCon->Ub = pCon->Ud*sin(pCon->theta_e) + pCon->Uq*cos(pCon->theta_e);
		
//		uint8_t Usar = 3;
//		if(pCon->Ua > Usar){	
//			pCon->Ua = Usar;
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//		}
//		if(pCon->Ub > Usar){	
//			pCon->Ub = Usar;
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//		}
//		if(pCon->Ua < -Usar){	
//			pCon->Ua = -Usar;
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//		}
//		if(pCon->Ub < -Usar){	
//			pCon->Ub = -Usar;
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
//		}
		
		
		/*Cal dutycycle*/
		pCon->dutyA = (int)((pCon->Ua/hsm_var.V_max+1)*pCon->duty_max/2);
		pCon->dutyB = (int)((pCon->Ub/hsm_var.V_max+1)*pCon->duty_max/2);
		/*Put duty of timer pulse generation*/
		htim1.Instance->CCR1 = pCon->dutyA;		
		htim1.Instance->CCR2 = pCon->dutyB;

	}
	
/*_________________________________________________________________________________________________________________________*/
	/*----------THIS IS THE OPEN LOOP OF CURRENT USE FOR TEST------------*/ 
	void OPEN_LOOP(CONTROL_OBJECT *pCon, float Uq, float rpm_sp){   
		pCon->Uq = Uq;
		pCon->rpm_sp = rpm_sp;
		pCon->T_sin = 60.0/((double)pCon->rpm_sp*50);
		pCon->f_sin = 1.0/ pCon->T_sin;
		pCon->k_max = pCon->T_sin/pCon->Ts+0.01;

		pCon->theta_e = 2*PI*pCon->f_sin*pCon->cnt_k*pCon->Ts;		// cal theta electric theta = w.t = 2.Pi.f.k.Ts

		pCon->theta_m = CAL_THETA(&theta_var, pCon->cnt_k);

		pCon->Ua = pCon->Ud*cos(pCon->theta_e) - pCon->Uq*sin(pCon->theta_e);
		pCon->Ub = pCon->Ud*sin(pCon->theta_e) + pCon->Uq*cos(pCon->theta_e);	
		
		pCon->dutyA = (int)((pCon->Ua/hsm_var.V_max+1)*pCon->duty_max/2);
		pCon->dutyB = (int)((pCon->Ub/hsm_var.V_max+1)*pCon->duty_max/2);
		
		htim1.Instance->CCR1 = pCon->dutyA;		
		htim1.Instance->CCR2 = pCon->dutyB;
		
		pCon->cnt_k++;
		if(pCon->cnt_k >= pCon->k_max)	pCon->cnt_k = 0;
	}	
/*_________________________________________________________________________________________________________________________*/
	/*Initiate SVPWM parameters*/
	void SVPWM_Init(SVPWM *pSV){
		pSV->angle_sector = 0;
		pSV->V_star = 0; pSV->V_phay = 0;
		pSV->sector = 0;
		pSV->Ts_half = 25e-6;
		pSV->t_A = 0; pSV->t_B = 0;
		pSV->t10 = 0; pSV->A = 0;
		pSV->t20 = 0;	pSV->B = 0;
		pSV->t11 = 0; pSV->C = 0;
		pSV->t21 = 0; pSV->D = 0;
	}
	/*Execute SVPWM*/
	void SVPWM_ComputeTime(SVPWM *pSV){
		double temp = pSV->V_phay/(sin(pSV->angle_sector)+cos(pSV->angle_sector));
		pSV->A = pSV->Ts_half*cos(pSV->angle_sector)*(temp + pSV->V_star)/(2*pSV->V_phay);
		pSV->B = pSV->Ts_half*sin(pSV->angle_sector)*(temp + pSV->V_star)/(2*pSV->V_phay);
		pSV->C = pSV->Ts_half*cos(pSV->angle_sector)*(temp - pSV->V_star)/(2*pSV->V_phay);
		pSV->D = pSV->Ts_half*sin(pSV->angle_sector)*(temp - pSV->V_star)/(2*pSV->V_phay);
	}
	
	void SVPWM_RUN(CONTROL_OBJECT *pCon, SVPWM *pSV, float Vdc){
		pSV->V_phay = Vdc*sqrt(2);
		pSV->V_star = sqrt(pCon->Ua*pCon->Ua+pCon->Ub*pCon->Ub);	
		pSV->angle_sector = atan2(pCon->Ub,pCon->Ua);
		if(pSV->angle_sector > PI/4 && pSV->angle_sector <= 3*PI/4){
			pSV->sector = 1;
		}
		else if(pSV->angle_sector > -PI/4 && pSV->angle_sector <= PI/4){
			pSV->sector = 4;
		}
		else if(pSV->angle_sector > -3*PI/4 && pSV->angle_sector <= -PI/4){
			pSV->sector = 3;
		}
	  else pSV->sector = 2;

		if(pSV->sector == 1){
			pSV->angle_sector = pSV->angle_sector - PI/4;		// Goc giua vecto U1 voi U*
			SVPWM_ComputeTime(&svpwm_var);
			pSV->t10 = pSV->A;
			pSV->t20 = pSV->B;
			pSV->t11 = pSV->C;
			pSV->t21 = pSV->D;
			pSV->t_A =  pSV->t10 + pSV->t20 - pSV->t11 - pSV->t21;
			pSV->t_B = -pSV->t10 + pSV->t20 + pSV->t11 - pSV->t21;
		}
		else if(pSV->sector == 2){
			if(pSV->angle_sector > 0)
				pSV->angle_sector = -pSV->angle_sector + 5*PI/4;	// Goc giua vecto U* va U3
			else
				pSV->angle_sector = pSV->angle_sector + 3*PI/4;
			SVPWM_ComputeTime(&svpwm_var);
			pSV->t10 = pSV->D;
			pSV->t20 = pSV->B;
			pSV->t11 = pSV->A;
			pSV->t21 = pSV->C;
			pSV->t_A = pSV->t10 - pSV->t20 - pSV->t11 + pSV->t21;
			pSV->t_B = pSV->t10 + pSV->t20 - pSV->t11 - pSV->t21;
		}
		else if(pSV->sector == 3){
			pSV->angle_sector = -pSV->angle_sector - 3*PI/4;
			SVPWM_ComputeTime(&svpwm_var);			
			pSV->t10 = pSV->D;
			pSV->t20 = pSV->C;
			pSV->t11 = pSV->A;
			pSV->t21 = pSV->B;
			pSV->t_A = -pSV->t10 + pSV->t20 + pSV->t11 - pSV->t21;
			pSV->t_B = -pSV->t10 - pSV->t20 + pSV->t11 + pSV->t21;
		}
		else{
			if(pSV->angle_sector > 0)
				pSV->angle_sector = -pSV->angle_sector + PI/4;	// Goc giua vecto U1 va U*
			else
				pSV->angle_sector = pSV->angle_sector - PI/4;
			SVPWM_ComputeTime(&svpwm_var);
			pSV->t10 = pSV->A;
			pSV->t20 = pSV->C;
			pSV->t11 = pSV->D;
			pSV->t21 = pSV->B;
			pSV->t_A = -pSV->t10 - pSV->t20 + pSV->t11 + pSV->t21;
			pSV->t_B =  pSV->t10 - pSV->t20 - pSV->t11 + pSV->t21;
		}
		pCon->dutyA = 2100*(pSV->t_A/(25e-6)+1);
		pCon->dutyB = 2100*(pSV->t_B/(25e-6)+1);
	}
	

