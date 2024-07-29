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

//========================================================================================================================
	/*Initiate the controller parameters*/
	void CONTROLLER_Init(CONTROL_OBJECT *pCon, double Ts){
		pCon->Id_sp = 4; pCon->Id_fb = 0;
		pCon->Iq_sp = 0, pCon->Iq_fb = 0;
		pCon->EBF_d = 0, pCon->EBF_q = 0;

		pCon->Ud = 0, pCon->Uq = 0, pCon->Ua = 0, pCon->Ub = 0;

		pCon->theta_e = 0; pCon->theta_m = 0;
		pCon->Ts = Ts;			// 50us
		pCon->T_sin = 0, pCon->f_sin = 0;
		pCon->k_max = 0, pCon->cnt_k = 0;

		pCon->rpm_sp = 0,	pCon->w_sp = 0;

		pCon->motor_dir = LR;
		pCon->V_ref = 0;
		pCon->duty_max = htim1.Instance->ARR; // AutoReload 
		pCon->dutyA = pCon->duty_max/2; 		// put 0V to phase A
		pCon->dutyB = pCon->duty_max/2;		// put 0V to phase B
	}
//========================================================================================================================
	/*Initiate the speed PI controller*/ 
	void PI_SPEED_Init(PI_OBJECT *pPI, double Ts){
		pPI->Ek = 0; pPI->Ek_1 = 0; pPI->Esk = 0; pPI->Esk_1 = 0;			// error
		pPI->Ik = 0; pPI->Ik_1 = 0; pPI->Isk = 0; pPI->Isk_1 = 0;		// current output
		pPI->Imax = 4;
		pPI->Imin = -4;
		pPI->Ts = Ts; 			//20ms
		pPI->Kp = 0.001; //0.1*pPI->Imax/3.14;   // 0.5*Imax/Emax 
		pPI->Ti = 0.5; 
		pPI->Ka = pPI->Kp;
		pPI->D = 1 - pPI->Ts/pPI->Ti;
		pPI->Uk = NULL; pPI->Uk_1 = NULL; pPI->Usk = NULL; pPI->Usk_1 = NULL; pPI->Umax = NULL; pPI->Umin = NULL; // not use
	}
//========================================================================================================================
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
		pPI->D = 1 - pPI->Ts/pPI->Ti;
		pPI->Ik = NULL; pPI->Ik_1 = NULL; pPI->Isk = NULL; pPI->Isk_1 = NULL; pPI->Imax = NULL; pPI->Imin = NULL;  // not use
	}
//========================================================================================================================
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
		pPI->Ek = SP - FB;
		//---------------------------------------
		/*This for PI controller*/
		/*Error bo chong bao hoa tich phan*/ 
		pPI->Esk = pPI->Ek_1 + pPI->Ka*(pPI->Isk_1 - pPI->Ik_1);		
		/*Phuong phap gian doan hoa (xap xi khau tich bang pp hinh thang), su dung toan tu lui ZOH*/
		pPI->Ik = pPI->Isk_1 + pPI->Kp*(pPI->Ek - pPI->Esk_1+(pPI->Ts/(2*pPI->Ti))*(pPI->Ek + pPI->Esk_1));
		/*Limit output*/ 
		if(pPI->Ik > pPI->Imax)
				pPI->Isk = pPI->Imax;
		else if(pPI->Ik < pPI->Imin)
				pPI->Isk = pPI->Imin;
		else
				pPI->Isk = pPI->Ik;
		/*Update data*/
		pPI->Ek_1 = pPI->Ek;
		pPI->Esk_1 = pPI->Esk;
		pPI->Ik_1 = pPI->Ik;
		pPI->Isk_1 = pPI->Isk;
		return pPI->Isk;
//		pPI->Ek = SP - FB;
//		pPI->Ik = pPI->Kp*pPI->Ek;
//		if(pPI->Ik > pPI->Imax)
//				pPI->Isk = pPI->Imax;
//		else if(pPI->Ik < pPI->Imin)
//				pPI->Isk = pPI->Imin;
//		else
//				pPI->Isk = pPI->Ik;
//		return pPI->Isk;

	}
//========================================================================================================================
//	/*Closed Speed Loop*/ 
	void SPEED_LOOP(CONTROL_OBJECT *pCon, PI_OBJECT *pPI, float rpm_sp)	{
		pCon->Id_sp = 0; // setpoint
		
		pCon->rpm_sp = rpm_sp;
		pCon->w_sp = (double)pCon->rpm_sp/9.5493;
		pCon->T_sin = 60.0/((double)pCon->rpm_sp*50);
		pCon->f_sin = 1.0/ pCon->T_sin;
		pCon->k_max = pCon->T_sin/pCon->Ts+0.01;
		
		pCon->Iq_sp = IP_Run(&*pPI, pCon->rpm_sp, encoder_var.rpm_fb);
		//pCon->Iq_sp = 4;
	}

//========================================================================================================================
	/*-------------THE CURRENT CLOSED LOOP-------------*/ 
	void CURRENT_LOOP(CONTROL_OBJECT *pCon, PI_OBJECT *pPI_Id, PI_OBJECT *pPI_Iq){		
		pCon->theta_e = 50*CAL_THETA(&theta_var, pCon->cnt_k);  // theta_electric = 50*theta_mechanical 
		//pCon->theta_e = 0;
		
		/*Park transfer*/
		pCon->Id_fb = adc_var.Ia_fb*cos(pCon->theta_e) + adc_var.Ib_fb*sin(pCon->theta_e);
		pCon->Iq_fb = -adc_var.Ia_fb*sin(pCon->theta_e) + adc_var.Ib_fb*cos(pCon->theta_e);
		/*PI Controller*/
		pCon->Ud = PI_Run(&*pPI_Id, pCon->Id_sp, pCon->Id_fb);			// Id_sp = const = 0
		pCon->Uq = PI_Run(&*pPI_Iq, pCon->Iq_sp, pCon->Iq_fb);		
		/*Clark transfer*/
		pCon->Ua = pCon->Ud*cos(pCon->theta_e) - pCon->Uq*sin(pCon->theta_e);
		pCon->Ub = pCon->Ud*sin(pCon->theta_e) + pCon->Uq*cos(pCon->theta_e);
		/*Cal dutycycle*/
		pCon->dutyA = (int)((pCon->Ua/hsm_var.V_max+1)*pCon->duty_max/2);
		pCon->dutyB = (int)((pCon->Ub/hsm_var.V_max+1)*pCon->duty_max/2);
		/*Put duty of timer pulse generation*/
		htim1.Instance->CCR1 = pCon->dutyA;		
		htim1.Instance->CCR2 = pCon->dutyB;
		
		pCon->cnt_k++;		// next sin cycle
		if(pCon->cnt_k >= pCon->k_max)			// When finish a electric cycle
			pCon->cnt_k = 0;				
		
	}
	
//========================================================================================================================
	/*----------THIS IS THE OPEN LOOP OF CURRENT USE FOR TEST------------*/ 
	void CURRENT_OPENLOOP(CONTROL_OBJECT *pCon, ADC_OBJECT *pADC, float Uq){   
		pCon->Uq = Uq;

		pCon->theta_e = 2*PI*pCon->f_sin*pCon->cnt_k*pCon->Ts;		// cal theta electric theta = w.t = 2.Pi.f.k.Ts

		pCon->theta_m = CAL_THETA(&theta_var, pCon->cnt_k);

		pCon->Ua = pCon->Ud*cos(pCon->theta_e) - pCon->Uq*sin(pCon->theta_e);
		pCon->Ub = pCon->Ud*sin(pCon->theta_e) + pCon->Uq*cos(pCon->theta_e);	
		
		pCon->dutyA = (int)((pCon->Ua/hsm_var.V_max+1)*pCon->duty_max/2);
		pCon->dutyB = (int)((pCon->Ub/hsm_var.V_max+1)*pCon->duty_max/2);
		//SVPWM_RUN(&control_var, &svpwm_var, Uq);
		
		htim1.Instance->CCR1 = pCon->dutyA;		
		htim1.Instance->CCR2 = pCon->dutyB;
		
		pCon->cnt_k++;
		if(pCon->cnt_k >= pCon->k_max)	pCon->cnt_k = 0;
	}	
//========================================================================================================================
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
//========================================================================================================================	
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
	
//pCon->dutyA = 2100*((-pSV->t10 - pSV->t20 + pSV->t11 + pSV->t21)/(25e-6)+1);
//pCon->dutyB = 2100*((pSV->t10 - pSV->t20 - pSV->t11 + pSV->t21)/(25e-6)+1);


//pCon->Ua = 12.0;
//pCon->Ub = 12.0;
//float kUmax = 0.95; 	//Use 20% of V_max
//if(pCon->Ua > kUmax*hsm_var.V_max)
//pCon->Ua = kUmax*hsm_var.V_max;
//if(pCon->Ub > kUmax*hsm_var.V_max)
//pCon->Ub = kUmax*hsm_var.V_max;
//if(pCon->Ua < -kUmax*hsm_var.V_max)
//pCon->Ua = -kUmax*hsm_var.V_max;
//if(pCon->Ub < -kUmax*hsm_var.V_max)
//pCon->Ub = -kUmax*hsm_var.V_max;

	
//============================= TEST HERE===========================================================================================
//	/*MACH VONG HO*/ 
//	void OPEN_LOOP_U(CONTROL_OBJECT *pCon, uint16_t rpm_sp, DIR dir, float U_hold){
//		pCon->U_hold = U_hold;
//		pCon->rpm_SP = (float)rpm_sp;		// toc do dat
//		pCon->motor_dir = dir;		// xet chieu quay
//		pCon->delta_us = ((60.0/pCon->rpm_SP)*(1000000.0/pCon->num_ppr));
//		
//		float theta;
//		theta = 2*PI*((float)pCon->cnt)/((float)pCon->k) ;
//		
//		if(pCon->motor_dir == RL){
//			pCon->Ua = pCon->U_hold*cos(theta+PI/4);
//			pCon->Ub = pCon->U_hold*sin(theta+PI/4);
//		}
//		else if(pCon->motor_dir == LR){
//			pCon->Ua = pCon->U_hold*cos(-theta-PI/4);
//			pCon->Ub = pCon->U_hold*sin(-theta-PI/4);
//		}
//		pCon->dutyA = pCon->duty_zero + (pCon->Ua*2100.0)/hsm_var.V_max;
//		pCon->dutyB = pCon->duty_zero + (pCon->Ub*2100.0)/hsm_var.V_max;
//		
//		htim1.Instance->CCR1 = pCon->dutyA;
//		htim1.Instance->CCR2 = pCon->dutyB;
//		pCon->cnt++;
//		if(pCon->cnt >= pCon->k) pCon->cnt = 0;
//		
////		if(pCon->delta_us > 60000){
////			for(int32_t i = pCon->delta_us; i>= 60000; i-=60000){
////				delay_us(60000);
////				delay_us(i - 60000);
////			}
////		}	
////		else delay_us(pCon->delta_us);
//		
//		delay_us(pCon->delta_us);
//		
//		//CAL_SPEED(&encoder_var, pCon->delta_us/1000000.0);
//		
//	}
//	

//OPEN_LOOP_U(&control_var, 60, RL, 12);				/*OPEN_LOOP(variable control, speed (v/p), direction, voltage(0-24V))*/
//OPEN_LOOP_U(&control_var, 6, LR, 5);				/*OPEN_LOOP(variable control, speed (v/p), direction, voltage(0-24V))*/
//========================================================================================================================
//	void OPEN_LOOP(CONTROL_OBJECT *pCon, uint16_t rpm_sp, DIR dir, uint16_t delta_duty_hold){
//		pCon->rpm_SP = (float)rpm_sp;		// toc do dat
//		pCon->motor_dir = dir;		// xet chieu quay
//		pCon->delta_duty_hold = delta_duty_hold;
//		pCon->delta_us = ((60.0/pCon->rpm_SP)*(1000000.0/pCon->num_ppr));
//		float theta;
//		theta = 2*PI*((float)pCon->cnt)/((float)pCon->k);
//		if(pCon->motor_dir == RL){
//			pCon->dutyA = pCon->duty_zero + pCon->delta_duty_hold*cos(theta);
//			pCon->dutyB = pCon->duty_zero + pCon->delta_duty_hold*sin(theta);
//		}
//		else if(pCon->motor_dir == LR){
//			pCon->dutyA = pCon->duty_zero + pCon->delta_duty_hold*cos(-theta);
//			pCon->dutyB = pCon->duty_zero + pCon->delta_duty_hold*sin(-theta);
//		}
//		htim1.Instance->CCR1 = pCon->dutyA;
//		htim1.Instance->CCR2 = pCon->dutyB;
//		pCon->cnt++;
//		if(pCon->cnt >= pCon->k) pCon->cnt = 0;
//		delay_us(pCon->delta_us);
//		//delay_us(2500);


//	}
