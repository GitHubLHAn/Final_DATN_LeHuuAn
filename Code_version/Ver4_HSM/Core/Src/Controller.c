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


const double sin_table[80] = {0,0.078460, 0.15643, 0.23345, 0.30902, 0.38268, 0.45399, 0.52250, 0.58779, 0.64945, 0.70711,
															0.76041, 0.80902, 0.85264, 0.89101, 0.92388, 0.95106, 0.97237, 0.98769, 0.99692, 1,	0.99692, 0.98769,
															0.97237, 0.95106, 0.92388, 0.89101, 0.85264, 0.80902, 0.76041, 0.70711, 0.64945, 0.58779, 0.52250,
															0.45399, 0.38268, 0.30902, 0.23345, 0.15643, 0.078460, 0, -0.078460, -0.15643, -0.23345, -0.30902,
															-0.38268, -0.45399, -0.52250, -0.58779, -0.64945, -0.70711, -0.76041, -0.80902, -0.85264, -0.89101,
															-0.92388, -0.95106, -0.97237, -0.98769,-0.99692, -1, - 0.99692, -0.98769, -0.97237, -0.95106, -0.92388,
															-0.89101, -0.85264, -0.80902, -0.76041, -0.70711, -0.64945, -0.58779, -0.52250, -0.45399,-0.38268,
															-0.30902, -0.23345, -0.15643, -0.078460};
const double cos_table[80] = {1,0.99692, 0.98769, 0.97237, 0.95106, 0.92388, 0.89101, 0.85264, 0.80902, 0.76041, 0.70711,
															0.64945, 0.58779, 0.52250, 0.45399, 0.38268, 0.30902, 0.23345, 0.15643, 0.078460, 0,-0.078460, 
															-0.15643, -0.23345, -0.30902, -0.38268, -0.45399, -0.52250, -0.58779, -0.64945, -0.70711, -0.76041,
															-0.80902, -0.85264, -0.89101, -0.92388, -0.95106, -0.97237, -0.98769, -0.99692, -1, -0.99692, -0.98769, 
															-0.97237, -0.95106, -0.92388, -0.89101, -0.85264, -0.80902, -0.76041, -0.70711, -0.64945, -0.58779,
															-0.52250, -0.45399, -0.38268, -0.30902, -0.23345, -0.15643, -0.078460, 0,0.078460, 0.15643, 0.23345,
															0.30902, 0.38268, 0.45399, 0.52250, 0.58779, 0.64945, 0.70711, 0.76041, 0.80902, 0.85264, 0.89101, 
															0.92388, 0.95106, 0.97237, 0.98769, 0.99692};

/*_________________________________________________________________________________________________________________________*/
	/*Initiate the controller parameters*/
	void CONTROLLER_Init(CONTROL_OBJECT *pCon, double Ts){
		pCon->Id_sp = 0;			pCon->Id_fb = 0;
		pCon->Iq_sp = 0;			pCon->Iq_fb = 0;
		pCon->EBF_d = 0;			pCon->EBF_q = 0;

		pCon->Ud = 0; 				pCon->Uq = 0; 
		pCon->Ua = 0; 				pCon->Ub = 0;

		pCon->theta_e = 0;		pCon->theta_m = 0;		pCon->pul_theta_e = 0;
		pCon->cos_theta = 0;	pCon->sin_theta = 0;	pCon->ptr_theta = 0;
		pCon->Ts = Ts;// 50us
		pCon->T_sin = 0;			pCon->f_sin = 0;
		pCon->k_max = 0; 			pCon->cnt_k = 0;

		pCon->rpm_sp = 0;			pCon->w_sp = 0;

		pCon->motor_dir = CCW;
		pCon->V_ref = 0;
		pCon->duty_max = htim1.Instance->ARR; // AutoReload 
		pCon->dutyA = pCon->duty_max/2; 		// put 0V to phase A
		pCon->dutyB = pCon->duty_max/2;		// put 0V to phase B
	}
/*_________________________________________________________________________________________________________________________*/
	/*Initiate the speed PI controller*/ 
	void PI_SPEED_Init(PI_OBJECT *pPI, double Ts)
	{
		pPI->Ek = 0; pPI->Ek_1 = 0; pPI->Esk = 0; pPI->Esk_1 = 0;			// error
		pPI->Ik = 0; pPI->Ik_1 = 0; pPI->Isk = 0; pPI->Isk_1 = 0;		// current output
		pPI->Imax = 4;
		pPI->Imin = -4;
		pPI->Ts = Ts; 			//20ms
		pPI->Kp = 0.015; 					//0.1*pPI->Imax/3.14;   // 0.5*Imax/Emax 
		pPI->Ti = 0.5; 
		pPI->Ka = pPI->Kp;
		pPI->Uk = NULL; pPI->Uk_1 = NULL; pPI->Usk = NULL; pPI->Usk_1 = NULL; pPI->Umax = NULL; pPI->Umin = NULL; // not use
	}
/*_________________________________________________________________________________________________________________________*/
	/*Initiate the current PI controller*/ 
	void PI_CURRENT_Init(PI_OBJECT *pPI, double Ts)				//CURRENT PI
	{		
		pPI->Ek = 0; pPI->Ek_1 = 0; pPI->Esk = 0; pPI->Esk_1 = 0;
		pPI->Uk = 0; pPI->Uk_1 = 0; pPI->Usk = 0; pPI->Usk_1 = 0;
		pPI->Umax = 24;
		pPI->Umin = -24; 
		pPI->Ts = Ts; 
		pPI->Kp = 1.6	;		//0.4*pPI->Umax/hsm_var.I_max;		// 1.6					// 0.4*Umax/Emax as Emax = Imax		
		pPI->Ti = 0.009;			 // tuned
		pPI->Ka = pPI->Kp;
		pPI->Ik = NULL; pPI->Ik_1 = NULL; pPI->Isk = NULL; pPI->Isk_1 = NULL; pPI->Imax = NULL; pPI->Imin = NULL;  // not use
	}
/*_________________________________________________________________________________________________________________________*/
	/*RUN PI CONTROLLER*/ 
	float PI_Run(PI_OBJECT *pPI, double SP, double FB)				// for current loop
	{	
		pPI->Ek = SP - FB;
		/*Error bo chong bao hoa tich phan*/ 
		pPI->Esk_1 = pPI->Ek_1 + (1/pPI->Ka)*(pPI->Usk_1 - pPI->Uk_1);		
		/*Phuong phap gian doan hoa (xap xi khau tich bang pp hinh thang), su dung toan tu lui ZOH*/
		pPI->Uk = pPI->Usk_1 + pPI->Kp*(pPI->Ek - pPI->Esk_1*(1-pPI->Ts/pPI->Ti));

		/*Limit output*/ 
		if(pPI->Uk > pPI->Umax)
				pPI->Usk = pPI->Umax;
		else if(pPI->Uk < pPI->Umin)
				pPI->Usk = pPI->Umin;
		else
				pPI->Usk = pPI->Uk;

		/*Update data*/
		pPI->Ek_1 = pPI->Ek;
		pPI->Uk_1 = pPI->Uk;
		pPI->Usk_1 = pPI->Usk;

		return pPI->Usk;	
	}
	//--------------------------------------------------------------------------
	/*RUN IP CONTROLLER for speed loop*/
	float IP_Run(PI_OBJECT *pPI, double SP, double FB){						// for speed loop
		
//		pPI->Ek = SP - FB;
//		/*Error bo chong bao hoa tich phan*/ 
//		pPI->Esk_1 = pPI->Ek_1 + (1/pPI->Ka)*(pPI->Isk_1 - pPI->Ik_1);		
//		/*Phuong phap gian doan hoa (xap xi khau tich bang pp hinh thang), su dung toan tu lui ZOH*/
//		pPI->Ik = pPI->Isk_1 + pPI->Kp*(pPI->Ek - pPI->Esk_1*(1-pPI->Ts/pPI->Ti));

//		/*Limit output*/ 
//		if(pPI->Ik > pPI->Imax)
//				pPI->Isk = pPI->Imax;
//		else if(pPI->Ik < pPI->Imin)
//				pPI->Isk = pPI->Imin;
//		else
//				pPI->Isk = pPI->Ik;

//		/*Update data*/
//		pPI->Ek_1 = pPI->Ek;
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
	void SPEED_LOOP(CONTROL_OBJECT *pCon, PI_OBJECT *pPI, float rpm_sp)
	{
		//pCon->rpm_sp = rpm_sp;
		pCon->rpm_sp = RAMP_RUN(rpm_sp, &ramp_rpm_var);
		pCon->Id_sp = 0;
		pCon->Iq_sp = IP_Run(&*pPI, pCon->rpm_sp, encoder_var.rpm_fb);
		mState_var.flag_set_rpm = 1;
	}

/*_________________________________________________________________________________________________________________________*/
	/*-------------THE CURRENT CLOSED LOOP-------------*/ 
	void CURRENT_CLOSELOOP(CONTROL_OBJECT *pCon, PI_OBJECT *pPI_Id, PI_OBJECT *pPI_Iq)
	{		
		if(mState_var.flag_set_rpm)
		{
			pCon->pul_theta_e = CAL_DELTA_PULSE(&theta_var);  // use value table
			while(pCon->pul_theta_e >= 80)
					pCon->pul_theta_e = pCon->pul_theta_e - 80;
		}
		else 
			pCon->pul_theta_e = 0;				// when speedloop has not operated yet 
		
		pCon->ptr_theta = pCon->pul_theta_e;	
		pCon->cos_theta = cos_table[pCon->ptr_theta];
		pCon->sin_theta = sin_table[pCon->ptr_theta]; 
		
		/*Park transfer*/
		pCon->Id_fb = adc_var.Ia_fb*pCon->cos_theta + adc_var.Ib_fb*pCon->sin_theta;
		pCon->Iq_fb = -adc_var.Ia_fb*pCon->sin_theta + adc_var.Ib_fb*pCon->cos_theta;
		/*PI Controller*/
		pCon->Ud = PI_Run(&*pPI_Id, pCon->Id_sp, pCon->Id_fb);			// Id_sp = const = 0
		pCon->Uq = PI_Run(&*pPI_Iq, pCon->Iq_sp, pCon->Iq_fb);		
		/*Clark transfer*/
		pCon->Ua = pCon->Ud*pCon->cos_theta - pCon->Uq*pCon->sin_theta;
		pCon->Ub = pCon->Ud*pCon->sin_theta + pCon->Uq*pCon->cos_theta;
		

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
		pCon->T_sin = 1.2/pCon->rpm_sp;
		pCon->f_sin = 1.0/ pCon->T_sin;
		pCon->k_max = pCon->T_sin/pCon->Ts+0.01;
	
		pCon->theta_e = 2*pCon->f_sin*pCon->cnt_k*pCon->Ts;				// cal theta electric theta = w.t = 2.Pi.f.k.Ts
		pCon->pul_theta_e = (int)(pCon->theta_e*40);						// convert to pulse
		
		pCon->Ua = pCon->Ud*cos_table[pCon->pul_theta_e] - pCon->Uq*sin_table[pCon->pul_theta_e];
		pCon->Ub = pCon->Ud*sin_table[pCon->pul_theta_e] + pCon->Uq*cos_table[pCon->pul_theta_e];	

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
	

