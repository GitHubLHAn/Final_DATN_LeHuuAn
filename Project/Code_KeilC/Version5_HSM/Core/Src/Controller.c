#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern ENCODER_OBJECT encoder_var, theta_var;
extern ADC_OBJECT adc_var;
extern CONTROL_OBJECT control_var;
extern HSM hsm_var;
extern FLAG flag_var;
extern StateMachine mState_var;
extern RAMP ramp_rpm_var;
extern SMC SMC_var;
extern LPF LPF_EMFa, LPF_EMFb, LPF_rpm_est;
extern SVPWM svpwm_var;


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
		pCon->dutyA = 0; 		// put 0V to phase A
		pCon->dutyB = 0;		// put 0V to phase B
	}
/*_________________________________________________________________________________________________________________________*/
	/*----------THIS IS THE OPEN LOOP OF CURRENT USE FOR TEST------------*/ 
	void OPEN_LOOP_Fcn(CONTROL_OBJECT *pCon, float Uq, float rpm_sp)
	{   
		pCon->Uq = Uq;
		pCon->rpm_sp = rpm_sp;
		pCon->T_sin = 1.2/pCon->rpm_sp;
		pCon->f_sin = 1.0/pCon->T_sin;
		pCon->k_max = (int)(pCon->T_sin/pCon->Ts + 0.001);
		
		pCon->theta_m  = CAL_THETA(&theta_var, NULL);
		
		pCon->theta_e = 2*PI*pCon->f_sin*pCon->cnt_k*pCon->Ts;		// cal theta electric theta = w.t = 2.Pi.f.k.Ts
		pCon->Ua = pCon->Ud*cal_cos(pCon->theta_e) - pCon->Uq*cal_sin(pCon->theta_e);
		pCon->Ub = pCon->Ud*cal_sin(pCon->theta_e) + pCon->Uq*cal_cos(pCon->theta_e);

		pCon->dutyA = (1 + pCon->Ua/24.0f)/2.0f;
		pCon->dutyB = (1 + pCon->Ub/24.0f)/2.0f;
		
		htim1.Instance->CCR1 = (int)(8400*pCon->dutyA);		
		htim1.Instance->CCR2 = (int)(8400*(1.0f - pCon->dutyA));
		htim1.Instance->CCR3 = (int)(8400*pCon->dutyB);		
		htim1.Instance->CCR4 = (int)(8400*(1.0f - pCon->dutyB));
		
		//SVPWM_RUN(&svpwm_var, pCon->Ua, pCon->Ub);			
		pCon->cnt_k++;
		if(pCon->cnt_k >= pCon->k_max)	pCon->cnt_k = 0;
	}	
/*_________________________________________________________________________________________________________________________*/
	
	void SMC_Init(SMC *pSMC)
	{
		pSMC->Ts = 5e-5; 
		pSMC->Ts_s_max = 100;	
		pSMC->flag_PIs = pSMC->Ts_s_max - 1;
		pSMC->Ra = hsm_var.Ra;		pSMC->La = hsm_var.La;
		pSMC->RPM_max = hsm_var.Speed_max;
		pSMC->imax = hsm_var.I_max;		pSMC->imin = -hsm_var.I_max;	
		pSMC->RPM_sp = 0;
		
		// PI para
		pSMC->Kps = 0.01f*pSMC->imax/pSMC->RPM_max;
		pSMC->Tis = 0.1f;
		pSMC->isk_1 = 0;		pSMC->ik_1 = 0; 	pSMC->ik = 0;	pSMC->isk = 0;
		pSMC->ek = 0;				pSMC->esk_1 = 0; 	pSMC->ek_1 = 0;
		
		pSMC->lamda = 0.92f;
		pSMC->EMFak_1 = 0;	pSMC->EMFbk_1 = 0; 
		pSMC->erIak = 0; 		pSMC->erIbk = 0;
		pSMC->Iak_ref = 0; 	pSMC->Ibk_ref = 0;		
		pSMC->Iak = 0;			pSMC->Iak_1 = 0;		pSMC->Ibk = 0;		pSMC->Ibk_1 = 0;
		pSMC->Vak = 0;			pSMC->Vak_1 = 0;		pSMC->Vbk = 0;		pSMC->Vbk_1 = 0;
		pSMC->theta_est = 0;	pSMC->theta_set = 0; pSMC->theta_atan2= 0;
		
		pSMC->k_cnt = 0;
		pSMC->Iq_ref = 0;
		pSMC->EMFa_est = 0;		pSMC->EMFb_est = 0;
		pSMC->Va_ref = 0;			pSMC->Vb_ref = 0; 
		pSMC->dutyA = 0; 			pSMC->dutyB = 0;
		
		pSMC->pk = 0;		 pSMC->pk_1 = 0;	 pSMC->theta_k = 0; 	pSMC->theta_k_1 = 0;
		pSMC->psk = 0;	 pSMC->psk_1 = 0;	 pSMC->Wk = 0;				pSMC->Wk_1 = 0;
		//pSMC->Kp_pll = 100;		 pSMC->Ti_pll = pSMC->Kp_pll/2000;
		pSMC->Kp_pll = 100;		 pSMC->Ti_pll = 0.1;
		
		pSMC->Direction = 1;
		pSMC->flag_speed = 0; pSMC->flag_fc = 0;
		pSMC->flag_trans = 0;
		pSMC->delta_theta = 0; pSMC->rpm_fb_est = 0, pSMC->rpm_fb_est_fil = 0;
	}
	
	float Speed_SMC_Loop(SMC *pSMC, int16_t RPM_sp)
	{
//		pSMC->flag_speed++;
//		if(pSMC->flag_speed < 600)
//			pSMC->RPM_sp = RAMP_RUN(150, &ramp_rpm_var);
//		else if(pSMC->flag_speed < 1200)
//			pSMC->RPM_sp = RAMP_RUN(100, &ramp_rpm_var);
//		else if(pSMC->flag_speed < 1800)
//			pSMC->RPM_sp = RAMP_RUN(120, &ramp_rpm_var);
//		else if(pSMC->flag_speed < 2400)
//			pSMC->RPM_sp = RAMP_RUN(60, &ramp_rpm_var);
//		else if(pSMC->flag_speed < 3000)
//			pSMC->RPM_sp = RAMP_RUN(125, &ramp_rpm_var);
//		else if(pSMC->flag_speed < 3600)
//			pSMC->RPM_sp = RAMP_RUN(80, &ramp_rpm_var);
//		else if(pSMC->flag_speed < 4200)
//			pSMC->RPM_sp = RAMP_RUN(110, &ramp_rpm_var);
		
		pSMC->RPM_sp = RAMP_RUN(RPM_sp, &ramp_rpm_var);

		
		//pSMC->ek = pSMC->RPM_sp - encoder_var.rpm_fb;
		pSMC->ek = pSMC->RPM_sp - pSMC->rpm_fb_est;

		pSMC->esk_1 = pSMC->ek_1 + (1/pSMC->Kps)*(pSMC->isk_1 - pSMC->ik_1);
		pSMC->ik = pSMC->isk_1 + pSMC->Kps*(pSMC->ek - (1-pSMC->Ts/pSMC->Tis)*pSMC->esk_1);

		if(pSMC->ik > pSMC->imax)       pSMC->isk  = pSMC->imax;
		else if(pSMC->ik < -pSMC->imax) pSMC->isk  = -pSMC->imax;
		else                pSMC->isk  = pSMC->ik;

		pSMC->ek_1 = pSMC->ek;
		pSMC->ik_1 = pSMC->ik;
		pSMC->isk_1 = pSMC->isk;
		
		return pSMC->isk;
				
	}
	
	void Est_Speed(SMC *pSMC, int32_t RPM_sp){
		// estimate speed
		pSMC->theta_atan2 = ApproxAtan2((float)(-pSMC->EMFa_est), (float)(pSMC->EMFb_est));
		if(RPM_sp >= 0){
			if(pSMC->theta_atan2 < 0 && pSMC->theta_atan2 > -PI)
				pSMC->theta_atan2 += 2*PI;
		}
		else{
			if(pSMC->theta_atan2 > 0 && pSMC->theta_atan2 < PI)
				pSMC->theta_atan2 -= 2*PI;
		}
		
		pSMC->theta_k = fabs(pSMC->theta_atan2/50.0f);
		
		if(pSMC->theta_k > pSMC->theta_k_1)
				pSMC->delta_theta = pSMC->theta_k - pSMC->theta_k_1;
		else
			pSMC->delta_theta = pSMC->theta_k + (2*PI/50 - pSMC->theta_k_1);
		
		pSMC->rpm_fb_est_fil = LPF_Run(&LPF_rpm_est, (int)((pSMC->delta_theta/(pSMC->Ts*pSMC->Ts_s_max))*9.5493f));
		if(RPM_sp >= 0){
			pSMC->rpm_fb_est = pSMC->rpm_fb_est_fil;
		}
		else{
			pSMC->rpm_fb_est = -pSMC->rpm_fb_est_fil;
		}
		
		//pSMC->rpm_fb_est = (pSMC->delta_theta/(pSMC->Ts*pSMC->Ts_s_max))*9.5493f;
		
		pSMC->theta_k_1 = pSMC->theta_k;
	
	}
	
	void SMC_OL_Run(SMC *pSMC, int16_t rpm_sp)
	{
		//pSMC->RPM_sp = rpm_sp;
		// open loop
		volatile float T_sin = 1.2f/fabs((float)(rpm_sp));
		volatile float f_sin = 1.0f/T_sin;
		volatile uint16_t k_max = (int)(T_sin/pSMC->Ts + 0.001);
		if(rpm_sp >= 0){
			pSMC->theta_set = 2*PI*f_sin*pSMC->k_cnt*pSMC->Ts;		// cal theta electric theta = w.t = 2.Pi.f.k.Ts		
		}				
		else{
			pSMC->theta_set = -2*PI*f_sin*pSMC->k_cnt*pSMC->Ts;
		}
		
		pSMC->k_cnt++;
		if(pSMC->k_cnt >= k_max)	pSMC->k_cnt = 0;
	}
//-----------------------------------------------------------------------------------------------------	
	
	void SMC_Fcn(SMC *pSMC, int16_t RPM_ref)
	{	
		//pSMC->RPM_sp = RAMP_RUN(RPM_ref, &ramp_rpm_var);
		//pSMC->RPM_sp = RPM_ref;
		
//		if(RPM_ref < 50 &&RPM_ref > -50){		// neu toc do dat nho hon 50 thi chay vong ho
//			pSMC->Iq_ref = 3.0;
//			SMC_OL_Run(&SMC_var, RPM_ref);	// open loop
//		}
//		else{
			if(pSMC->flag_trans == 0){

				if(fabs((float)(encoder_var.rpm_fb)) > 30 && pSMC->flag_PIs == pSMC->Ts_s_max-1){ 
						pSMC->flag_trans = 1;
				}
				else{
					pSMC->Iq_ref = 1;	
				}
			}

		//pSMC->Iq_ref = 5; 
		//SMC_OL_Run(&SMC_var, 60);	// open loop
		if(pSMC->flag_trans == 1)
			pSMC->theta_set = pSMC->theta_est;
		
		/*--------------- Set current ref -------------------*/
		pSMC->Iak_ref = -pSMC->Iq_ref*cal_sin(pSMC->theta_set);
		pSMC->Ibk_ref = pSMC->Iq_ref*cal_cos(pSMC->theta_set);
	
		pSMC->Iak = adc_var.Ia_fb;
		pSMC->erIak = pSMC->Iak_ref - pSMC->Iak;
		pSMC->Ibk = adc_var.Ib_fb;
		pSMC->erIbk = pSMC->Ibk_ref - pSMC->Ibk;
		/*-------------------------------- Disturbance estimation --------------------------------*/
		//volatile double La_dv_Ts = pSMC->La/pSMC->Ts;
		pSMC->EMFak_1 = -(pSMC->La/pSMC->Ts)*pSMC->Iak + pSMC->Vak_1 + (pSMC->La/pSMC->Ts - pSMC->Ra)*pSMC->Iak_1;
		pSMC->EMFbk_1 = -(pSMC->La/pSMC->Ts)*pSMC->Ibk + pSMC->Vbk_1 + (pSMC->La/pSMC->Ts - pSMC->Ra)*pSMC->Ibk_1;
		
		pSMC->EMFa_est = LPF_Run(&LPF_EMFa, pSMC->EMFak_1);
		pSMC->EMFb_est = LPF_Run(&LPF_EMFb, pSMC->EMFbk_1);
		
		//pSMC->theta_real = CAL_THETA(&theta_var, RPM_ref);
		
		/*-----------------------Theta_est from ATAN2---------------------------------------*/
		pSMC->theta_atan2 = ApproxAtan2((float)(-pSMC->EMFa_est), (float)(pSMC->EMFb_est));
		
		if(pSMC->theta_atan2 < 0 && pSMC->theta_atan2 > -PI)
				pSMC->theta_atan2 += 2*PI;

		pSMC->theta_est = pSMC->theta_atan2;
		
		pSMC->theta_set = pSMC->theta_est;

		/*--------------- Compute output voltage -----------------------*/
		//Phase A
		pSMC->Vak = (pSMC->La/pSMC->Ts)*(pSMC->Iak_ref - pSMC->lamda*pSMC->erIak - 
																						(1-pSMC->Ra*pSMC->Ts/pSMC->La)*pSMC->Iak + (pSMC->Ts/pSMC->La)*pSMC->EMFa_est);

		//Phase B   );//
		pSMC->Vbk = (pSMC->La/pSMC->Ts)*(pSMC->Ibk_ref - pSMC->lamda*pSMC->erIbk - 
																						(1-pSMC->Ra*pSMC->Ts/pSMC->La)*pSMC->Ibk + (pSMC->Ts/pSMC->La)*pSMC->EMFb_est);
		
		/*-------------- Saturation output voltage ---------------------*/
		if(pSMC->Vak > 25.0f)	pSMC->Va_ref = 25.0;
		else if(pSMC->Vak < -25.0f)	pSMC->Va_ref = -25.0f;
		else	pSMC->Va_ref = pSMC->Vak;
				
		if(pSMC->Vbk > 25.0f)	pSMC->Vb_ref = 25.0f;
		else if(pSMC->Vbk < -25.0f)	pSMC->Vb_ref = -25.0f;
		else	pSMC->Vb_ref = pSMC->Vbk;
		
		pSMC->dutyA = (1 + pSMC->Va_ref/25.0f)/2.0f;
		pSMC->dutyB = (1 + pSMC->Vb_ref/25.0f)/2.0f;
		
		htim1.Instance->CCR1 = (int)(8400*pSMC->dutyA);		
		htim1.Instance->CCR2 = (int)(8400*(1.0f - pSMC->dutyA));
		htim1.Instance->CCR3 = (int)(8400*pSMC->dutyB);		
		htim1.Instance->CCR4 = (int)(8400*(1.0f - pSMC->dutyB));
		
		
		/*-------------------------------- Update system state --------------------------------*/
		pSMC->Iak_1 = pSMC->Iak;
		pSMC->Ibk_1 = pSMC->Ibk;
		pSMC->Vak_1 = pSMC->Vak;
		pSMC->Vbk_1 = pSMC->Vbk;

	}
	
	void SVPWM_Init(SVPWM *pSV)
	{
			pSV->Vdc = 24.0f;
			pSV->sector = 0;;
			pSV->angle = 0;
			pSV->t1 = 0; pSV->t2 = 0; pSV->t5 = 0; 
			pSV->Ts = 5e-5;
			pSV->m1 = 0; pSV->m2 = 0; pSV->m3 = 0; pSV->m4 = 0;
	}
	
	
	void SVPWM_RUN(SVPWM *pSV,float Vak, float Vbk)
	{
		pSV->angle = ApproxAtan2(Vbk,Vak);
		if(pSV->angle>0 && pSV->angle<=PI/2)
				pSV->sector = 1;
		else if(pSV->angle>PI/2 && pSV->angle<=PI)
				pSV->sector = 2;
		else if(pSV->angle>-PI && pSV->angle<=-PI/2)
				pSV->sector = 3;
		else
				pSV->sector = 4;
	
		pSV->t1 = (Vak*pSV->Ts*pow(-1,pSV->sector+1))/pSV->Vdc;

		if(pSV->sector > 2)
				pSV->t2 = -(Vbk*pSV->Ts)/pSV->Vdc;
		else
				pSV->t2 = (Vbk*pSV->Ts)/pSV->Vdc;

		pSV->t5 = (pSV->Ts - pSV->t1 - pSV->t2)/4;

		if (pSV->sector == 1)
		{
				pSV->m1 = (pSV->t1 + 2*pSV->t5)/pSV->Ts;
				pSV->m2 = (2*pSV->t5)/pSV->Ts;
				pSV->m3 = (pSV->t1+pSV->t2+2*pSV->t5)/pSV->Ts;
				pSV->m4 = (pSV->t1+2*pSV->t5)/pSV->Ts;
		}
		else if (pSV->sector == 2)
		{
				pSV->m1 = (2*pSV->t5)/pSV->Ts;
				pSV->m2 = (pSV->t1+2*pSV->t5)/pSV->Ts;
				pSV->m3 = (pSV->t1+pSV->t2+2*pSV->t5)/pSV->Ts;
				pSV->m4 = (pSV->t1+2*pSV->t5)/pSV->Ts;
		}
		else if (pSV->sector == 3)
		{
				pSV->m1 = (pSV->t1 + 2*pSV->t5)/pSV->Ts;
				pSV->m2 = (2*pSV->t5)/pSV->Ts;
				pSV->m3 = (pSV->t1+2*pSV->t5)/pSV->Ts;
				pSV->m4 = (pSV->t1+pSV->t2+2*pSV->t5)/pSV->Ts;
		}
		else 
		{
				pSV->m1 = (2*pSV->t5)/pSV->Ts;
				pSV->m2 = (pSV->t1+2*pSV->t5)/pSV->Ts;
				pSV->m3 = (pSV->t1+2*pSV->t5)/pSV->Ts;
				pSV->m4 = (pSV->t1+pSV->t2+2*pSV->t5)/pSV->Ts;   
		}
		
		htim1.Instance->CCR1 = (int)(8400*pSV->m1);		
		htim1.Instance->CCR2 = (int)(8400*pSV->m2);	
		htim1.Instance->CCR3 = (int)(8400*pSV->m3);		
		htim1.Instance->CCR4 = (int)(8400*pSV->m4);	
	
	}

	
	



