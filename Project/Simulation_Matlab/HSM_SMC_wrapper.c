
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>

#define PI 3.14159265
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 4

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
unsigned int flag_PIs, Ts_s_max;
double rpm_fb;
double rpm_sp;

/*---------- Motor pars ----------*/
double R, L, Ts, Vdc;
double RPM_max;

/*---------- Sliding mode control ----------*/
double lamda;
double erIak, erIbk;
double Vak, Vbk, Vak_1, Vbk_1;
double Iak_ref, Ibk_ref;
double Iak, Ibk, Iak_1, Ibk_1;
double EMFak, EMFbk;
double theta;

/*---------- Pars of Speed controller ----------*/
double Kps;
double Tis;
double isk_1, ik_1, ik;
double ek, esk_1, ek_1;
double imax, imin;
double Iq_ref;

/*---------- SVPWM ----------*/
double angle, sector;
double t1, t2, t5;

/*---------- Low pass filter ----------*/
typedef struct
{
 double Fc; 		//conrner frequency
 double a1, a2, b1, b2;	//parameters of the filter
 double alpha;
}LPF;
LPF LPFpar;
LPF LPFSpd;

typedef struct
{
  double yk, yk_1, yk_2, uk, uk_1;
}LPFio;
static LPFio LPFa;	//Perturbation Filter
static LPFio LPFb;
static LPFio LPFw;

double temp1, temp2;
double Temp1, Temp2;

/*---------- RAMP ----------*/
typedef enum
{
  ACC, RUN, DEC
}RAMP_STATE;

typedef struct
{
  double inputMax, inputMin;
  int T_Acc, T_Dec;
  int set_AccCnt, set_DecCnt;
  double delta_Acc, delta_Dec;
  double SPk, Output;
  RAMP_STATE State;
}RAMP;
RAMP ramp_rpm_var;

void RAMP_Init(RAMP *pRamp, int Tacc_Sec, int Tdec_Sec, double inputMin, double inputMax, double Ts)
{
pRamp->inputMax = inputMax;
pRamp->inputMin = inputMin;
pRamp->T_Acc = Tacc_Sec;
pRamp->T_Dec = Tdec_Sec;
		
//Compute the set count - 10ms resolution
pRamp->set_AccCnt = pRamp->T_Acc/Ts;	
pRamp->set_DecCnt = pRamp->T_Dec/Ts;
//Compute the step size
pRamp->delta_Acc = (double)(pRamp->inputMax - pRamp->inputMin)/pRamp->set_AccCnt;
pRamp->delta_Dec = (double)(pRamp->inputMax - pRamp->inputMin)/pRamp->set_DecCnt;

pRamp->SPk = 0;
pRamp->Output = 60;
}

double RAMP_RUN(double SP, RAMP *pRamp)
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

/*---------- PLL ----------*/
double pk, pk_1, theta_k, theta_k_1;
double psk, psk_1, Wk, Wk_1;
double Kp_pll, Ti_pll;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void HSM_SMC_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
Ts_s_max = 10;     // 20ms
flag_PIs = Ts_s_max - 1;
/* Motor pars */
R = 2.1;        L = 4.2e-3;
RPM_max = 120;  imax = 6;
Ts = 5e-5;
Vdc = 24;

/* Compute speed controller */
//Kps = 0.2*imax/RPM_max;
Kps = 0.55*imax/RPM_max;
Tis = 0.034;
isk_1 = 0; ik_1 = 0; ik = 0;
ek = 0; esk_1 = 0; ek_1 = 0;

/* Sliding mode control */
lamda = 0.1;
EMFak = 0;
EMFbk = 0;
erIak = 0;      erIbk = 0;
Iak_ref = 0;    Ibk_ref = 0;
Iak = 0;    Iak_1 = 0;  Ibk_1 = 0;  Ibk = 0;
Vak = 0;    Vak_1 = 0;  Vbk_1 = 0;  Vbk = 0;
theta = 0;

/* RAMP */
RAMP_Init(&ramp_rpm_var, 2, 2, -100, 100, 5e-4);

/* Low pass filter */
// Fc for back-emf estimation
LPFpar.Fc = 300;

// Fc for speed estimation
LPFSpd.Fc = 50;
LPFSpd.alpha = 1/(2*PI*Ts*LPFSpd.Fc);
Temp1 = (1+LPFSpd.alpha); Temp2 = Temp1*Temp1;
LPFSpd.a1 = (1+2*LPFSpd.alpha)/Temp2; LPFSpd.a2 = -2*LPFSpd.alpha/Temp2;
LPFSpd.b1 = -2*LPFSpd.alpha/Temp1; LPFSpd.b2 = LPFSpd.alpha*LPFSpd.alpha/Temp2;

LPFa.yk_1 = 0;
LPFa.uk_1 = 0;
LPFb.yk_1 = 0; 
LPFb.uk_1 = 0;
LPFw.yk_1 = 0;
LPFw.uk_1 = 0;

/* PLL */
pk = 0;     pk_1 = 0;   theta_k = 0;    theta_k_1 = 0;
psk = 0;    psk_1 = 0;  Wk = 0;        Wk_1 = 0;  
Kp_pll = 150; 
Ti_pll = 0.1;
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void HSM_SMC_Outputs_wrapper(const real_T *RPM_sp,
			const real_T *RPM_fb,
			const real_T *Ia_fb,
			const real_T *Ib_fb,
			real_T *m,
			real_T *ramp,
			real_T *Theta_est,
			real_T *Iq_sp,
			real_T *RPM_est)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/*-------------------------------- Speed controller --------------------------------*/
if(RPM_fb[0]>45)    // Closed loop
{
    rpm_sp = RAMP_RUN(RPM_sp[0], &ramp_rpm_var);
    ramp[0] = rpm_sp;
    flag_PIs++;
    if(flag_PIs == Ts_s_max)
    {
        ek = rpm_sp - LPFw.yk*9.55/50;
        esk_1 = ek_1 + (1/Kps)*(isk_1 - ik_1);
        ik = isk_1 + Kps*(ek - (1-Ts/Tis)*esk_1);

        if(ik > imax)       Iq_ref = imax;
        else if(ik < -imax) Iq_ref = -imax;
        else                Iq_ref = ik;

        ek_1 = ek;
        ik_1 = ik;
        isk_1 = Iq_ref;
        flag_PIs = 0;
        Iq_sp[0] = Iq_ref;
    }
}
else            // Open loop
{
    Iq_ref = 0.4;
    Iq_sp[0] = Iq_ref;
}

/*-------------------------------- Set current ref --------------------------------*/
Iak_ref = -Iq_ref*sin(theta_k);
Ibk_ref = Iq_ref*cos(theta_k);

Iak = Ia_fb[0];
erIak = Iak_ref - Iak;
Ibk = Ib_fb[0];
erIbk = Ibk_ref - Ibk;

/*-------------------------------- Fuzzy logic --------------------------------*/
if(RPM_fb[0]>=0)
    rpm_fb = RPM_fb[0];
else
    rpm_fb = -RPM_fb[0];

if(rpm_fb>0 && rpm_fb<=100)
{
    LPFpar.Fc = 300;
}
if(rpm_fb>100 && rpm_fb<=200)
{
    LPFpar.Fc = 400;
}

LPFpar.alpha = 1/(2*PI*Ts*LPFpar.Fc);
temp1 = (1+LPFpar.alpha); temp2 = temp1*temp1;
LPFpar.a1 = (1+2*LPFpar.alpha)/temp2; LPFpar.a2 = -2*LPFpar.alpha/temp2;
LPFpar.b1 = -2*LPFpar.alpha/temp1; LPFpar.b2 = LPFpar.alpha*LPFpar.alpha/temp2;

/*-------------------------------- Disturbance estimation --------------------------------*/
// Phase A
EMFak = -(L/Ts)*Iak + Vak_1 + (L/Ts - R)*Iak_1;
LPFa.uk = EMFak; 
LPFa.yk = -LPFpar.b1*LPFa.yk_1 - LPFpar.b2*LPFa.yk_2 + LPFpar.a1*LPFa.uk + LPFpar.a2*LPFa.uk_1; 
LPFa.yk_2 = LPFa.yk_1; LPFa.yk_1 = LPFa.yk; LPFa.uk_1 = LPFa.uk;

// Phase B
EMFbk = -(L/Ts)*Ibk + Vbk_1 + (L/Ts - R)*Ibk_1;
LPFb.uk = EMFbk; 
LPFb.yk = -LPFpar.b1*LPFb.yk_1 - LPFpar.b2*LPFb.yk_2 + LPFpar.a1*LPFb.uk + LPFpar.a2*LPFb.uk_1; 
LPFb.yk_2 = LPFb.yk_1; LPFb.yk_1 = LPFb.yk; LPFb.uk_1 = LPFb.uk;

/*-------------------------------- Theta_e estimation --------------------------------*/
pk = -EMFak*cos(theta_k_1) - EMFbk*sin(theta_k_1);
Wk = Wk_1 + Kp_pll*(pk - (1-Ts/Ti_pll)*pk_1);
if(RPM_fb[0]>=0)
    theta_k = theta_k_1 + Ts*Wk_1;
else
    theta_k = theta_k_1 - Ts*Wk_1;

pk_1 = pk;
Wk_1 = Wk;
theta_k_1 = theta_k;

// LPF for speed est
LPFw.uk = Wk; 
LPFw.yk = -LPFSpd.b1*LPFw.yk_1 - LPFSpd.b2*LPFw.yk_2 + LPFSpd.a1*LPFw.uk + LPFSpd.a2*LPFw.uk_1; 
LPFw.yk_2 = LPFw.yk_1; LPFw.yk_1 = LPFw.yk; LPFw.uk_1 = LPFw.uk;

/*-------------------------------- Compute output voltage --------------------------------*/
//Phase A
Vak = (L/Ts)*(Iak_ref - lamda*erIak - (1-R*Ts/L)*Iak + (Ts/L)*LPFa.yk);

//Phase B
Vbk = (L/Ts)*(Ibk_ref - lamda*erIbk - (1-R*Ts/L)*Ibk + (Ts/L)*LPFb.yk);

/*-------------------------------- Saturation output voltage --------------------------------*/
if(Vak > 24.0)
    Vak = 24.0;
if(Vak < -24.0)
    Vak = -24.0;

if(Vbk > 24.0)
    Vbk = 24.0;
if(Vbk < -24.0)
    Vbk = -24.0;

/*-------------------------------- Update system state --------------------------------*/
Iak_1 = Iak;
Ibk_1 = Ibk;
Vak_1 = Vak;
Vbk_1 = Vbk;

/*-------------------------------- SVPWM --------------------------------*/
angle = atan2(Vbk,Vak);
if(angle>0 && angle<=PI/2)
    sector = 1;
else if(angle>PI/2 && angle<=PI)
    sector = 2;
else if(angle>-PI && angle<=-PI/2)
    sector = 3;
else
    sector = 4;

t1 = (Vak*Ts*pow(-1,sector+1))/Vdc;

if(sector > 2)
    t2 = -(Vbk*Ts)/Vdc;
else
    t2 = (Vbk*Ts)/Vdc;

t5 = (Ts - t1 - t2)/4;

if (sector == 1)
{
    m[0] = (t1 + 2*t5)/Ts;
    m[1] = (2*t5)/Ts;
    m[2] = (t1+t2+2*t5)/Ts;
    m[3] = (t1+2*t5)/Ts;
}
else if (sector == 2)
{
    m[0] = (2*t5)/Ts;
    m[1] = (t1+2*t5)/Ts;
    m[2] = (t1+t2+2*t5)/Ts;
    m[3] = (t1+2*t5)/Ts;
}
else if (sector == 3)
{
    m[0] = (t1 + 2*t5)/Ts;
    m[1] = (2*t5)/Ts;
    m[2] = (t1+2*t5)/Ts;
    m[3] = (t1+t2+2*t5)/Ts;
}
else 
{
    m[0] = (2*t5)/Ts;
    m[1] = (t1+2*t5)/Ts;
    m[2] = (t1+2*t5)/Ts;
    m[3] = (t1+t2+2*t5)/Ts;   
}

/*-------------------------------- Export data --------------------------------*/
Theta_est[0] = fmod(theta_k, 2*PI);
RPM_est[0] = Wk*9.55/50.0;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


