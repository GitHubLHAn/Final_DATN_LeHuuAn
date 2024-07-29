
/*
 * Include Files
 *
 */
#include "simstruc.h"



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#define obF1k_1 xD[0]
#define obF2k_1 xD[1]
#define obVok_1 xD[2]
#define obILk_1 xD[3]
#define Uik_1   xD[4]
#define Vok_1   xD[5]
#define iLk_1   xD[6]

#define ILMAX 100
#define ILMIN -100
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
double L1[2][2], L2[2][2], Phi[2][2], Gamma[2][1];
double erX1k, erX2k; 
double ILk1_ref, ConstR, Lambda, Ui, Dtemp;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void T_TypePOb_Verify_Start_wrapper(real_T *xD,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
Lambda = 0.01;

L1[0][0] =  1.3047 ;  L1[0][1] = 3.6854; 
L1[1][0] =  -0.2027 ; L1[1][1] = 1.1777; 

L2[0][0] =  0.8304;   L2[0][1] =  0.0; 
L2[1][0] =  0.0;      L2[1][1] = 0.72; 

Phi[0][0] =  0.4777;    Phi[0][1] = 3.6854; 
Phi[1][0] = -0.2027;    Phi[1][1] = 0.4777; 

Gamma[0][0] = 208.9273; 
Gamma[1][0] = 81.0904;
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void T_TypePOb_Verify_Outputs_wrapper(const real_T *iL,
			const real_T *Vo,
			const real_T *Vo_refk1,
			const real_T *Vo_refk,
			const real_T *PWM,
			real_T *D,
			real_T *obFk,
			real_T *obXk,
			real_T *Iref,
			const real_T *xD,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/*Current command generator*/  
    ConstR = Gamma[1][0]/Gamma[0][0];
    ILk1_ref = ConstR*(Vo_refk1[0] + Lambda*Vo_refk[0]) + (Phi[1][1]-ConstR*Phi[0][1])*iL[0] 
            + (Phi[1][0] - ConstR*(Phi[0][0]+Lambda))*Vo[0] + obFk[1] - ConstR*obFk[0];
    if(ILk1_ref > ILMAX)
        ILk1_ref = ILMAX; 
    if(ILk1_ref < ILMIN)
        ILk1_ref = ILMIN; 
    /*Current control*/
    Ui = (1/Gamma[1][0])*(ILk1_ref - Phi[1][0]*Vo[0] - Phi[1][1]*iL[0] - obFk[1]);
    if(Ui > 1)
        Ui = 0.98;
    else if(Ui < -1)
        Ui = -0.98; 
    Iref[0] = ILk1_ref;
    D[0] = Ui; 
    //D[0] = 0.5*(Ui+1);
    /*Run the piOb*/
    erX1k = Vok_1-obVok_1;            //Voltage estimation error
    erX2k = iLk_1-obILk_1;            //Current estimation error

    obXk[0] = Phi[0][0]*obVok_1 + Phi[0][1]*obILk_1 + Gamma[0][0]*Uik_1 + L1[0][0]*erX1k + L1[0][1]*erX2k + obF1k_1;
    obXk[1] = Phi[1][0]*obVok_1 + Phi[1][1]*obILk_1 + Gamma[1][0]*Uik_1 + L1[1][0]*erX1k + L1[1][1]*erX2k + obF2k_1;
    obFk[0] = obF1k_1 + L2[0][0]*erX1k + L2[0][1]*erX2k; 
    obFk[1] = obF2k_1 + L2[1][0]*erX1k + L2[1][1]*erX2k;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void T_TypePOb_Verify_Update_wrapper(const real_T *iL,
			const real_T *Vo,
			const real_T *Vo_refk1,
			const real_T *Vo_refk,
			const real_T *PWM,
			real_T *D,
			real_T *obFk,
			real_T *obXk,
			real_T *Iref,
			real_T *xD,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/*Update system state*/
    obF1k_1 = obFk[0];
    obF2k_1 = obFk[1];
    obVok_1 = obXk[0];
    obILk_1 = obXk[1];
    //Uik_1 = D[0]*2-1; 
    Uik_1 = D[0];
    Vok_1 = Vo[0];
    iLk_1 = iL[0];
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}

