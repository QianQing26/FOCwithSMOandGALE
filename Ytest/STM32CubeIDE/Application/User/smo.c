/*
 * smo.c
 *
 *  Created on: Jun 8, 2025
 *      Author: 79489
 */

#include "smo.h"
#include "pid_regulator.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "speed_pos_fdbk.h"
#include "sto_speed_pos_fdbk.h"

#include "mc_math.h"

#include <stdint.h>

void SMO_InitFromSTO(SMO_Handle_t *pSMO, const STO_PLL_Handle_t *pSTO){
    if ((pSMO == NULL) || (pSTO == NULL))
        return;

    pSMO->hF1 = pSTO->hF1;
    pSMO->hF2 = pSTO->hF2;
    pSMO->hC1 = pSTO->hC1;
    pSMO->hC2 = pSTO->hC2;
    pSMO->hC3 = pSTO->hC3;
    pSMO->hC4 = pSTO->hC4;
    pSMO->hC5 = pSTO->hC5;
    pSMO->hC6 = pSTO->hC6;
    pSMO->F1LOG = pSTO->F1LOG;
    pSMO->F2LOG = pSTO->F2LOG;
    pSMO->F3POW2 = pSTO->F3POW2;
    pSMO->hF3 = pSTO->hF3;
    pSMO->hForcedDirection = pSTO->hForcedDirection;

    pSMO->Ialfa_est = pSTO->Ialfa_est;
    pSMO->Ibeta_est = pSTO->Ibeta_est;
    pSMO->wBemf_alfa_est = pSTO->wBemf_alfa_est;
    pSMO->wBemf_beta_est = pSTO->wBemf_beta_est;
    pSMO->hBemf_alfa_est = pSTO->hBemf_alfa_est;
    pSMO->hBemf_beta_est = pSTO->hBemf_beta_est;

    pSMO->hElSpeedDpp = pSTO->_Super.hElSpeedDpp;
    pSMO->InstantaneousElSpeedDpp = pSTO->_Super.InstantaneousElSpeedDpp;
    pSMO->hElAngle = pSTO->_Super.hElAngle;

    pSMO->PIRegulator = pSTO->PIRegulator;
//    pSMO->Speed = 0;
    pSMO->Speed = 4 * pSMO->InstantaneousElSpeedDpp;
}

void SMO_CopyFromSTO(SMO_Handle_t *pSMO, const STO_PLL_Handle_t *pSTO)
{
    if ((pSMO == NULL) || (pSTO == NULL))
        return;

    pSMO->hF1 = pSTO->hF1;
    pSMO->hF2 = pSTO->hF2;
    pSMO->hC1 = pSTO->hC1;
    pSMO->hC2 = pSTO->hC2;
    pSMO->hC3 = pSTO->hC3;
    pSMO->hC4 = pSTO->hC4;
    pSMO->hC5 = pSTO->hC5;
    pSMO->hC6 = pSTO->hC6;
    pSMO->F1LOG = pSTO->F1LOG;
    pSMO->F2LOG = pSTO->F2LOG;
    pSMO->F3POW2 = pSTO->F3POW2;
    pSMO->hF3 = pSTO->hF3;
    pSMO->hForcedDirection = pSTO->hForcedDirection;

    pSMO->Ialfa_est = pSTO->Ialfa_est;
    pSMO->Ibeta_est = pSTO->Ibeta_est;
    pSMO->wBemf_alfa_est = pSTO->wBemf_alfa_est;
    pSMO->wBemf_beta_est = pSTO->wBemf_beta_est;
    pSMO->hBemf_alfa_est = pSTO->hBemf_alfa_est;
    pSMO->hBemf_beta_est = pSTO->hBemf_beta_est;

    pSMO->hElSpeedDpp = pSTO->_Super.hElSpeedDpp;
    pSMO->InstantaneousElSpeedDpp = pSTO->_Super.InstantaneousElSpeedDpp;
    pSMO->hElAngle = pSTO->_Super.hElAngle;
}

int16_t SMO_PLL_CalcSpeed(SMO_Handle_t *pHandle, Observer_Inputs_t *pInputs)
{
    int16_t retValue;

    if ((MC_NULL == pHandle) || (MC_NULL == pInputs))
    {
        retValue = 0;
    }
    else
    {
        int32_t wAux;
        int32_t wDirection;
        int32_t wIalfa_est_Next;
        int32_t wIbeta_est_Next;
        int32_t wBemf_alfa_est_Next;
        int32_t wBemf_beta_est_Next;
        int16_t hAux;
        int16_t hAux_Alfa;
        int16_t hAux_Beta;
        int16_t hIalfa_err;
        int16_t hIbeta_err;
        int16_t hRotor_Speed;
        int16_t hValfa;
        int16_t hVbeta;

        if (pHandle->wBemf_alfa_est > (((int32_t)pHandle->hF2) * INT16_MAX))
        {
            pHandle->wBemf_alfa_est = INT16_MAX * ((int32_t)pHandle->hF2);
        }
        else if (pHandle->wBemf_alfa_est <= (-INT16_MAX * ((int32_t)pHandle->hF2)))
        {
            pHandle->wBemf_alfa_est = -INT16_MAX * ((int32_t)pHandle->hF2);
        }
        else
        {
            /* Nothing to do */
        }
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est >> pHandle->F2LOG);
#else
        hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est / pHandle->hF2);
#endif

        if (pHandle->wBemf_beta_est > (INT16_MAX * ((int32_t)pHandle->hF2)))
        {
            pHandle->wBemf_beta_est = INT16_MAX * ((int32_t)pHandle->hF2);
        }
        else if (pHandle->wBemf_beta_est <= (-INT16_MAX * ((int32_t)pHandle->hF2)))
        {
            pHandle->wBemf_beta_est = (-INT16_MAX * ((int32_t)pHandle->hF2));
        }
        else
        {
            /* Nothing to do */
        }
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        hAux_Beta = (int16_t)(pHandle->wBemf_beta_est >> pHandle->F2LOG);
#else
        hAux_Beta = (int16_t)(pHandle->wBemf_beta_est / pHandle->hF2);
#endif

        if (pHandle->Ialfa_est > (INT16_MAX * ((int32_t)pHandle->hF1)))
        {
            pHandle->Ialfa_est = INT16_MAX * ((int32_t)pHandle->hF1);
        }
        else if (pHandle->Ialfa_est <= (-INT16_MAX * ((int32_t)pHandle->hF1)))
        {
            pHandle->Ialfa_est = -INT16_MAX * ((int32_t)pHandle->hF1);
        }
        else
        {
            /* Nothing to do */
        }

        if (pHandle->Ibeta_est > (INT16_MAX * ((int32_t)pHandle->hF1)))
        {
            pHandle->Ibeta_est = INT16_MAX * ((int32_t)pHandle->hF1);
        }
        else if (pHandle->Ibeta_est <= (-INT16_MAX * ((int32_t)pHandle->hF1)))
        {
            pHandle->Ibeta_est = -INT16_MAX * ((int32_t)pHandle->hF1);
        }
        else
        {
            /* Nothing to do */
        }

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        hIalfa_err = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
        hIalfa_err = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

        hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.alpha;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        hIbeta_err = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
        hIbeta_err = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif

        hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.beta;

        wAux = ((int32_t)pInputs->Vbus) * pInputs->Valfa_beta.alpha;
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        hValfa = (int16_t)(wAux >> 16); // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
        hValfa = (int16_t)(wAux / 65536);
#endif

        wAux = ((int32_t)pInputs->Vbus) * pInputs->Valfa_beta.beta;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        hVbeta = (int16_t)(wAux >> 16); // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
        hVbeta = (int16_t)(wAux / 65536);
#endif

        /*alfa axes observer*/
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        hAux = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
        hAux = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

        wAux = ((int32_t)pHandle->hC1) * hAux;
        wIalfa_est_Next = pHandle->Ialfa_est - wAux;

//        wAux = ((int32_t)pHandle->hC2) * hIalfa_err;
        wAux = ((int32_t)pHandle->hC2) * saturation(hIalfa_err);
        wIalfa_est_Next += wAux;

        wAux = ((int32_t)pHandle->hC5) * hValfa;
        wIalfa_est_Next += wAux;

        wAux = ((int32_t)pHandle->hC3) * hAux_Alfa;
        wIalfa_est_Next -= wAux;

//        wAux = ((int32_t)pHandle->hC4) * hIalfa_err;
        wAux = ((int32_t)pHandle->hC4) * saturation(hIalfa_err);
        wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        wAux = (int32_t)hAux_Beta >> pHandle->F3POW2; // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
        wAux = ((int32_t)hAux_Beta) / pHandle->hF3;
#endif

        wAux = wAux * pHandle->hC6;
        wAux = pHandle->hElSpeedDpp * wAux;
        wBemf_alfa_est_Next += wAux;

        /*beta axes observer*/
#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
        hAux = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
        hAux = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif

        wAux = ((int32_t)pHandle->hC1) * hAux;
        wIbeta_est_Next = pHandle->Ibeta_est - wAux;

//        wAux = ((int32_t)pHandle->hC2) * hIbeta_err;
        wAux = ((int32_t)pHandle->hC2) * saturation(hIbeta_err);
        wIbeta_est_Next += wAux;

        wAux = ((int32_t)pHandle->hC5) * hVbeta;
        wIbeta_est_Next += wAux;

        wAux = ((int32_t)pHandle->hC3) * hAux_Beta;
        wIbeta_est_Next -= wAux;

//        wAux = ((int32_t)pHandle->hC4) * hIbeta_err;
        wAux = ((int32_t)pHandle->hC4) * saturation(hIbeta_err);
        wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
        wAux = (int32_t)hAux_Alfa >> pHandle->F3POW2; // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
        wAux = ((int32_t)hAux_Alfa) / pHandle->hF3;
#endif

        wAux = wAux * pHandle->hC6;
        wAux = pHandle->hElSpeedDpp * wAux;
        wBemf_beta_est_Next -= wAux;

        /*Calls the PLL blockset*/
        pHandle->hBemf_alfa_est = hAux_Alfa;
        pHandle->hBemf_beta_est = hAux_Beta;

        if (0 == pHandle->hForcedDirection)
        {
            /* we are in auxiliary mode, then rely on the speed detected */
            if (pHandle->hElSpeedDpp >= 0)
            {
                wDirection = 1;
            }
            else
            {
                wDirection = -1;
            }
        }
        else
        {
            /* we are in main sensor mode, use a forced direction */
            wDirection = pHandle->hForcedDirection;
        }

        hAux_Alfa = (int16_t)(hAux_Alfa * wDirection);
        hAux_Beta = (int16_t)(hAux_Beta * wDirection);

        hRotor_Speed = SMO_ExecutePLL(pHandle, hAux_Alfa, -hAux_Beta);
        pHandle->InstantaneousElSpeedDpp = hRotor_Speed;
        pHandle->Speed = (int16_t)(3.9f * (float)hRotor_Speed);

        // STO_Store_Rotor_Speed(pHandle, hRotor_Speed);

        pHandle->hElAngle += hRotor_Speed;

        /*storing previous values of currents and bemfs*/
        pHandle->Ialfa_est = wIalfa_est_Next;
        pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

        pHandle->Ibeta_est = wIbeta_est_Next;
        pHandle->wBemf_beta_est = wBemf_beta_est_Next;
        // retValue = pHandle->hElAngle;
        retValue = hRotor_Speed;
    }
    return (retValue);
}

int16_t SMO_ExecutePLL(SMO_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est)
{
    int32_t wAlfa_Sin_tmp;
    int32_t wBeta_Cos_tmp;
    Trig_Components Local_Components;
    int16_t hAux1;
    int16_t hAux2;
    int16_t hOutput;

    Local_Components = MCM_Trig_Functions(pHandle->hElAngle);

    /* Alfa & Beta BEMF multiplied by Cos & Sin*/
    wAlfa_Sin_tmp = ((int32_t)hBemf_alfa_est) * ((int32_t)Local_Components.hSin);
    wBeta_Cos_tmp = ((int32_t)hBemf_beta_est) * ((int32_t)Local_Components.hCos);

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    hAux1 = (int16_t)(wBeta_Cos_tmp >> 15); // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    hAux1 = (int16_t)(wBeta_Cos_tmp / 32768);
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_PLL
    hAux2 = (int16_t)(wAlfa_Sin_tmp >> 15); // cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    hAux2 = (int16_t)(wAlfa_Sin_tmp / 32768);
#endif

    /* Speed PI regulator */
    hOutput = PI_Controller(&pHandle->PIRegulator, (int32_t)(hAux1)-hAux2);
    return (hOutput);
}


int16_t SMO_GetSpeed(SMO_Handle_t *pHandle){
//	return pHandle->InstantaneousElSpeedDpp;
	return pHandle->Speed;
}

int16_t saturation(int16_t err){
	err = err>20?20:err;
	err = err<-20?-20:err;
	err *= 5;
	return err;
}





