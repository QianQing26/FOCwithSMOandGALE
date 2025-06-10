/*
 * smo.h
 *
 *  Created on: Jun 8, 2025
 *      Author: 79489
 */

#ifndef APPLICATION_USER_SMO_H_
#define APPLICATION_USER_SMO_H_

#include "pid_regulator.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "speed_pos_fdbk.h"
#include "sto_speed_pos_fdbk.h"
#include <stdint.h>

typedef struct
{
    int16_t hF1;
    int16_t hF2;
    int16_t hC1;
    int16_t hC2;
    int16_t hC3;
    int16_t hC4;
    int16_t hC5;
    int16_t hC6;
    int16_t F1LOG;
    int16_t F2LOG;
    int16_t F3POW2;
    int16_t hF3;
    int16_t hForcedDirection;

    int32_t Ialfa_est;
    int32_t Ibeta_est;
    int32_t wBemf_alfa_est;
    int32_t wBemf_beta_est;

    int16_t hBemf_alfa_est;
    int16_t hBemf_beta_est;

    int16_t hElSpeedDpp;
    int16_t InstantaneousElSpeedDpp;
    int16_t hElAngle;

    PID_Handle_t PIRegulator;

    int16_t Speed;


} SMO_Handle_t;

int16_t saturation(int16_t err);

void SMO_InitFromSTO(SMO_Handle_t *pSMO, const STO_PLL_Handle_t *pSTO);
void SMO_CopyFromSTO(SMO_Handle_t *pSMO, const STO_PLL_Handle_t *pSTO);
int16_t SMO_PLL_CalcSpeed(SMO_Handle_t *pHandle, Observer_Inputs_t *pInputs);
int16_t SMO_ExecutePLL(SMO_Handle_t *pHandle, int16_t hBemf_alfa_est, int16_t hBemf_beta_est);
int16_t SMO_GetSpeed(SMO_Handle_t *pHandle);

#endif /* APPLICATION_USER_SMO_H_ */
