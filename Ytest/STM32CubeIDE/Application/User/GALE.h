/*
 * GALE.h
 *
 *  Created on: Jun 6, 2025
 *      Author: 79489
 */

#ifndef APPLICATION_USER_GALE_H_
#define APPLICATION_USER_GALE_H_

#include "kalman_filter.h"
#include "filter.h"
#include "GALE.h"
#include "welford.h"

#include "mc_interface.h"

#define FILTER_NUM 4

typedef struct{

	int16_t input_speed;

	kalman_filter _Super;

	float prev_input;
	float output;

	KalmanFilter kalman;
	LowPassFilter lpf;
	MovingAverageFilter maf;
	AdaLMSFilter lms;

	float u[FILTER_NUM]; // default weight
	float z[FILTER_NUM]; // filter output
	WelfordStats stats[FILTER_NUM];

	int16_t speed;
	bool initialized;
}GALE;


void GALE_Init(GALE* fushion);
float GALE_Update(GALE* fusion, float input);
void GALE_Reset(GALE* fusion);
int16_t GetGALESpeed(GALE* fusion);
int16_t GetGALEInput(GALE* fusion);

#endif /* APPLICATION_USER_GALE_H_ */
