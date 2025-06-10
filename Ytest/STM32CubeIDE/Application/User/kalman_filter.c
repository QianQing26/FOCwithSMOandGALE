/*
 * kalman_filter.c
 *
 *  Created on: May 22, 2025
 *      Author: 79489
 */

#include "kalman_filter.h"
#include "mc_type.h"
#include "mc_config.h"
#include "mc_interface.h"
#include<math.h>
#include<stdint.h>

extern kalman_filter gSpeedFilter;

void kf_init(kalman_filter* kf, float Q, float R){
	kf->speed = 0.0f;
	kf->speed_int16 = (int16_t)0;
	kf->P = 1.0f;
	kf->Q = Q;
	kf->R = R;
	kf->K = 0.0f;
}

void kf_update(kalman_filter *kf, int16_t measurement){
	float m = (float)measurement;
	float tempR;
	tempR = kf->R;
	if(Mci[M1].State==START || Mci[M1].State==SWITCH_OVER){
		tempR = 0.001f;
	}
	//
	kf->P += kf->Q;
	//
	kf->K = kf->P / (kf->P + tempR);
	kf->speed += kf->K * (m - kf->speed);

	kf->P *= (1 - kf->K);

	kf->speed_int16 = (int16_t)round(kf->speed);
}

float kf_GetFilteredSpeed(kalman_filter *kf){
	return kf->speed;
}

int16_t kf_GetFilteredSpeedInt16(kalman_filter *kf){
	return kf->speed_int16;
}


void kf_ClearSpeed(){
	gSpeedFilter.speed = 0.0f;
	gSpeedFilter.speed_int16 = (int16_t)0;
}
