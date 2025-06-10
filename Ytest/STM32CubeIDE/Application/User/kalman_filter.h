/*
 * kalman_filter.h
 *
 *  Created on: May 22, 2025
 *      Author: 79489
 */

#ifndef APPLICATION_USER_KALMAN_FILTER_H_
#define APPLICATION_USER_KALMAN_FILTER_H_

#include<stdint.h>

typedef struct{
	float speed;	//速度估计值
	int16_t speed_int16;
	float P;		//误差协方差
	float Q;		//过程噪声
	float R;		//测量噪声
	float K;
}kalman_filter;

void kf_init(kalman_filter* kf, float Q, float R);
void kf_update(kalman_filter *kf, int16_t measurement);
float kf_GetFilteredSpeed(kalman_filter *kf);
int16_t kf_GetFilteredSpeedInt16(kalman_filter *kf);
void kf_ClearSpeed();

#endif /* APPLICATION_USER_KALMAN_FILTER_H_ */
