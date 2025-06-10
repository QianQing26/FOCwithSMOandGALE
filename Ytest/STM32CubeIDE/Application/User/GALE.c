/*
 * GALE.c
 *
 *  Created on: Jun 6, 2025
 *      Author: 79489
 */

#include <string.h>

#include "kalman_filter.h"
#include "filter.h"
#include "GALE.h"
#include "welford.h"


#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
//#include "kalman_filter.h"
//#include "filter.h"
//#include "GALE.h"
//#include "smo.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "dac_ui.h"


void GALE_Init(GALE* gale){
	kalman_init(&gale->kalman, 0.0005, 0.0f);
	LowPassFilter_Init(&gale->lpf, 0.3f, 0.0f);
	MovingAverageFilter_Init(&gale->maf, 32, 0.0f);
	AdaLMSFilter_Init(&gale->lms);

	gale->u[0] = 1.5f;
	gale->u[1] = 0.001f;
	gale->u[2] = 0.001f;
	gale->u[3] = 0.000f;

	for (int i=0; i<FILTER_NUM; i++){
		WelfordStats_Init(&gale->stats[i], 32);
	}

	memset(gale->z, 0, sizeof(gale->z));

	gale->prev_input = 0.0f;
	gale->output = 0.0f;

	gale->initialized = true;

	kf_init(&gale->_Super, 1.0f, 1.0f);


	return;
}

float GALE_Update(GALE* gale, float input){

	gale->input_speed = (int16_t)input;

	input = 0.2 * input + 0.8 * gale->prev_input;

	gale->prev_input = input;

	// 1.计算滤波器输出
	gale->z[0] = kalman_update(&gale->kalman, input);
	gale->z[1] = LowPassFilter_Update(&gale->lpf, input);
	gale->z[2] = MovingAverageFilter_Update(&gale->maf, input);

	float desired = gale->kalman.x[0] + gale->kalman.dt * gale->kalman.x[1];
	desired = 0.8 * desired + 0.2 * gale->z[2];

	gale->z[3] = AdaLMSFilter_Update(&gale->lms, input, desired);

    float weights[FILTER_NUM];
    float weighted_sum = 0.0f;
    float total_weight = 0.0f;

    // 2. 更新历史数据
    for (int i = 0; i < FILTER_NUM; i++) {
//        gale->history[i][gale->index] = gale->z[i];
    	WelfordStats_Update(&gale->stats[i], gale->z[i]);
    }

//    gale->index = (gale->index + 1) % HISTORY_LEN;

    float diff, exponent, L, mean, var;

    // 3，计算高斯似然并融合
    for (int i = 0; i < FILTER_NUM; i++){
//    	float* data = gale->history[i];
//    	mean = calc_mean(data, HISTORY_LEN);
    	mean = WelfordStats_GetMean(&gale->stats[i]);
//    	var = calc_variance(data, HISTORY_LEN, mean);
    	var = WelfordStats_GetVariance(&gale->stats[i]);
    	if (var<1e-6) var = 1e-6;

    	diff = gale->z[i] - mean;
    	exponent = - (diff * diff) / (2.0f* var);
    	L = fast_expf(exponent);

    	weights[i] = L * gale->u[i];
    	total_weight += weights[i];
    }

    // 4. 加权融合输出
    for (int i = 0; i < FILTER_NUM; i++) {
        if (total_weight > 0)
            weighted_sum += weights[i] / total_weight * gale->z[i];
        else
            weighted_sum += gale->u[i] * gale->z[i]; // fallback
    }

    kf_update(&gale->_Super, (int16_t)input);

    if(gale->output<10.0f){
    	gale->output = weighted_sum;
    }
    else{
    	gale->output = gale->output*0.5 + weighted_sum*0.5;
    }

//    gale->speed = ((int16_t)gale->output);
    gale->speed = kf_GetFilteredSpeedInt16(&gale->_Super);

    return weighted_sum;
}

int16_t GetGALESpeed(GALE* fusion){
	if((Mci[M1].State==IDLE) || (Mci[M1].State==STOP)){
	return 0;
	}
	else
		return fusion->speed;
}


int16_t GetGALEInput(GALE* fusion){
	return fusion->input_speed;
}

void GALE_Reset(GALE* fusion){
	kalman_reset(&fusion->kalman);
		LowPassFilter_Reset(&fusion->lpf);
		MovingAverageFilter_Reset(&fusion->maf);
		AdaLMSFilter_Reset(&fusion->lms);
		for (int i=0; i<FILTER_NUM; i++){
			WelfordStats_Init(&fusion->stats[i], 30);
		};
		fusion->_Super.speed = 0.0f;
		fusion->_Super.speed_int16 = (int16_t)0;
}
