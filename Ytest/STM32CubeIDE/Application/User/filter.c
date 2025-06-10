/*
 * filter.c
 *
 *  Created on: Jun 4, 2025
 *      Author: 79489
 */


#include "mc_type.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "kalman_filter.h"
#include "filter.h"
#include "matrix.h"
#include "welford.h"
#include<math.h>
#include<stdint.h>
#include<stdbool.h>
#include<string.h>


// 卡尔曼滤波
void kalman_init(KalmanFilter* kf, float dt, float init_speed){
	kf->dt = dt;

	// 初始化状态向量
	kf->x[0] = init_speed;
	kf->x[1] = 0.0f;

	// state transition matrix
	kf->F[0][0] = 1.0f;	kf->F[0][1] = dt;
	kf->F[1][0] = 0.0f;	kf->F[1][1] = 1.0f;

	// measurement matrix
	kf->H[0][0] = 1.0f;
	kf->H[0][1] = 0.0f;

    // P covariance
    kf->P[0][0] = 1.0f;  kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;  kf->P[1][1] = 3000.0f;

	// process noise
    kf->Q[0][0] = 0.1f;  kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;  kf->Q[1][1] = 10.0f;

    // measurement noise
    kf->R[0][0] = 15.0f;

	kf->initialized = true;
	return;
}


float kalman_update(KalmanFilter* kf, float measurement_speed) {
    //-----Prediction-----
#if USE_MATRIX
    // x_pred = F * x
    matrix_multiply_2x1(kf->F, kf->x, kf->x_pred);

    // P_pred = F * P * F^T + Q
    matrix_multiply_2x2(kf->F, kf->P, kf->temp_2x2);  // temp = F * P
    float Ft[2][2] = {
        { kf->F[0][0], kf->F[1][0] },
        { kf->F[0][1], kf->F[1][1] }
    };
    matrix_multiply_2x2(kf->temp_2x2, Ft, kf->P_pred);
    matrix_add_2x2(kf->P_pred, kf->Q, kf->P_pred);
#else
    // x_pred = F * x
    kf->x_pred[0] = kf->F[0][0] * kf->x[0] + kf->F[0][1] * kf->x[1];
    kf->x_pred[1] = kf->F[1][0] * kf->x[0] + kf->F[1][1] * kf->x[1];

    // P_pred = F * P * F^T + Q
    float FP[2][2];
    FP[0][0] = kf->F[0][0] * kf->P[0][0] + kf->F[0][1] * kf->P[1][0];
    FP[0][1] = kf->F[0][0] * kf->P[0][1] + kf->F[0][1] * kf->P[1][1];
    FP[1][0] = kf->F[1][0] * kf->P[0][0] + kf->F[1][1] * kf->P[1][0];
    FP[1][1] = kf->F[1][0] * kf->P[0][1] + kf->F[1][1] * kf->P[1][1];

    kf->P_pred[0][0] = FP[0][0] * kf->F[0][0] + FP[0][1] * kf->F[0][1] + kf->Q[0][0];
    kf->P_pred[0][1] = FP[0][0] * kf->F[1][0] + FP[0][1] * kf->F[1][1] + kf->Q[0][1];
    kf->P_pred[1][0] = FP[1][0] * kf->F[0][0] + FP[1][1] * kf->F[0][1] + kf->Q[1][0];
    kf->P_pred[1][1] = FP[1][0] * kf->F[1][0] + FP[1][1] * kf->F[1][1] + kf->Q[1][1];
#endif

    //-----Update-----
    // y = z - H * x_pred
    float z = measurement_speed;
    float y = z - (kf->H[0][0] * kf->x_pred[0] + kf->H[0][1] * kf->x_pred[1]);

#if USE_MATRIX
    matrix_multiply_1x2(kf->H, kf->P_pred, kf->temp_1x2); // temp = H * P_pred
    float Ht[2][1] = { {kf->H[0][0]}, {kf->H[0][1]} };
    matrix_multiply_1x1(kf->temp_1x2, Ht, kf->temp_1x1);
    kf->temp_1x1[0][0] += kf->R[0][0];
    float S = kf->temp_1x1[0][0];
#else
    float S =
        kf->H[0][0] * (kf->P_pred[0][0] * kf->H[0][0] + kf->P_pred[0][1] * kf->H[0][1]) +
        kf->H[0][1] * (kf->P_pred[1][0] * kf->H[0][0] + kf->P_pred[1][1] * kf->H[0][1]);
    S += kf->R[0][0];
#endif

    float S_inv = (fabsf(S) < 1e-6f) ? 1e6f : 1.0f / S;

    // Kalman 增益 K = P_pred * H^T * S^-1
    kf->K[0][0] = (kf->P_pred[0][0] * kf->H[0][0] + kf->P_pred[0][1] * kf->H[0][1]) * S_inv;
    kf->K[1][0] = (kf->P_pred[1][0] * kf->H[0][0] + kf->P_pred[1][1] * kf->H[0][1]) * S_inv;

    // 状态更新：x = x_pred + K * y
    kf->x[0] = kf->x_pred[0] + kf->K[0][0] * y;
    kf->x[1] = kf->x_pred[1] + kf->K[1][0] * y;

    // 协方差更新：P = P_pred - K * H * P_pred
    kf->P[0][0] = kf->P_pred[0][0] - kf->K[0][0] * (kf->H[0][0] * kf->P_pred[0][0] + kf->H[0][1] * kf->P_pred[1][0]);
    kf->P[0][1] = kf->P_pred[0][1] - kf->K[0][0] * (kf->H[0][0] * kf->P_pred[0][1] + kf->H[0][1] * kf->P_pred[1][1]);
    kf->P[1][0] = kf->P_pred[1][0] - kf->K[1][0] * (kf->H[0][0] * kf->P_pred[0][0] + kf->H[0][1] * kf->P_pred[1][0]);
    kf->P[1][1] = kf->P_pred[1][1] - kf->K[1][0] * (kf->H[0][0] * kf->P_pred[0][1] + kf->H[0][1] * kf->P_pred[1][1]);

    return kf->x[0];
}

void kalman_reset(KalmanFilter* kf) {
    // 状态归零
    kf->x[0] = 0.0f;  // 速度
    kf->x[1] = 0.0f;  // 加速度

    kf->x_pred[0] = 0.0f;
    kf->x_pred[1] = 0.0f;

    // 协方差设为初始高不确定性
    kf->P[0][0] = 1000.0f;  kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;     kf->P[1][1] = 1000.0f;

    kf->P_pred[0][0] = 1000.0f;  kf->P_pred[0][1] = 0.0f;
    kf->P_pred[1][0] = 0.0f;     kf->P_pred[1][1] = 1000.0f;
}

//----------------------------------------------------------------------------------------------------------------------------------------

// 一阶低通滤波
void LowPassFilter_Init(LowPassFilter* filter, float alpha, float init_value){
    if (alpha <= 0.0f) alpha = 0.01f;
    if (alpha >= 1.0f) alpha = 0.99f;
    filter->alpha = alpha;
    filter->value = init_value;
}

float LowPassFilter_Update(LowPassFilter* filter, float input_value) {
    filter->value = filter->alpha * input_value + (1.0f - filter->alpha) * filter->value;
    return filter->value;
}

void LowPassFilter_Reset(LowPassFilter* filter) {
    filter->value = 0;
}

//-----------------------------------------------------------------------------------------------------------------------------------------

// 滑动平均滤波
static int Log2Int(int n) {
    int power = 0;
    while (n >>= 1) ++power;
    return power;
}

void MovingAverageFilter_Init(MovingAverageFilter* filter, int window_size, float init_value) {

    if (window_size > MOVING_AVERAGE_MAX_SIZE) {
        window_size = MOVING_AVERAGE_MAX_SIZE;
    }

    filter->size = window_size;
    filter->power = Log2Int(window_size);
    filter->index = 0;
    filter->sum = init_value * window_size;

    for (int i = 0; i < window_size; i++) {
        filter->buffer[i] = init_value;
    }
}

float MovingAverageFilter_Update(MovingAverageFilter* filter, float input_value) {
    filter->sum -= filter->buffer[filter->index];
    filter->buffer[filter->index] = input_value;
    filter->sum += input_value;

    filter->index = (filter->index + 1) & (filter->size - 1);  // 取模优化

    // 用右移代替除法
    return filter->sum / filter->size;
}

void MovingAverageFilter_Reset(MovingAverageFilter* filter) {
    filter->sum = 0;
    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->index = 0;
}


//-----------------------------------------------------------------------------------------------------------------------------------

//自适应LMS滤波器
static inline float clamp_value(float value, float min_val, float max_val){
	value = ( value < min_val ) ? min_val : value;
	value = ( value > max_val ) ? max_val : value;
	return value;
}

void AdaLMSFilter_Init(AdaLMSFilter* filter) {
//    if (filter == NULL) return;

    filter->weight = 1.0f;
    filter->bias = 0.0f;
    filter->mu = 0.01f;

    filter->error = 0.0f;
    filter->output = 0.0f;
    filter->prev_input = 0.0f;

    filter->weight_max = 1.2f;
    filter->weight_min = 0.8f;
    filter->bias_max = 10.0f;
    filter->bias_min = -10.0f;

    filter->initialized = true;
}

float AdaLMSFilter_Update(AdaLMSFilter* filter, float input_value, float desired_value){
//	float desired = filter->prev_input;
	float desired = desired_value;
	filter->output = filter->weight * input_value + filter->bias;

	filter->error = filter->output - desired;

	filter->weight -= filter->mu * filter->error * input_value;
	filter->bias -= filter->mu * filter->error;

	filter->weight = clamp_value(filter->weight, filter->weight_min, filter->weight_max);
	filter->bias = clamp_value(filter->bias, filter->bias_min, filter->bias_max);

	filter->prev_input = input_value;

	return filter->output;
}

void AdaLMSFilter_Reset(AdaLMSFilter* filter){

    filter->weight = 1.0f;
    filter->bias = 0.0f;
    filter->mu = 0.01f;

    filter->error = 0.0f;
    filter->output = 0.0f;
    filter->prev_input = 0.0f;
}

//-----------------------------------------------------------------------------------------------------------------------------------

//MMAE
float fast_expf(float x) {
	// IEEE float trick快速exp算法
    union {
        float f;
        int32_t i;
    } result;
    result.i = (int32_t)(12102203 * x + 1065353216); // 约等于 2^23 / ln(2)
    return result.f;
}

//void LMMAE_Init(LMMAE* fusion){
//	kalman_init(&fusion->kalman, 0.0005, 0.0f);
//	LowPassFilter_Init(&fusion->lpf, 0.3f, 0.0f);
//	MovingAverageFilter_Init(&fusion->maf, 8, 0.0f);
//	AdaLMSFilter_Init(&fusion->lms);
//
//    fusion->u[0] = 0.4f;
//    fusion->u[1] = 0.2f;
//    fusion->u[2] = 0.2f;
//    fusion->u[3] = 0.2f;
//
//    for (int i=0; i<FILTER_NUM; i++){
//    	WelfordStats_Init(&fusion->stats[i], 32);
//    }
//
//
////    memset(fusion->history, 0, sizeof(fusion->history));
//    memset(fusion->z, 0, sizeof(fusion->z));
//
////    fusion->index = 0;
//    fusion->initialized = true;
//    return;
//}
//
//float LMMAE_Update(LMMAE* fusion, float input){
//	// 1.计算滤波器输出
//	fusion->z[0] = kalman_update(&fusion->kalman, input);
//	fusion->z[1] = LowPassFilter_Update(&fusion->lpf, input);
//	fusion->z[2] = MovingAverageFilter_Update(&fusion->maf, input);
//	fusion->z[3] = AdaLMSFilter_Update(&fusion->lms, input);
//
//    float weights[FILTER_NUM];
//    float weighted_sum = 0.0f;
//    float total_weight = 0.0f;
//
//    // 2. 更新历史数据
//    for (int i = 0; i < FILTER_NUM; i++) {
////        fusion->history[i][fusion->index] = fusion->z[i];
//    	WelfordStats_Update(&fusion->stats[i], fusion->z[i]);
//    }
//
////    fusion->index = (fusion->index + 1) % HISTORY_LEN;
//
//    float diff, exponent, L, mean, var;
//
//    // 3，计算高斯似然并融合
//    for (int i = 0; i < FILTER_NUM; i++){
////    	float* data = fusion->history[i];
////    	mean = calc_mean(data, HISTORY_LEN);
//    	mean = WelfordStats_GetMean(&fusion->stats[i]);
////    	var = calc_variance(data, HISTORY_LEN, mean);
//    	var = WelfordStats_GetVariance(&fusion->stats[i]);
//    	if (var<1e-6) var = 1e-6;
//
//    	diff = fusion->z[i] - mean;
//    	exponent = - (diff * diff) / (2.0f* var);
//    	L = fast_expf(exponent);
//
//    	weights[i] = L * fusion->u[i];
//    	total_weight += weights[i];
//    }
//
//    // 4. 加权融合输出
//    for (int i = 0; i < FILTER_NUM; i++) {
//        if (total_weight > 0)
//            weighted_sum += weights[i] / total_weight * fusion->z[i];
//        else
//            weighted_sum += fusion->u[i] * fusion->z[i]; // fallback
//    }
//
//    kf_update(&fusion->_Super, (int16_t)input);
//
////    fusion->speed = (int16_t)weighted_sum;
//    fusion->speed = kf_GetFilteredSpeedInt16(&fusion->_Super);
//
//    return weighted_sum;
//}
//
//void LMMAE_Reset(LMMAE* fusion){
//	kalman_reset(&fusion->kalman);
//	LowPassFilter_Reset(&fusion->lpf);
//	MovingAverageFilter_Reset(&fusion->maf);
//	AdaLMSFilter_Reset(&fusion->lms);
//	for (int i=0; i<FILTER_NUM; i++){
//		WelfordStats_Init(&fusion->stats[i], 30);
//	}
//	return;
//}
//
//int16_t GetLMMAESpeed(LMMAE* fusion){
//	return fusion->speed;
//}

//-------------------------------------------------------------------------------
