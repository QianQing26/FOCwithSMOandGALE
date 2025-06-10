/*
 * filter.h
 *
 *  Created on: Jun 4, 2025
 *      Author: 79489
 */

#ifndef APPLICATION_USER_FILTER_H_
#define APPLICATION_USER_FILTER_H_


#include<stdint.h>
#include<stdbool.h>

#include "welford.h"

#define STATE_DIM 2
#define MEASURE_DIM 1

#define USE_MATRIX 0

//Kalman Filter
typedef struct{
	float x[STATE_DIM];
	float x_pred[STATE_DIM];

    float P[STATE_DIM][STATE_DIM];        // 状态协方差矩阵
    float P_pred[STATE_DIM][STATE_DIM];   // 预测协方差矩阵

    float F[STATE_DIM][STATE_DIM];        // 状态转移矩阵
    float H[MEASURE_DIM][STATE_DIM];      // 观测矩阵
    float Q[STATE_DIM][STATE_DIM];        // 过程噪声协方差
    float R[MEASURE_DIM][MEASURE_DIM];    // 测量噪声协方差

    float K[STATE_DIM][MEASURE_DIM];      // Kalman增益矩阵

    float temp_2x2[STATE_DIM][STATE_DIM];
    float temp_2x1[STATE_DIM];
    float temp_1x1[MEASURE_DIM][MEASURE_DIM];
    float temp_1x2[MEASURE_DIM][STATE_DIM];


    float dt;                     // 采样时间
    bool initialized;             // 初始化标志
}KalmanFilter;

void kalman_init(KalmanFilter* kf, float dt, float init_speed);
float kalman_update(KalmanFilter* kf, float measurement_speed);
void kalman_reset(KalmanFilter* kf);

//----------------------------------------------------------------------------------------------------------------------------

// 一阶低通滤波器
typedef struct {
    float alpha;    // 滤波系数，0 < alpha < 1
    float value;    // 当前滤波输出值
} LowPassFilter;

void LowPassFilter_Init(LowPassFilter* filter, float alpha, float init_value);
float LowPassFilter_Update(LowPassFilter* filter, float input_value);
void LowPassFilter_Reset(LowPassFilter* filter);

//-----------------------------------------------------------------------------------------------------------------------------

// 滑动平均滤波器
#define MOVING_AVERAGE_MAX_SIZE 32
typedef struct {
    float buffer[MOVING_AVERAGE_MAX_SIZE];  // 滤波缓存
    int size;       // 滤波窗口大小（2 的次幂）
    int power;      // 滤波窗口大小的对数值（如 size=8，power=3）
    int index;      // 当前写入位置
    float sum;      // 当前总和
} MovingAverageFilter;

void MovingAverageFilter_Init(MovingAverageFilter* filter, int window_size, float init_value);
float MovingAverageFilter_Update(MovingAverageFilter* filter, float input_value);
void MovingAverageFilter_Reset(MovingAverageFilter* filter);

//-------------------------------------------------------------------------------------------------------------------------------------------

// 自适应LMS滤波器
typedef struct{
	float weight; //权重系数
	float bias;	//偏置系数
	float mu;	//步长因子
	float error;	//误差
	float output;	//输出
	float prev_input;	//上一次输入

	//权重限制范围
	float weight_max;
	float weight_min;
	float bias_max;
	float bias_min;

	//初始化标记
	bool initialized;
}AdaLMSFilter;

void AdaLMSFilter_Init(AdaLMSFilter* filter);
float AdaLMSFilter_Update(AdaLMSFilter* filter, float input_value, float desired_value);
void AdaLMSFilter_Reset(AdaLMSFilter* filter);

//--------------------------------------------------------------------------------------------------------


float fast_expf(float x);

//-------------------------------------------------------------------------------------------------------------------


#endif /* APPLICATION_USER_FILTER_H_ */
