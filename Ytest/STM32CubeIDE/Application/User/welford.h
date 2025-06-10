/*
 * welford.h
 *
 *  Created on: Jun 6, 2025
 *      Author: 79489
 */

#ifndef APPLICATION_USER_WELFORD_H_
#define APPLICATION_USER_WELFORD_H_

typedef struct WelfordCore{
    float mean;
    float MS;
    unsigned int n;
} WelfordCore;

typedef struct {
    WelfordCore core[2];        // 双 core
    int active_index;           // 当前输出的 core
    unsigned int max_count;     // 每个 core 的周期样本数
} WelfordStats;


void WelfordCore_Init(WelfordCore* core);
void WelfordStats_Init(WelfordStats* stats, unsigned int max_count);
void WelfordStats_Update(WelfordStats* stats, float x);
float WelfordStats_GetMean(const WelfordStats* stats);
float WelfordStats_GetVariance(const WelfordStats* stats);
float WelfordStats_GetStdDev(const WelfordStats* stats);

#endif /* APPLICATION_USER_WELFORD_H_ */
