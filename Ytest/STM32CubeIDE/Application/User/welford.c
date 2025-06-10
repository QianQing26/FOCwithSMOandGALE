/*
 * welford.c
 *
 *  Created on: Jun 6, 2025
 *      Author: 79489
 */


#include <math.h>
#include <stdio.h>
#include "welford.h"

void WelfordCore_Init(WelfordCore* core) {
    core->mean = 0.0f;
    core->MS = 0.0f;
    core->n = 0;
}

void WelfordStats_Init(WelfordStats* stats, unsigned int max_count) {
    WelfordCore_Init(&stats->core[0]);
    WelfordCore_Init(&stats->core[1]);
    stats->active_index = 0;
    stats->max_count = max_count;

}

void WelfordStats_Update(WelfordStats* stats, float x) {
    int a = stats->active_index;
    int b = 1 - a;

    WelfordCore* active  = &stats->core[a];
    WelfordCore* standby = &stats->core[b];

    // 更新 active core（主统计器）
    float delta_a = x - active->mean;
    active->n += 1;
    active->mean += delta_a / active->n;
    active->MS += delta_a * (x - active->mean);

    // 如果 active 超过半周期，则 standby 也开始更新
    if (active->n > stats->max_count / 2) {
        float delta_b = x - standby->mean;
        standby->n += 1;
        standby->mean += delta_b / standby->n;
        standby->MS += delta_b * (x - standby->mean);
    }

    // 如果 active 达到 max_count，则进行切换
    if (active->n >= stats->max_count) {
        // 切换主 core
        stats->active_index = b;

        // 被切换下来的 core重置
        active->n = 0;
        active->mean = 0.0f;
        active->MS = 0.0f;
    }
}


float WelfordStats_GetMean(const WelfordStats* stats) {
    return stats->core[stats->active_index].mean;
}

float WelfordStats_GetVariance(const WelfordStats* stats) {
    const WelfordCore* core = &stats->core[stats->active_index];
    return (core->n < 2) ? 0.0f : core->MS / (core->n - 1);
}
