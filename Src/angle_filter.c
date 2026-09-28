#include "angle_filter.h"
#include <math.h>

#define TWO_PI 6.28318530717958647692f

static inline float wrap_pm_cpr(float delta, float cpr)
{
    const float half_cpr = cpr * 0.5f;

    while(delta > half_cpr)
    {
        delta -= cpr;
    }

    while(delta < -half_cpr)
    {
        delta += cpr;
    }
    return delta;
}

void AngleFilter_SetBandwidth(AngleFilterPLL_t *filter, float bandwidth_hz)
{
    filter->bandwidth_hz = bandwidth_hz;

    float omega = TWO_PI * bandwidth_hz;

    filter->pll_kp = 2.0f * omega;
    filter->pll_ki = 0.25f * (filter->pll_kp * filter->pll_kp); 

    filter->snap_threshold = 0.5f * filter->ts * filter->pll_ki;
}

void AngleFilter_Init(AngleFilterPLL_t *filter, float cpr, float ts_sec, float bandwitdth_hz, float initial_raw_count)
{
    filter->cpr = cpr;
    filter->ts = ts_sec;

    AngleFilter_SetBandwidth(filter, bandwitdth_hz);

    filter->pos_estimate_counts = initial_raw_count;
    filter->vel_estimate_counts = 0.0f;
}

void AngleFilter_Update(AngleFilterPLL_t *filter, float raw_count)
{
    //prediction step
    filter->pos_estimate_counts += filter->ts * filter->vel_estimate_counts;

    // calculate error [0 ; CPR)
    float delta_pos = raw_count - filter->pos_estimate_counts;
    delta_pos = wrap_pm_cpr(delta_pos, filter->cpr);

    filter->pos_estimate_counts += filter->ts * filter->pll_kp * delta_pos;
    filter->vel_estimate_counts += filter->ts * filter->pll_ki * delta_pos;

    if (filter->pos_estimate_counts >= filter->cpr)
    {
        filter->pos_estimate_counts -= filter->cpr;
    }
    else if (filter->pos_estimate_counts < 0.0f)
    {
        filter->pos_estimate_counts += filter->cpr;
    }

}

float AngleFilter_GetAngleRad(const AngleFilterPLL_t *filter) {
    return (filter->pos_estimate_counts / filter->cpr) * TWO_PI;
}

float AngleFilter_GetVelocityRad_s(const AngleFilterPLL_t *filter) {
    return (filter->vel_estimate_counts / filter->cpr) * TWO_PI;
}

float AngleFilter_GetAngleCounts(const AngleFilterPLL_t *filter) {
    return filter->pos_estimate_counts;
}

float AngleFilter_GetVelocityCounts_s(const AngleFilterPLL_t *filter) {
    return filter->vel_estimate_counts;
}

float AngleFilter_GetAngleDeg(const AngleFilterPLL_t *filter) {
    return (filter->pos_estimate_counts / filter->cpr) * 360.0f;
}

float AngleFilter_GetVelocityDeg_s(const AngleFilterPLL_t *filter) {
    return (filter->vel_estimate_counts / filter->cpr) * 360.0f;
}