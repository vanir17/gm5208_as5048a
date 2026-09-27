#ifndef ANGLE_FILTER_H
#define ANGLE_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct 
{
    float cpr; // count per revolution
    float ts; // sampling cycle Ts
    float bandwidth_hz;

    float pll_kp; // Kp = 2 * omega
    float pll_ki; // Ki = omega ^ 2

    float pos_estimate_counts;
    float vel_estimate_counts;

    float snap_threshold; //deadband
}AngleFilterPLL_t;



void AngleFilter_Init(AngleFilterPLL_t *filter, float cpr, float ts_sec, float bandwitdth_hz, float initial_raw_count);

void AngleFilter_SetBandwidth(AngleFilterPLL_t *filter, float bandwidth_hz);

void AngleFilter_Update(AngleFilterPLL_t *filter, float raw_count);

float AngleFilter_GetAngleRad(const AngleFilterPLL_t *filter);

float AngleFilter_GetVelocityRad_s(const AngleFilterPLL_t *filter);

float AngleFilter_GetAngleCounts(const AngleFilterPLL_t *filter);

float AngleFilter_GetVelocityCounts_s(const AngleFilterPLL_t *filter);


#ifdef __cplusplus
}
#endif

#endif // ANGLE_FILTER_H