#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#define FOC_2PI             6.28318530718f
#define FOC_ENCODER_CPR     16384.0f  

#define FOC_LOOP_FREQ_HZ   10000.0f
#define FOC_ENCODER_DIR     1
#define FOC_POLE_PAIRS      7
#define FOC_VQ_LIMIT 1.0f

extern volatile float g_pp;
extern volatile int32_t g_dir;
extern volatile float g_enc_off_counts;
extern volatile float g_theta_e;
extern volatile int32_t g_detect_dir;
extern volatile float g_detect_pp;
extern volatile uint8_t g_calib_err;
extern volatile uint32_t g_dgb_ccr1, g_dbg_ccr2, g_dbg_ccr3;



void FOC_M1_Init(void);
void FOC_M1_EmergencyStop(void);

/**
 * SVPWM - SetPhaseDuty
 */
void SetPhaseDuty(float ua, float ub, float uc);

/**
 * FOC_AutoCalibrate
 * 0: OK
 * 1: SPI Error
 * 2: Rotor do not rotate
 * 3: The number of pole pairs is not an integer
 */
uint8_t FOC_AutoCalibrate(float v);

/**
 * Call in ISR func
 * enc_counts: [0;16384)
 */
void FOC_VoltageMode_Step(float enc_counts, float vq);





#endif /* FOC_H */