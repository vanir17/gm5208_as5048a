#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define ONE_BY_SQRT3    0.57735026919f
#define SQRT3_BY_2      0.86602540378f
#define TIM_1_8_CLOCK_HZ 168000000.0f  // Tần số Timer STM32F4 (168MHz)

typedef enum {
    MOTOR_ERROR_NONE = 0,
    MOTOR_ERROR_CONTROLLER_INITIALIZING,
    MOTOR_ERROR_BAD_TIMING,
    MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND,
    MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE,
    MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE,
    MOTOR_ERROR_UNKNOWN_GAINS,
    MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT,
    MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND,
    MOTOR_ERROR_MODULATION_IS_NAN,
    MOTOR_ERROR_MODULATION_MAGNITUDE
} MotorError;

typedef struct {
    // Thông số đo lường & Cấu hình
    float vbus_voltage;
    bool  has_vbus_voltage;
    
    float Ialpha;
    float Ibeta;
    bool  has_Ialpha_beta;
    
    uint32_t i_timestamp;
    uint32_t ctrl_timestamp;
    float    current_meas_period; // Ví dụ: 1.0f / 24000.0f
    
    // Trạng thái điều khiển
    bool  enable_current_control;
    float phase;
    float phase_vel;
    bool  has_phase;
    
    // Setpoints
    float Id_setpoint;
    float Iq_setpoint;
    bool  has_Idq_setpoint;
    
    float Vd_setpoint;
    float Vq_setpoint;
    bool  has_Vdq_setpoint;
    
    // PI Gains
    float p_gain;
    float i_gain;
    bool  has_pi_gains;
    
    // Integrator states
    float v_current_control_integral_d;
    float v_current_control_integral_q;
    
    // Output states & Feedback
    float Id_measured;
    float Iq_measured;
    float I_measured_report_filter_k;
    
    float final_v_alpha;
    float final_v_beta;
    float power;
    float ibus;
} FOCController;

// Prototype các hàm
void FOC_Reset(FOCController *foc);
MotorError FOC_OnMeasurement(FOCController *foc, float *currents_abc, float vbus_voltage, uint32_t timestamp);
MotorError FOC_GetAlphaBetaOutput(FOCController *foc, uint32_t output_timestamp, float *mod_alpha, float *mod_beta);
MotorError FOC_GetPWMOutput(FOCController *foc, uint32_t output_timestamp, float pwm_timings[3]);

#endif // FOC_H