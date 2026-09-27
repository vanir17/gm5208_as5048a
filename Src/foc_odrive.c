#include "foc.h"
#include "arm_math.h" // Sử dụng arm_sin_f32 và arm_cos_f32 của CMSIS-DSP

#define MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA 10000 // Tùy chọn theo cấu hình clock

// Hàm Space Vector Modulation (SVM) chuẩn C
static bool SVM(float alpha, float beta, float *tA, float *tB, float *tC) {
    float tA_val, tB_val, tC_val;
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            // Sector 1 hoặc 2
            if (ONE_BY_SQRT3 * beta > alpha) Sextant = 2;
            else Sextant = 1;
        } else {
            // Sector 2 hoặc 3
            if (-ONE_BY_SQRT3 * beta > alpha) Sextant = 3;
            else Sextant = 2;
        }
    } else {
        if (alpha >= 0.0f) {
            // Sector 5 hoặc 6
            if (-ONE_BY_SQRT3 * beta > alpha) Sextant = 5;
            else Sextant = 6;
        } else {
            // Sector 4 hoặc 5
            if (ONE_BY_SQRT3 * beta > alpha) Sextant = 4;
            else Sextant = 5;
        }
    }

    switch (Sextant) {
        // Sector 1: V1(0 deg) và V2(60 deg)
        case 1: {
            float t1 = alpha - ONE_BY_SQRT3 * beta;
            float t2 = 2.0f * ONE_BY_SQRT3 * beta;
            tA_val = (1.0f + t1 + t2) * 0.5f;
            tB_val = tA_val - t1;
            tC_val = tB_val - t2;
        } break;
        case 2: {
            float t2 = alpha + ONE_BY_SQRT3 * beta;
            float t3 = -alpha + ONE_BY_SQRT3 * beta;
            tB_val = (1.0f + t2 + t3) * 0.5f;
            tA_val = tB_val - t3;
            tC_val = tA_val - t2;
        } break;
        case 3: {
            float t3 = 2.0f * ONE_BY_SQRT3 * beta;
            float t4 = -alpha - ONE_BY_SQRT3 * beta;
            tB_val = (1.0f + t3 + t4) * 0.5f;
            tC_val = tB_val - t3;
            tA_val = tC_val - t4;
        } break;
        case 4: {
            float t4 = -alpha + ONE_BY_SQRT3 * beta;
            float t5 = -2.0f * ONE_BY_SQRT3 * beta;
            tC_val = (1.0f + t4 + t5) * 0.5f;
            tB_val = tC_val - t4;
            tA_val = tB_val - t5;
        } break;
        case 5: {
            float t5 = -alpha - ONE_BY_SQRT3 * beta;
            float t6 = alpha - ONE_BY_SQRT3 * beta;
            tC_val = (1.0f + t5 + t6) * 0.5f;
            tA_val = tC_val - t5;
            tB_val = tA_val - t6;
        } break;
        case 6: {
            float t6 = -2.0f * ONE_BY_SQRT3 * beta;
            float t1 = alpha + ONE_BY_SQRT3 * beta;
            tA_val = (1.0f + t6 + t1) * 0.5f;
            tC_val = tA_val - t6;
            tB_val = tC_val - t1;
        } break;
        default:
            return false;
    }

    // Kiểm tra giới hạn duty cycle [0, 1]
    if (tA_val < 0.0f || tA_val > 1.0f ||
        tB_val < 0.0f || tB_val > 1.0f ||
        tC_val < 0.0f || tC_val > 1.0f) {
        return false;
    }

    *tA = tA_val;
    *tB = tB_val;
    *tC = tC_val;
    return true;
}

void FOC_Reset(FOCController *foc) {
    foc->v_current_control_integral_d = 0.0f;
    foc->v_current_control_integral_q = 0.0f;
    foc->has_vbus_voltage = false;
    foc->has_Ialpha_beta = false;
    foc->power = 0.0f;
    foc->Id_measured = 0.0f;
    foc->Iq_measured = 0.0f;
}

// Chạy trong ngắt ADC (Đo dòng và điện áp Bus)
MotorError FOC_OnMeasurement(FOCController *foc, float *currents_abc, float vbus_voltage, uint32_t timestamp) {
    foc->i_timestamp = timestamp;
    foc->vbus_voltage = vbus_voltage;
    foc->has_vbus_voltage = true;

    if (currents_abc != NULL) {
        // Clarke transform
        foc->Ialpha = currents_abc[0];
        foc->Ibeta  = ONE_BY_SQRT3 * (currents_abc[1] - currents_abc[2]);
        foc->has_Ialpha_beta = true;
    } else {
        foc->has_Ialpha_beta = false;
    }

    return MOTOR_ERROR_NONE;
}

// Tính toán FOC ra Mod_alpha và Mod_beta
MotorError FOC_GetAlphaBetaOutput(FOCController *foc, uint32_t output_timestamp, float *mod_alpha, float *mod_beta) {
    if (!foc->has_vbus_voltage || !foc->has_Ialpha_beta) {
        return MOTOR_ERROR_CONTROLLER_INITIALIZING;
    }

    if (abs((int32_t)(foc->i_timestamp - foc->ctrl_timestamp)) > MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA) {
        return MOTOR_ERROR_BAD_TIMING;
    }

    if (!foc->has_Vdq_setpoint) return MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND;
    if (!foc->has_phase)        return MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE;

    float Vd = foc->Vd_setpoint;
    float Vq = foc->Vq_setpoint;
    float phase = foc->phase;
    float phase_vel = foc->phase_vel;
    float vbus_voltage = foc->vbus_voltage;

    float Id = 0.0f;
    float Iq = 0.0f;

    // Park transform (nắn dòng theo góc điện tại thời điểm đo dòng)
    if (foc->has_Ialpha_beta) {
        float dt_meas = (float)(int32_t)(foc->i_timestamp - foc->ctrl_timestamp) / TIM_1_8_CLOCK_HZ;
        float I_phase = phase + phase_vel * dt_meas;

        float c_I = arm_cos_f32(I_phase);
        float s_I = arm_sin_f32(I_phase);

        Id = c_I * foc->Ialpha + s_I * foc->Ibeta;
        Iq = c_I * foc->Ibeta  - s_I * foc->Ialpha;

        // Lọc thông thấp giá trị dòng điện để giám sát
        foc->Id_measured += foc->I_measured_report_filter_k * (Id - foc->Id_measured);
        foc->Iq_measured += foc->I_measured_report_filter_k * (Iq - foc->Iq_measured);
    }

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d;
    float mod_q;

    if (foc->enable_current_control) {
        if (!foc->has_pi_gains)        return MOTOR_ERROR_UNKNOWN_GAINS;
        if (!foc->has_Ialpha_beta)     return MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT;
        if (!foc->has_Idq_setpoint)    return MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND;

        float Ierr_d = foc->Id_setpoint - Id;
        float Ierr_q = foc->Iq_setpoint - Iq;

        // Bộ điều khiển PI (Vdq_setpoint hoạt động như Feed-forward)
        mod_d = V_to_mod * (Vd + foc->v_current_control_integral_d + Ierr_d * foc->p_gain);
        mod_q = V_to_mod * (Vq + foc->v_current_control_integral_q + Ierr_q * foc->p_gain);

        // Chống bão hòa điện áp (Anti-windup clamping)
        float mod_sq = mod_d * mod_d + mod_q * mod_q;
        float mod_scalefactor = 0.80f * SQRT3_BY_2 * (1.0f / sqrtf(mod_sq));

        if (mod_scalefactor < 1.0f) {
            mod_d *= mod_scalefactor;
            mod_q *= mod_scalefactor;
            // Xả bớt tích phân khi điện áp chạm trần modulation
            foc->v_current_control_integral_d *= 0.99f;
            foc->v_current_control_integral_q *= 0.99f;
        } else {
            foc->v_current_control_integral_d += Ierr_d * (foc->i_gain * foc->current_meas_period);
            foc->v_current_control_integral_q += Ierr_q * (foc->i_gain * foc->current_meas_period);
        }
    } else {
        // Voltage control mode
        mod_d = V_to_mod * Vd;
        mod_q = V_to_mod * Vq;
    }

    // Inverse Park transform (bù trễ pha từ lúc tính đến lúc cập nhật duty timer)
    float dt_pwm = (float)(int32_t)(output_timestamp - foc->ctrl_timestamp) / TIM_1_8_CLOCK_HZ;
    float pwm_phase = phase + phase_vel * dt_pwm;

    float c_p = arm_cos_f32(pwm_phase);
    float s_p = arm_sin_f32(pwm_phase);

    *mod_alpha = c_p * mod_d - s_p * mod_q;
    *mod_beta  = c_p * mod_q + s_p * mod_d;

    // Lưu lại điện áp pha thực tế để sensorless estimator dùng (nếu có)
    foc->final_v_alpha = mod_to_V * (*mod_alpha);
    foc->final_v_beta  = mod_to_V * (*mod_beta);

    // Tính dòng tiêu thụ và công suất
    if (foc->has_Ialpha_beta) {
        foc->ibus = mod_d * Id + mod_q * Iq;
        foc->power = vbus_voltage * foc->ibus;
    }

    return MOTOR_ERROR_NONE;
}

// Chạy cuối ngắt ADC để đẩy thẳng duty cycle ra PWM
MotorError FOC_GetPWMOutput(FOCController *foc, uint32_t output_timestamp, float pwm_timings[3]) {
    float mod_alpha, mod_beta;
    MotorError status = FOC_GetAlphaBetaOutput(foc, output_timestamp, &mod_alpha, &mod_beta);
    if (status != MOTOR_ERROR_NONE) {
        return status;
    }

    if (isnan(mod_alpha) || isnan(mod_beta)) {
        return MOTOR_ERROR_MODULATION_IS_NAN;
    }

    float tA, tB, tC;
    if (!SVM(mod_alpha, mod_beta, &tA, &tB, &tC)) {
        return MOTOR_ERROR_MODULATION_MAGNITUDE;
    }

    pwm_timings[0] = tA;
    pwm_timings[1] = tB;
    pwm_timings[2] = tC;

    return MOTOR_ERROR_NONE;
}