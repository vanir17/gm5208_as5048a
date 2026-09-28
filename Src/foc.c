#include "foc.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "drv8301.h"
#include "as5047.h"
#include "as5048a.h"
#include <math.h>
#include <stdlib.h>

#define _2PI_F 6.28318530718f

/* ===================== Thông số phần cứng M1 (Odrive v3.6) ===================== */
#define R_SHUNT          0.0005f      // Ohm
#define CSA_GAIN         20.0f        // V/V - PHẢI khớp với gain nạp vào DRV8301_M1_Init()
#define ADC_VREF         3.3f
#define ADC_MAX_COUNT    4096.0f
#define PWM_PERIOD       3500         // trùng TIM_1_8_PERIOD_CLOCKS trong .ioc

/* ===================== Giới hạn an toàn ===================== */
#define IQ_MAX               1.5f     // A
#define ID_MAX               1.5f     // A
#define CURRENT_TRIP_MARGIN  1.5f     // vượt quá 1.5x giới hạn -> cắt xung ngay
#define DUTY_MAX_RATIO       0.15f    // |Vd|,|Vq| <= 0.15 (tức 15% duty lệch khỏi 50%)
#define DUTY_MIN             0.02f
#define DUTY_CEIL            0.98f


static const float DT = 1.0f / FOC_LOOP_FREQ_HZ;

/* ===================== Trạng thái nội bộ ===================== */
static volatile uint8_t s_calibrated = 0;
static float s_offset_ib = 2048.0f;
static float s_offset_ic = 2048.0f;

static volatile float s_theta = 0.0f;
static volatile int32_t s_encoder_offset_raw = 0; // giá trị AS5048a raw tại theta_elec = 0
volatile float s_encoder_offset_rad;
extern float angle_filtered_rad;
static volatile float s_id_target = 0.0f;
volatile float s_iq_target = 0.0f;
volatile float s_id_meas = 0.0f;
volatile float s_iq_meas = 0.0f;


volatile uint32_t g_dbg_ccr1 = 0;
volatile uint32_t g_dbg_ccr2 = 0;
volatile uint32_t g_dbg_ccr3 = 0;

typedef struct {
    float kp, ki;
    float integrator;
    float limit;
} PI_t;

/* Gain khởi điểm bảo thủ - CẦN tune lại theo motor thực tế (đo R, L pha rồi tính,
 * hoặc tune thực nghiệm bắt đầu từ giá trị nhỏ tăng dần). */
static PI_t s_pi_id = { .kp = 0.05f, .ki = 15.0f, .integrator = 0.0f, .limit = DUTY_MAX_RATIO };
static PI_t s_pi_iq = { .kp = 0.05f, .ki = 15.0f, .integrator = 0.0f, .limit = DUTY_MAX_RATIO };

/* ===================== Hàm phụ trợ ===================== */
static float PI_Update(PI_t *pi, float error)
{
    pi->integrator += pi->ki * error * DT;
    if (pi->integrator > pi->limit)  pi->integrator = pi->limit;
    if (pi->integrator < -pi->limit) pi->integrator = -pi->limit;

    float out = pi->kp * error + pi->integrator;
    if (out > pi->limit)  out = pi->limit;
    if (out < -pi->limit) out = -pi->limit;
    return out;
}

static void SetPhaseDuty(float ua, float ub, float uc)
{
    float v_max = ua;
    if(ub > v_max) v_max = ub;
    if(uc > v_max) v_max = uc;

    float v_min = ua;
    if(ub < v_min) v_min = ub;
    if(uc < v_min) v_min = uc;

    float v_neutral = -0.5f *(v_max + v_min);

    //tao song yen ngua svpwm
    ua += v_neutral;
    ub += v_neutral;
    uc += v_neutral;

    float duty_a = 0.5f + ua;
    float duty_b = 0.5f + ub;
    float duty_c = 0.5f + uc;

    if (duty_a < DUTY_MIN) duty_a = DUTY_MIN; 
    if (duty_a > DUTY_CEIL) duty_a = DUTY_CEIL;
    if (duty_b < DUTY_MIN) duty_b = DUTY_MIN; 
    if (duty_b > DUTY_CEIL) duty_b = DUTY_CEIL;
    if (duty_c < DUTY_MIN) duty_c = DUTY_MIN; 
    if (duty_c > DUTY_CEIL) duty_c = DUTY_CEIL;

    TIM8->CCR1 = (uint32_t)(duty_a * PWM_PERIOD);
    TIM8->CCR2 = (uint32_t)(duty_b * PWM_PERIOD);
    TIM8->CCR3 = (uint32_t)(duty_c * PWM_PERIOD);

    g_dbg_ccr1 = TIM8->CCR1;
    g_dbg_ccr2 = TIM8->CCR2;
    g_dbg_ccr3 = TIM8->CCR3;
}

void FOC_M1_EmergencyStop(void)
{
    TIM8->BDTR &= ~TIM_BDTR_MOE;                       // High-Z ngay lập tức
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // EN_GATE = LOW
    s_calibrated = 0;
}

/* ===================== API ===================== */
void FOC_M1_Init(void)
{
    s_calibrated = 0;
    s_pi_id.integrator = 0.0f;
    s_pi_iq.integrator = 0.0f;
    s_id_target = 0.0f;
    s_iq_target = 0.0f;
}

void FOC_M1_CalibrateCurrentOffsets(void)
{
    /* Đảm bảo cầu H ở trạng thái ngắt xung */
    TIM8->BDTR &= ~TIM_BDTR_MOE; //[cite: 1]
    HAL_Delay(10); //[cite: 1]

    /* 1. Lưu lại cấu hình External Trigger hiện tại của ADC2 và ADC3 */
    uint32_t cr2_adc2_ext = ADC2->CR2 & (ADC_CR2_EXTEN | ADC_CR2_EXTSEL);
    uint32_t cr2_adc3_ext = ADC3->CR2 & (ADC_CR2_EXTEN | ADC_CR2_EXTSEL);

    /* 2. Tắt External Trigger để cho phép kích hoạt bằng phần mềm (Software Trigger) */
    ADC2->CR2 &= ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL);
    ADC3->CR2 &= ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL);

    uint32_t sum_ib = 0, sum_ic = 0; //[cite: 1]
    const int N = 2000; //[cite: 1]
    for (int i = 0; i < N; i++) { //[cite: 1]
        HAL_ADC_Start(&hadc2); //[cite: 1]
        if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
            sum_ib += HAL_ADC_GetValue(&hadc2);
        }
        HAL_ADC_Stop(&hadc2);

        HAL_ADC_Start(&hadc3); //[cite: 1]
        if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
            sum_ic += HAL_ADC_GetValue(&hadc3);
        }
        HAL_ADC_Stop(&hadc3);
    }
    s_offset_ib = (float)sum_ib / (float)N; //[cite: 1]
    s_offset_ic = (float)sum_ic / (float)N; //[cite: 1]

    /* 3. Khôi phục lại cấu hình Trigger Timer cứng cho FOC vòng kín sau này */
    ADC2->CR2 |= cr2_adc2_ext;
    ADC3->CR2 |= cr2_adc3_ext;

    /* Lưu ý: Vẫn giữ comment 3 dòng Start_IT này như bước trước */
    TIM8->BDTR |= TIM_BDTR_MOE;
    // HAL_ADC_Start_IT(&hadc2);
    // HAL_ADC_Start_IT(&hadc3);

    s_calibrated = 1;
}

void FOC_M1_CalibrateEncoderOffset(void)
{
    /* Đọc trung bình vài mẫu cho ổn định, rotor phải đang được giữ CỐ ĐỊNH bởi
     * open-loop align (theta_dien = 0) ngay trước khi gọi hàm này. */
    uint32_t sum = 0;
    const int N = 32;
    for (int i = 0; i < N; i++) {
        sum += AS5048A_ReadRaw();
        HAL_Delay(1);
    }
    s_encoder_offset_raw = (int32_t)(sum / N);
}

uint8_t FOC_M1_IsCalibrated(void) { return s_calibrated; }

void FOC_M1_SetElectricalAngle(float theta_rad) { s_theta = theta_rad; }

float FOC_M1_GetElectricalAngle(void) { return s_theta; }

/* Đọc AS5047, quy đổi ra góc điện (rad, đã trừ offset, nhân số cặp cực, wrap [0,2pi)) */
// static inline float ReadElectricalAngleFromEncoder(void)
// {
//     int32_t raw = (int32_t)AS5048A_ReadRaw();
//     int32_t delta = raw - s_encoder_offset_raw;

//     /* wrap delta về khoảng [0, CPR) để xử lý điểm quay vòng qua 0/16383 */
//     delta &= 0x3FFF;

//     float mech_angle = ((float)delta / FOC_ENCODER_CPR) * _2PI_F;
//     float theta_e = fmodf(mech_angle * (float)FOC_POLE_PAIRS * (float)FOC_ENCODER_DIR, _2PI_F);
//     if (theta_e < 0.0f) theta_e += _2PI_F;
//     return theta_e;
// }
static inline float ReadElectricalAngleFromEncoder(void)
{
    // s_encoder_offset_rad là offset quy đổi ra radian sau bước Align:
    s_encoder_offset_rad = ((float)s_encoder_offset_raw / 16384.0f) * _2PI_F;
    float mech_angle = angle_filtered_rad - s_encoder_offset_rad;

    // Tính góc điện có xét đến chiều encoder (+1 hoặc -1)
    float theta_e = mech_angle * (float)FOC_POLE_PAIRS * (float)FOC_ENCODER_DIR;

    // Chuẩn hóa theta_e về dải [0, 2*PI)
    theta_e = fmodf(theta_e, _2PI_F);
    if (theta_e < 0.0f) {
        theta_e += _2PI_F;
    }

    return theta_e;
}
void FOC_M1_SetIqTarget(float iq_amps)
{
    if (iq_amps > IQ_MAX)  iq_amps = IQ_MAX;
    if (iq_amps < -IQ_MAX) iq_amps = -IQ_MAX;
    s_iq_target = iq_amps;
}

void FOC_M1_SetIdTarget(float id_amps)
{
    if (id_amps > ID_MAX)  id_amps = ID_MAX;
    if (id_amps < -ID_MAX) id_amps = -ID_MAX;
    s_id_target = id_amps;
}

void FOC_M1_GetMeasuredCurrents(float *id, float *iq)
{
    if (id) *id = s_id_meas;
    if (iq) *iq = s_iq_meas;
}

/* Gọi từ HAL_ADC_ConvCpltCallback khi có đủ 2 mẫu IB (ADC2) + IC (ADC3) mới cùng chu kỳ */
void FOC_M1_CurrentLoopUpdate(uint16_t adc2_ib_raw, uint16_t adc3_ic_raw)
{
    if (!s_calibrated) return;

    /* 1. ADC -> dòng thực (A). denom = 4096*20*0.0005 = 40.96 */
    const float denom = ADC_MAX_COUNT * CSA_GAIN * R_SHUNT;
    float i_b = ((float)adc2_ib_raw - s_offset_ib) * ADC_VREF / denom;
    float i_c = ((float)adc3_ic_raw - s_offset_ic) * ADC_VREF / denom;
    float i_a = -(i_b + i_c); // KCL: Ia + Ib + Ic = 0

    /* 2. Bảo vệ quá dòng phần cứng - kiểm tra TRƯỚC khi tính toán thêm */
    if (fabsf(i_a) > IQ_MAX * CURRENT_TRIP_MARGIN * 2.0f ||
        fabsf(i_b) > IQ_MAX * CURRENT_TRIP_MARGIN * 2.0f ||
        fabsf(i_c) > IQ_MAX * CURRENT_TRIP_MARGIN * 2.0f) {
        FOC_M1_EmergencyStop();
        return;
    }

    /* 3. Clarke transform */
    float i_alpha = i_a;
    float i_beta  = (i_b - i_c) * 0.57735026919f; // 1/sqrt(3)

    /* 4. Đọc góc điện từ AS5047 (SPI3, ~12us cho 2 lần transfer @2.625MHz) rồi Park transform.
     * Lưu ý: việc đọc SPI trong ISR chiếm một phần chu kỳ 41.6us (24kHz) - nếu sau này cần
     * tăng tần số vòng dòng cao hơn, nên chuyển việc đọc encoder sang một timer/DMA riêng
     * chạy song song thay vì blocking ngay trong ADC ISR như ở đây. */
    float theta = ReadElectricalAngleFromEncoder();
    s_theta = theta;
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);
    float i_d =  i_alpha * cos_t + i_beta * sin_t;
    float i_q = -i_alpha * sin_t + i_beta * cos_t;

    s_id_meas = i_d;
    s_iq_meas = i_q;

    if (fabsf(i_d) > ID_MAX * CURRENT_TRIP_MARGIN || fabsf(i_q) > IQ_MAX * CURRENT_TRIP_MARGIN) {
        FOC_M1_EmergencyStop();
        return;
    }

    /* 5. PI dòng điện */
    float v_d = PI_Update(&s_pi_id, s_id_target - i_d);
    float v_q = PI_Update(&s_pi_iq, s_iq_target - i_q);

    /* 6. Inverse Park */
    float v_alpha = v_d * cos_t - v_q * sin_t;
    float v_beta  = v_d * sin_t + v_q * cos_t;

    /* 7. Inverse Clarke -> 3 pha ABC */
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + 0.86602540378f * v_beta;
    float v_c = -0.5f * v_alpha - 0.86602540378f * v_beta;

    SetPhaseDuty(v_a, v_b, v_c);
}

/* ===================== Callback ADC dùng chung cho ADC2/ADC3 ===================== */
static volatile uint16_t s_last_ib_raw = 0;
static volatile uint16_t s_last_ic_raw = 0;
static volatile uint8_t  s_have_ib = 0, s_have_ic = 0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2) {
        s_last_ib_raw = HAL_ADC_GetValue(&hadc2);
        s_have_ib = 1;
    } else if (hadc->Instance == ADC3) {
        s_last_ic_raw = HAL_ADC_GetValue(&hadc3);
        s_have_ic = 1;
    }

    if (s_have_ib && s_have_ic) {
        s_have_ib = 0;
        s_have_ic = 0;
        FOC_M1_CurrentLoopUpdate(s_last_ib_raw, s_last_ic_raw);
    }
}


void M1_OpenLoopAlign(float align_voltage_ratio, uint32_t hold_time_ms)
{
    if (align_voltage_ratio > 0.05f) {
        align_voltage_ratio = 0.05f;
    }

    /* Đặt vector điện áp tại theta_e = 0 */
    float ua = align_voltage_ratio;
    float ub = -0.5f * align_voltage_ratio;
    float uc = -0.5f * align_voltage_ratio;

    SetPhaseDuty(ua, ub, uc);

    /* Bật cầu H phát xung */
    TIM8->BDTR |= TIM_BDTR_MOE;

    /* Giữ nguyên vị trí để rotor ổn định cơ học */
    HAL_Delay(hold_time_ms);

    /* Đọc và chốt mốc raw encoder */
    FOC_M1_CalibrateEncoderOffset();

    /* Tắt cầu H trở lại trạng thái High-Z để an toàn */
    TIM8->BDTR &= ~TIM_BDTR_MOE;

    s_calibrated = 1;
}


void FOC_VoltageMode_Step(float vq_ratio)
{
    const float VQ_LIMIT = 0.30f;
    if (vq_ratio > VQ_LIMIT)  vq_ratio = VQ_LIMIT;
    if (vq_ratio < -VQ_LIMIT) vq_ratio = -VQ_LIMIT;

    // 1. Đọc góc điện từ cảm biến
    float theta = ReadElectricalAngleFromEncoder();
    float sin_t = sinf(theta);
    float cos_t = cosf(theta);

    // 2. Ép Vd = 0, Vq = điện áp mong muốn (Inverse Park)
    // vq_ratio nằm trong khoảng [-0.15, 0.15] (tương ứng 0 - 15% điện áp nguồn)
    float v_d = 0.0f;
    float v_q = vq_ratio;

    float v_alpha = v_d * cos_t - v_q * sin_t;
    float v_beta  = v_d * sin_t + v_q * cos_t;

    // 3. Inverse Clarke ra 3 pha
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + 0.86602540378f * v_beta;
    float v_c = -0.5f * v_alpha - 0.86602540378f * v_beta;

    // 4. Xuất duty cycle
    SetPhaseDuty(v_a, v_b, v_c);
}

void FOC_OpenLoopSpin(float vq, float elec_speed_rad_s, uint32_t ms)
{
    float theta = 0.0f;
    const float dt = 1.0f / 8000.0f; // sửa theo tần số ISR thực tế
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        theta += elec_speed_rad_s * dt;
        if (theta > _2PI_F) theta -= _2PI_F;
        float s = sinf(theta), c = cosf(theta);
        float va = -vq * s;
        float vb = -0.5f * va + 0.86602540378f * (vq * c);
        float vc = -0.5f * va - 0.86602540378f * (vq * c);
        SetPhaseDuty(va, vb, vc);
        // delay đúng dt (dùng DWT hoặc timer)
    }
}



void FOC_OpenLoopSpin1(float vq, float elec_speed_rad_s, uint32_t ms)
{
    float theta = 0.0f;
    const float dt = 0.000125f; // 125 us (8 kHz)
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < ms) {
        theta += elec_speed_rad_s * dt;
        if (theta >= _2PI) {
            theta -= _2PI;
        } else if (theta < 0.0f) {
            theta += _2PI;
        }

        float s = sinf(theta);
        float c = cosf(theta);

        float va = -vq * s;
        float vb = -0.5f * va + 0.86602540378f * (vq * c);
        float vc = -0.5f * va - 0.86602540378f * (vq * c);

        SetPhaseDuty(va, vb, vc);

        // DELAY CHÍNH XÁC 125 MICROSECOND ĐỂ ĐÚNG CHU KỲ dt
        DWT_Delay_us(125);
    }

    // Kết thúc: hạ duty về 0 để ngắt dòng giữ mát cuộn dây
    SetPhaseDuty(0.0f, 0.0f, 0.0f);
}

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL); // 168 ticks mỗi microsecond
    while ((DWT->CYCCNT - start) < ticks);
}
