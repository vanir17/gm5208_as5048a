#ifndef FOC_H
#define FOC_H

#include <stdint.h>

/* Tần số vòng dòng = tần số update của TIM8 (PWM ~ 168MHz/(3500*2) = 24kHz center-aligned,
 * RepetitionCounter=2 -> 1 update mỗi chu kỳ PWM đầy đủ => 24kHz vòng dòng). Sửa lại nếu khác. */
#define FOC_LOOP_FREQ_HZ   24000.0f

/* ===== Motor AS2212 13T 1000KV + Encoder AS5047 =====
 * AS2212 13T 1000KV thường là motor 14 cực (7 cặp cực) - KIỂM TRA LẠI với motor thực tế của bạn. */
#define FOC_POLE_PAIRS      7
#define FOC_ENCODER_CPR     16384.0f   // AS5047: 14-bit = 16384 count / vòng cơ khí

/* Đổi thành -1 nếu sau khi chạy closed-loop, dòng Iq/Id không bám được setpoint hoặc
 * motor có xu hướng tăng tốc mất kiểm soát (dấu hiệu kinh điển của sai chiều encoder
 * gây phản hồi dương). Luôn test lần đầu với Iq rất nhỏ và tay sẵn sàng ngắt nguồn. */
#define FOC_ENCODER_DIR     1

void FOC_M1_Init(void);

/* Hiệu chuẩn offset dòng điện. Gọi 1 lần lúc khởi động, khi PWM đang bị tắt (MOE=0). */
void FOC_M1_CalibrateCurrentOffsets(void);

/* Hiệu chuẩn offset góc encoder. PHẢI gọi NGAY SAU khi rotor đã được giữ cố định ở
 * theta_dien = 0 bằng open-loop align (M1_OpenLoopAlign trong main.c), vì hàm này
 * đọc góc thô AS5047 tại thời điểm đó và lưu làm mốc 0 điện. */
void FOC_M1_CalibrateEncoderOffset(void);

uint8_t FOC_M1_IsCalibrated(void);

/* Góc điện (rad) đặt thủ công - CHỈ dùng trong giai đoạn open-loop align trước khi
 * bật closed-loop. Sau khi closed-loop chạy, FOC_M1_CurrentLoopUpdate() tự đọc AS5047
 * và ghi đè giá trị này mỗi chu kỳ, gọi hàm này lúc đó sẽ không có tác dụng. */
void FOC_M1_SetElectricalAngle(float theta_rad);

/* Đọc góc điện hiện tại (đã tính từ encoder) - phục vụ debug/log */
float FOC_M1_GetElectricalAngle(void);

/* Đặt dòng mục tiêu Iq (mô-men) và Id (thường = 0 với motor không có reluctance) */
void FOC_M1_SetIqTarget(float iq_amps);
void FOC_M1_SetIdTarget(float id_amps);

/* Được gọi từ HAL_ADC_ConvCpltCallback() khi có đủ mẫu ADC2 (IB) + ADC3 (IC) mới.
 * Đây là vòng lặp điều khiển dòng chạy trong ngắt. */
void FOC_M1_CurrentLoopUpdate(uint16_t adc2_ib_raw, uint16_t adc3_ic_raw);

/* Đọc dòng Id/Iq đo được gần nhất, phục vụ debug/log qua UART/CAN */
void FOC_M1_GetMeasuredCurrents(float *id, float *iq);

/* Dừng khẩn cấp: High-Z ngay lập tức + tắt EN_GATE */
void FOC_M1_EmergencyStop(void);

void M1_OpenLoopAlign(float align_voltage_ratio, uint32_t hold_time_ms);

void FOC_VoltageMode_Step(float vq_ratio);
#endif /* FOC_H */