#ifndef AS5047_H
#define AS5047_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

/* ===== CS của AS5047 =====
 * AS5047 và DRV8301 (M1_nCS=PC14) dùng CHUNG SPI3, nên encoder cần 1 chân CS riêng.
 * Đang gán TẠM vào GPIO_5 = PC4 (chân IO tự do trên header Odrive v3.6 theo Odrive.ioc).
 * NẾU bạn đấu dây CS của AS5047 vào chân khác, SỬA 2 macro dưới cho khớp phần cứng. */
#define AS5047_CS_PORT   GPIOA
#define AS5047_CS_PIN    GPIO_PIN_3

/* Địa chỉ thanh ghi AS5047 (giao thức 16-bit: bit15=parity chẵn, bit14=R/W, bit13..0=addr/data) */
#define AS5047_REG_ANGLECOM   0x3FFF  // Góc đã bù lỗi động (DAEC)
#define AS5047_REG_DIAAGC     0x3FFC  // AGC / chẩn đoán cường độ từ trường

/* Cấu hình chân CS làm output, mặc định HIGH (không chọn chip) - gọi 1 lần lúc init */
void AS5047_GPIO_Init(void);

/* Đọc góc thô 14-bit tuyệt đối, giá trị 0..16383 tương ứng 0..2*pi cơ khí */
uint16_t AS5047_ReadAngleRaw(void);

/* Đọc thanh ghi chẩn đoán (bit AGC, quá/thiếu từ trường) để kiểm tra lắp đặt nam châm */
uint16_t AS5047_ReadDiagnostics(void);

#endif /* AS5047_H */