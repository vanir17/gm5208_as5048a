#ifndef DRV8301_H
#define DRV8301_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

/* ===== Chân theo Odrive.ioc: M1_nCS = PC14 (PC14-OSC32_IN) ===== */
#define DRV8301_M1_CS_PORT   GPIOC
#define DRV8301_M1_CS_PIN    GPIO_PIN_14

/* Địa chỉ thanh ghi DRV8301 */
#define DRV8301_REG_STATUS1   0x00
#define DRV8301_REG_STATUS2   0x01
#define DRV8301_REG_CONTROL1  0x02
#define DRV8301_REG_CONTROL2  0x03

/* Gain của Current Shunt Amplifier, bit[3:2] Control Reg 2 */
typedef enum {
    DRV8301_GAIN_10VpV = 0x0,
    DRV8301_GAIN_20VpV = 0x1,
    DRV8301_GAIN_40VpV = 0x2,
    DRV8301_GAIN_80VpV = 0x3,
} DRV8301_Gain_t;

uint16_t DRV8301_M1_WriteReg(uint8_t regAddr, uint16_t data);
uint16_t DRV8301_M1_ReadReg(uint8_t regAddr);

/* Trả về 0 nếu init OK và không có fault, khác 0 nếu lỗi */
uint8_t DRV8301_M1_Init(DRV8301_Gain_t gain);

/* Đọc lại Status1 (địa chỉ 0x00). != 0 nghĩa là đang có fault (quá dòng/quá nhiệt/sụt áp) */
uint16_t DRV8301_M1_CheckFault(void);

/* Đổi enum gain sang giá trị số thực (V/V) để dùng khi quy đổi ADC -> Ampe */
float DRV8301_GainToFloat(DRV8301_Gain_t gain);

#endif /* DRV8301_H */