#include "drv8301.h"

static inline void CS_LOW(void)  { HAL_GPIO_WritePin(DRV8301_M1_CS_PORT, DRV8301_M1_CS_PIN, GPIO_PIN_RESET); }
static inline void CS_HIGH(void) { HAL_GPIO_WritePin(DRV8301_M1_CS_PORT, DRV8301_M1_CS_PIN, GPIO_PIN_SET); }

/* Lưu ý: PC13 (M0_nCS) đã được MX_GPIO_Init() set HIGH sẵn theo .ioc,
 * nên không đụng vào bus SPI3 khi ta giao tiếp với M1. */

uint16_t DRV8301_M1_WriteReg(uint8_t regAddr, uint16_t data)
{
    uint16_t txData = ((uint16_t)(regAddr & 0x0F) << 11) | (data & 0x07FF);
    uint16_t rxData = 0;

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&txData, (uint8_t*)&rxData, 1, 100);
    CS_HIGH();

    return rxData;
}

uint16_t DRV8301_M1_ReadReg(uint8_t regAddr)
{
    uint16_t txData = (1u << 15) | ((uint16_t)(regAddr & 0x0F) << 11);
    uint16_t rxData = 0;

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&txData, (uint8_t*)&rxData, 1, 100);
    CS_HIGH();

    return rxData;
}

float DRV8301_GainToFloat(DRV8301_Gain_t gain)
{
    switch (gain) {
        case DRV8301_GAIN_10VpV: return 10.0f;
        case DRV8301_GAIN_20VpV: return 20.0f;
        case DRV8301_GAIN_40VpV: return 40.0f;
        case DRV8301_GAIN_80VpV: return 80.0f;
        default: return 20.0f;
    }
}

uint8_t DRV8301_M1_Init(DRV8301_Gain_t gain)
{
    CS_HIGH();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // EN_GATE = HIGH[cite: 2]
    HAL_Delay(20);

    /* 1. Ghi cấu hình Gain vào CONTROL2 */
    uint16_t control_reg_2 = ((uint16_t)gain & 0x3) << 2;
    DRV8301_M1_WriteReg(DRV8301_REG_CONTROL2, control_reg_2);
    HAL_Delay(1);

    /* 2. Đọc lại CONTROL2 để xác nhận:
     * Lần đọc 1: gửi địa chỉ CONTROL2
     * Lần đọc 2: DRV8301 mới trả về dữ liệu của CONTROL2 */
    (void)DRV8301_M1_ReadReg(DRV8301_REG_CONTROL2);
    uint16_t readback = DRV8301_M1_ReadReg(DRV8301_REG_CONTROL2);
    if ((readback & 0x07FF) != control_reg_2) {
        return 1; // Ghi/đọc SPI thất bại[cite: 2]
    }

    /* 3. Đọc STATUS1 kiểm tra Fault:
     * Cần đọc 2 lần để xả dữ liệu cũ và nhận STATUS1 */
    (void)DRV8301_M1_ReadReg(DRV8301_REG_STATUS1);
    uint16_t stat1 = DRV8301_M1_ReadReg(DRV8301_REG_STATUS1);

    /* Mask 0x07FF: chỉ kiểm tra bit lỗi (Bit 10 là FAULT, Bit 9..0 là các mã lỗi UV/OCP/OTW) */
    if ((stat1 & 0x07FF) != 0) {
        return 2; // Đang có lỗi phần cứng thật sự[cite: 2]
    }

    return 0; // OK[cite: 2]
}

uint16_t DRV8301_M1_CheckFault(void)
{
    (void)DRV8301_M1_ReadReg(DRV8301_REG_STATUS1);
    uint16_t stat1 = DRV8301_M1_ReadReg(DRV8301_REG_STATUS1);
    return (stat1 & 0x07FF); // Trả về 0 nếu không có lỗi[cite: 2]
}