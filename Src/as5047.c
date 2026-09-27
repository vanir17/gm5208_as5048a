#include "as5047.h"

static inline void CS_LOW(void)  { HAL_GPIO_WritePin(AS5047_CS_PORT, AS5047_CS_PIN, GPIO_PIN_RESET); }
static inline void CS_HIGH(void) { HAL_GPIO_WritePin(AS5047_CS_PORT, AS5047_CS_PIN, GPIO_PIN_SET); }

void AS5047_GPIO_Init(void)
{
    CS_HIGH(); // đảm bảo HIGH trước, tránh chọm bus SPI3 khi DRV8301 đang giao tiếp

    GPIO_InitTypeDef gi = {0};
    gi.Pin   = AS5047_CS_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AS5047_CS_PORT, &gi);

    HAL_GPIO_WritePin(AS5047_CS_PORT, AS5047_CS_PIN, GPIO_PIN_SET);}

static uint8_t EvenParityBit(uint16_t v)
{
    /* tính parity chẵn trên 15 bit thấp (bit14..0) */
    v &= 0x7FFF;
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return (uint8_t)(v & 0x1);
}

static uint16_t AS5047_Transfer(uint16_t command_frame)
{
    uint16_t rx = 0;
    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&command_frame, (uint8_t*)&rx, 1, 10);
    CS_HIGH();
    return rx;
}

static uint16_t AS5047_ReadRegister(uint16_t reg_addr)
{
    /* Frame lệnh đọc: bit14=1 (Read), bit13..0 = địa chỉ, bit15 = parity chẵn */
    uint16_t frame = (1u << 14) | (reg_addr & 0x3FFF);
    if (EvenParityBit(frame)) frame |= (1u << 15);

    /* AS5047 trả dữ liệu của lệnh TRƯỚC ĐÓ trên transfer hiện tại (kiểu daisy-chain 1 tầng),
     * nên phải gửi lệnh 2 lần: lần 1 "nạp" lệnh đọc, lần 2 mới lấy được dữ liệu tương ứng. */
    (void)AS5047_Transfer(frame);
    uint16_t resp = AS5047_Transfer(frame);

    /* Bit14 của response là Error Flag (EF). Nếu set, dữ liệu 14-bit thấp không đáng tin. */
    return resp; // caller tự lọc EF/parity nếu cần kiểm tra chặt
}

uint16_t AS5047_ReadAngleRaw(void)
{
    uint16_t resp = AS5047_ReadRegister(AS5047_REG_ANGLECOM);
    return resp & 0x3FFF; // 14-bit angle, 0..16383 = 0..360 độ cơ khí
}

uint16_t AS5047_ReadDiagnostics(void)
{
    uint16_t resp = AS5047_ReadRegister(AS5047_REG_DIAAGC);
    return resp & 0x3FFF;
}