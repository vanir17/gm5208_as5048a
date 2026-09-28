#include "as5048a.h"

static SPI_HandleTypeDef *as5048a_spi = NULL;

#define SPI_WAIT_LOOPS 20000u

static inline uint16_t CalcParityBit(uint16_t v)
{
    return (__builtin_popcount(v) & 1) ? 0x8000 : 0x0000;
}

static uint16_t BuildReadCommand(uint16_t address)
{
    uint16_t cmd = 0x4000 | (address & 0x3FFF);
    return cmd | CalcParityBit(cmd);
}

static inline void Delay350ns(void)
{
    for(volatile int i = 0; i < 12; i++)
    {
        __NOP();
    }
}

static inline void CS_Low(void)
{
    AS5048A_CS_PORT->BSRR = ((uint32_t)AS5048A_CS_PIN) << 16;
    Delay350ns();
}

static inline void CS_High(void)
{
    AS5048A_CS_PORT->BSRR = AS5048A_CS_PIN; 
    Delay350ns();
}

uint8_t AS5048A_Init(SPI_HandleTypeDef *hspi)
{
    as5048a_spi = hspi;
    CS_High();
    if (hspi->Init.DataSize != SPI_DATASIZE_16BIT) return 1;
    __HAL_SPI_ENABLE(hspi);
    return 0;
}

static uint16_t Transfer(uint16_t tx)
{
    SPI_TypeDef *spi = as5048a_spi->Instance;
    uint32_t t;
    uint16_t rx = AS5048A_ERR;
 
    if (spi->SR & SPI_SR_RXNE) { (void)spi->DR; }   /* xả dữ liệu cũ */
 
    CS_Low();
 
    t = SPI_WAIT_LOOPS;
    while (!(spi->SR & SPI_SR_TXE) && --t) {}
    if (t) {
        spi->DR = tx;
        t = SPI_WAIT_LOOPS;
        while (!(spi->SR & SPI_SR_RXNE) && --t) {}
        if (t) rx = (uint16_t)spi->DR;
        t = SPI_WAIT_LOOPS;
        while ((spi->SR & SPI_SR_BSY) && --t) {}
    }
 
    CS_High();
    return rx;
}

static uint16_t DecodeResponse(uint16_t resp)
{
    if (resp == AS5048A_ERR)          return AS5048A_ERR;   /* timeout */
    if (resp & 0x4000)                return AS5048A_ERR;   /* cờ lỗi */
    if (__builtin_popcount(resp) & 1) return AS5048A_ERR;   /* sai parity (even) */
    return resp & 0x3FFF;
}

uint16_t AS5048A_ReadRaw(void)
{
    Transfer(BuildReadCommand(AS5048A_CMD_ANGLE));   /* frame 1: gửi lệnh */
    return DecodeResponse(Transfer(0x0000));         /* frame 2: NOP, nhận kết quả frame 1 */
}
 
void AS5048A_Prime(void)
{
    Transfer(BuildReadCommand(AS5048A_CMD_ANGLE));
}


uint16_t AS5048A_ReadFast(void)
{
    return DecodeResponse(Transfer(BuildReadCommand(AS5048A_CMD_ANGLE)));
}
 
float AS5048A_ReadAngle(void)
{
    uint16_t raw = AS5048A_ReadRaw();
    if (raw == AS5048A_ERR) return -1.0f;
    return (float)raw * 360.0f / 16384.0f;
}
 
float AS5048A_ReadRad(void)
{
    uint16_t raw = AS5048A_ReadRaw();
    if (raw == AS5048A_ERR) return -1.0f;
    return (float)raw * 6.28318530718f / 16384.0f;
}










