#include "as5048a.h"

static SPI_HandleTypeDef *as5048a_spi = NULL;

static uint16_t AS5048A_CalcParity(uint16_t value)
{
    uint8_t count = 0;
    for(int i = 0; i < 16; i++)
    {
        if(value & (1 << i))
        {
            count++;
        }
    }
    
    return (count & 1) ? 0x8000 : 0x0000;
}

static uint16_t AS5048A_BuildReadCommand(uint16_t address)
{
    uint16_t cmd = 0x4000 | (address & 0x3FFF);
    cmd |= AS5048A_CalcParity(cmd);
    return cmd;
}

static void AS5048A_CS_Low(void)
{
    HAL_GPIO_WritePin(AS5048A_CS_PORT, AS5048A_CS_PIN, GPIO_PIN_RESET);
}

static void AS5048A_CS_High(void)
{
    HAL_GPIO_WritePin(AS5048A_CS_PORT, AS5048A_CS_PIN, GPIO_PIN_SET);
}


/* Init function*/
void AS5048A_Init(SPI_HandleTypeDef *hspi)
{
    as5048a_spi = hspi;
    AS5048A_CS_High();
}

/* Communication 1 frame 16-bit*/
static uint16_t AS5048A_Transfer(uint16_t tx_data)
{
    uint16_t rx_data = 0;

    /*Low*/
    AS5048A_CS_Low();

    /*Transmit and receive 16-bit*/
    HAL_SPI_TransmitReceive(as5048a_spi, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 1,10);

    /*High*/
    AS5048A_CS_High();
    
    return rx_data;

}

uint16_t AS5048A_ReadRaw(void)
{
    uint16_t cmd = AS5048A_BuildReadCommand(AS5048A_CMD_ANGLE);

    //Request to read angle at frame 1
    AS5048A_Transfer(cmd);

    //At frame 2, request NOP to receive frame 1 's result
    uint16_t response = AS5048A_Transfer (0x0000);

    if(response & 0x4000)
    {
        return 0xFFFF;
    }

    return (response & 0x3FFF);
}

float AS5048A_ReadAngle(void) 
{
    uint16_t raw = AS5048A_ReadRaw();
    if (raw == 0xFFFF) 
    {
        return -1.0f; 
    }
    return (float)raw * 360.0f / 16384.0f;
}

float AS5048A_ReadRad(void)
{
    uint16_t raw = AS5048A_ReadRaw();
    if(raw == 0xFFFF)
    {
        return -1.0f;
    }
    return (float)raw * 6.28318530718f / 16384.0f;
}