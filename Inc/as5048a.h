#ifndef __AS5048A_H
#define __AS5048A_H

#include "main.h"
#include "stm32f4xx_hal.h"

/* Định nghĩa chân CSn tại PA3 */
#define AS5048A_CS_PIN        GPIO_PIN_4
#define AS5048A_CS_PORT       GPIOC

#define AS5048A_CMD_ANGLE 0x3FFE
#define AS5048A_ERR 0xFFFF


uint8_t AS5048A_Init(SPI_HandleTypeDef *hspi);

uint16_t AS5048A_ReadRaw(void);

void AS5048A_Prime(void);
uint16_t AS5048A_ReadFast(void);

float AS5048A_ReadAngle(void);
float AS5048A_ReadRad(void);


#endif /* __AS5048A_H */