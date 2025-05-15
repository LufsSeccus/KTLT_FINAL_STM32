#include "main.h"

#ifndef _STM32_MPU9250_I2C_H_
#define _STM32_MPU9250_I2C_H_

#define MPU_SCL_Pin        GPIO_PIN_6
#define MPU_SCL_GPIO_Port  GPIOB

#define MPU_SDA_Pin        GPIO_PIN_7
#define MPU_SDA_GPIO_Port  GPIOB

int stm32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data);
int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data);

#endif // _STM32_MPU9250_I2C_H_
