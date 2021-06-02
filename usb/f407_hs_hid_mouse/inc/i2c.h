#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

#include "../inc/cmsis/device/stm32f4xx.h"

void i2c_init();
void i2c_write(uint8_t dev_addr, uint8_t mem_addr,  uint8_t *pData, int num_bytes);
void i2c_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t *pData, int num_bytes);

#endif /* I2C_H_ */
