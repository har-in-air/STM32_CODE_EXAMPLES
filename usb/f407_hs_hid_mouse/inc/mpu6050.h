#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

#define MPU6050_ADDR 		0xD0

#define SMPLRT_DIV_REG 		0x19
#define GYRO_CONFIG_REG 	0x1B
#define ACCEL_CONFIG_REG 	0x1C
#define ACCEL_XOUT_H_REG 	0x3B
#define TEMP_OUT_H_REG 		0x41
#define GYRO_XOUT_H_REG 	0x43
#define PWR_MGMT_1_REG 		0x6B
#define WHO_AM_I_REG 		0x75

extern int8_t Axi, Ayi;

void mpu6050_init(void);
void mpu6050_read_accel(void);

#endif /* MPU6050_H_ */
