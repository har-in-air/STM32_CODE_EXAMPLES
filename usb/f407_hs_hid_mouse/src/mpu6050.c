#include "i2c.h"
#include "mpu6050.h"

int8_t Axi, Ayi;

void mpu6050_init (void){
	uint8_t device_id;
	uint8_t reg_val;

	i2c_read(MPU6050_ADDR, WHO_AM_I_REG, &device_id, 1);

	if (device_id == 0x68)  {// 0x68 will be returned by the sensor if everything goes well
		// power management register 0X6B we should write all 0's to wake the sensor up
		reg_val = 0;
		i2c_write( MPU6050_ADDR, PWR_MGMT_1_REG,&reg_val, 1);
		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		reg_val = 0x07;
		i2c_write( MPU6050_ADDR, SMPLRT_DIV_REG, &reg_val, 1);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		reg_val = 0x00;
		i2c_write( MPU6050_ADDR, ACCEL_CONFIG_REG, &reg_val, 1);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		reg_val = 0x00;
		i2c_write( MPU6050_ADDR, GYRO_CONFIG_REG, &reg_val, 1);
		}
	}


void mpu6050_read_accel (void){
	uint8_t raw_data[4];
	int16_t x_raw, y_raw;

	// Read accelerometer data
	i2c_read(MPU6050_ADDR, ACCEL_XOUT_H_REG, raw_data, 4);

	x_raw = (int16_t)(raw_data[0] << 8 | raw_data [1]);
	y_raw = (int16_t)(raw_data[2] << 8 | raw_data [3]);

	// scaled values in -127 to 127 range for mouse movement
	Axi = (int8_t)(x_raw/1638);
	Ayi = (int8_t)(y_raw/1638);
	}



