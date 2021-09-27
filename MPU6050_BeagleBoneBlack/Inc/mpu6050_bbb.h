/*
 * mpu6050_bbb.h
 *
 *  Created on: 27-Sep-2021
 *      Author: neeraj
 */

#ifndef MPU6050_BBB_H_
#define MPU6050_BBB_H_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/* Linux OS device file for I2C2 controller of the SOC */
#define I2C_DEVICE_FILE   		 "/dev/i2c-2"

/* I2C slave address of MPU6050 */
#define MPU6050_SLAVE_ADDR				 0x68

/*MPU6050 register addresses */

#define MPU6050_REG_POWER                0x6B
#define MPU6050_REG_ACCEL_CONFIG         0x1C

/* MPU6050 registers for accelerator data */
#define MPU6050_REG_ACC_X_HIGH           0x3B
#define MPU6050_REG_ACC_X_LOW            0x3C
#define MPU6050_REG_ACC_Y_HIGH           0x3D
#define MPU6050_REG_ACC_Y_LOW            0x3E
#define MPU6050_REG_ACC_Z_HIGH           0x3F
#define MPU6050_REG_ACC_Z_LOW			 0x40

/* MPU6050 full scale ranges for accelerator sensitivity */
#define ACCL_FS_SENSITIVITY_0			16384
#define ACCL_FS_SENSITIVITY_1		     8192
#define ACCL_FS_SENSITIVITY_2		     4096
#define ACCL_FS_SENSITIVITY_3		     2048


/* function prototypes */
void mpu6050_init();
int mpu6050_write(uint8_t addr, uint8_t data);
int mpu6050_read(uint8_t base_addr, char *pBuffer, uint32_t len);
void get_accl_data(short int *pBuffer);

#endif /* MPU6050_BBB_H_ */
