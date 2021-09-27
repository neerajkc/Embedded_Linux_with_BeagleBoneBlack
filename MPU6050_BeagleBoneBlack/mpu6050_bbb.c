/*
 * mpu6050_bbb.c
 *
 * This application collects and reports the acceleration and gyro data from the
 * MPU6050 sensor. The sensor interfaces via I2C protocol.
 *
 *  Created on: 27-Sep-2021
 *      Author: neeraj
 */


#include "mpu6050_bbb.h"

int fd;	// file descriptor

int main(void)
{

	short accl_val_arr[3], gyro_val_arr[3]; // These hold acceleration and gyro data
	double accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z;

	/* Open the I2C device file */
	if ((fd = open(I2C_DEVICE_FILE, O_RDWR)) < 0)
	{
	   perror("Failed to open I2C device file.\n");
	   return -1;
	}

	/* Set the I2C slave address using ioctl I2C_SLAVE command */
	if (ioctl(fd, I2C_SLAVE, MPU6050_SLAVE_ADDR) < 0)
	{
		   perror("Failed to set I2C slave address.\n");
		   close(fd);
		   return -1;
	}

	while(1)
	{

		mpu6050_init();

		get_accl_data(accl_val_arr);
		//get_gyro_data(gyro_val_arr); // TODO

		/* convert raw readings to values relative to 'g' */
        accl_x = (double) accl_val_arr[0] / ACCL_FS_SENSITIVITY_0;
        accl_y = (double) accl_val_arr[1] / ACCL_FS_SENSITIVITY_0;
        accl_z = (double) accl_val_arr[2] / ACCL_FS_SENSITIVITY_0;

        printf("%0.2f	%0.2f	%0.2f\n",accl_x, accl_y, accl_z);

        usleep(50 * 1000);
	}
}



/* By default mpu6050 is in sleep mode, so wake it up and configure
 * the full scale ranges for acc */
void mpu6050_init()
{
    // Disable sleep mode
    mpu6050_write(MPU6050_REG_POWER, 0x00);
    usleep(500);

    // Adjust full scale values for accelerator
    mpu6050_write(MPU6050_REG_ACCEL_CONFIG, 0x00);
    usleep(500);
}



/* write 8bit "data" to the sensor address "addr" */
int mpu6050_write(uint8_t addr, uint8_t data)
{
  int ret;
  char buf[2];

  buf[0]=addr;
  buf[1]=data;

  ret = write(fd, buf, 2);
  if (ret <= 0)
  {
      perror("write failed\n");
      return -1;
  }
  return 0;
}



/* read "len" bytes from sensor "addr" in to pBuffer */
int mpu6050_read(uint8_t base_addr, char *pBuffer, uint32_t len)
{
	int ret;
	char buf[2];

	buf[0] = base_addr;

	ret = write(fd, buf, 1);
	if (ret <= 0)
	{
	  perror("write address failed\n");
	  return -1;
	}

	ret = read(fd, pBuffer, len);
	if(ret <= 0)
	{
	  perror("read failed\n");
	}
	return 0;
}



/* read acceleration x,y,z values in to pBuffer */
void get_accl_data(short int *pBuffer)
{
    //each axis value is of 2byte, so we need a buffer of 6bytes.
    char accl_buffer[6];

    //start reading from the base address of accelerometer values i.e MPU6050_REG_ACC_X_HIGH
    mpu6050_read(MPU6050_REG_ACC_X_HIGH, accl_buffer, 6);

    /* pBuffer[0] = accl x axis value, pBuffer[1] = accl y axis value, pBuffer[2] = accl z axis value  */
    pBuffer[0] = (int) ( (accl_buffer[0] << 8) |  accl_buffer[1] );
    pBuffer[1] = (int) ( (accl_buffer[2] << 8) |  accl_buffer[3] );
    pBuffer[2] = (int) ( (accl_buffer[4] << 8) |  accl_buffer[5] );

}

