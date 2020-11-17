/*
 * MPU9250_registers.h
 *
 *  Created on: 09.03.2015
 *      Author: Ecki
 */

#ifndef MPU9250_REGISTERS_H_
#define MPU9250_REGISTERS_H_

/*
 * I2CSlave
 */
// This is the devices I2C Adress, when AD0 Pin is connected to low (0)
#define MPU_SLAVE_ADDR_0 0b01101000
// This is the devices I2C Adress, when AD0 Pin is connected to high (1)
#define MPU_SLAVE_ADDR_1 0b01101001

/*
 * Registers
 * Add more if needed
 */
#define SMPLRT_DIV 			0x19
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG	 	0x1C
#define I2C_MST_CTRL		0x24
#define PWR_MGMT_1 			0x6B
#define INT_PIN_CFG			0x37
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define USER_CTRL 			0x6A

// Test register
#define WHO_AM_I 			0x75	// for testing the communication
#define I_AM_9250			0x71	// Who am I answer on 9250
#define I_AM_9255			0x73	// Who am I answer on 9255

#define MAGNET_ADRESS_R		0x0C + 0x80 // I2C Read Adress for Magnetometer (AK8963)
#define MAGNET_ADRESS_W		0x0C // I2C Write Adress for Magnetometer (AK8963)
#define MAGNET_ADRESS		0x0C // I2C Adress for Magnetometer (AK8963)
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV0_DO			0x63

// Magnetometer Registers
#define MAGNET_WIA		0x00	// Device ID of AKM. It is described in one byte and fixed value (0x48)
#define MAGNET_HXL		0x03	// magnet x-axis low byte
#define MAGNET_ASAX		0x10
#define MAGNET_ASAY		0x11
#define MAGNET_ASAZ		0x12
#define MAGNET_CNTL1	0x0A
#define MAGNET_CNTL2	0x0B
#define MAGNET_ASTC		0x0C // self-test register

/*
 * Options
 */

#define ACCEL_FS_2g			0x00 + 0b00 << 3
#define ACCEL_FS_4g			0x00 + 0b01 << 3
#define ACCEL_FS_8g			0x00 + 0b10 << 3
#define ACCEL_FS_16g		0x00 + 0b11 << 3
#define GYRO_FS_250dps		0x00 + 0b00 << 3
#define GYRO_FS_500dps		0x00 + 0b01 << 3
#define GYRO_FS_1000dps		0x00 + 0b10 << 3
#define GYRO_FS_2000dps		0x00 + 0b11 << 3

#define BYPASS_EN			0x02
#define ALL_DISABLED		0x00
#define INT_20MHZ_CLOCK		0x00
#define H_RESET				0x80

#define I2C_IF_DIS_SET		0b00010000
#define I2C_MST_EN			0b00100000
#define WAIT_FOR_ES			0b01000000
#define I2C_MST_RST			0b00000010
#define I2C_MST_P_NSR		0b00010000
#define I2C_MST_CLK_258kHz	0x08
#define I2C_MST_CLK_400kHz	0x0D

#define I2C_SLV_EN			0b10000000
#define I2C_SLV_REG_DIS_EN	0b00100000
#define I2C_SLV_LENG_1		0x01
#define I2C_SLV_LENG_2		0x02
#define I2C_SLV_LENG_6		0x06
#define I2C_SLV_LENG_7		0x07

#define MAGNET_CONTINUOUS_MODE1	0x02
#define MAGNET_CONTINUOUS_MODE2	0x06
#define MAGNET_16_BIT_OUTPUT	0x10
#define MAGNET_SRST				0x01
#define MAGNET_SELFTEST_ON		0x40 // Generate magnetic field for self-test
#define MAGNET_SELFTEST_OFF		0x00 // Normal mode (no self-test)

// Add to MPU Register Adress to make a read access, otherwise data is written
#define MPU_READ 0x80



#endif /* MPU9250_REGISTERS_H_ */
