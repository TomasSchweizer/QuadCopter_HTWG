/*
 * ms5611_register.h
 *
 *  Created on: 02.01.2021
 *      Author: Tomas Schweizer
 */

#ifndef MS5611_REGISTER_H_
#define MS5611_REGISTER_H_


// i2c address of the MS5611
#define BARO_ADDRESS                0x77

// MS5611 commands and registers
#define BARO_RESET                  0x1E //Needs to be called at start-up

#define BARO_PROM_READ                   0xA0 // Needs to be read out to get 6 calibration values before start-up
/*
 * The calibration values lie at: 0xA0 to 0xAE
 * In this application we read them in 2 uint8 values at a time
 */

#define BARO_CONVER_PRESS_256       0x40 // Conversion request for pressure with precision OSR=256
#define BARO_CONVER_PRESS_512       0x42 // Conversion request for pressure with precision OSR=512
#define BARO_CONVER_PRESS_1024      0x44 // Conversion request for pressure with precision OSR=1024
#define BARO_CONVER_PRESS_2048      0x46 // Conversion request for pressure with precision OSR=2048
#define BARO_CONVER_PRESS_4096      0x48 // Conversion request for pressure with precision OSR=4096 !!! This is used !!!

#define BARO_CONVER_TEMP_256        0x50 // Conversion request for temperature with precision OSR=256
#define BARO_CONVER_TEMP_512        0x52 // Conversion request for temperature with precision OSR=512
#define BARO_CONVER_TEMP_1024       0x54 // Conversion request for temperature with precision OSR=1024
#define BARO_CONVER_TEMP_2048       0x56 // Conversion request for temperature with precision OSR=2048
#define BARO_CONVER_TEMP_4096       0x58 // Conversion request for temperature with precision OSR=4096 !!! This is used !!!

#define BARO_ADC_READ_COMMAND       0x00 // To read out conversion result






#endif /* MS5611_REGISTER_H_ */
