//=====================================================================================================
// @file LidarLitev3HP_register.h
//=====================================================================================================
//
// @brief List of used registers from LidarLite_v3HP
//
// Date                 Author                      Notes
// @date 24/12/2020     @author Tomas Schweizer     Implementation
//
// Source:
// Data sheet LidarLite_v3HP
//
//=====================================================================================================

#ifndef LIDARLITEV3HP_REGISTER_H_
#define LIDARLITEV3HP_REGISTER_H_


/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/
// 2 i2c addresses one for write and  one for read of the LidarLite_v3HP
#define LIDARLITEV3HP_ADDRESS                       0x62

// control registers
#define LIDARLITEv3HP_ACQ_COMMAND                   0x00
#define LIDARLITEv3HP_STATUS                        0x01
#define LIDARLITEv3HP_SIG_COUNT_VAL                 0x02
#define LIDARLITEv3HP_ACQ_CONFIG_REG                0x04
#define LIDARLITEv3HP_REF_COUNT_VAL                 0x12
#define LIDARLITEv3HP_THRESHOLD_BYPASS              0x1C
// read registers
#define LIDARLITEv3HP_DISTANCE_HIGH_BYTE            0x0f
#define LIDARLITEv3HP_DISTANCE_LOW_BYTE             0x10

// bit values for balanced mode !!! is used !!!
#define LIDARLITEv3HP_SIGCOUNTVAL_BALANCED          0x80
#define LIDARLITEv3HP_ACQCONFIGREG_BALANCED         0x08
#define LIDARLITEv3HP_REFCOUNTVAL_BALANCED          0x05
#define LIDARLITEv3HP_THRESHOLDBYPASS_BALANCED      0x00

// bit for short range high speed
#define LIDARLITEv3HP_SIGCOUNTVAL_SPEED             0x1D
#define LIDARLITEv3HP_ACQCONFIGREG_SPEED            0x08
#define LIDARLITEv3HP_REFCOUNTVAL_SPEED             0x03
#define LIDARLITEv3HP_THRESHOLDBYPASS_SPEED         0x00

// bit to initiate a measurement
#define LIDARLITEv3HP_ACQ_COMMAND_INIT_MEAS         0x01

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/
#endif /* LIDARLITEV3HP_REGISTER_H_ */

//=====================================================================================================
// End of file
//=====================================================================================================


