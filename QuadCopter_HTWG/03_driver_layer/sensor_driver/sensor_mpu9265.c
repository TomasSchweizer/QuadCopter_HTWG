//=====================================================================================================
// @file sensor_mpu9265.h
//=====================================================================================================
//
// @brief Implementation of a mpu9265 function library
//
// Date                 Author                      Notes
// @date 06/12/2020     @author Tomas Schweizer     Overall changes to fit to application
//
// Source:
// TivaWare SensorLib
//
//=====================================================================================================

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/
// standard libaries
#include <stdint.h>

#include "sensorlib/i2cm_drv.h"

#include "mpu9265_registers.h"
#include "ak8963_registers.h"

#include "sensor_mpu9265.h"


/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/
enum mpu9265State_e
{
    MPU9265_STATE_IDLE,                     // state machine is idle
    MPU9265_STATE_INIT_RESET,               // reset request issued.
    MPU9265_STATE_INIT_RESET_WAIT,          // polling wait for reset complete
    MPU9265_STATE_INIT_PWR_MGMT,            // wake up the device.
    MPU9265_STATE_INIT_SAMPLE_RATE_CFG,     // init the sensors
    MPU9265_STATE_INIT_ACCEL_CONFIG,        // init accelerometer configuration
    MPU9265_STATE_INIT_GYRO_CONFIG,         // init gyro configuration
    MPU9265_STATE_INIT_I2C_BYPASS,          // init i2c bypass pin
    MPU9265_STATE_INIT_AK8963,              // config ak8963
    MPU9265_STATE_INIT_FINISHED,             // init of mpu and ak is finished
    MPU9265_STATE_READ_DATA,                // waiting for data read MPU9256
    MPU9265_STATE_READ_DATA_AK8693          // waiting for data read AK8693
};

enum ak8963State_e
{
    AK8963_STATE_IDLE,                      // state machine is idle
    AK8963_STATE_INIT_RESET,                // state requested reset
    AK8963_STATE_INIT_CONFIG_FINISHED,      // configured AK8963
    AK8963_STATE_READ_DATA                  // waiting AK8693 read data

};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/
static void MPU9265Callback(void *p_MPU9265CallbackData, uint_fast8_t ui8_i2cState);
static AK8963_s* MPU9265GetAK8963Inst(MPU9265_s *ps_inst);

static void AK8963Callback(void *p_AK8693CallbackData, uint_fast8_t ui8_i2cState);
static uint8_t AK8963_Init(AK8963_s *ps_ak_inst, tI2CMInstance *ps_I2CMInst, uint8_t ui8_AK8963Address,
                           tSensorCallback *fp_AK8963Callback, void *p_AK8963CallbackData);
static uint8_t AK8963_ReadData(AK8963_s *ps_ak_inst, tSensorCallback *fp_AK8963Callback, void *p_AK8963CallbackData);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * \brief   The callback function that is called when I2C transactions to/from the
 *          MPU9150 have completed. Implemented as state machine.
 */
static void MPU9265Callback(void *p_MPU9265CallbackData, uint_fast8_t ui8_i2cState){

    MPU9265_s *ps_inst;

    // Convert the instance data into a pointer to a MPU9265 structure
    ps_inst = p_MPU9265CallbackData;

    // If the I2C master driver encountered a failure, force the state machine
    // to the idle state (which will also result in a callback to propagate the
    // error). Except in the case that we are in the reset wait state and the
    // error is an address NACK.  This error is handled by the reset wait
    // state.
    if((ui8_i2cState != I2CM_STATUS_SUCCESS) &&
       !((ui8_i2cState == I2CM_STATUS_ADDR_NACK) &&
         (ps_inst->ui8_MPU9265State == MPU9265_STATE_INIT_RESET_WAIT)))
    {
        ps_inst->ui8_MPU9265State = MPU9265_STATE_IDLE;
    }

    // Determine the current state of the MPU9265 state machine.
    switch(ps_inst->ui8_MPU9265State)
    {

        // MPU9150 Device reset was issued
        case MPU9265_STATE_INIT_RESET:
        {
            // Issue a read of the status register to confirm reset is done.
            ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_PWR_MGMT_1;
            I2CMRead(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
                     ps_inst->pui8_MPU9265WriteBuffer, 1,
                     ps_inst->pui8_MPU9265ReadBuffer, 1, MPU9265Callback, ps_inst);

            ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_RESET_WAIT;
            break;
        }

        // Status register was read, check if reset is done before proceeding.
        case MPU9265_STATE_INIT_RESET_WAIT:
        {
            // Check the value read back from status to determine if device
            // is still in reset or if it is ready.  Reset state for this
            // register is 0x01. Device may also respond with an address NACK during very early stages of the
            // its internal reset.  Keep polling until we verify device is ready.
            if((ps_inst->pui8_MPU9265ReadBuffer[0] != 0x01) ||
                    (ui8_i2cState == I2CM_STATUS_ADDR_NACK))
            {
                // Device still in reset so begin polling this register.
                ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_PWR_MGMT_1;
                I2CMRead(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
                         ps_inst->pui8_MPU9265WriteBuffer, 1,
                         ps_inst->pui8_MPU9265ReadBuffer, 1, MPU9265Callback, ps_inst);

                // Stay in the state for polling effect
                ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_RESET_WAIT;
            }
            else
            {
                // Device finished reset active PWR MODE 1
                ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_PWR_MGMT_1;
                ps_inst->pui8_MPU9265WriteBuffer[1] = MPU9265_PWR_MGMT_1_INT_20MHZ_CLOCK;
                I2CMWrite(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
                          ps_inst->pui8_MPU9265WriteBuffer, 2,
                          MPU9265Callback, ps_inst);

                // Update state to show we are modifing user control and
                // power management 1 regs.
                ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_PWR_MGMT;
            }

            break;
        }

        // Reset complete and PWR MODE 1 started now configure internal sample rate
        case MPU9265_STATE_INIT_PWR_MGMT:
        {
            // Load index 0 with the sample rate register number.
            ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_SMPLRT_DIV;
            // set sample rate divider to Internal_Sample_Rate / (1 + SMPLRT_DIV)
            ps_inst->pui8_MPU9265WriteBuffer[1] = MPU9265_SMPLRT_DIV_S;
            I2CMWrite(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
                        ps_inst->pui8_MPU9265WriteBuffer, 2,
                        MPU9265Callback, ps_inst);

            // update state to show are in process of configuring sensors.
            ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_SAMPLE_RATE_CFG;
            break;
        }

        // internal sample rate is configured now set accel sensitivity
        case MPU9265_STATE_INIT_SAMPLE_RATE_CFG:
        {
            // configure accelerometer for 2g sensitivity
            ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_ACCEL_CONFIG;
            ps_inst->pui8_MPU9265WriteBuffer[1] = MPU9265_ACCEL_CONFIG_AFS_SEL_2G;
            I2CMWrite(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
                        ps_inst->pui8_MPU9265WriteBuffer, 2,
                        MPU9265Callback, ps_inst);

           // update state to show are in process of configuring accel.
           ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_ACCEL_CONFIG;
           break;
        }

        // accel is configured now configure gyro
        case MPU9265_STATE_INIT_ACCEL_CONFIG:
        {
            ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_GYRO_CONFIG;
            ps_inst->pui8_MPU9265WriteBuffer[1] = MPU9265_GYRO_CONFIG_FS_SEL_500;
            I2CMWrite(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
                        ps_inst->pui8_MPU9265WriteBuffer, 2,
                        MPU9265Callback, ps_inst);

            // update state to show are in process of configuring accel.
            ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_GYRO_CONFIG;
            break;

        }

        // gyro is configured now enable i2c bypass to AK8963
        case MPU9265_STATE_INIT_GYRO_CONFIG:
        {
            // INT Pin / Bypass Enable Configuration
            ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_INT_PIN_CFG;
            ps_inst->pui8_MPU9265WriteBuffer[1] = MPU9265_INT_PIN_CFG_I2C_BYPASS_EN;
            I2CMWrite(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
                           ps_inst->pui8_MPU9265WriteBuffer, 2,
                           MPU9265Callback, ps_inst);

            ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_I2C_BYPASS;
            break;
        }

        // i2c bypass to AK8963 is enabled
        case MPU9265_STATE_INIT_I2C_BYPASS:
        {
            // get AK8693 Instance
            AK8963_s *ps_ak_inst = MPU9265GetAK8963Inst(ps_inst);

            // init AK8963 sensor
            AK8963_Init(ps_ak_inst, ps_inst->ps_i2cMastInst, AK8963_ADDRESS, MPU9265Callback, ps_inst);

            // take AK8963 instance and initialize it
            ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_AK8963;
            break;

        }

              // MPU9256 and AK8963 are initialized
        case MPU9265_STATE_INIT_AK8963:
        {
            // All sensors are initialized go into IDLE
            ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_FINISHED;
            break;
        }

        case MPU9265_STATE_INIT_FINISHED:
        {
            // All sensors are initialized go into IDLE
            ps_inst->ui8_MPU9265State = MPU9265_STATE_IDLE;
            break;
        }


        // Wait for MPU9265 data
        case MPU9265_STATE_READ_DATA:
        {
            // Change to Data read AK8693
            AK8963_s *ps_ak_inst = MPU9265GetAK8963Inst(ps_inst);

            AK8963_ReadData(ps_ak_inst, MPU9265Callback, ps_inst);

            ps_inst->ui8_MPU9265State = MPU9265_STATE_READ_DATA_AK8693;
            break;
        }

        // MPU9265 data should be read no wait for AK8693 data
        case MPU9265_STATE_READ_DATA_AK8693:
        {
            // MPU9265 and AK8693 data should be read now go into IDLE
            ps_inst->ui8_MPU9265State = MPU9265_STATE_IDLE;
            break;
        }

    }

    if((ps_inst->ui8_MPU9265State == MPU9265_STATE_IDLE) && ps_inst->fp_MPU9265Callback)
    {
        // Call the application-supplied callback function.
        ps_inst->fp_MPU9265Callback(ps_inst->p_MPU9265CallbackData, ui8_i2cState);

    }

}

/**
 * \brief   Initializes the MPU9265 struct and requests a reset
 *
 */
uint8_t MPU9265_Init(MPU9265_s *ps_inst, tI2CMInstance *ps_I2CMInst, uint8_t ui8_MPU9265Address,
                    tSensorCallback *fp_MPU9265Callback, void *p_MPU9265CallbackData){

    // Initialize the MPU9265 instance struct
    ps_inst->ps_i2cMastInst = ps_I2CMInst;
    ps_inst->ui8_MPU9265Address = ui8_MPU9265Address;

    // save callback information
    ps_inst->fp_MPU9265Callback = fp_MPU9265Callback;
    ps_inst->p_MPU9265CallbackData = p_MPU9265CallbackData;

    // set intial state to device reset
    ps_inst->ui8_MPU9265State = MPU9265_STATE_INIT_RESET;

    // write i2c reset command to MPU9256
    ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_PWR_MGMT_1;
    ps_inst->pui8_MPU9265WriteBuffer[1] = MPU9265_PWR_MGMT_1_DEVICE_RESET;

    if(I2CMWrite(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
               ps_inst->pui8_MPU9265WriteBuffer, 2,
               MPU9265Callback, ps_inst) == 0)
    {
        // Reset couldn't be send
        ps_inst->ui8_MPU9265State = MPU9265_STATE_IDLE;
        return(0);
    }

    // MPU9265 struct init and reset requested
    return(1);

}

/**
 * \brief   Starts the read of accelerometer and gyroscope data from the MPU9150 and the
 *          magnetometer data from the on-chip aK8975.
 */
uint8_t MPU9265_ReadData(MPU9265_s *ps_inst, tSensorCallback *fp_MPU9265Callback, void *p_MPU9265CallbackData)
{

    // Return a failure if the MPU9150 driver is not idle
    if(ps_inst->ui8_MPU9265State != MPU9265_STATE_IDLE)
    {
        return(0);
    }

    // save callback information
    ps_inst->fp_MPU9265Callback = fp_MPU9265Callback;
    ps_inst->p_MPU9265CallbackData = p_MPU9265CallbackData;

    // Move the state machine to the wait for data read state.
    ps_inst->ui8_MPU9265State = MPU9265_STATE_READ_DATA;

    // Read the data registers from the MPU9265.
    ps_inst->pui8_MPU9265WriteBuffer[0] = MPU9265_ACCEL_XOUT_H;
    if(I2CMRead(ps_inst->ps_i2cMastInst, ps_inst->ui8_MPU9265Address,
                ps_inst->pui8_MPU9265WriteBuffer, 1,
                ps_inst->pui8_MPU9265ReadBuffer, 16, MPU9265Callback, ps_inst) == 0)
    {
        // i2c failed return 0 and move back to IDLE state
        ps_inst->ui8_MPU9265State = MPU9265_STATE_IDLE;
        return(0);
    }

    // i2c was successful
    return(1);
}

// Returns the pointer to the AK8963 struct from the MPU9265 struct
static AK8963_s* MPU9265GetAK8963Inst(MPU9265_s *ps_inst)
{
    return (&(ps_inst->s_AK8963Inst));
}

/**
 * \brief    The callback function that is called when I2C transations to/from the
 *           AK8963 have completed.
 *
 */
static void AK8963Callback(void *p_AK8693CallbackData, uint_fast8_t ui8_i2cState){

    AK8963_s *ps_ak_inst;

    // Convert the instance data into a pointer to a tAK8963 structure
    ps_ak_inst = p_AK8693CallbackData;

    // If the I2C master driver encountered a failure, force the state machine
    // to the idle state (which will also result in a callback to propagate
    // the error).
    if(ui8_i2cState != I2CM_STATUS_SUCCESS)
    {
        ps_ak_inst->ui8_AK8963State = AK8963_STATE_IDLE;
    }

    // Determine the current state of the AK8963 state machine.
    switch(ps_ak_inst->ui8_AK8963State)
    {

        // reset was requested and should be performed now configure AK8963
        case AK8963_STATE_INIT_RESET:
        {
            ps_ak_inst->pui8_AK8963WriteBuffer[0] = AK8963_CNTL1;
            ps_ak_inst->pui8_AK8963WriteBuffer[1] = (AK8963_CNTL_MODE_CONT_2 | AK8963_CNTL_BITM_16BIT);
            I2CMWrite(ps_ak_inst->ps_i2cMastInst, ps_ak_inst->ui8_AK8693Address,
                      ps_ak_inst->pui8_AK8963WriteBuffer, 2, AK8963Callback, ps_ak_inst);

            ps_ak_inst->ui8_AK8963State = AK8963_STATE_INIT_CONFIG_FINISHED;
        }

        // AK8963 should be configured for use now change ot idle
        case AK8963_STATE_INIT_CONFIG_FINISHED:
        {
            // AK8963 should be configured and ready to use
            ps_ak_inst->ui8_AK8963State = AK8963_STATE_IDLE;
        }

        // AK8963 should have read data change back to IDLE
        case AK8963_STATE_READ_DATA:
        {
            // change to IDLE state
            ps_ak_inst->ui8_AK8963State = AK8963_STATE_IDLE;
        }

    }

    if((ps_ak_inst->ui8_AK8963State == AK8963_STATE_IDLE) && ps_ak_inst->fp_AK8963Callback)
    {
        // Call the application-supplied callback function.
        ps_ak_inst->fp_AK8963Callback(ps_ak_inst->p_AK8963CallbackData, ui8_i2cState);

    }
}

/**
 * \brief    Initializes the AK8963 driver and request a reset
 *
 */
uint8_t AK8963_Init(AK8963_s *ps_ak_inst, tI2CMInstance *ps_I2CMInst, uint8_t ui8_AK8963Address,
                       tSensorCallback *fp_AK8963Callback, void *p_AK8963CallbackData)
{
    // Initialize the AK8963 instance structure
    ps_ak_inst->ps_i2cMastInst = ps_I2CMInst;
    ps_ak_inst->ui8_AK8693Address = ui8_AK8963Address;

    // save callback information
    ps_ak_inst->fp_AK8963Callback = fp_AK8963Callback;
    ps_ak_inst->p_AK8963CallbackData = p_AK8963CallbackData;


    ps_ak_inst->ui8_AK8963State = AK8963_STATE_INIT_RESET;

    ps_ak_inst->pui8_AK8963WriteBuffer[0] = AK8963_CNTL2;
    ps_ak_inst->pui8_AK8963WriteBuffer[1] = AK8963_CNTL2_SRST;

    if(I2CMWrite(ps_ak_inst->ps_i2cMastInst, ps_ak_inst->ui8_AK8693Address,
                 ps_ak_inst->pui8_AK8963WriteBuffer, 2, AK8963Callback, ps_ak_inst) == 0)
    {
        // if I2CM Write is not succesful change into IDLE state and return 0
        ps_ak_inst->ui8_AK8963State = AK8963_STATE_IDLE;
        return(0);
    }

    // struct is initialized and reset is requested
    return(1);

}
/**
 * \brief    Initializes a data read for the AK8963
 *
 */
uint8_t AK8963_ReadData(AK8963_s *ps_ak_inst, tSensorCallback *fp_AK8963Callback, void *p_AK8963CallbackData){

    // Return a failure if the AK8963 driver is not idle
       if(ps_ak_inst->ui8_AK8963State != AK8963_STATE_IDLE)
       {
           return(0);
       }

       // save callback information
       ps_ak_inst->fp_AK8963Callback = fp_AK8963Callback;
       ps_ak_inst->p_AK8963CallbackData = p_AK8963CallbackData;

       // Move the state machine to the wait for data read state.
       ps_ak_inst->ui8_AK8963State = AK8963_STATE_READ_DATA;

       // Read the data registers from the MPU9265.
       ps_ak_inst->pui8_AK8963WriteBuffer[0] = AK8963_HXL;
       if(I2CMRead(ps_ak_inst->ps_i2cMastInst, ps_ak_inst->ui8_AK8693Address,
                   ps_ak_inst->pui8_AK8963WriteBuffer, 1,
                   ps_ak_inst->pui8_AK8963ReadBuffer, 7, AK8963Callback, ps_ak_inst) == 0)
       {
           // i2c failed return 0 and move back to IDLE state
           ps_ak_inst->ui8_AK8963State = AK8963_STATE_IDLE;
           return(0);
       }

       // i2c was successful
       return(1);

}

void  MPU9265_AK8963_GetRawData(MPU9265_s *ps_inst, MPU9265_AK8963_rawData_s *s_rawData)
{
    // Copy accel data
    s_rawData->i16_accX = (ps_inst->pui8_MPU9265ReadBuffer[0]<<8) + ps_inst->pui8_MPU9265ReadBuffer[1];
    s_rawData->i16_accY = (ps_inst->pui8_MPU9265ReadBuffer[2]<<8) + ps_inst->pui8_MPU9265ReadBuffer[3];
    s_rawData->i16_accZ = (ps_inst->pui8_MPU9265ReadBuffer[4]<<8) + ps_inst->pui8_MPU9265ReadBuffer[5];

    // Copy gyro data skip the two read temperature values
    s_rawData->i16_gyroX = (ps_inst->pui8_MPU9265ReadBuffer[8]<<8) + ps_inst->pui8_MPU9265ReadBuffer[9];
    s_rawData->i16_gyroY = (ps_inst->pui8_MPU9265ReadBuffer[10]<<8) + ps_inst->pui8_MPU9265ReadBuffer[11];
    s_rawData->i16_gyroZ = (ps_inst->pui8_MPU9265ReadBuffer[12]<<8) + ps_inst->pui8_MPU9265ReadBuffer[13];

    // Copy magnetometer values from AK8963 instance !!! Order of bytes is different then for the other sensors !!!
    s_rawData->i16_magX = (ps_inst->s_AK8963Inst.pui8_AK8963ReadBuffer[1]<<8) + ps_inst->s_AK8963Inst.pui8_AK8963ReadBuffer[0];
    s_rawData->i16_magY = (ps_inst->s_AK8963Inst.pui8_AK8963ReadBuffer[3]<<8) + ps_inst->s_AK8963Inst.pui8_AK8963ReadBuffer[2];
    s_rawData->i16_magZ = (ps_inst->s_AK8963Inst.pui8_AK8963ReadBuffer[5]<<8) + ps_inst->s_AK8963Inst.pui8_AK8963ReadBuffer[4];


}

//=====================================================================================================
// End of file
//=====================================================================================================
