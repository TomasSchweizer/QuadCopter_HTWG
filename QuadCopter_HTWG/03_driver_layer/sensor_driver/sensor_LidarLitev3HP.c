/*===================================================================================================*/
/*  sensor_LidarLitev3HP.c                                                                                 */
/*===================================================================================================*/

/*
*   file   sensor_LidarLitev3HP.c
*
*   brief  API to interact with the sensor LidarLitev3HP.
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>18/12/2021      <td>Tomas Schweizer     <td>Implementation
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   - GARMIN github: https://github.com/garmin/LIDARLite_Arduino_Library
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/
// standard libaries
#include <stdint.h>

#include "sensorlib/i2cm_drv.h"

#include "register_maps/LidarLitev3HP_register.h"
#include "sensor_LidarLitev3HP.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/
// states for state machine
enum LidarLitev3HPState_e{

    LIDARLITEv3HP_STATE_IDLE,

    // init states
    LIDARLITEv3HP_STATE_INIT_SIGCOUNTVAL,
    LIDARLITEv3HP_STATE_INIT_ACQCONFIGREG,
    LIDARLITEv3HP_STATE_INIT_REFCOUNTVAL,
    LIDARLITEv3HP_STATE_INIT_THRESHOLDBYPASS,
    LIDARLITEv3HP_STATE_INIT_FINISHED,

    // state for operation
    LIDARLITEv3HP_STATE_BUSY,
    LIDARLITEv3HP_STATE_READ_DISTANCE,
    LIDARLITEv3HP_STATE_REQUEST_DISTANCE

};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/
static void LidarLitev3HPCallback(void *p_LidarLitev3HPCallbackData, uint_fast8_t ui8_i2cState);

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
 * @brief   The callback function that is called a I2C transactions to/from the
 *          LidarLitev3HP has completed. Implemented as state machine.
 */
static void LidarLitev3HPCallback(void *p_LidarLitev3HPCallbackData, uint_fast8_t ui8_i2cState)
{

    LidarLitev3HP_s *ps_li_inst;

    // Convert the instance data into a pointer to a LidarLitev3HP structure
    ps_li_inst = p_LidarLitev3HPCallbackData;

    // If the I2C master driver encountered a failure, force the state machine
    // to the idle state (which will also result in a callback to propagate the
    // error).
    if(ui8_i2cState != I2CM_STATUS_SUCCESS)
    {
        ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_IDLE;
    }

    switch(ps_li_inst->ui8_LidarLitev3HPState)
    {
        // The i2c write from the init function is finished and the sigcountval register should be set
        case LIDARLITEv3HP_STATE_INIT_SIGCOUNTVAL:
        {

            // set the acquisition mode control to the balanced mode
            ps_li_inst->pui8_LidarLitev3HPWriteBuffer[0] = LIDARLITEv3HP_ACQ_CONFIG_REG;
            ps_li_inst->pui8_LidarLitev3HPWriteBuffer[1] = LIDARLITEv3HP_ACQCONFIGREG_BALANCED;
            I2CMWrite(ps_li_inst->ps_i2cMastInst, ps_li_inst->ui8_LidarLitev3HPAddress,
                      ps_li_inst->pui8_LidarLitev3HPWriteBuffer, 2,
                      LidarLitev3HPCallback, ps_li_inst);

            // change to the next init state
            ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_INIT_REFCOUNTVAL;
            break;

        }

        // The acquisition mode control register should be set now set refcountval register one
        case LIDARLITEv3HP_STATE_INIT_REFCOUNTVAL:
        {
            ps_li_inst->pui8_LidarLitev3HPWriteBuffer[0] = LIDARLITEv3HP_REF_COUNT_VAL;
            ps_li_inst->pui8_LidarLitev3HPWriteBuffer[1] = LIDARLITEv3HP_REFCOUNTVAL_BALANCED;
            I2CMWrite(ps_li_inst->ps_i2cMastInst, ps_li_inst->ui8_LidarLitev3HPAddress,
                     ps_li_inst->pui8_LidarLitev3HPWriteBuffer, 2,
                     LidarLitev3HPCallback, ps_li_inst);

            // change to the next init state
            ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_INIT_THRESHOLDBYPASS;
            break;

        }

        // The reference aquisition count register should be set to normal mode now set the THRESHOLDBYPASS
        // register
        case LIDARLITEv3HP_STATE_INIT_THRESHOLDBYPASS:
        {
            ps_li_inst->pui8_LidarLitev3HPWriteBuffer[0] = LIDARLITEv3HP_THRESHOLD_BYPASS;
            ps_li_inst->pui8_LidarLitev3HPWriteBuffer[1] = LIDARLITEv3HP_THRESHOLDBYPASS_BALANCED;
            I2CMWrite(ps_li_inst->ps_i2cMastInst, ps_li_inst->ui8_LidarLitev3HPAddress,
                     ps_li_inst->pui8_LidarLitev3HPWriteBuffer, 2,
                     LidarLitev3HPCallback, ps_li_inst);

            // change to the next init state
            ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_INIT_FINISHED;
            break;

        }

        // All register are set for operation init is finished
        case LIDARLITEv3HP_STATE_INIT_FINISHED:
        {
            // change to IDLE state
            ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_IDLE;
            break;
        }

        // operation states

        // status bit should be read check if LiderLitev3HP is finished with measurement
        case LIDARLITEv3HP_STATE_BUSY:
        {
            // checks if the first bit of the status register is 1 which means the device is busy
            if((ps_li_inst->pui8_LidarLitev3HPReadBuffer[0] &= 0x01))
            {
                // if device is still busy go back to IDLE state
                ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_IDLE;
            }
            // the status bit is 0 therefore the device has finished a measurement
            else
            {
                // trigger new measurement before read because then is faster ready
                ps_li_inst->pui8_LidarLitev3HPWriteBuffer[0] = LIDARLITEv3HP_ACQ_COMMAND;
                ps_li_inst->pui8_LidarLitev3HPWriteBuffer[1] = LIDARLITEv3HP_ACQ_COMMAND_INIT_MEAS;
                I2CMWrite(ps_li_inst->ps_i2cMastInst, ps_li_inst->ui8_LidarLitev3HPAddress,
                         ps_li_inst->pui8_LidarLitev3HPWriteBuffer, 2,
                         LidarLitev3HPCallback, ps_li_inst);

                // change into request state
                ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_REQUEST_DISTANCE;
            }
            break;
        }

        // request is send now read distance
        case LIDARLITEv3HP_STATE_REQUEST_DISTANCE:
        {
            // read out the to distance register for total 2 bytes first HSB then LSB
            ps_li_inst->pui8_LidarLitev3HPWriteBuffer[0] = LIDARLITEv3HP_DISTANCE_HIGH_BYTE;
            I2CMRead(ps_li_inst->ps_i2cMastInst, ps_li_inst->ui8_LidarLitev3HPAddress,
                    ps_li_inst->pui8_LidarLitev3HPWriteBuffer, 1,
                    ps_li_inst->pui8_LidarLitev3HPReadBuffer, 2, LidarLitev3HPCallback, ps_li_inst);

            // change to read state
            ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_READ_DISTANCE;
            break;
        }

        // data should be read change back to IDLE state
        case LIDARLITEv3HP_STATE_READ_DISTANCE:
        {
            ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_IDLE;
            break;
        }
    }

    // in the IDLE state a the sensor callbackfunction from sensor_driver is called
    if((ps_li_inst->ui8_LidarLitev3HPState == LIDARLITEv3HP_STATE_IDLE) && ps_li_inst->fp_LidarLitev3HPCallback)
    {
       // Call the application-supplied callback function.
       ps_li_inst->fp_LidarLitev3HPCallback(ps_li_inst->p_LidarLitev3HPCallbackData, ui8_i2cState);

    }


}

/**
 * @brief   Initializes the LidarLitev3HP struct and write the first config register
 */
uint8_t LidarLitev3HP_Init(LidarLitev3HP_s *ps_li_inst, tI2CMInstance *ps_I2CMInst,
                          uint8_t ui8_LidarLitev3HPAddress, tSensorCallback *fp_LidarLitev3HPCallback,
                          void *p_LidarLitev3HPCallbackData)
{
    // Initialize the LidarLitev3HP instance struct
    ps_li_inst->ps_i2cMastInst = ps_I2CMInst;
    ps_li_inst->ui8_LidarLitev3HPAddress = ui8_LidarLitev3HPAddress;

    // save callback information
    ps_li_inst->fp_LidarLitev3HPCallback = fp_LidarLitev3HPCallback;
    ps_li_inst->p_LidarLitev3HPCallbackData = p_LidarLitev3HPCallbackData;

    // set initial state of device
    ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_INIT_SIGCOUNTVAL;

    // write to the SIG_COUNT_VAL register the value for balanced mode
    ps_li_inst->pui8_LidarLitev3HPWriteBuffer[0] = LIDARLITEv3HP_SIG_COUNT_VAL;
    ps_li_inst->pui8_LidarLitev3HPWriteBuffer[1] = LIDARLITEv3HP_SIGCOUNTVAL_BALANCED;

    if(I2CMWrite(ps_li_inst->ps_i2cMastInst, ps_li_inst->ui8_LidarLitev3HPAddress,
                 ps_li_inst->pui8_LidarLitev3HPWriteBuffer, 2,
                 LidarLitev3HPCallback, ps_li_inst) == 0)
    {
        // first write to LidarLite wasn't successful
        ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_IDLE;
        return (0);
    }

    // LidarLitev3HP struct was initialized and first write initiated
    return (1);

}

/**
 * @brief   Starts the read of distance data from LidarLitev3HP.
 */
uint8_t LidarLitev3HP_ReadData(LidarLitev3HP_s *ps_li_inst, tSensorCallback *fp_LidarLitev3HPCallback,
                               void *p_LidarLitev3HPCallbackData)
{
    // Return a failure if the LidarLitev3HP driver is not idle
    if(ps_li_inst->ui8_LidarLitev3HPState != LIDARLITEv3HP_STATE_IDLE)
    {
       return(0);
    }

    // save callback information
    ps_li_inst->fp_LidarLitev3HPCallback = fp_LidarLitev3HPCallback;
    ps_li_inst->p_LidarLitev3HPCallbackData = p_LidarLitev3HPCallbackData;

    ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_BUSY;

    // read status register and check if device busy
    ps_li_inst->pui8_LidarLitev3HPWriteBuffer[0] = LIDARLITEv3HP_STATUS;
    if(I2CMRead(ps_li_inst->ps_i2cMastInst, ps_li_inst->ui8_LidarLitev3HPAddress,
                ps_li_inst->pui8_LidarLitev3HPWriteBuffer, 1,
                ps_li_inst->pui8_LidarLitev3HPReadBuffer, 1, LidarLitev3HPCallback, ps_li_inst) == 0)
    {
      // i2c failed return 0 and move back to IDLE state
        ps_li_inst->ui8_LidarLitev3HPState = LIDARLITEv3HP_STATE_IDLE;
       return(0);
    }
    // i2c was successful added to queue
    return (1);

}

/**
 * @brief    Copies the data from the read buffer into a variable
 */
void LidarLitev3HP_GetRawData(LidarLitev3HP_s *ps_li_inst, uint16_t *ui16_distance, float *f_distance)
{
    // byte in read buffer 0 is HSB and in read buffer 1 LSB
    *ui16_distance = (ps_li_inst->pui8_LidarLitev3HPReadBuffer[0]<<8) + ps_li_inst->pui8_LidarLitev3HPReadBuffer[1];
    *f_distance = (float) *ui16_distance;

}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
