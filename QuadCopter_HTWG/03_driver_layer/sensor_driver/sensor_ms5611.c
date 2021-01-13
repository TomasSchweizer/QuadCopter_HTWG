//=====================================================================================================
// @file sensor_ms5611.c
//=====================================================================================================
//
// @brief API to interact with the sensors.
//
// Date                 Author                      Notes
// @date 06/12/2020     @author Tomas Schweizer     Implementation
//
// Source:
// YMCA-Quadcopter: http://www.brokking.net/ymfc-32_auto_main.html
//
//=====================================================================================================

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/
// standard libaries
#include <stdint.h>

#include "sensorlib/i2cm_drv.h"
#include "busy_delay.h"

#include "ms5611_registers.h"

#include "sensor_ms5611.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/
// states for state machine
enum MS5611State_e{

    MS5611_STATE_IDLE,
    // init states is takes a long time for MS5611 because it need multiple adjust reads till ready
    MS5611_STATE_INIT_RESET,
    MS5611_STATE_INIT_READ_PROM,
    MS5611_STATE_INIT_SEQUENCE_WAITuREAD,
    MS5611_STATE_INIT_SEQUENCE_COMMAND,
    MS5611_STATE_INIT_FINISHED,

    // states for operation
    MS5611_STATE_READ_TEMP_REQ_PRESS,
    MS5611_STATE_READ_PRESS_REQ_TEMP,
    MS5611_STATE_READ_PRESS_REQ_PRESS,
    MS5611_STATE_READ_REQ_SEQUENCE_FINISHED,

};

enum MS5611DataType_e{

    MS5611_TEMP,
    MS5611_PRESS

};


/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/
static void MS5611Callback(void *p_MS5611CallbackData, uint_fast8_t ui8_i2cState);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/
static uint8_t ui8_MS5611InitSequenceCounter = 0;
/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/**
 * \brief   The callback function that is called a I2C transactions to/from the
 *          MS5611 has completed. Implemented as state machine.
 */
static void MS5611Callback(void *p_MS5611CallbackData, uint_fast8_t ui8_i2cState)
{

    MS5611_s *ps_ms_inst;

    // Convert the instance data into a pointer to a MS5611 structure
    ps_ms_inst = p_MS5611CallbackData;

    // If the I2C master driver encountered a failure, force the state machine
    // to the idle state (which will also result in a callback to propagate the
    // error).
    if(ui8_i2cState != I2CM_STATUS_SUCCESS)
    {
        ps_ms_inst->ui8_MS5611State = MS5611_STATE_IDLE;
    }

    // Determine the current state of the MPU9265 state machine.
    switch(ps_ms_inst->ui8_MS5611State)
    {

        // Reset is initiated wait for device to power up and then read prom
        case MS5611_STATE_INIT_RESET:
        {
            static uint8_t i = 0;

            if(i == 0)
            {
            // After reset it takes > 10ms to get back up busy delay not critical because in init phase
            // flight task hasn't started running
            BusyDelay_Ms(15);
            }
            // Issue a read of the PROM register to get calibration values and save them in
            // pui8_MS5611PROMValues to extract them later
            ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_PROM_READ + i;
            I2CMRead(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                     ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                     ps_ms_inst->pui8_MS5611PROMValues+i, 2, MS5611Callback, ps_ms_inst);
            i += 2;
            if(i<12)
            {
                // read next prom value
                ps_ms_inst->ui8_MS5611State = MS5611_STATE_INIT_RESET;

            }
            else
            {
                // PROM should be read only one value left
                ps_ms_inst->ui8_MS5611State = MS5611_STATE_INIT_READ_PROM;
            }

            break;

        }
        // PROM is read now start the init sequence
        case MS5611_STATE_INIT_READ_PROM:
        {
            // request a temp value to start the init sequence
            ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_CONVER_TEMP_4096;
            I2CMWrite(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                      ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                      MS5611Callback, ps_ms_inst);

            // go into init sequence wait state
            ps_ms_inst->ui8_MS5611State = MS5611_STATE_INIT_SEQUENCE_WAITuREAD;
            break;
        }

        // Wait till data is ready then read data in init sequence
        case MS5611_STATE_INIT_SEQUENCE_WAITuREAD:
        {
            // wait 10ms till measurement is finished
            BusyDelay_Ms(10);

            // It doesn't matter if the values are temp or press values because this are just init reads
            // therefore we can discard the measurements after reading
            ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_ADC_READ_COMMAND;
            I2CMRead(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                     ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                     ps_ms_inst->pui8_MS5611ReadBuffer, 3, MS5611Callback, ps_ms_inst);

            // go into the send command state
            ps_ms_inst->ui8_MS5611State = MS5611_STATE_INIT_SEQUENCE_COMMAND;
            break;
        }
        case MS5611_STATE_INIT_SEQUENCE_COMMAND:
        {
            // increase the counter after every read -> 50 temp and 50 press values
            ui8_MS5611InitSequenceCounter++;

            // Switch between the sequence states till 100 measurements have been taken
            if(ui8_MS5611InitSequenceCounter >= 100)
            {
                // request a temp value as the request of the init sequence to setup for operation
                ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_CONVER_TEMP_4096;
                I2CMWrite(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                          ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                          MS5611Callback, ps_ms_inst);

                // the init is finished after this write
                ps_ms_inst->ui8_MS5611State = MS5611_STATE_INIT_FINISHED;
            }
            else
            {
                //  if the counter is an even number temp is requested else press
                if(ui8_MS5611InitSequenceCounter % 2 == 0){
                    ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_CONVER_TEMP_4096;
                }
                else
                {
                    ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_CONVER_PRESS_4096;
                }

                I2CMWrite(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                          ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                          MS5611Callback, ps_ms_inst);

                // go back to sequence state wait
                ps_ms_inst->ui8_MS5611State = MS5611_STATE_INIT_SEQUENCE_WAITuREAD;
            }
            break;
        }

        // The init is finished now go into IDLE mode and call application callback function
        case MS5611_STATE_INIT_FINISHED:
        {
            ps_ms_inst->ui8_MS5611State = MS5611_STATE_IDLE;
            break;
        }

        // Read states


        // state read temp now request press
        case MS5611_STATE_READ_TEMP_REQ_PRESS:
        {
            // request a press adc conversion
            ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_CONVER_PRESS_4096;
            I2CMWrite(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                      ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                      MS5611Callback, ps_ms_inst);

            // Set data type indicator to read out data
            ps_ms_inst->ui8_MS5611DataType = MS5611_TEMP;

            ps_ms_inst->ui8_MS5611State = MS5611_STATE_READ_REQ_SEQUENCE_FINISHED;
            break;
        }

        // state read temp now request press
        case MS5611_STATE_READ_PRESS_REQ_TEMP:
        {
            // request a press adc conversion
            ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_CONVER_TEMP_4096;
            I2CMWrite(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                      ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                      MS5611Callback, ps_ms_inst);

            // Set data type indicator to read out data
            ps_ms_inst->ui8_MS5611DataType = MS5611_PRESS;

            ps_ms_inst->ui8_MS5611State = MS5611_STATE_READ_REQ_SEQUENCE_FINISHED;
            break;
        }

        // state read temp now request press
        case MS5611_STATE_READ_PRESS_REQ_PRESS:
        {
            // request a press adc conversion
            ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_CONVER_PRESS_4096;
            I2CMWrite(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                      ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                      MS5611Callback, ps_ms_inst);

            // Set data type indicator to read out data
            ps_ms_inst->ui8_MS5611DataType = MS5611_PRESS;

            ps_ms_inst->ui8_MS5611State = MS5611_STATE_READ_REQ_SEQUENCE_FINISHED;
            break;
        }

        // one read request sequence is finished
        case MS5611_STATE_READ_REQ_SEQUENCE_FINISHED:
        {
            ps_ms_inst->ui8_MS5611State = MS5611_STATE_IDLE;
            break;
        }

    }

    // in the IDLE state a the sensor callbackfunction from sensor_driver is called
    if((ps_ms_inst->ui8_MS5611State == MS5611_STATE_IDLE) && ps_ms_inst->fp_MS5611Callback)
    {
        // Call the application-supplied callback function.
        ps_ms_inst->fp_MS5611Callback(ps_ms_inst->p_MS5611CallbackData, ui8_i2cState);

    }


}

/**
 * \brief   Initializes the MS5611 struct and requests a reset
 *
 */
uint8_t MS5611_Init(MS5611_s *ps_ms_inst, tI2CMInstance *ps_I2CMInst, uint8_t ui8_MS5611Address,
                           tSensorCallback *fp_MS5611Callback, void *p_MS5611CallbackData)
{
    // Initialize the MS5611 instance struct
    ps_ms_inst->ps_i2cMastInst = ps_I2CMInst;
    ps_ms_inst->ui8_MS5611Address = ui8_MS5611Address;

    // save callback information
    ps_ms_inst->fp_MS5611Callback = fp_MS5611Callback;
    ps_ms_inst->p_MS5611CallbackData = p_MS5611CallbackData;

    // set intial state to device reset
    ps_ms_inst->ui8_MS5611State = MS5611_STATE_INIT_RESET;

    // write i2c reset command to MS5611
    ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_RESET;

    if(I2CMWrite(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                 ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                 MS5611Callback, ps_ms_inst) == 0)
    {
        // Reset couldn't be send
        ps_ms_inst->ui8_MS5611State = MS5611_STATE_IDLE;
        return(0);
    }

    // MS5611 struct init and reset requested
    return (1);

}

/**
 * \brief   Starts the read of barometer temp or pressure data from the MS5611
 */
uint8_t MS5611_ReadData(MS5611_s *ps_ms_inst, tSensorCallback *fp_MS5611Callback,
                        void *p_MS5611CallbackData, uint8_t ui8_MS5611ReadSequence)
{
    // Return a failure if the MS5611 driver is not idle
    if(ps_ms_inst->ui8_MS5611State != MS5611_STATE_IDLE)
    {
        return(0);
    }

    // save callback information
    ps_ms_inst->fp_MS5611Callback = fp_MS5611Callback;
    ps_ms_inst->p_MS5611CallbackData = p_MS5611CallbackData;

    // Select the right read sequence
    if(ui8_MS5611ReadSequence == MS5611_STATE_READ_TEMP_REQ_PRESS)
    {
        // Move the state machine into read temp req press state
        ps_ms_inst->ui8_MS5611State = MS5611_STATE_READ_TEMP_REQ_PRESS;
    }
    else if(ui8_MS5611ReadSequence == MS5611_STATE_READ_PRESS_REQ_TEMP)
    {
        // Move the state machine into read press req temp state
        ps_ms_inst->ui8_MS5611State = MS5611_STATE_READ_PRESS_REQ_TEMP;
    }
    else if(ui8_MS5611ReadSequence == MS5611_STATE_READ_PRESS_REQ_PRESS)
    {
        // Move the state machine into read press req press state
        ps_ms_inst->ui8_MS5611State = MS5611_STATE_READ_PRESS_REQ_PRESS;
    }

    // First read command is the same for all states only the follow up in the states is different
    ps_ms_inst->pui8_MS5611WriteBuffer[0] = MS5611_ADC_READ_COMMAND;
    if(I2CMRead(ps_ms_inst->ps_i2cMastInst, ps_ms_inst->ui8_MS5611Address,
                ps_ms_inst->pui8_MS5611WriteBuffer, 1,
                ps_ms_inst->pui8_MS5611ReadBuffer, 3, MS5611Callback, ps_ms_inst) == 0)
    {
      // i2c failed return 0 and move back to IDLE state
       ps_ms_inst->ui8_MS5611State = MS5611_STATE_IDLE;
       return(0);
    }
    // i2c was successful added to queue
    return (1);

}

/**
 * \brief   Can be called after Init function was called copies the MS5611 calibration values
 *          out of the buffer into int16 variables
 */
void MS5611_GetCalibrationValues(MS5611_s *ps_ms_inst, uint16_t *ui16_baroCalValues){

    ui16_baroCalValues[0] = (ps_ms_inst->pui8_MS5611PROMValues[0]<<8) + ps_ms_inst->pui8_MS5611PROMValues[1];
    ui16_baroCalValues[1] = (ps_ms_inst->pui8_MS5611PROMValues[2]<<8) + ps_ms_inst->pui8_MS5611PROMValues[3];
    ui16_baroCalValues[2] = (ps_ms_inst->pui8_MS5611PROMValues[4]<<8) + ps_ms_inst->pui8_MS5611PROMValues[5];
    ui16_baroCalValues[3] = (ps_ms_inst->pui8_MS5611PROMValues[6]<<8) + ps_ms_inst->pui8_MS5611PROMValues[7];
    ui16_baroCalValues[4] = (ps_ms_inst->pui8_MS5611PROMValues[8]<<8) + ps_ms_inst->pui8_MS5611PROMValues[9];
    ui16_baroCalValues[5] = (ps_ms_inst->pui8_MS5611PROMValues[10]<<8) + ps_ms_inst->pui8_MS5611PROMValues[11];
}

// TODO add rotating memory to struct and calculate moving average but mayby better in sensor driver
/**
 * \brief   Can be called after Read data function, copies read data from the buffers into a rawData struct *
 */
uint8_t MS5611_GetRawData(MS5611_s *ps_ms_inst, MS5611_rawData_s *s_rawData)
{
    static uint8_t ui8_tempRotMemCounter = 0;
    static uint32_t ui32_tempSum = 0;
    static uint32_t ui32_tempRotMem[5];

    static uint8_t ui8_pressRotMemCounter = 0;
    static uint32_t ui32_pressSum = 0;
    static uint32_t ui32_pressRotMem[20];


    if(ps_ms_inst->ui8_MS5611DataType == MS5611_TEMP)
    {
        // Read temp value in rotating memory and calculate average of the last 5 measurements
        //to help against temperature spikes
        ui32_tempSum -= ui32_tempRotMem[ui8_tempRotMemCounter];
        // measurements values are 24uint values
        ui32_tempRotMem[ui8_tempRotMemCounter] = (ps_ms_inst->pui8_MS5611ReadBuffer[0] << 16) |
                                                 (ps_ms_inst->pui8_MS5611ReadBuffer[1] << 8)  |
                                                  ps_ms_inst->pui8_MS5611ReadBuffer[2];
        ui32_tempSum += ui32_tempRotMem[ui8_tempRotMemCounter];
        ui8_tempRotMemCounter++;

        if(ui8_tempRotMemCounter >= 5)
        {
            ui8_tempRotMemCounter = 0;
        }

        if(ui32_tempRotMem[4] != 0 && ui32_pressRotMem[19] != 0)
        {
            s_rawData->ui32_baroT = ui32_tempSum / 5;
            return (1);
        }
        return (0);


    }
    else if(ps_ms_inst->ui8_MS5611DataType == MS5611_PRESS)
    {
        // Read pressure value in rotating memory and calculate average
        //of the last 20 measurements to smoothen pressure readings
        ui32_pressSum -= ui32_pressRotMem[ui8_pressRotMemCounter];
        // measurements values are 24uint values
        ui32_pressRotMem[ui8_pressRotMemCounter] = (ps_ms_inst->pui8_MS5611ReadBuffer[0] << 16) |
                                                   (ps_ms_inst->pui8_MS5611ReadBuffer[1] << 8)  |
                                                    ps_ms_inst->pui8_MS5611ReadBuffer[2];

        ui32_pressSum += ui32_pressRotMem[ui8_pressRotMemCounter];
        ui8_pressRotMemCounter++;
        if(ui8_pressRotMemCounter == 20)
        {
            ui8_pressRotMemCounter = 0;
        }

        if(ui32_pressRotMem[19] != 0 && ui32_tempRotMem[4] != 0 )
        {
            s_rawData->ui32_baroP = ui32_pressSum / 20;
            return (1);
        }
        return (0);



    }
    else
    {
        // This should never happen mistake
        return(0);

    }
}

















