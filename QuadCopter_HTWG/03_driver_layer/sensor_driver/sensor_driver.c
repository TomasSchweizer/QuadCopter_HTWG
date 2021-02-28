/*===================================================================================================*/
/*  sensor_driver.c                                                                                  */
/*===================================================================================================*/

/*
*   file   sensor_driver.c
*
*   brief  Implementation of sensor initialization, reading, calibration and fusion.
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>31/05/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
*   <tr><td>29/12/2020      <td>Tomas Schweizer     <td>Completely overworked because of new sensors
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>
#include <math.h>

// Setup
#include "peripheral_setup.h"
#include "prioritys.h"

// Drivers
#include "sensor_driver.h"
#include "display_driver.h"
#include "debug_interface.h"
#include "MadgwickAHRS.h"
#include "basic_filters.h"


// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"

// Utilities
#include "qc_math.h"
#include "workload.h"
#include "fault.h"
#include "busy_delay.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

#define SENSOR_MPU_INIT_TIMEOUT_MS                  ( 5 )                               ///< Timeout for MPU init sequence [ms]
#define SENSOR_READ_TIMEOUT_MS		                ( 1.1 )                             ///< Timeout for MPU data read sequence [ms]

#define event_SENSOR_BIT_COUNT                      ( 3 )                               ///< Sensor event bits
#define event_SENSOR_RECEIVED                       ( 1 << 0 )                          ///< Event bit for sensor MPU received

#define CALIBRATE_MIN_TIME_MS                       ( 5000 )                            ///< Time for calibration [ms]
#define CALIBRATE_BEFORE_FIRST_START                ( -2 )                              ///< Start up flag for calibration state machine
#define CALIBRATE_STOP                              ( -1 )                              ///< Stop flag for calibration state machine
#define CALIBRATE_START                             ( 0 )                               ///< Start flag for calibration state machine
#define IS_CALIBRAE_REQUIRED(stateTime)             (stateTime>=0 || stateTime == -2)   ///< Macro to check if calibration is required

#define dt 0.002f



#if (setup_ALT_BARO)
    #define SENSOR_BARO_INIT_TIMEOUT_MS             ( 3000 )                            ///< Timeout for Barometer init sequence [ms]
    #define event_SENSOR_BARO_RECEIVED              ( 1 << 1 )                          ///< Event bit for sensor barometer received

    // Defines for read timing barometer
    #define SENSOR_BARO_MEASUREMENT                 ( 5 )                               ///< Every 5th loop read data (5 loops = 10ms)
    #define SENSOR_BARO_MEASUREMENT_TEMP            ( 20 )                              ///< Every 20th measurement read temp
    #define SENSOR_BARO_READ_TEMP_REQ_PRESS         ( 6 )                               ///< Macro for barometer control sequence
    #define SENSOR_BARO_READ_PRESS_REQ_TEMP         ( 7 )                               ///< Macro for barometer control sequence
    #define SENSOR_BARO_READ_PRESS_REQ_PRESS        ( 8 )                               ///< Macro for barometer control sequence


#endif

#if(setup_ALT_LIDAR)
    #define SENSOR_LIDAR_INIT_TIMEOUT_MS            ( 5 )                               ///< Timeout for Lidar init sequence [ms]
    #define event_SENSOR_LIDAR_RECEIVED             (1 << 2)                            ///< Event bit for sensor lidar received

    #define SENSOR_LIDAR_MEASUREMENT                ( 2 )                               ///< Every 2th loop read lidar data (2 loops = 4ms)

#endif


///< Enum for postion of sensor values in array
enum sensorAxis_e{

    X_ACC,
    Y_ACC,
    Z_ACC,
    X_GYRO,
    Y_GYRO,
    Z_GYRO,
    X_MAG,
    Y_MAG,
    Z_MAG,

};

///< Enum for position of barometer sensor values in array
enum sensorBaro_e{

    T_BARO,
    P_BARO,
};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/
uint8_t blockTaskTillEventBitIsSet(uint32_t ui32_eventBits, uint32_t ui32_eventTimeout_Ms);


#if ( setup_DEV_SUM_RECEIVS ) || DOXYGEN
    extern void HIDE_senMesRec_SetEventName(uint32_t ui32_eventBit,const char* pc_name);
    extern void HIDE_senMesRec_DrawDisplay(void);
    extern void HIDE_senMesRec_Increment(uint32_t ui32_receiveEventBit);
#else
    #define HIDE_senMesRec_SetEventName(ui32_eventBit,pc_name)            // this define will be kicked off from the preprocessor
    #define HIDE_senMesRec_DrawDisplay                            0       // 0 pointer
    #define HIDE_senMesRec_Increment(ui32_receiveEventBit)                // this define will be kicked off from the preprocessor
#endif

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/
volatile float gf_sensor_attitudeQuaternion[4] = {1.0, 0.0, 0.0, 0.0};  ///< Global variable for attitude quaternion
volatile float gf_sensor_dotAttitudeQuaternion[4];                      ///< Global variable for derivative of attitude quaternion
float gf_sensor_fusedAngles[3];                                         ///< Global variable for fused Tait-Bryan angles [rad] (roll, pitch, yaw)
float gf_sensor_angularVelocity[3];                                     ///< Global variable for angular velocity [rad/s] calculated from quaternions
float gf_sensor_gyroAngularVelocity[3] = {0.0, 0.0, 0.0};               ///< Global variable for angular velocity [rad/s] directly from gyroscope
float gf_sensor_pressure;                                               ///< Global variable for pressure [mPa] of barometer
float gf_sensor_baroAltitude;                                           ///< Global variable for altitude [cm] measured by barometer
float gf_sensor_lidarAltitude;                                          ///< Global variable for altitude [cm] measured by lidar

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

volatile EventGroupHandle_t gx_sensor_EventGroup;                       ///< Event group handle for sensor receive events
static workload_handle_p p_workHandle;                                  ///< Variable to check i2c workload
volatile countEdges_handle_p p_senMesRec_countEdges;                    ///< Count received sensor measurements

static int32_t i32_stateTime = CALIBRATE_BEFORE_FIRST_START;            ///< Variable for calibration state machine

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/


#if ( setup_DEV_DEBUG_USB ) || DOXYGEN


    /**
    *   @brief   Send data from sensor driver to over USB to PC application
    *
    *   @note   This happens each 100ms !!!NOT USEABLE FOR FREQUENCY ANALYSIS OF SIGNALS!!!
    */
    void HIDE_Sensor_SendDataOverUSB(void)
    {
        float f_usb_debug[9];
        /*
         * Array has to be filled with variables you wish to send
         * f_usb_debug[0] = ...
         * f_usb_debug[1] = ...
         * ...
         */
        HIDE_Debug_USB_InterfaceSend(f_usb_debug, sizeof(f_usb_debug)/ sizeof(f_usb_debug[0]), debug_FLOAT);
    }

#endif

#if ( setup_DEV_SUM_RECEIVS )
    /**
     * @brief   Draw info about the sensor receive counter on the display
     *
     * @note    To enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
     */
    void HIDE_senMesRec_DrawDisplay(void)
    {
        EventBits_t x_faultEventBits = xEventGroupGetBits( gx_sensor_EventGroup );
        const uint8_t drawHight     = 6;
        const uint8_t xOffset       = 55;
        u8g_SetFont(&gs_display, u8g_font_04b_03r);     // u8g_font_unifont
        uint8_t i;
        uint16_t hight=drawHight+6;
        const char* p_name;
        for(i=0;i<event_SENSOR_BIT_COUNT;++i)
        {
            p_name=CountEdges_GetName(p_senMesRec_countEdges,i);
            if(p_name!=0)
            {
                u8g_DrawStr(&gs_display, xOffset + 0,  hight,p_name);                       // Name
                u8g_DrawStr(&gs_display, xOffset + 15+1, hight, u8g_u16toa(CountEdges_Get(p_senMesRec_countEdges,i),3));      // How often fired
                hight+=drawHight;
            }
        }

    }

    /**
     * @brief   Store the name of the receive eventBit into p_receive_coundEdges
     */
    void HIDE_senMesRec_SetEventName(uint32_t ui32_eventBit,const char* pc_name)
    {
        CountEdges_SetName(p_senMesRec_countEdges,CountEdges_Bit2Num(ui32_eventBit),pc_name);
    }

    /**
     * @brief   Increment at ui32_receiveEventSensorBit in p_senMesRec_countEdges
     *
     * @param   ui32_receiveEventSensorBit  the received eventBit
     *
     * @note    To enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
     */
    void HIDE_senMesRec_Increment(uint32_t ui32_receiveEventSensorBit)
    {
        CountEdges_Increment(p_senMesRec_countEdges,CountEdges_Bit2Num(ui32_receiveEventSensorBit));
    }

#endif

/**
 * @brief   Draw info about Sensors on the Display
 */
void Sensor_DrawDisplay(void)
{
    static const char* p_sign = " ";
    static const char* m_sign = "-";
    const uint8_t yOffset       = 58;

    u8g_SetFont(&gs_display, u8g_font_04b_03r);     // u8g_font_unifont
    u8g_DrawStr(&gs_display,0,      yOffset + 0,"Ro");
    u8g_DrawStr(&gs_display,0+12,   yOffset + 0,u8g_8toa((int8_t)(math_RAD2DEC(gf_sensor_fusedAngles[0])),3));
    u8g_DrawStr(&gs_display,36,     yOffset + 0,"Pi");
    u8g_DrawStr(&gs_display,36+12,  yOffset + 0,u8g_8toa((int8_t)(math_RAD2DEC(gf_sensor_fusedAngles[1])),3));
    u8g_DrawStr(&gs_display,72,     yOffset + 0,"Ya");

    // print yaw on display -180 to 180
    uint8_t yaw_value;
    if (gf_sensor_fusedAngles[2] < 0.0){

        yaw_value = (uint8_t) math_RAD2DEC(-gf_sensor_fusedAngles[2]);
        u8g_DrawStr(&gs_display,72+12,     yOffset + 0, m_sign);
        u8g_DrawStr(&gs_display,72+16,  yOffset + 0, u8g_u8toa(yaw_value, 3));
    }
    else
    {
        yaw_value = (uint8_t) math_RAD2DEC(gf_sensor_fusedAngles[2]);
        u8g_DrawStr(&gs_display,72+12,     yOffset + 0, p_sign);
        u8g_DrawStr(&gs_display,72+16,  yOffset + 0, u8g_u8toa(yaw_value, 3));
    }

    #if ( setup_ALT_BARO || setup_ALT_LIDAR )


    u8g_DrawStr(&gs_display, 58, 32,"aB");

    float alt_print;
    if(gf_sensor_baroAltitude <= 0){

        alt_print = 0.0;
    }
    else
    {
        alt_print = gf_sensor_baroAltitude * 100;
    }
    u8g_DrawStr(&gs_display, 66, 32, u8g_u16toa((uint16_t)alt_print,4));
    u8g_DrawStr(&gs_display, 58, 44,"aL");
    u8g_DrawStr(&gs_display, 66, 44, u8g_u16toa((uint16_t)gf_sensor_lidarAltitude,4));

    #endif

}

/**
 * @brief   Blocks FreeRTOS Task and sets timeout faults and i2c faults
 *
 * @param   ui32_eventBits Event bit/bits for which the task waits
 * @param   ui32_eventTimeout_Ms Maximum block time [ms]
 *
 * @return 1 Timeout or error has happend
 *         0 Event bit was received before timeout
 */
uint8_t blockTaskTillEventBitIsSet(uint32_t ui32_eventBits, uint32_t ui32_eventTimeout_Ms)
{

    volatile EventBits_t x_sensorEventBits;

    //  Wait for sensor received event
    xEventGroupClearBits(gx_sensor_EventGroup,
                         event_SENSOR_RECEIVED |
                         event_SENSOR_BARO_RECEIVED|
                         event_SENSOR_LIDAR_RECEIVED);

    x_sensorEventBits = xEventGroupWaitBits(gx_sensor_EventGroup,
                                            ui32_eventBits,
                                            pdTRUE,          // clear Bits before returning.
                                            pdTRUE,          // wait for all Bits
                                            ui32_eventTimeout_Ms / portTICK_PERIOD_MS ); // maximum wait time

    // Unblock because of timeout, or i2cError?
    uint8_t ui8_error= (ui32_eventBits & x_sensorEventBits) != ui32_eventBits;

    // set an error if a change from ui8_error from 0 to 1 occurs (rising edge)
    HIDE_Fault_Increment(fault_SENSOR, ui8_error);

    if( ui8_error )
    {
       xEventGroupSetBits(gx_fault_EventGroup,fault_SENSOR);
       return (1);

    }
    else
    {
       xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);
       HIDE_senMesRec_Increment(event_SENSOR_RECEIVED);
       return(0);
    }


}

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Select Mode                                                   */
/* ---------------------------------------------------------------------------------------------------*/

#if ( setup_SENSOR_I2C == (setup_SENSOR&setup_MASK_OPT1) ) || DOXYGEN

    /* -----------------------------------------------------------------------------------------------*/
    /*                                     Include File Definitions i2c Mode                          */
    /* -----------------------------------------------------------------------------------------------*/


    //  Hardware Specific
    #include "inc/hw_memmap.h"
    #include "inc/hw_ints.h"
    #include "driverlib/sysctl.h"
    #include "driverlib/pin_map.h"
    #include "driverlib/gpio.h"
    #include "driverlib/rom.h"

    // i2c sensorlib
    #include "sensorlib/i2cm_drv.h"

    // Sensor register files
    #include "register_maps/mpu9265_registers.h"
    #include "register_maps/ak8963_registers.h"

    // Sensor lib files
    #include "sensor_mpu9265.h"

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local defines i2c Mode                                     */
    /* -----------------------------------------------------------------------------------------------*/

    #define I2C_MPU_ADRESS                      MPU9265_ADDRESS             ///< I2C address of MPU
    #define I2C_AK_ADDRESS                      AK8963_ADDRESS              ///< I2C address of magnetometer



    ///< If Barometer of Lidar are selected in qc_setup defines for I2C2 get set
    #if ( setup_ALT_BARO || setup_ALT_LIDAR )

        #include "register_maps/ms5611_registers.h"
        #include "sensor_ms5611.h"

        #include "register_maps/LidarLitev3HP_register.h"
        #include "sensor_LidarLitev3HP.h"

        #define I2C_MS_ADDRESS                  MS5611_ADDRESS
        #define I2C_LI_ADDRESS                  LIDARLITEV3HP_ADDRESS

        #define I2C_ALT_SYSCTL_PERIPH           SYSCTL_PERIPH_I2C2
        #define I2C_ALT_PERIPH_BASE             I2C2_BASE
        #define I2C_ALT_SYSCTL_PERIPH_PORT      SYSCTL_PERIPH_GPIOE
        #define I2C_ALT_PORT_BASE               GPIO_PORTE_BASE
        #define I2C_ALT_SCL                     GPIO_PE4_I2C2SCL
        #define I2C_ALT_SCL_PIN                 GPIO_PIN_4
        #define I2C_ALT_SDA                     GPIO_PE5_I2C2SDA
        #define I2C_ALT_SDA_PIN                 GPIO_PIN_5
    #endif

    ///< Set defines for I2C1 for commmunication with MPU
    #if ( periph_SENSOR_INT == INT_I2C1 )
        #define I2C_SYSCTL_PERIPH           SYSCTL_PERIPH_I2C1
        #define I2C_PERIPH_BASE             I2C1_BASE
        #define I2C_SYSCTL_PERIPH_PORT      SYSCTL_PERIPH_GPIOA
        #define I2C_PORT_BASE               GPIO_PORTA_BASE
        #define I2C_SCL                     GPIO_PA6_I2C1SCL
        #define I2C_SCL_PIN                 GPIO_PIN_6
        #define I2C_SDA                     GPIO_PA7_I2C1SDA
        #define I2C_SDA_PIN                 GPIO_PIN_7
    #else
        #error  ERROR implement the defines above here
    #endif

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local Type Definitions i2c Mode                            */
    /* -----------------------------------------------------------------------------------------------*/

    /* ------------------------------------------------------------------------------------------------*/
    /*                                      Forward Declarations i2c Mode                              */
    /* ------------------------------------------------------------------------------------------------*/
    static void SensorCalibrate(void);

    void Sensor_MPU9265Callback(void *pvCallbackData, uint_fast8_t ui8_i2cState);
    static void MPUrawData2Float(float* pf_sensorData);
    static void MPUAxis2QCAxis(float* pf_sensorData);
    static void convertMPUData(float* pf_sensorData);
    static void correctMPUOffset(float* pf_sensorData, uint8_t calibrate);
    static void filterGyroData(float *pf_sensorData);
    static void calculateAngularVelocity(volatile float *q, volatile float *qDot, float *angularVelocity, float *gyro, float *gyroAngularVelocity);

    #if ( setup_ALT_BARO )

        void Sensor_MS5611Callback(void *pvCallbackData, uint_fast8_t ui8_i2cState);
        static void calculatePressureAndTemp(MS5611_rawData_s *ps_rawData, float *pf_sensorBaroData);
        static float calculateAltitude(float *pf_sensorBaroData, uint8_t calibrate);

    #endif

    #if ( setup_ALT_LIDAR )

        void Sensor_LidarLitev3HPCallback(void *pvCallbackData, uint_fast8_t ui8_i2cState);
    #endif


    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Global Variables i2c Mode                                     */
    /* ---------------------------------------------------------------------------------------------------*/

    // MPU and Motors share the same i2c bus and master instance
    #if(periph_SENSOR_INT==periph_MOTOR_INT)
        extern tI2CMInstance i2cMastInst_s;             // I2C master instance
    #else
        static tI2CMInstance i2cMastInst_s;             // I2C master instance
    #endif

    // Ii2c master instance for barometer and lidar
    #if ( setup_ALT_BARO || setup_ALT_LIDAR )
        tI2CMInstance i2cAltMastInst_s;
    #endif

    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Local Variables i2c Mode                                      */
    /* ---------------------------------------------------------------------------------------------------*/

    // Flags for i2c transfer
    static volatile uint8_t ui8_i2cFinishedFlag = 0;
    static volatile uint8_t ui8_i2cErrorFlag = 0;
    static volatile uint8_t ui8_i2cDataReadFlag = 0;


    // variables for MPU sensor functions
    static MPU9265_s s_MPU9265Inst;
    static MPU9265_AK8963_rawData_s s_MPU9265_AK8963_rawData;
    static uint8_t ui8_mpuInitFlag = 0;


    // struct for low pass filtering for dc offset correction of acc data
    static struct lp_a_struct lp_offsets = {
            .alpha = LP_FREQ2ALPHA(0.3f, dt)
    };


    // variables for calibrating gyro
    static float f_gyro_cal[3] = { 0.0, 0.0, 0.0};
    static float f_index = 0;

    // structs for gyroscope filtering

    // lowpass at high cut off frequency 100Hz -> less delay and helps against mirroring of frequencies
    struct lp_a_struct s_lp_gyro = {
            .alpha = LP_FREQ2ALPHA(100.0f,dt)

    };

    // static notch filter for noise from wind of the props at ~ 80Hz
    static struct notch_a_s s_nf1_gyro = {

           .x = {0.0, 0.0, 0.0},
           .x1 = {0.0, 0.0, 0.0},
           .x2 ={0.0, 0.0, 0.0},

           .y = {0.0, 0.0, 0.0},
           .y1 ={0.0, 0.0, 0.0},
           .y2 = {0.0, 0.0, 0.0},

           // notch at q 5
           .b0 = 0.9032,
           .b1 = -0.8702,
           .b2 = 0.9032,

           .a0 = 1.0,
           .a1 = -0.8702,
           .a2 = 0.8063,

    };

    // static notch filter for motor vibrations at around 100 Hz
    static struct notch_a_s s_nf2_gyro = {

        .x = {0.0, 0.0, 0.0},
        .x1 = {0.0, 0.0, 0.0},
        .x2 = {0.0, 0.0, 0.0},

        .y = {0.0, 0.0, 0.0},
        .y1 = {0.0, 0.0, 0.0},
        .y2 = {0.0, 0.0, 0.0},

        // notch q 1 100 hz works really well should be ok
//        .b0 = 0.5792,
//        .b1 = -0.3580,
//        .b2 = 0.5792,
//
//        .a0 = 1.0,
//        .a1 = -0.3580,
//        .a2 = 0.1584,

        // notch q 0.8 100 hz works a little bit better but slower response test if worth
       .b0 = 0.5000,
       .b1 = -0.3090,
       .b2 = 0.5000,

       .a0 = 1.0,
       .a1 = -0.3090,
       .a2 = 0.0,

    };







    // Magnetometer calibration data
    #if(!(setup_ALT_BARO || setup_ALT_LIDAR))

        static const float f_mag_calibration_b[3] = { -37.8932,   -3.4380,  -66.0203};
        static const float f_mag_calibration_A[9] = { 1.0296,         0.0,         0.0,
                                                     0.0,    0.9850,         0.0,
                                                     0.0,         0.0,    0.9861};

    #else

        static const float f_mag_calibration_b[3] = { -15.9344,   -1.4309,  -74.7844};
        static const float f_mag_calibration_A[9] = { 1.0137,         -0.0086,         -0.0278,
                                                    -0.0086,    0.9866,         -0.0044,
                                                    -0.0278,         -0.0044,    1.0007};


    #endif





    #if ( setup_ALT_BARO )

        // variables for Baro sensor functions
        static MS5611_s s_MS5611Inst;
        static MS5611_rawData_s s_MS5611_rawData;
        static uint16_t ui16_MS5611CalVal[6];
        static uint8_t ui8_baroInitFlag = 0;
        static uint8_t ui8_MS5611LoopCounter = 0;
        static uint8_t ui8_MS5611MesCounter = 0;
        static uint8_t ui8_MS5611NewMesFlag = 0;
        static float f_pressureReference = 101325.0;
    #endif

    #if ( setup_ALT_LIDAR )
        static LidarLitev3HP_s s_LidarLitev3HPInst;
        static uint16_t ui16_lidarAltitude;
        static uint8_t ui8_lidarInitFlag = 0;
        static uint8_t ui8_lidarLoopCounter = 0;
        static uint8_t ui8_lidarNewMesFlag = 0;

    #endif


    /* -----------------------------------------------------------------------------------------------*/
    /*                                      Procedure Definitions i2c mode                            */
    /* -----------------------------------------------------------------------------------------------*/

    /**
     * @brief   ISR for I2C. Give the interrupt handle to the I2C Master instance.
     */
    void Sensor_I2CIntHandler(void)
    {
        I2CMIntHandler(&i2cMastInst_s);
    }

    /**
    * @brief   MPU9265 sensor callback function only set flag computation in other functions
    *
    */
    void Sensor_MPU9265Callback(void *pvCallbackData, uint_fast8_t ui8_i2cState)
    {
        // check if i2c was a success else save error
        if(ui8_i2cState == I2CM_STATUS_SUCCESS){

            ui8_i2cErrorFlag = 0;

            // fire eventBit to notify Sensor Read has finished
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if(pdFAIL==xEventGroupSetBitsFromISR(gx_sensor_EventGroup, event_SENSOR_RECEIVED, &xHigherPriorityTaskWoken))
                while(1);// you will come here, when configTIMER_QUEUE_LENGTH is full
            else
                portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }
        else
        {
            ui8_i2cErrorFlag = 1;
        }
    }



    /**
     * @brief   Convert raw int16 values from MPU to float values     *
     */
    static void MPUrawData2Float(float* pf_sensorData){

        pf_sensorData[X_ACC] = (float) s_MPU9265_AK8963_rawData.i16_accX;
        pf_sensorData[Y_ACC] = (float) s_MPU9265_AK8963_rawData.i16_accY;
        pf_sensorData[Z_ACC] = (float) s_MPU9265_AK8963_rawData.i16_accZ;

        pf_sensorData[X_GYRO] = (float) s_MPU9265_AK8963_rawData.i16_gyroX;
        pf_sensorData[Y_GYRO] = (float) s_MPU9265_AK8963_rawData.i16_gyroY;
        pf_sensorData[Z_GYRO] = (float) s_MPU9265_AK8963_rawData.i16_gyroZ;

        pf_sensorData[X_MAG] = (float) s_MPU9265_AK8963_rawData.i16_magX;
        pf_sensorData[Y_MAG] = (float) s_MPU9265_AK8963_rawData.i16_magY;
        pf_sensorData[Z_MAG] = (float) s_MPU9265_AK8963_rawData.i16_magZ;

    }

    /**
     * @brief   Matches the MPU axes to the QC axes
     */
    static void MPUAxis2QCAxis(float* pf_sensorData){

        pf_sensorData[X_ACC]    =   pf_sensorData[X_ACC];
        pf_sensorData[Y_ACC]    =   -pf_sensorData[Y_ACC];
        pf_sensorData[Z_ACC]    =   pf_sensorData[Z_ACC];

        pf_sensorData[X_GYRO]     =   -pf_sensorData[X_GYRO];
        pf_sensorData[Y_GYRO]     =   pf_sensorData[Y_GYRO];
        pf_sensorData[Z_GYRO]     =   -pf_sensorData[Z_GYRO];

        float temp = 0.0;

        temp = pf_sensorData[X_MAG];                      // save x_mag value for swap

        pf_sensorData[X_MAG]   =   -pf_sensorData[Y_MAG];   // switch x and y axes of the magnetometer to correlate to the accel and gyro axes and inverse it (x axis points to front of quadcopter)
        pf_sensorData[Y_MAG]   =   temp;                  // switch x and y axes of the magnetometer to correlate to the accel and gyro axes
        pf_sensorData[Z_MAG]   =   pf_sensorData[Z_MAG];

    }

    /**
     * @brief   Convert raw MPU data to SI units
     */
    static void convertMPUData(float* pf_sensorData){

        // convert Acc LSB values to a_g multiple of earth acceleration [1g = 9,81m/s^2]
        // Formula: a_g = (2 * a_lsb * sensitivity) / 2^16
        pf_sensorData[X_ACC]    =   (2.0 * pf_sensorData[X_ACC] * 2.0) / 65520.0;
        pf_sensorData[Y_ACC]    =   (2.0 * pf_sensorData[Y_ACC] * 2.0) / 65520.0;
        pf_sensorData[Z_ACC]    =   (2.0 * pf_sensorData[Z_ACC] * 2.0) / 65520.0;

        // Convert from Gyro LSB values to g_w angular velocity [°/s]
        // Formula: g_w = (2 * w_lsb * sensitivity) / 2^16
        pf_sensorData[X_GYRO]     =   (2.0 * pf_sensorData[X_GYRO] * 500.0) / 65520.0;
        pf_sensorData[Y_GYRO]     =   (2.0 * pf_sensorData[Y_GYRO] * 500.0) / 65520.0;
        pf_sensorData[Z_GYRO]     =   (2.0 * pf_sensorData[Z_GYRO] * 500.0) / 65520.0;

        // Convert from degree to rad
        pf_sensorData[X_GYRO]     =   pf_sensorData[X_GYRO] * math_PI / 180.0;
        pf_sensorData[Y_GYRO]     =   pf_sensorData[Y_GYRO] * math_PI / 180.0;
        pf_sensorData[Z_GYRO]     =   pf_sensorData[Z_GYRO] * math_PI / 180.0;


        // Adjust Magnetometer by sensitivity values from manufacturer
        // Formula: m_adj = m_raw * (((ASA_[x,y,z] - 128.0) * 0.5) / 128.0 + 1.0);
        pf_sensorData[X_MAG]   =   pf_sensorData[X_MAG] * (((176.0 - 128.0) * 0.5) / 128.0 + 1.0);
        pf_sensorData[Y_MAG]   =   pf_sensorData[Y_MAG] * (((178.0 - 128.0) * 0.5) / 128.0 + 1.0);
        pf_sensorData[Z_MAG]   =   pf_sensorData[Z_MAG] * (((167.0 - 128.0) * 0.5) / 128.0 + 1.0);

        // Convert Mag LSB values to magnetic flux [uT]
        // Formula: g_w = (2 * w_lsb * sensitivity) / 2^16
        pf_sensorData[X_MAG]   =   (2.0 * pf_sensorData[X_MAG] * 4900.0) / 65520.0;
        pf_sensorData[Y_MAG]   =   (2.0 * pf_sensorData[Y_MAG] * 4900.0) / 65520.0;
        pf_sensorData[Z_MAG]   =   (2.0 * pf_sensorData[Z_MAG] * 4900.0) / 65520.0;
    }



    /**
     * @brief   Correct the MPU offsets after calibration phase
     */
    static void correctMPUOffset(float* pf_sensorData,uint8_t calibrate){

        // calibrate accelerometer statically for 5 secs

        if (calibrate == 1){

            //calculate Sensor offsets
            lp_offsets.x[0] = pf_sensorData[X_ACC];
            lp_offsets.x[1] = pf_sensorData[Y_ACC];
            lp_offsets.x[2] = pf_sensorData[Z_ACC] - 1.0f;

            lowPassArray(&lp_offsets);

            f_gyro_cal[0] += pf_sensorData[X_GYRO];
            f_gyro_cal[1] += pf_sensorData[Y_GYRO];
            f_gyro_cal[2] += pf_sensorData[Z_GYRO];

            f_index++;

        }
        else{

            // perform offset correction
            pf_sensorData[X_ACC]    -= lp_offsets.y[0];
            pf_sensorData[Y_ACC]    -= lp_offsets.y[1];
            pf_sensorData[Z_ACC]    -= lp_offsets.y[2];

            if(f_index != 0.0){
            pf_sensorData[X_GYRO]     -= f_gyro_cal[0] / f_index;
            pf_sensorData[Y_GYRO]     -= f_gyro_cal[1] / f_index;
            pf_sensorData[Z_GYRO]     -= f_gyro_cal[2] / f_index;
            }

            // reduce hard and soft iron biases
            float x_mag_b0 = pf_sensorData[X_MAG] - f_mag_calibration_b[0];
            float y_mag_b1 = pf_sensorData[Y_MAG] - f_mag_calibration_b[1];
            float z_mag_b2 = pf_sensorData[Z_MAG] - f_mag_calibration_b[2];


            pf_sensorData[X_MAG]   = (x_mag_b0 * f_mag_calibration_A[0]) +
                                        (y_mag_b1 * f_mag_calibration_A[3]) +
                                            (z_mag_b2 * f_mag_calibration_A[6]);
            pf_sensorData[Y_MAG]   = (x_mag_b0 * f_mag_calibration_A[1]) +
                                        (y_mag_b1 * f_mag_calibration_A[4]) +
                                            (z_mag_b2 * f_mag_calibration_A[7]);
            pf_sensorData[Z_MAG]   = (x_mag_b0 * f_mag_calibration_A[3]) +
                                        (y_mag_b1 * f_mag_calibration_A[5]) +
                                            (z_mag_b2 * f_mag_calibration_A[8]);

        }
    }

    /**
    * @brief   Filter gyroscope data
    *
    * @details
    *
    * Procedure:
    *
    * 1. lowpass filter
    * 2. First notch filter
    * 3. Second notch filter
    */
    static void filterGyroData(float *pf_sensorData)
    {

        s_lp_gyro.x[0] = pf_sensorData[3];
        s_lp_gyro.x[1] = pf_sensorData[4];
        s_lp_gyro.x[2] = pf_sensorData[5];

        lowPassArray(&s_lp_gyro);

        pf_sensorData[3] = s_lp_gyro.y[0];
        pf_sensorData[4] = s_lp_gyro.y[1];
        pf_sensorData[5] = s_lp_gyro.y[2];

        notchFilterArray(pf_sensorData, &s_nf1_gyro);

        notchFilterArray(pf_sensorData, &s_nf2_gyro);
    }



    /**
    * @brief   Calculate angular velocity from attitude quaternion and copy gyro angular velocity
    */
    static void calculateAngularVelocity(volatile float *q, volatile float *qDot,  float *angularVelocity,
                                         float * pf_sensorData, float*gyroAngularVelocity)
    {



        float quatC1, quatC2, quatC3, quatC4, quatDot1, quatDot2, quatDot3, quatDot4, w1, wx, wy, wz;

        quatC1 = q[0];
        quatC2 = -q[1];
        quatC3 = -q[2];
        quatC4 = -q[3];

        quatDot1 = qDot[0];
        quatDot2 = qDot[1];
        quatDot3 = qDot[2];
        quatDot4 = qDot[3];

        w1 = 2.0 * (quatDot1*quatC1 - quatDot2*quatC2 - quatDot3*quatC3 - quatDot4 * quatC4);
        wx = 2.0 * (quatDot1*quatC2 + quatDot2*quatC1 + quatDot3*quatC4 - quatDot4 * quatC3);
        wy = 2.0 * (quatDot1*quatC3 - quatDot2*quatC4 + quatDot3*quatC1 + quatDot4 * quatC2);
        wz = 2.0 * (quatDot1*quatC4 + quatDot2*quatC3 - quatDot3*quatC2 + quatDot4 * quatC1);

        if(w1 > -0.1 && w1 < 0.1)
        {
            angularVelocity[0] = wx;
            angularVelocity[1] = wy;
            angularVelocity[2] = wz;
        }



        // copy gyro values
        gyroAngularVelocity[0] = pf_sensorData[3];
        gyroAngularVelocity[1] = pf_sensorData[4];
        gyroAngularVelocity[2] = pf_sensorData[5];





    }


    #if (setup_ALT_BARO || setup_ALT_LIDAR)

        /**
          * @brief   ISR for I2C. Give the interrupt handle to the I2C Master instance.
          */
         void Sensor_Alt_I2CIntHandler(void)
         {
             I2CMIntHandler(&i2cAltMastInst_s);
         }

    #endif



    #if (setup_ALT_BARO)

       /**
       * @brief   MS5611 sensor callback function only set flag computation in other functions
       */
       void Sensor_MS5611Callback(void *pvCallbackData, uint_fast8_t ui8_i2cState)
       {
           // check if i2c was a success else save error
           if(ui8_i2cState == I2CM_STATUS_SUCCESS){

               ui8_i2cErrorFlag = 0;

               // fire eventBit to notify Sensor Read has finished
               BaseType_t xHigherPriorityTaskWoken = pdFALSE;
               if(pdFAIL==xEventGroupSetBitsFromISR(gx_sensor_EventGroup, event_SENSOR_BARO_RECEIVED, &xHigherPriorityTaskWoken))
                   while(1);// you will come here, when configTIMER_QUEUE_LENGTH is full
               else
                   portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
           }
           else
           {
               ui8_i2cErrorFlag = 1;
           }

       }


        /**
         * @brief   Calculate temp and pressure from raw sensor values after datasheet
         */
        static void calculatePressureAndTemp(MS5611_rawData_s *ps_rawData, float pf_sensorBaroData[2]){

            // help variables to calculate smoth pressure values (look at datasheet of MS5611)
            int64_t OFF, OFF_C2, SENS, SENS_C1;
            int32_t dT, TEMP, P;
            float f_pressureDiff;
            float pf_help;
            static float f_pressureBase = 0.0;


            //Calculate pressure as explained in the datasheet of the MS-5611.
            dT = (int32_t) (ps_rawData->ui32_baroT - ui16_MS5611CalVal[4] * 256);
            //Calculate Temperature in °C [2000 = 20,00C°] not necessary for application
            TEMP = (int32_t) (2000 + dT * ui16_MS5611CalVal[5] / 8388608);
            pf_sensorBaroData[T_BARO] = ((float) TEMP) *0.01;

            OFF_C2 = (int64_t) ui16_MS5611CalVal[1] * 65536;
            SENS_C1 =(int64_t) ui16_MS5611CalVal[0] * 32768;

            OFF = OFF_C2 + ((int64_t)dT * (int64_t)ui16_MS5611CalVal[3]) / 128;
            SENS = SENS_C1 + ((int64_t)dT * (int64_t)ui16_MS5611CalVal[2]) / 256;
            P = (int32_t)(((ps_rawData->ui32_baroP * SENS) / 2097152 - OFF) / 32768);

            pf_help= (float) P;

            // Use a complementary filter to get a smoother pressure curve
            f_pressureBase = f_pressureBase * 0.985 + pf_help * 0.015;

            f_pressureDiff = f_pressureBase - pf_help;
            // To still guarantee fast behaviour if the difference is to big and secure for malfunctions
            f_pressureDiff = math_LIMIT(f_pressureDiff, -8.0, 8.0);
            if(f_pressureDiff > 1.0 || f_pressureDiff < -1.0)
            {
                f_pressureBase -= f_pressureDiff / 6.0;
            }

            // set f_pressureBase
            pf_sensorBaroData[P_BARO] = f_pressureBase;

        }



        /**
        * @brief   Calculate altitude from temp and pressure values also after calibrate set
        *          pressure reference to zero
        */
        static float calculateAltitude(float pf_sensorBaroData[2] , uint8_t calibrate)
        {

            float f_altitude;


            // clean noise/values outside the resolution of the sensor from the pressure value
            float f_pressureClean;
            modff(pf_sensorBaroData[P_BARO], &f_pressureClean); // stores int part of pressure in pressureClean


            if(calibrate == 1)
            {
                f_pressureReference = pf_sensorBaroData[P_BARO];
            }
            else
            {
                f_altitude = ((powf((f_pressureReference/f_pressureClean),
                                  (1.0/5.257)) - 1.0)*(pf_sensorBaroData[T_BARO] + 273.15)) / 0.0065;
            }

            return f_altitude;

        }

    #endif

    #if ( setup_ALT_LIDAR )


        /**
        *  @brief   LidarLitev3HP sensor callback function only set flag computation in other functions
        */
        void Sensor_LidarLitev3HPCallback(void *pvCallbackData, uint_fast8_t ui8_i2cState)
        {
            // check if i2c was a success else save error
            if(ui8_i2cState == I2CM_STATUS_SUCCESS){

               ui8_i2cErrorFlag = 0;

               // fire eventBit to notify Sensor Read has finished
               BaseType_t xHigherPriorityTaskWoken = pdFALSE;
               if(pdFAIL==xEventGroupSetBitsFromISR(gx_sensor_EventGroup, event_SENSOR_LIDAR_RECEIVED, &xHigherPriorityTaskWoken))
                   while(1);// you will come here, when configTIMER_QUEUE_LENGTH is full
               else
                   portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
            }
            else
            {
               ui8_i2cErrorFlag = 1;
            }

        }

    #endif


    /**
     * @brief   Init the peripheral for the sensor driver
     *
     * @return  void
     */
    void Sensor_InitPeriph(void)
    {
        BusyDelay_Init();

        gx_sensor_EventGroup = xEventGroupCreate();

        // Was the event group created successfully?
        if( gx_sensor_EventGroup == math_NULL )
                while(1);

        ROM_IntPrioritySet(periph_SENSOR_INT, priority_SENSOR_ISR);      // I2C Sensorbus (MPU9265;Baro;LIDAR)

        // Enable peripherial I2C
        ROM_SysCtlPeripheralEnable(I2C_SYSCTL_PERIPH);
        ROM_SysCtlPeripheralEnable(I2C_SYSCTL_PERIPH_PORT);

        // Set GPIOs as I2C pins.
        ROM_GPIOPinConfigure(I2C_SCL);
        ROM_GPIOPinConfigure(I2C_SDA);

        // open drain with pullups
        ROM_GPIOPinTypeI2CSCL(I2C_PORT_BASE, I2C_SCL_PIN);
        ROM_GPIOPinTypeI2C(I2C_PORT_BASE, I2C_SDA_PIN);


        // i2c driver unit
        I2CMInit(   &i2cMastInst_s,
                    I2C_PERIPH_BASE,
                    periph_SENSOR_INT,
                    0xff,
                    0xff,
                    ROM_SysCtlClockGet());



        #if (setup_ALT_BARO || setup_ALT_LIDAR)

            ROM_IntPrioritySet(periph_SENSOR_ALT_INT, priority_SENSOR_ALT_ISR);      // I2C Sensorbus (MPU9265;Baro;LIDAR)

            // Enable peripherial I2C
            ROM_SysCtlPeripheralEnable(I2C_ALT_SYSCTL_PERIPH);
            ROM_SysCtlPeripheralEnable(I2C_ALT_SYSCTL_PERIPH_PORT);

            // Set GPIOs as I2C pins.
            ROM_GPIOPinConfigure(I2C_ALT_SCL);
            ROM_GPIOPinConfigure(I2C_ALT_SDA);

            // open drain with pullups
            ROM_GPIOPinTypeI2CSCL(I2C_ALT_PORT_BASE, I2C_ALT_SCL_PIN);
            ROM_GPIOPinTypeI2C(I2C_ALT_PORT_BASE, I2C_ALT_SDA_PIN);


            // i2c driver unit
            I2CMInit(   &i2cAltMastInst_s,
                        I2C_ALT_PERIPH_BASE,
                        periph_SENSOR_ALT_INT,
                        0xff,
                        0xff,
                        ROM_SysCtlClockGet());


        #endif

        // set optional eventBits name
        HIDE_Fault_SetEventName(fault_SENSOR,"Sen");

        // workload estimation for I2C
        HIDE_Workload_EstimateCreate(&p_workHandle, "sI2C");

        // clear all event bits
        xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);

        #if ( setup_DEV_SUM_RECEIVS )
            HIDE_Display_InsertDrawFun(HIDE_senMesRec_DrawDisplay);
            p_senMesRec_countEdges = CountEdges_Create(event_SENSOR_BIT_COUNT);
            if( p_senMesRec_countEdges == math_NULL )
            {
                while(1); // creation didn't work
            }
            HIDE_senMesRec_SetEventName(event_SENSOR_RECEIVED, "Sen");
        #endif
    }

    /**
     * @brief   Init through the peripheral the sensors
     *
     * @return  void
     */
    void Sensor_InitSensor(void)
    {
        // if returned zero init was not succesfull
        if(!MPU9265_Init(&s_MPU9265Inst, &i2cMastInst_s, I2C_MPU_ADRESS, Sensor_MPU9265Callback, &s_MPU9265Inst))
            while(1); // couldn't initialize

        // blocks task till i2c communication is finished then sets mpu init flag if succesful 2c com
        ui8_mpuInitFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED, SENSOR_MPU_INIT_TIMEOUT_MS);


    #if ( setup_ALT_BARO )

        if(!MS5611_Init(&s_MS5611Inst, &i2cAltMastInst_s, I2C_MS_ADDRESS, Sensor_MS5611Callback, &s_MS5611Inst))
            while(1); // couldn't initialize

        // blocks task till i2c communication is finished then sets baro init flag if succesful 2c com
        ui8_baroInitFlag = blockTaskTillEventBitIsSet(event_SENSOR_BARO_RECEIVED, SENSOR_BARO_INIT_TIMEOUT_MS);

        // Copy calibration values into local variable from buffer
        MS5611_GetCalibrationValues(&s_MS5611Inst, ui16_MS5611CalVal);

    #endif

    #if ( setup_ALT_LIDAR )

        if(!LidarLitev3HP_Init(&s_LidarLitev3HPInst, &i2cAltMastInst_s, LIDARLITEV3HP_ADDRESS,
                               Sensor_LidarLitev3HPCallback, &s_LidarLitev3HPInst))
            while(1); // couldn't initialize

        ui8_lidarInitFlag = blockTaskTillEventBitIsSet(event_SENSOR_LIDAR_RECEIVED,
                                                       SENSOR_LIDAR_INIT_TIMEOUT_MS);
    #endif

    }

    /**
     * @brief   Collect data to calibrate the sensor
     */
    static void SensorCalibrate(void)
    {
        float pf_sensorData[9];

        if(!MPU9265_ReadData(&s_MPU9265Inst, Sensor_MPU9265Callback, &s_MPU9265Inst))
        {
         // couln't initiate a read
        }

        // Start workload estimation for i2c
        HIDE_Workload_EstimateStart(p_workHandle);

        #if( !(setup_ALT_BARO || setup_ALT_LIDAR) )

             ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED, SENSOR_READ_TIMEOUT_MS);

        #endif

     #if (setup_ALT_LIDAR)

         ui8_lidarLoopCounter++;

         if(ui8_lidarLoopCounter == SENSOR_LIDAR_MEASUREMENT)
         {

             ui8_lidarNewMesFlag = 1;
             // start lidar read sequence for altitude measurement
             LidarLitev3HP_ReadData(&s_LidarLitev3HPInst, Sensor_LidarLitev3HPCallback, &s_LidarLitev3HPInst);
         }

         #if (!setup_ALT_BARO)

             ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED | event_SENSOR_LIDAR_RECEIVED,
                                                              SENSOR_READ_TIMEOUT_MS);

          #endif
     #endif

     #if ( setup_ALT_BARO )

         float pf_sensorBaroData[2];
         // decide which baro read seuqence should be initiated
         // after init temp data should be ready to read

         // increase the loop counter each time
         ui8_MS5611LoopCounter++;

         // Every 5th loop (~10ms) a new pressure value is read
         if(ui8_MS5611LoopCounter == SENSOR_BARO_MEASUREMENT)
         {
             // reset
             ui8_MS5611LoopCounter = 0;

             if(ui8_MS5611MesCounter == SENSOR_BARO_MEASUREMENT_TEMP)
             {
                 ui8_MS5611MesCounter = 0;
                 MS5611_ReadData(&s_MS5611Inst, Sensor_MS5611Callback, &s_MS5611Inst,
                                                 SENSOR_BARO_READ_TEMP_REQ_PRESS);

                 ui8_MS5611MesCounter = 0;
             }
             else if(ui8_MS5611MesCounter == SENSOR_BARO_MEASUREMENT_TEMP - 1)
             {
                 MS5611_ReadData(&s_MS5611Inst, Sensor_MS5611Callback, &s_MS5611Inst,
                                                 SENSOR_BARO_READ_PRESS_REQ_TEMP);
                 ui8_MS5611MesCounter++;
             }
             else
             {
                 MS5611_ReadData(&s_MS5611Inst, Sensor_MS5611Callback, &s_MS5611Inst,
                                 SENSOR_BARO_READ_PRESS_REQ_PRESS);
                 ui8_MS5611MesCounter++;
             }

             ui8_MS5611NewMesFlag = 1;

             #if (setup_ALT_LIDAR)

                 if(ui8_lidarLoopCounter == SENSOR_LIDAR_MEASUREMENT)
                 {
                     ui8_lidarLoopCounter = 0;
                     ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_BARO_RECEIVED  |
                                                                      event_SENSOR_LIDAR_RECEIVED |
                                                                      event_SENSOR_RECEIVED,
                                                                      SENSOR_READ_TIMEOUT_MS);

                 }
                 else
                 {
                     ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_BARO_RECEIVED  |
                                                                      event_SENSOR_RECEIVED,
                                                                      SENSOR_READ_TIMEOUT_MS);
                 }
             #else


             ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_BARO_RECEIVED |
                                                              event_SENSOR_RECEIVED,
                                                              SENSOR_READ_TIMEOUT_MS);

             #endif

         }

         else
         {
            #if (setup_ALT_LIDAR)

                 if(ui8_lidarLoopCounter == SENSOR_LIDAR_MEASUREMENT )
                 {
                     ui8_lidarLoopCounter = 0;
                     ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_LIDAR_RECEIVED |
                                                                      event_SENSOR_RECEIVED,
                                                                      SENSOR_READ_TIMEOUT_MS);
                 }
                 else
                 {
                     ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED,
                                                                      SENSOR_READ_TIMEOUT_MS);
                 }

            #else

                 ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED,
                                                                  SENSOR_READ_TIMEOUT_MS);
            #endif
         }

     #endif

     // stop i2c workload
     HIDE_Workload_EstimateStop(p_workHandle);
    // Get MPU data and convert it
    MPU9265_AK8963_GetRawData(&s_MPU9265Inst, &s_MPU9265_AK8963_rawData);
    MPUrawData2Float(pf_sensorData);
    // Correct MPU data
    MPUAxis2QCAxis(pf_sensorData);
    convertMPUData(pf_sensorData);

    correctMPUOffset(pf_sensorData, 1);

    #if (setup_ALT_BARO)

                if((MS5611_GetRawData(&s_MS5611Inst, &s_MS5611_rawData) && ui8_MS5611NewMesFlag == 1))
                {

                    ui8_MS5611NewMesFlag = 0;
                    calculatePressureAndTemp(&s_MS5611_rawData, pf_sensorBaroData);
                    // in calibration mode altitude doesn't have to be saved
                    calculateAltitude(pf_sensorBaroData, 1);
                }

    #endif

    #if (setup_ALT_LIDAR)

        if(ui8_lidarNewMesFlag)
        {
            ui8_lidarNewMesFlag = 0;
            // copy the read buffer into global variables
            LidarLitev3HP_GetRawData(&s_LidarLitev3HPInst, &ui16_lidarAltitude, &gf_sensor_lidarAltitude);
        }
    #endif



    }
    // TODO delete Later
    static float f_usb_debug[9];
    /**
     * @brief   Read the sensors and prepare data for control algorithm
     */
    void Sensor_ReadAndFusion(void)
    {
        float pf_sensorData[9];

        if(!MPU9265_ReadData(&s_MPU9265Inst, Sensor_MPU9265Callback, &s_MPU9265Inst))
        {
            // couln't initiate a read
        }

        // Start workload estimation for i2c
        HIDE_Workload_EstimateStart(p_workHandle);

        #if( !(setup_ALT_BARO || setup_ALT_LIDAR) )

            ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED, SENSOR_READ_TIMEOUT_MS);

        #endif

        #if (setup_ALT_LIDAR)

            ui8_lidarLoopCounter++;
            if(ui8_lidarLoopCounter == SENSOR_LIDAR_MEASUREMENT)
            {

                 ui8_lidarNewMesFlag = 1;
                 // start lidar read sequence for altitude measurement
                 LidarLitev3HP_ReadData(&s_LidarLitev3HPInst, Sensor_LidarLitev3HPCallback, &s_LidarLitev3HPInst);
            }

            #if (!setup_ALT_BARO)

                ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED |
                                                                 event_SENSOR_LIDAR_RECEIVED,
                                                                 SENSOR_READ_TIMEOUT_MS);

            #endif
        #endif

        #if ( setup_ALT_BARO )

            float pf_sensorBaroData[2];
            // decide which baro read seuqence should be initiated
            // after init temp data should be ready to read

            // increase the loop counter each time
            ui8_MS5611LoopCounter++;

            // Every 5th loop (~10ms) a new pressure value is read
            if(ui8_MS5611LoopCounter == SENSOR_BARO_MEASUREMENT)
            {
                // reset
                ui8_MS5611LoopCounter = 0;

                if(ui8_MS5611MesCounter == SENSOR_BARO_MEASUREMENT_TEMP)
                {
                    ui8_MS5611MesCounter = 0;
                    MS5611_ReadData(&s_MS5611Inst, Sensor_MS5611Callback, &s_MS5611Inst,
                                       SENSOR_BARO_READ_TEMP_REQ_PRESS);

                    ui8_MS5611MesCounter = 0;
                }
                else if(ui8_MS5611MesCounter == SENSOR_BARO_MEASUREMENT_TEMP - 1)
                {
                    MS5611_ReadData(&s_MS5611Inst, Sensor_MS5611Callback, &s_MS5611Inst,
                                           SENSOR_BARO_READ_PRESS_REQ_TEMP);
                    ui8_MS5611MesCounter++;
                }
                else
                {
                    MS5611_ReadData(&s_MS5611Inst, Sensor_MS5611Callback, &s_MS5611Inst,
                           SENSOR_BARO_READ_PRESS_REQ_PRESS);
                    ui8_MS5611MesCounter++;
                }

                ui8_MS5611NewMesFlag = 1;

            #if (setup_ALT_LIDAR)
                if(ui8_lidarLoopCounter == SENSOR_LIDAR_MEASUREMENT)
                {
                    ui8_lidarLoopCounter = 0;
                    ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_BARO_RECEIVED  |
                                                                     event_SENSOR_LIDAR_RECEIVED |
                                                                     event_SENSOR_RECEIVED,
                                                                     SENSOR_READ_TIMEOUT_MS);

                }
                else
                {
                    ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_BARO_RECEIVED  |
                                                                     event_SENSOR_RECEIVED,
                                                                     SENSOR_READ_TIMEOUT_MS);
                 }
            #else


                    ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_BARO_RECEIVED |
                                                            event_SENSOR_RECEIVED,
                                                            SENSOR_READ_TIMEOUT_MS);

            #endif

            }

            else
            {
                #if (setup_ALT_LIDAR)

                    if(ui8_lidarLoopCounter == SENSOR_LIDAR_MEASUREMENT )
                    {
                        ui8_lidarLoopCounter = 0;
                        ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_LIDAR_RECEIVED |
                                                                         event_SENSOR_RECEIVED,
                                                                         SENSOR_READ_TIMEOUT_MS);
                    }
                    else
                    {
                        ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED,
                                                                         SENSOR_READ_TIMEOUT_MS);
                    }
                #else

                    ui8_i2cDataReadFlag = blockTaskTillEventBitIsSet(event_SENSOR_RECEIVED,
                                            SENSOR_READ_TIMEOUT_MS);
                #endif
            }

        #endif

    // stop i2c workload
    HIDE_Workload_EstimateStop(p_workHandle);
    // Get MPU data and convert it
    MPU9265_AK8963_GetRawData(&s_MPU9265Inst, &s_MPU9265_AK8963_rawData);
    MPUrawData2Float(pf_sensorData);
    // Correct MPU data
    MPUAxis2QCAxis(pf_sensorData);
    convertMPUData(pf_sensorData);
    correctMPUOffset(pf_sensorData, 0);




    filterGyroData(&pf_sensorData[3]);



//


    // Get attitude quaternion via sensor fusion from MPU data
    MadgwickAHRSupdate(pf_sensorData[X_ACC], pf_sensorData[Y_ACC], pf_sensorData[Z_ACC],
                      pf_sensorData[X_GYRO],  pf_sensorData[Y_GYRO], pf_sensorData[Z_GYRO],
                      pf_sensorData[X_MAG], pf_sensorData[Y_MAG], pf_sensorData[Z_MAG],
                      gf_sensor_attitudeQuaternion, gf_sensor_dotAttitudeQuaternion, dt);

    // Get Euler/Tait-Bryan angles from quaternion
    Math_QuatToEuler(gf_sensor_attitudeQuaternion, gf_sensor_fusedAngles);
    //Math_QuatToEuler(gf_sensor_dotAttitudeQuaternion, gf_sensor_angularVelocity);



    calculateAngularVelocity(gf_sensor_attitudeQuaternion, gf_sensor_dotAttitudeQuaternion, gf_sensor_angularVelocity, pf_sensorData, gf_sensor_gyroAngularVelocity);

     #if (setup_ALT_BARO)


        // get the new raw baro data and calculate is firstly started after
        // 20 pressure and 5 temp values have been read then calculate pressure/altitude
        if((MS5611_GetRawData(&s_MS5611Inst, &s_MS5611_rawData) && ui8_MS5611NewMesFlag == 1))
        {

            // reset
            ui8_MS5611NewMesFlag = 0;

            calculatePressureAndTemp(&s_MS5611_rawData, pf_sensorBaroData);
            // in calibration mode altitude doesn't have to be saved
            gf_sensor_baroAltitude = calculateAltitude(pf_sensorBaroData, 0);


        }


    #endif

    #if (setup_ALT_LIDAR)
        if(ui8_lidarNewMesFlag)
        {
            ui8_lidarNewMesFlag = 0;
            // copy the read buffer into global variables
            LidarLitev3HP_GetRawData(&s_LidarLitev3HPInst, &ui16_lidarAltitude, &gf_sensor_lidarAltitude);


        }
    #endif

    }

#elif ( setup_SENSOR_NONE == (setup_SENSOR&setup_MASK_OPT1) )

	void Sensor_InitPeriph(void){}
	void Sensor_InitSensor(void){}
	void SensorCalibrate(void){}
	void Sensor_ReadAndFusion(void){}
#else
	#error ERROR: define setup_SENSOR (in qc_setup.h)
#endif

/**
 * @brief   If calibration is required, start calibration.
 *
 * @details
 * Calibrate for a minimum time of x ms. (x is defined in the driver).
 *
 * @param   elapseTimeMS --> elapsed time between last call
 *
 * @return  void
 */
void Sensor_Calibrate(int32_t elapseTimeMS)
{
    if(IS_CALIBRAE_REQUIRED(i32_stateTime))
    {

        SensorCalibrate();
        i32_stateTime+=elapseTimeMS;
        if(i32_stateTime>=CALIBRATE_MIN_TIME_MS)
            i32_stateTime=CALIBRATE_MIN_TIME_MS;
    }
}

/**
 * @brief   Stops the requirement of the sensor to calibrate
 *
 * @return  void
 */
void Sensor_CalibrateStop(void)
{
    i32_stateTime=CALIBRATE_STOP;

}

/**
 * @brief   Requires the sensor to start calibration
 *
 * @details
 * This does not calibrate, only requires. Use Sensor_Calibrate to calibrate
 *
 * @return  void
 */
void Sensor_CalibrateRequire(void)
{
    if(!IS_CALIBRAE_REQUIRED(i32_stateTime))
        i32_stateTime=CALIBRATE_START;
}

/**
 * @brief   Get the state of the sensor calibration
 *
 * @return  true --> If calibration is not running or ( calibration is running but long enough )\n
 *          false --> Else
 */
uint8_t Sensor_IsCalibrateReady(void)
{
    if(i32_stateTime==CALIBRATE_STOP)
        return true;
    if(i32_stateTime>=CALIBRATE_MIN_TIME_MS)
        return true;
    return false;
}

/**
 * @brief   Check if calibration is required
 *
 * @return  true --> If calibration is required\n
 *          false --> else
 */
uint8_t Sensor_IsCalibrateRequired(void)
{

    return IS_CALIBRAE_REQUIRED(i32_stateTime);

}

//=====================================================================================================
// End of file
//=====================================================================================================
