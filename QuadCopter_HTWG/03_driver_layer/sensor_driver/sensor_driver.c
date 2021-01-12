//=====================================================================================================
// @file sensor_driver.c
//=====================================================================================================
//
// @brief Implementation of Madgwick's AHRS algorithm.
//
// Date                 Author                      Notes
// @date 31/05/2016     @author Tobias Grimm        Implementation
// @date 06/12/2020     @author Tomas Schweizer     Overall changes
//
// Source:
//
//
//=====================================================================================================

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/
#include <stdint.h>

// own header file
#include "sensor_driver.h"

// setup
#include "peripheral_setup.h"
#include "prioritys.h"

// drivers
#include "display_driver.h"
#include "debug_interface.h"
#include "MadgwickAHRS.h"
#include "basic_filters.h"

// utils
#include "qc_math.h"
#include "workload.h"
#include "fault.h"
#include "busy_delay.h"


// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"






/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/
#define SENSOR_INIT_TIMEOUT_MS      ( 10 )
#define SENSOR_READ_TIMEOUT_MS		( 1 )


#define CALIBRATE_MIN_TIME_MS               5000//###5000
#define CALIBRATE_BEFORE_FIRST_START        -2
#define CALIBRATE_STOP                      -1
#define CALIBRATE_START                     0
#define IS_CALIBRAE_REQUIRED(stateTime)     (stateTime>=0 || stateTime == -2)

/** \brief  sensor eventBit*/
#define event_SENSOR_BIT_COUNT                           ( 2 )
#define event_RECEIVED_pf_sensorData                ( 1 << 0 )

#define dt 0.002f

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
    T_BARO,
    P_BARO,
    A_LIDAR

};

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/

#if ( setup_DEV_SUM_RECEIVS )
    extern void HIDE_senMesRec_SetEventName(uint32_t ui32_eventBit,const char* pc_name);
    extern void HIDE_senMesRec_DrawDisplay(void);
    extern void HIDE_senMesRec_Increment(uint32_t ui32_receiveEventBit);
#else
    #define HIDE_senMesRec_SetEventName(ui32_eventBit,pc_name)            // this define will be kicked off from the preprocessor
    #define HIDE_senMesRec_DrawDisplay                            0       // 0 pointer
    #define HIDE_senMesRec_Increment(ui32_receiveEventBit)
#endif

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/
volatile float gf_sensor_attitudeQuaternion[4] = {1.0, 0.0, 0.0, 0.0};
volatile float gf_sensor_dotAttitudeQuaternion[4];
float gf_sensor_fusedAngles[3];
float gf_sensor_angularVelocity[3];
float gf_sensor_pressure; // TODO maybe later just altitude global
float gf_sensor_altitude;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/
volatile EventGroupHandle_t gx_sensor_EventGroup = 0;
// variable to check i2c workload
static workload_handle_p p_workHandle;
// count received sensor measurements
countEdges_handle_p p_senMesRec_countEdges;

static int32_t i32_stateTime = CALIBRATE_BEFORE_FIRST_START;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/**
* \brief   send data of Sensor over USB to PC application
*/
#if ( setup_DEV_DEBUG_USB )

    static float gf_usb_debug[9];

    void HIDE_Sensor_SendDataOverUSB(void)
    {
        gf_usb_debug[0] = gf_sensor_pressure;
        HIDE_Debug_USB_InterfaceSend(gf_usb_debug, sizeof(gf_usb_debug)/ sizeof(gf_usb_debug[0]), debug_FLOAT);
    }

#endif

#if ( setup_DEV_SUM_RECEIVS )
    /**
     * \brief   Draw info about receive Counts on the Display
     * \note    to enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
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
     * \brief   store the name of the receive eventBit into p_receive_coundEdges
     *
     */
    void HIDE_senMesRec_SetEventName(uint32_t ui32_eventBit,const char* pc_name)
    {
        CountEdges_SetName(p_senMesRec_countEdges,CountEdges_Bit2Num(ui32_eventBit),pc_name);
    }

    /**
     * \brief   increment at ui32_receiveEventBit in p_receive_coundEdges
     * \param   ui32_receiveEventBit    the receive eventBit (see receiver_task.h)
     * \note    to enable this HIDE function set setup_DEV_SUM_RECEIVS in qc_setup.h
     */
    void HIDE_senMesRec_Increment(uint32_t ui32_receiveEventBit)
    {
        CountEdges_Increment(p_senMesRec_countEdges,CountEdges_Bit2Num(ui32_receiveEventBit));
    }

#endif

/**
 * \brief   Draw info about Sensor on the Display
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

}

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Select Mode                                                   */
/* ---------------------------------------------------------------------------------------------------*/

#if   ( setup_SENSOR_I2C == (setup_SENSOR&setup_MASK_OPT1) ) || DOXYGEN

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

    // sensor register files
    #include "mpu9265_registers.h"
    #include "ak8963_registers.h"
    #include "ms5611_registers.h"

    // sensor lib files
    #include "sensor_mpu9265.h"





    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local defines i2c Mode                                     */
    /* -----------------------------------------------------------------------------------------------*/

    // define if barometer and lidar should be enabled
    #define BARO_INIT                           ( 0 )
    #define LIDAR_INIT                          ( 0 )

    // define i2c addresses of sensors
    #define I2C_MPU_ADRESS                      MPU9265_ADDRESS
    #define I2C_AK_ADDRESS                      AK8963_ADDRESS
    #define I2C_MS_ADDRESS                      MS5611_ADDRESS
    #define I2C_LI_ADDRESS                      0 // TODO change after implementation

    // define the desired peripheral setup (see peripheral_setup.h)
    #if ( periph_SENSOR_INT == INT_I2C2 )
        #define SYSCTL_PERIPH_I2C           SYSCTL_PERIPH_I2C2
        #define I2C_PERIPH_BASE             I2C2_BASE
        #define SYSCTL_PERIPH_PORT          SYSCTL_PERIPH_GPIOE
        #define PORT_BASE                   GPIO_PORTE_BASE
        #define I2C_SCL                     GPIO_PE4_I2C2SCL
        #define SCL_PIN                     GPIO_PIN_4
        #define I2C_SDA                     GPIO_PE5_I2C2SDA
        #define SDA_PIN                     GPIO_PIN_5
    #elif ( periph_SENSOR_INT == INT_I2C1 )
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
    static void MPUrawData2Float(float* pf_sensorData);
    static void MPUAxis2QCAxis(float* pf_sensorData);
    static void convertMPUData(float* pf_sensorData);
    static void correctMPUOffset(float* pf_sensorData, uint8_t calibrate);
    static void calculateAngularVelocity(float *pf_sensorData, float *angularVelocity);

    void Sensor_MPU9265Callback(void *pvCallbackData, uint_fast8_t ui8Status);

    // TODO baro functions enable later
    //static baroData_s calculateBaroData(uint32_t t_baro, uint32_t p_baro, uint8_t calibrate);
    //static float calculateAltitude(baroData_s baroData, float accel_z, uint8_t calibrate);


    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Global Variables i2c Mode                                      */
    /* ---------------------------------------------------------------------------------------------------*/

    #if(periph_SENSOR_INT==periph_MOTOR_INT)
        extern tI2CMInstance i2cMastInst_s;             // I2C master instance
    #else
        static tI2CMInstance i2cMastInst_s;             // I2C master instance
    #endif

    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Local Variables i2c Mode                                      */
    /* ---------------------------------------------------------------------------------------------------*/

    // varibales for sensor functions
    MPU9265_s s_MPU9265Inst;
    MPU9265_AK8963_rawData_s s_MPU9265_AK8963_rawData;




    // struct for low pass filtering of acc data
    struct lp_a_struct lp_offsets = {
            .alpha = LP_FREQ2ALPHA(0.3f, dt)
    };
    // variables for calibrating gyro
    static float f_gyro_cal[3] = { 0.0, 0.0, 0.0};
    static float f_index = 0;
    // Magnetometer calibration data
    static const float f_mag_calibration_b[3] = { -37.8932,   -3.4380,  -66.0203};
    static const float f_mag_calibration_A[9] = { 1.0296,         0.0,         0.0,
                                                 0.0,    0.9850,         0.0,
                                                 0.0,         0.0,    0.9861};

    // variable for barometer calibration values
    //static uint16_t ui16_baro_calibration[6];

    // setup flags
    static uint8_t ui8_mpuInitFlag = 0;

    // flags for i2c transfer
    static volatile uint8_t ui8_i2cFinishedFlag = 0;
    static volatile uint8_t ui8_i2cErrorFlag = 0;
    static volatile uint8_t ui8_i2cMPU9265DataReadFlag = 0;

//  TODO check which variables are needed for barometer
//    // Counter variables for Barometer (MS5611) loop
//    static uint8_t ui8_i2cPressCounter = 0;
//    static uint8_t ui8_i2cTempCounter = 0;
//
//    // variables for temp from barometer
//    static uint32_t ui32_tempRotMem[5];
//    static uint8_t ui8_rotMemCounter;
//    static uint32_t ui32_tempSum;
//
//    // variables for pressure from barometer
//    static uint32_t ui32_pressRotMem[20];
//    static uint8_t ui8_pressMemCounter;
//    static uint32_t ui32_pressSum;
//    static float f_pressureBase = 97500; // Set to a typical value for around 300m to get faster filter convergence
//
//    //flag new pressure measurement is there
//    static uint8_t ui8_newPressValue = 0;
//
//    static float f_pressureReference = 101325.0; // pressure at sea level gets changed in calibrate to local pressure on ground
//
//    static rawData_s s_rawData;
//    static baroData_s s_baroData;

    /* -----------------------------------------------------------------------------------------------*/
    /*                                      Procedure Definitions i2c mode                                    */
    /* -----------------------------------------------------------------------------------------------*/

    /**
     * \brief   Convert raw int16 values from MPU to float values
     *
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
     * \brief   Matches the MPU axes to the QC axes
     *
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
     * \brief   Convert raw MPU data to SI units
     *
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
     * \brief   Correct the MPU offsets after calibration phase
     *
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
    * \brief   calculate angulat velocity from tait-bryan angles
    */
    static void calculateAngularVelocity(float *angles, float *angularVelocity){

        float w_angles[3];
        static float anglesOld[3];

        uint8_t i;
        for(i=0; i<3; i++)
        {
            w_angles[i] = (angles[i] - anglesOld[i]) / dt;
            // TODO check if usable limits
            math_LIMIT(w_angles[i], -2.0, 2.0);
            gf_sensor_angularVelocity[i] = w_angles[i];
            anglesOld[i] = angles[i];
        }

    }

    /**
     * \brief   ISR for I2C
     *          Give the interrupt handle to the I2C Master instance
     */
    void Sensor_I2CIntHandler(void)
    {
        I2CMIntHandler(&i2cMastInst_s);
    }

    /**
    * \brief   MPU9265 sensor callback function only set flag computation in other functions
    *
    */
    void Sensor_MPU9265Callback(void *pvCallbackData, uint_fast8_t ui8_i2cState){

        // check if i2c was a success else save error
        if(ui8_i2cState == I2CM_STATUS_SUCCESS){

            ui8_i2cErrorFlag = 0;

            if(ui8_mpuInitFlag){
                HIDE_Workload_EstimateStop(p_workHandle);
            }

            // fire eventBit to notify Sensor Read has finished
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if(pdFAIL==xEventGroupSetBitsFromISR(gx_sensor_EventGroup, event_RECEIVED_pf_sensorData, &xHigherPriorityTaskWoken))
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
     * \brief   Init the peripheral for the sensor driver
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

         // set optional eventBits name
        HIDE_Fault_SetEventName(fault_SENSOR,"Sen");

        // workload estimation for I2C
        HIDE_Workload_EstimateCreate(&p_workHandle, "sI2C");

        #if ( setup_DEV_SUM_RECEIVS )
            HIDE_Display_InsertDrawFun(HIDE_senMesRec_DrawDisplay);
            p_senMesRec_countEdges = CountEdges_Create(event_SENSOR_BIT_COUNT);
            if( p_senMesRec_countEdges == math_NULL )
            {
                while(1); // creation didn't work
            }
            HIDE_senMesRec_SetEventName(event_RECEIVED_pf_sensorData, "Sen");
        #endif
    }

    /**
     * \brief   Init through the peripheral the sensor chip itself
     */
    void Sensor_InitSensor(void)
    {

        xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);
        xEventGroupClearBits(gx_sensor_EventGroup,event_RECEIVED_pf_sensorData);

        // if returned zero init was not succesfull
        if(!MPU9265_Init(&s_MPU9265Inst, &i2cMastInst_s, I2C_MPU_ADRESS, Sensor_MPU9265Callback, &s_MPU9265Inst))
            while(1);

        EventBits_t x_sensorEventBits;
        //  Wait for sensor received event
        x_sensorEventBits = xEventGroupWaitBits(gx_sensor_EventGroup,
        event_RECEIVED_pf_sensorData,
        pdTRUE,          // clear Bits before returning.
        pdTRUE,          // wait for all Bits
        SENSOR_INIT_TIMEOUT_MS / portTICK_PERIOD_MS ); // maximum wait time

        // unblock because of timeout, or i2cError?
        uint8_t ui8_error=(event_RECEIVED_pf_sensorData & x_sensorEventBits == 0) || ui8_i2cErrorFlag != 0;
        if( ui8_error )
        {
           xEventGroupSetBits(gx_fault_EventGroup,fault_SENSOR);
        }
        else
        {
           xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);
           ui8_mpuInitFlag = 1;
        }
        HIDE_Fault_Increment(fault_SENSOR,ui8_error);

    }

    /**
     * \brief   collect data to calibrate the sensor
     */
    static void SensorCalibrate(void)
    {
        float pf_sensorData[12];

        if(!MPU9265_ReadData(&s_MPU9265Inst, Sensor_MPU9265Callback, &s_MPU9265Inst))
        {
            // couln't initiate a read
        }


        HIDE_Workload_EstimateStart(p_workHandle);
        EventBits_t x_sensorEventBits;
        //  Wait for sensor received event
        x_sensorEventBits = xEventGroupWaitBits(gx_sensor_EventGroup,
               event_RECEIVED_pf_sensorData,
               pdTRUE,          // clear Bits before returning.
               pdTRUE,          // wait for all Bits
               SENSOR_READ_TIMEOUT_MS / portTICK_PERIOD_MS ); // maximum wait time

        // unblock because of timeout, or i2cError?
        uint8_t ui8_error=(event_RECEIVED_pf_sensorData & x_sensorEventBits == 0) || ui8_i2cErrorFlag != 0;
        if( ui8_error )
        {
           xEventGroupSetBits(gx_fault_EventGroup,fault_SENSOR);
        }
        else
        {
           xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);
           HIDE_senMesRec_Increment(event_RECEIVED_pf_sensorData);
        }
        HIDE_Fault_Increment(fault_SENSOR,ui8_error);

        // Get MPU data and convert it
        MPU9265_AK8963_GetRawData(&s_MPU9265Inst, &s_MPU9265_AK8963_rawData);
        MPUrawData2Float(pf_sensorData);
        // Correct MPU data
        MPUAxis2QCAxis(pf_sensorData);
        convertMPUData(pf_sensorData);
        correctMPUOffset(pf_sensorData, 1);



    }
    /**
     * \brief   read the sonsor and prepare data (sensor fusion).
     *
     *          fire eventBit fault_SENSOR if there is a fault,
     *          else clear the bit.
     * \note    Events: EventBit fault_SENSOR will be set or cleared in gx_fault_EventGroup
     */
    void Sensor_ReadAndFusion(void)
    {
        // Read out sensor data and perform formatting
        float pf_sensorData[12];
        // TODO test baro

        if(!MPU9265_ReadData(&s_MPU9265Inst, Sensor_MPU9265Callback, &s_MPU9265Inst))
        {
            // couln't initiate a read
        }

        HIDE_Workload_EstimateStart(p_workHandle);
        EventBits_t x_sensorEventBits;
        //  Wait for sensor received event
        x_sensorEventBits = xEventGroupWaitBits(gx_sensor_EventGroup,
                event_RECEIVED_pf_sensorData,
                pdTRUE,          // clear Bits before returning.
                pdTRUE,          // wait for all Bits
                SENSOR_READ_TIMEOUT_MS / portTICK_PERIOD_MS ); // maximum wait time

        // unblock because of timeout, or i2cError?
        uint8_t ui8_error=(event_RECEIVED_pf_sensorData & x_sensorEventBits == 0) || ui8_i2cErrorFlag != 0;
        if( ui8_error )
        {
           xEventGroupSetBits(gx_fault_EventGroup,fault_SENSOR);
        }
        else
        {
           xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);
           HIDE_senMesRec_Increment(event_RECEIVED_pf_sensorData);
        }
        HIDE_Fault_Increment(fault_SENSOR,ui8_error);

        // Get MPU data and convert it
        MPU9265_AK8963_GetRawData(&s_MPU9265Inst, &s_MPU9265_AK8963_rawData);
        MPUrawData2Float(pf_sensorData);
        // Correct MPU data
        MPUAxis2QCAxis(pf_sensorData);
        convertMPUData(pf_sensorData);
        correctMPUOffset(pf_sensorData, 0);

        // Get attitude quaternion via sensor fusion from MPU data
        MadgwickAHRSupdate(pf_sensorData[X_ACC], pf_sensorData[Y_ACC], pf_sensorData[Z_ACC],
                           pf_sensorData[X_GYRO],  pf_sensorData[Y_GYRO], pf_sensorData[Z_GYRO],
                           pf_sensorData[X_MAG], pf_sensorData[Y_MAG], pf_sensorData[Z_MAG],
                           gf_sensor_attitudeQuaternion, gf_sensor_dotAttitudeQuaternion, dt);

        // Get Euler/Tait-Bryan angles from quaternion
        Math_QuatToEuler(gf_sensor_attitudeQuaternion, gf_sensor_fusedAngles);
        calculateAngularVelocity(gf_sensor_fusedAngles, gf_sensor_angularVelocity);

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
 * \brief   if calibration is required, start calibration.
 *
 *          calibrate for a minimum time of x ms.
 *          (x is defined in the driver)
 * \param   elapseTimeMS    elapsed time between last call
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
 * \brief   stopts the requirement of the sensor to calibrate
 */
void Sensor_CalibrateStop(void)
{
    i32_stateTime=CALIBRATE_STOP;

}

/**
 * \brief   requires the sensor to start calibration
 *
 *          (this does not calibrate, only requires
 *          use Sensor_Calibrate to calibrate)
 */
void Sensor_CalibrateRequire(void)
{
    if(!IS_CALIBRAE_REQUIRED(i32_stateTime))
        i32_stateTime=CALIBRATE_START;
}

/**
 * \brief   get the state of the sensor calibration
 * \return  true if calibration is not running or ( calibration is running but long enough )
 *          false else
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
 * \brief   return true if calibration is required
 * \return  true  if calibration is required,
 *          false else
 */
uint8_t Sensor_IsCalibrateRequired(void)
{

    return IS_CALIBRAE_REQUIRED(i32_stateTime);

}

//=====================================================================================================
// End of file
//=====================================================================================================
