/**
 * 		@file 	sensor_driver.c
 * 		@brief	functions to interact with the 9-Axis Gyro+Accel+Magn sensor
 *//*	@author Tobias Grimm
 *//*
 *//*	@modifier Tomas Schweizer
 * 		@date 	24.11.2020	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>


//  Hardware Specific
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "sensorlib/i2cm_drv.h"
#include "peripheral_setup.h"

// setup
#include "prioritys.h"

// drivers
#include "basic_filters.h"
#include "display_driver.h"
#include "debug_interface.h"
#include "MadgwickAHRS.h"


// utils
#include "qc_math.h"
#include "workload.h"
#include "fault.h"



// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"
#include "receiver_task.h"



/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

#define SENSOR_READ_TIMEOUT_MS		( 1 )

#define dt 0.002f
#define PI 3.14159265358979323846f

#define X_ACCEL		0
#define Y_ACCEL		1
#define Z_ACCEL		2
#define X_GYRO		3
#define Y_GYRO		4
#define Z_GYRO		5
#define X_MAGNET	6
#define Y_MAGNET	7
#define Z_MAGNET	8
#define AXIS_COUNT	9

#define CALIBRATE_MIN_TIME_MS				5000//###5000
#define CALIBRATE_BEFORE_FIRST_START		-2
#define CALIBRATE_STOP						-1
#define CALIBRATE_START						0
#define IS_CALIBRAE_REQUIRED(stateTime)		(stateTime>=0)


/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

// TODO only test variable delete, change later
int16_t gi16_sensor_data[9];
float new_quaternion[4];
float gf_sensor_data[9];


float gf_sensor_fusedAngles[3];
float gf_sensor_pressure;



/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */
static int32_t i32_stateTime = CALIBRATE_BEFORE_FIRST_START;

// struct for low pass filtering of acc data
struct lp_a_struct lp_offsets = {
		.alpha = LP_FREQ2ALPHA(0.3f, dt)
};

// variables for calibrating gyro
static float f_gyro_cal[3] = { 0.0, 0.0, 0.0};
static float f_index = 0;


// Magnetometer calibration data
static float f_mag_calibration_b[3] = { -37.8932,   -3.4380,  -66.0203};
static float f_mag_calibration_A[9] = { 1.0296,         0.0,         0.0,
                                        0.0,    0.9850,         0.0,
                                        0.0,         0.0,    0.9861};

// variable for barometer calibration values
static uint16_t ui16_baro_calibration[6];


// For display
static const char* p_sign = " ";
static const char* m_sign = "-";


/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/**
 * \brief	Draw info about Sensor on the Display
 */
void Sensor_DrawDisplay(void)
{
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


// TODO check if it works/test: function to convert sensor axis to quadcopter axis

static void IMUAxis2QCAxis(float* sensor_data){

    sensor_data[X_ACCEL]    =   sensor_data[X_ACCEL];
    sensor_data[Y_ACCEL]    =   -sensor_data[Y_ACCEL];
    sensor_data[Z_ACCEL]    =   sensor_data[Z_ACCEL];

    // TODO Important for alining with OPENGL probably must be changed for controller
    sensor_data[X_GYRO]     =   -sensor_data[X_GYRO];
    sensor_data[Y_GYRO]     =   sensor_data[Y_GYRO];
    sensor_data[Z_GYRO]     =   -sensor_data[Z_GYRO];

    float temp = 0.0;

    temp = sensor_data[X_MAGNET]; // save x_mag value for swap

    sensor_data[X_MAGNET]   =   -sensor_data[Y_MAGNET]; // switch x and y axes of the magnetometer to correlate to the accel and gyro axes and inverse it (x axis points to front of quadcopter)
    sensor_data[Y_MAGNET]   =   temp; // switch x and y axes of the magnetometer to correlate to the accel and gyro axes
    sensor_data[Z_MAGNET]   =   sensor_data[Z_MAGNET];



}

static void convertIMUData(float* sensor_data){

    // convert Acc LSB values to a_g multiple of earth acceleration [1g = 9,81m/s^2]
    // Formula: a_g = (2 * a_lsb * sensitivity) / 2^16
    sensor_data[X_ACCEL]    =   (2.0 * sensor_data[X_ACCEL] * 2.0) / 65520.0;
    sensor_data[Y_ACCEL]    =   (2.0 * sensor_data[Y_ACCEL] * 2.0) / 65520.0;
    sensor_data[Z_ACCEL]    =   (2.0 * sensor_data[Z_ACCEL] * 2.0) / 65520.0;

    // Convert from Gyro LSB values to g_w angular velocity [°/s]
    // Formula: g_w = (2 * w_lsb * sensitivity) / 2^16
    sensor_data[X_GYRO]     =   (2.0 * sensor_data[X_GYRO] * 500.0) / 65520.0;
    sensor_data[Y_GYRO]     =   (2.0 * sensor_data[Y_GYRO] * 500.0) / 65520.0;
    sensor_data[Z_GYRO]     =   (2.0 * sensor_data[Z_GYRO] * 500.0) / 65520.0;

    // Convert from degree to rad
    sensor_data[X_GYRO]     =   sensor_data[X_GYRO] * PI / 180.0;
    sensor_data[Y_GYRO]     =   sensor_data[Y_GYRO] * PI / 180.0;
    sensor_data[Z_GYRO]     =   sensor_data[Z_GYRO] * PI / 180.0;


    // Adjust Magnetometer by sensitivity values from manufacturer
    // Formula: m_adj = m_raw * (((ASA_[x,y,z] - 128.0) * 0.5) / 128.0 + 1.0);
    sensor_data[X_MAGNET]   =   sensor_data[X_MAGNET] * (((176.0 - 128.0) * 0.5) / 128.0 + 1.0);
    sensor_data[Y_MAGNET]   =   sensor_data[Y_MAGNET] * (((178.0 - 128.0) * 0.5) / 128.0 + 1.0);
    sensor_data[Z_MAGNET]   =   sensor_data[Z_MAGNET] * (((167.0 - 128.0) * 0.5) / 128.0 + 1.0);

    // Convert Mag LSB values to magnetic flux [uT]
    // Formula: g_w = (2 * w_lsb * sensitivity) / 2^16
    sensor_data[X_MAGNET]   =   (2.0 * sensor_data[X_MAGNET] * 4900.0) / 65520.0;
    sensor_data[Y_MAGNET]   =   (2.0 * sensor_data[Y_MAGNET] * 4900.0) / 65520.0;
    sensor_data[Z_MAGNET]   =   (2.0 * sensor_data[Z_MAGNET] * 4900.0) / 65520.0;
}


// TODO own implementation of Offset always after senAxis2CXAxis
static void correctIMUOffset(float* sensor_data,uint8_t calibrate){

    // calibrate accelerometer statically for 5 secs

    if (calibrate == 1){

        //calculate Sensor offsets
        lp_offsets.x[0] = sensor_data[X_ACCEL];
        lp_offsets.x[1] = sensor_data[Y_ACCEL];
        lp_offsets.x[2] = sensor_data[Z_ACCEL] - 1.0f;

        lowPassArray(&lp_offsets);

        f_gyro_cal[0] += sensor_data[X_GYRO];
        f_gyro_cal[1] += sensor_data[Y_GYRO];
        f_gyro_cal[2] += sensor_data[Z_GYRO];

        f_index++;

    }
    else{

        // perform offset correction
        sensor_data[X_ACCEL]    -= lp_offsets.y[0];
        sensor_data[Y_ACCEL]    -= lp_offsets.y[1];
        sensor_data[Z_ACCEL]    -= lp_offsets.y[2];

        if(f_index != 0.0){
        sensor_data[X_GYRO]     -= f_gyro_cal[0] / f_index;
        sensor_data[Y_GYRO]     -= f_gyro_cal[1] / f_index;
        sensor_data[Z_GYRO]     -= f_gyro_cal[2] / f_index;
        }

        // reduce hard and soft iron biases
        float x_mag_b0 = sensor_data[X_MAGNET] - f_mag_calibration_b[0];
        float y_mag_b1 = sensor_data[Y_MAGNET] - f_mag_calibration_b[1];
        float z_mag_b2 = sensor_data[Z_MAGNET] - f_mag_calibration_b[2];


        sensor_data[X_MAGNET]   = (x_mag_b0 * f_mag_calibration_A[0]) +
                                    (y_mag_b1 * f_mag_calibration_A[3]) +
                                        (z_mag_b2 * f_mag_calibration_A[6]);
        sensor_data[Y_MAGNET]   = (x_mag_b0 * f_mag_calibration_A[1]) +
                                    (y_mag_b1 * f_mag_calibration_A[4]) +
                                        (z_mag_b2 * f_mag_calibration_A[7]);
        sensor_data[Z_MAGNET]   = (x_mag_b0 * f_mag_calibration_A[3]) +
                                    (y_mag_b1 * f_mag_calibration_A[5]) +
                                        (z_mag_b2 * f_mag_calibration_A[8]);

    }
}


/* ------------------------------------------------------------ */
/*				Select the Mode									*/
/* ------------------------------------------------------------ */

#if   ( setup_SENSOR_I2C == (setup_SENSOR&setup_MASK_OPT1) ) || DOXYGEN

	/* ------------------------------------------------------------ */
	/*				Include File Definitions						*/
	/* ------------------------------------------------------------ */

	#include "mpu925x_registers.h"
    #include "ms5611_register.h"
	#include "driverlib/i2c.h"
	#include <busy_delay.h>

	/* ------------------------------------------------------------ */
	/*				Local Defines									*/
	/* ------------------------------------------------------------ */

	#define MAGNET_INIT 						(1)
    #define MAGNET_READ_SENSITIVITY_VALUES      (1)
    #define MAGNET_SELTEST_ON                   (1)

    #define BARO_INIT                           (1)

	#define I2CMPU_ADRESS 						MPU_SLAVE_ADDR_1
	#define ENABLE_BUSY_WAITING_READ			(0)					// for measurement purpose

	//
	// define the desired peripheral setup (see peripheral_setup.h)
	//
	#if ( periph_SENSOR_INT == INT_I2C2 )
		#define SYSCTL_PERIPH_I2C			SYSCTL_PERIPH_I2C2
		#define I2C_PERIPH_BASE				I2C2_BASE
		#define SYSCTL_PERIPH_PORT			SYSCTL_PERIPH_GPIOE
		#define PORT_BASE					GPIO_PORTE_BASE
		#define I2C_SCL						GPIO_PE4_I2C2SCL
		#define SCL_PIN						GPIO_PIN_4
		#define I2C_SDA						GPIO_PE5_I2C2SDA
		#define SDA_PIN						GPIO_PIN_5
	#elif ( periph_SENSOR_INT == INT_I2C1 )
		#define SYSCTL_PERIPH_I2C			SYSCTL_PERIPH_I2C1
		#define I2C_PERIPH_BASE				I2C1_BASE
		#define SYSCTL_PERIPH_PORT			SYSCTL_PERIPH_GPIOA
		#define PORT_BASE					GPIO_PORTA_BASE
		#define I2C_SCL						GPIO_PA6_I2C1SCL
		#define SCL_PIN						GPIO_PIN_6
		#define I2C_SDA						GPIO_PA7_I2C1SDA
		#define SDA_PIN						GPIO_PIN_7
	#else
		#error	ERROR implement the defines above here
	#endif

	/* ------------------------------------------------------------ */
	/*				Local Type Definitions							*/
	/* ------------------------------------------------------------ */

	enum sensorReadState_e
	{
		READY,			// Sensor is not busy
		READ_ACCEL,		// read accel  and next state will be READ_GYRO
		READ_GYRO,		// read gyro   and next state will be READ_MAGNET
		READ_MAGNET,		// read magnet and next state will be READY or READ_BARO
		READ_BARO       // read baro next state will be READY
	};

	typedef struct rawData_s {
	   int16_t x_accel;
	   int16_t y_accel;
	   int16_t z_accel;
	   int16_t x_gyro;
	   int16_t y_gyro;
	   int16_t z_gyro;
	   int16_t x_magnet;
	   int16_t y_magnet;
	   int16_t z_magnet;
	   uint32_t p_baro;
	   uint32_t t_baro;
	} rawData_s ;

	/* ------------------------------------------------------------ */
	/*				Forward Declarations							*/
	/* ------------------------------------------------------------ */

	static void SensorCalibrate(void);
	static void I2cBurstRead(uint8_t ui8_i2cAdress, uint8_t registerAdress, uint8_t *data_array, uint8_t data_length);
    static void I2cBurstReadBlocking(enum sensorReadState_e e_startState,uint8_t ui8_i2cAdress, uint8_t registerAdress, uint8_t *data_array, uint8_t data_length);
	static void I2CReadFinishCallback(void *pvData, uint_fast8_t ui8_status);
	static void I2CWriteFinishCallback(void *pvData, uint_fast8_t ui8_status);
	static void i2cMpuWrite(uint8_t ui8_i2cAdress, uint8_t registerAdress, uint8_t data);

	// TODO test baro
	static void i2cBaroWrite(uint8_t ui8_i2cAdress, uint8_t command);
	static float getPressureData(void);

	/* ------------------------------------------------------------ */
	/*				Global Variables								*/
	/* ------------------------------------------------------------ */

	#if(periph_SENSOR_INT==periph_MOTOR_INT)
		extern tI2CMInstance i2cMastInst_s;				// I2C master instance
	#else
		static tI2CMInstance i2cMastInst_s;				// I2C master instance
	#endif

	/* ------------------------------------------------------------ */
	/*				Local Variables									*/
	/* ------------------------------------------------------------ */

	static uint8_t ui8_i2cBufferRead[12];				// message for I2C
	static uint8_t ui8_i2cBufferWrite[12];
	static uint8_t ui8_mpuSetUpFlag, ui8_magSetupFlag, ui8_baroSetupFlag;
	static enum sensorReadState_e e_sensorReadState=READY;

	// Counter variables for Barometer (MS5611) loop
	static uint8_t ui8_i2cPressCounter = 0;
	static uint8_t ui8_i2cTempCounter = 0;

	// variables for temp from barometer
	static uint32_t ui32_tempRotMem[5];
	static uint8_t ui8_rotMemCounter;
	static uint32_t ui32_tempSum;

    // variables for pressure from barometer
    static uint32_t ui32_pressRotMem[20];
    static uint8_t ui8_pressMemCounter;
    static uint32_t ui32_pressSum;
    static float f_pressureBase;

    //flag new pressure measurement is there
    static uint8_t ui8_newPressValue = 0;



	static rawData_s s_rawData;


	static uint8_t ui8_i2cError=0;
	static workload_handle_p p_workHandle;

	#if	ENABLE_BUSY_WAITING_READ
		static uint8_t ui8_readFlag=0;
	#endif

	/* ------------------------------------------------------------ */
	/*				Procedure Definitions							*/
	/* ------------------------------------------------------------ */

	/**
	 * \brief	ISR for I2C
	 *
	 * 			Give the interrupt handle to the I2C Master instance
	 */
	void Sensor_I2CIntHandler(void) // dieser wird für master und slave verwendet?
	{
		I2CMIntHandler(&i2cMastInst_s);
	}

	/**
	 * \brief	I2C Callback funkction
	 *
	 * 			when I2C read transactions have completed, this funktion is called
	 *			in the context of the I2C master interrupthandler
	 * \param 	pvData		not used
	 * \param	ui8_status	status to find error in I2C transmission
	 */
	static void I2CReadFinishCallback(void *pvData, uint_fast8_t ui8_status)
	{
		// is there  an error in I2C transmission?
		if(ui8_status != I2CM_STATUS_SUCCESS)
			ui8_i2cError=true;
		else
			ui8_i2cError=false;

		// check if all sensors are initialized
		if(ui8_mpuSetUpFlag == 1 && ui8_magSetupFlag == 1 && ui8_baroSetupFlag == 1)
		{
            switch(e_sensorReadState)
            {
                case READY:		// this should never happen!
                {
                    while(1);
                }
                case READ_ACCEL:
                {
                    s_rawData.x_accel = (ui8_i2cBufferRead[0]<<8) + ui8_i2cBufferRead[1];
                    s_rawData.y_accel = (ui8_i2cBufferRead[2]<<8) + ui8_i2cBufferRead[3];
                    s_rawData.z_accel = (ui8_i2cBufferRead[4]<<8) + ui8_i2cBufferRead[5];
                    e_sensorReadState=READ_GYRO;
                    I2cBurstRead(I2CMPU_ADRESS,GYRO_XOUT_H, ui8_i2cBufferRead, 6);
                    break;
                }
                case READ_GYRO:
                {
                    s_rawData.x_gyro = (ui8_i2cBufferRead[0]<<8) + ui8_i2cBufferRead[1];
                    s_rawData.y_gyro = (ui8_i2cBufferRead[2]<<8) + ui8_i2cBufferRead[3];
                    s_rawData.z_gyro = (ui8_i2cBufferRead[4]<<8) + ui8_i2cBufferRead[5];
                    e_sensorReadState=READ_MAGNET;
                    I2cBurstRead(MAGNET_ADRESS,MAGNET_HXL, ui8_i2cBufferRead, 7);
                    break;
                }
                case READ_MAGNET:
                {
                    s_rawData.x_magnet = (ui8_i2cBufferRead[1]<<8) + ui8_i2cBufferRead[0];
                    s_rawData.y_magnet = (ui8_i2cBufferRead[3]<<8) + ui8_i2cBufferRead[2];
                    s_rawData.z_magnet = (ui8_i2cBufferRead[5]<<8) + ui8_i2cBufferRead[4];

                    if(ui8_i2cPressCounter == 5){

                        ui8_i2cPressCounter = 0;

                        // TODO can be used without if
                        if(ui8_i2cTempCounter == 0)
                        {
                            // read temperature value in OSC = 4096 (24 bit)
                            e_sensorReadState=READ_BARO;
                            I2cBurstRead(BARO_ADDRESS, BARO_ADC_READ_COMMAND, ui8_i2cBufferRead, 3);

                        }
                        else
                        {
                            // read pressure value in OSC = 4096 (24 bit)
                            e_sensorReadState=READ_BARO;
                            I2cBurstRead(BARO_ADDRESS, BARO_ADC_READ_COMMAND, ui8_i2cBufferRead, 3);
                        }

                        // increase the temperature counter with each measurement
                        ui8_i2cTempCounter++;

                    }
                    else
                    {
                        e_sensorReadState=READY;

                        // increase pressure counter so that 10ms elapse between requiring and reading the values
                        ui8_i2cPressCounter++;

                        HIDE_Workload_EstimateStop(p_workHandle);

                        #if	ENABLE_BUSY_WAITING_READ
                            ui8_readFlag=0;
                        #endif
                        // fire eventBit to notify Sensor Read has finished
                        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                        if(pdFAIL==xEventGroupSetBitsFromISR(gx_receiver_eventGroup,receiver_SENSOR_DATA, &xHigherPriorityTaskWoken))
                            while(1);// you will come here, when configTIMER_QUEUE_LENGTH is full
                        else
                            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
                    }


                    break;

                }
                case READ_BARO:
                {
                    // save barometer values into variables
                    if(ui8_i2cTempCounter == 1)
                    {
                        // Read temp value in rotating memory and calculate average of the last 5 measurements to help against temperature spikes
                        ui32_tempSum -= ui32_tempRotMem[ui8_rotMemCounter];
                        ui32_tempRotMem[ui8_rotMemCounter] = ui8_i2cBufferRead[0] << 16 | ui8_i2cBufferRead[1] << 8 | ui8_i2cBufferRead[2];
                        ui32_tempSum += ui32_tempRotMem[ui8_rotMemCounter];
                        ui8_rotMemCounter++;

                        if(ui8_rotMemCounter == 5)
                        {
                            ui8_rotMemCounter = 0;
                        }

                        s_rawData.t_baro = ui32_tempSum / 5;
                    }
                    else
                    {
                        // Read pressure value in rotating memory and calculate average of the last 20 measurements to smoothen pressure readings
                        ui32_pressSum -= ui32_pressRotMem[ui8_pressMemCounter];
                        ui32_pressRotMem[ui8_pressMemCounter] = ui8_i2cBufferRead[0] << 16 | ui8_i2cBufferRead[1] << 8 | ui8_i2cBufferRead[2];
                        ui32_pressSum += ui32_pressRotMem[ui8_pressMemCounter];
                        ui8_pressMemCounter++;

                        if(ui8_pressMemCounter == 20)
                        {
                            ui8_pressMemCounter = 0;
                        }

                        s_rawData.p_baro = ui32_pressSum / 20;
                        ui8_newPressValue = 1;
                    }



                    if(ui8_i2cTempCounter == 20)
                    {
                       // set temp counter to zero and require temp value
                       ui8_i2cTempCounter = 0;
                       i2cBaroWrite(BARO_ADDRESS, BARO_CONVER_TEMP_4096);
                    }
                    else
                    {   // require pressure value
                       i2cBaroWrite(BARO_ADDRESS, BARO_CONVER_PRESS_4096);
                    }

                    // increase pressure counter so that 10ms elapse between requiring and reading the values
                    ui8_i2cPressCounter++;

                    e_sensorReadState=READY;

                    HIDE_Workload_EstimateStop(p_workHandle);

                    #if ENABLE_BUSY_WAITING_READ
                        ui8_readFlag=0;
                    #endif
                    // fire eventBit to notify Sensor Read has finished
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    if(pdFAIL==xEventGroupSetBitsFromISR(gx_receiver_eventGroup,receiver_SENSOR_DATA, &xHigherPriorityTaskWoken))
                        while(1);// you will come here, when configTIMER_QUEUE_LENGTH is full
                    else
                        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

                    break;

                }
            }
		}
		else
		{
		    // If you get here setup of sensors isn't finished or wasn't succesful
		}
	}

	/**
	 * \brief	I2C Callback funkction
	 *
	 * 			when I2C trnsactions have completed, this funktion is called
	 *			in the context of the I2C master interrupthandler
	 * \param 	pvData		not used
	 * \param	ui8_status	status to find error in I2C transmission
	 */
	static void I2CWriteFinishCallback(void *pvData, uint_fast8_t ui8_status)
	{
		// is there  an error in I2C transmission?
		if(ui8_status != I2CM_STATUS_SUCCESS)// sometimes this happens, during init in debug mode (unplug and replug usb so that the Sensor loses voltage)
			xEventGroupSetBits(gx_fault_EventGroup,fault_SENSOR);
		else
			xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);
		HIDE_Fault_Increment(fault_SENSOR,ui8_status != I2CM_STATUS_SUCCESS);
	}

	static void I2cBurstReadBlocking(enum sensorReadState_e e_startState,uint8_t ui8_i2cAdress, uint8_t registerAdress, uint8_t *data_array, uint8_t data_length)
	{
		e_sensorReadState=e_startState;
		#if	ENABLE_BUSY_WAITING_READ
			ui8_readFlag=1;
		#endif
		HIDE_Workload_EstimateStart(p_workHandle);
		I2cBurstRead(ui8_i2cAdress,registerAdress, data_array, data_length);
		#if	ENABLE_BUSY_WAITING_READ
			while(ui8_readFlag);
		#endif

		EventBits_t x_receiverEventBits;
	    //  Wait for sensor receiver event
		x_receiverEventBits = xEventGroupWaitBits(gx_receiver_eventGroup,
				receiver_SENSOR_DATA,
				pdTRUE,          // clear Bits before returning.
				pdTRUE,          // wait for all Bits
				SENSOR_READ_TIMEOUT_MS / portTICK_PERIOD_MS ); // maximum wait time
		HIDE_Receive_Increment(receiver_SENSOR_DATA);

		// unblock because of timeout, or i2cError?
		uint8_t ui8_error=(receiver_SENSOR_DATA & x_receiverEventBits == 0) || ui8_i2cError;
		if( ui8_error )
			xEventGroupSetBits(gx_fault_EventGroup,fault_SENSOR);
		else
			xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);
		HIDE_Fault_Increment(fault_SENSOR,ui8_error);
	}

	static void I2cBurstRead(uint8_t ui8_i2cAdress, uint8_t registerAdress, uint8_t *data_array, uint8_t data_length)
	{
		ui8_i2cBufferWrite[0] = registerAdress;
		if(I2CMRead(	&i2cMastInst_s, 					// pointer to i2c master instance
					ui8_i2cAdress,						// address of the I2C device to access
					ui8_i2cBufferWrite,							// pointer to the data buffer to be written.
					1,									// the number of bytes to be written
					data_array,							// pointer to the buffer to be filled with the read data
					data_length,						// the number of bytes to be read


					I2CReadFinishCallback, 				// function to be called when the transfer has completed
					0)==0)									// pointer that is passed to the callback function.
			while(1);
	}

	static void i2cMpuWrite(uint8_t ui8_i2cAdress, uint8_t registerAdress, uint8_t data)
	{
		vTaskDelay(1);
		while(I2CMasterBusy(I2C_PERIPH_BASE));

		ui8_i2cBufferWrite[0] = registerAdress;
		ui8_i2cBufferWrite[1] = data;
		I2CMWrite(	&i2cMastInst_s, 					// pointer to i2c master instance
					ui8_i2cAdress, 						// I2C adress
					ui8_i2cBufferWrite, 							// I2C message
					2, 									// message length
					I2CWriteFinishCallback, 			// callback funktion (when message was sent)
					0);									// callback data
		while(I2CMasterBusy(I2C_PERIPH_BASE));
	}


	static void i2cGetMpuData(float sensor_data[]){

		// Read All Sensor Data
		I2cBurstReadBlocking(READ_ACCEL,I2CMPU_ADRESS,ACCEL_XOUT_H, ui8_i2cBufferRead, 6);

		sensor_data[X_ACCEL]	= (float)s_rawData.x_accel;
		sensor_data[Y_ACCEL]	= (float)s_rawData.y_accel;
		sensor_data[Z_ACCEL]	= (float)s_rawData.z_accel;
		sensor_data[X_GYRO]		= (float)s_rawData.x_gyro;
		sensor_data[Y_GYRO]		= (float)s_rawData.y_gyro;
		sensor_data[Z_GYRO]		= (float)s_rawData.z_gyro;

		sensor_data[X_MAGNET]   = (float)s_rawData.x_magnet;
		sensor_data[Y_MAGNET]   = (float)s_rawData.y_magnet;
		sensor_data[Z_MAGNET]    = (float)s_rawData.z_magnet;

	}



	// TODO Only test functions to send data delete or change for final implementation
	static void i2cCopyMPUData(int16_t sensor_data[]){


        sensor_data[X_ACCEL]    = s_rawData.x_accel;
        sensor_data[Y_ACCEL]    = s_rawData.y_accel;
        sensor_data[Z_ACCEL]    = s_rawData.z_accel;
        sensor_data[X_GYRO]     = s_rawData.x_gyro;
        sensor_data[Y_GYRO]     = s_rawData.y_gyro;
        sensor_data[Z_GYRO]     = s_rawData.z_gyro;

        sensor_data[X_MAGNET]   = s_rawData.x_magnet;
        sensor_data[Y_MAGNET]   = s_rawData.y_magnet;
        sensor_data[Z_MAGNET]   = s_rawData.z_magnet;


	}
	// TODO Only test functions to send data delete or change for final implementation
	static void i2cCopyFloatMPUData(float* local_sensor_data){


	    gf_sensor_data[X_ACCEL]    = local_sensor_data[X_ACCEL];
	    gf_sensor_data[Y_ACCEL]    = local_sensor_data[Y_ACCEL];
	    gf_sensor_data[Z_ACCEL]    = local_sensor_data[Z_ACCEL];
	    gf_sensor_data[X_GYRO]     = local_sensor_data[X_GYRO];
	    gf_sensor_data[Y_GYRO]     = local_sensor_data[Y_GYRO];
	    gf_sensor_data[Z_GYRO]     = local_sensor_data[Z_GYRO];

	    gf_sensor_data[X_MAGNET]   = local_sensor_data[X_MAGNET];
	    gf_sensor_data[Y_MAGNET]   = local_sensor_data[Y_MAGNET];
	    gf_sensor_data[Z_MAGNET]   = local_sensor_data[Z_MAGNET];


	    }


	// TODO test baro
        static void i2cBaroWrite(uint8_t ui8_i2cAdress, uint8_t command){

            //vTaskDelay(1);
            //while(I2CMasterBusy(I2C_PERIPH_BASE));

            ui8_i2cBufferWrite[0] = command;
            I2CMWrite(  &i2cMastInst_s,                     // pointer to i2c master instance
                        ui8_i2cAdress,                      // I2C adress
                        ui8_i2cBufferWrite,                          // I2C message
                        1,                                  // message length
                        I2CWriteFinishCallback,             // callback funktion (when message was sent)
                        0);                                 // callback data
            //while(I2CMasterBusy(I2C_PERIPH_BASE));

        }



        // TODO test baro
        static float getPressureData(void){

            // help variables to calculate smoth pressure values (look at datasheet of MS5611)
            int64_t OFF, OFF_C2, SENS, SENS_C1;
            int32_t dT, TEMP, P;
            float f_pressure, f_pressureDiff;

            //Calculate pressure as explained in the datasheet of the MS-5611.
            dT = (int32_t) (s_rawData.t_baro - ui16_baro_calibration[4] * 256);
            //Calculate Temperature in °C not necessary for application
            //TEMP = (int32_t) (2000 + dT * ui16_baro_calibration[5] / 8388608);

            OFF_C2 = (int64_t) ui16_baro_calibration[1] * 65536;
            SENS_C1 =(int64_t) ui16_baro_calibration[0] * 32768;

            OFF = OFF_C2 + ((int64_t)dT * (int64_t)ui16_baro_calibration[3]) / 128;
            SENS = SENS_C1 + ((int64_t)dT * (int64_t)ui16_baro_calibration[2]) / 256;
            P = (int32_t)(((s_rawData.p_baro * SENS) / 2097152 - OFF) / 32768);

            f_pressure = (float) P;

            // Use a complementary filter to get a smoother pressure curve
            f_pressureBase = f_pressureBase * 0.985 + f_pressure * 0.015;

            f_pressureDiff = f_pressureBase - f_pressure;
            // To still guarantee fast behaviour if the difference is to big and secure for malfunctions
            f_pressureDiff = math_LIMIT(f_pressureDiff, -8.0, 8.0);
            if(f_pressureDiff > 1.0 || f_pressureDiff < -1.0)
            {
                f_pressureBase -= f_pressureDiff / 6.0;
            }
            f_pressure = f_pressureBase;

            return f_pressure;
     }


	/**
	 * \brief	Init the peripheral for the sensor driver
	 */
	void Sensor_InitPeriph(void)
	{
		BusyDelay_Init();
		ROM_IntPrioritySet(periph_SENSOR_INT,    priority_SENSOR_ISR);		// I2C Sensorbus (MPU9150,BAro)

		// Enable peripherial I2C
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C);
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PORT);

		// Set GPIOs as I2C pins.
		ROM_GPIOPinConfigure(I2C_SCL);
		ROM_GPIOPinConfigure(I2C_SDA);

		// open drain with pullups
		ROM_GPIOPinTypeI2C(PORT_BASE, SDA_PIN);
		ROM_GPIOPinTypeI2CSCL(PORT_BASE, SCL_PIN);

		// Set the Master Clock and choose fast mode (400 kbps)
		ROM_I2CMasterInitExpClk(I2C_PERIPH_BASE, ROM_SysCtlClockGet(), true);

		ROM_I2CMasterEnable(I2C_PERIPH_BASE);


		// i2c driver unit
		I2CMInit(	&i2cMastInst_s,
					I2C_PERIPH_BASE,
					periph_SENSOR_INT,
					0xff,
					0xff,
					ROM_SysCtlClockGet());

		// enable interrupts
		ROM_IntEnable(i2cMastInst_s.ui8Int);
		ROM_I2CMasterIntEnableEx(i2cMastInst_s.ui32Base, I2C_MASTER_INT_DATA);

		// set optional eventBits name
		HIDE_Fault_SetEventName(fault_SENSOR,"Sen");
		HIDE_Receive_SetEventName(receiver_SENSOR_DATA,"Sen");

        // workload estimation for I2C
        HIDE_Workload_EstimateCreate(&p_workHandle, "sI2C");
	}

	/**
	 * \brief	Init through the peripheral the sensor chip itself
	 */
	void Sensor_InitSensor(void)
	{
		vTaskDelay( 50 );
		// do a reset
		i2cMpuWrite(I2CMPU_ADRESS,PWR_MGMT_1, H_RESET);
		                                                // TODO why twice check and change
		i2cMpuWrite(I2CMPU_ADRESS,PWR_MGMT_1, H_RESET);
		// configure Sensor to use internal 20 MHz clock
		i2cMpuWrite(I2CMPU_ADRESS,PWR_MGMT_1, INT_20MHZ_CLOCK);
		// set sample rate divider to Internal_Sample_Rate / (1 + SMPLRT_DIV)
		i2cMpuWrite(I2CMPU_ADRESS,SMPLRT_DIV, 0x00);
		// configure accelerometer
		i2cMpuWrite(I2CMPU_ADRESS,ACCEL_CONFIG, ACCEL_FS_2g);
		// configure gyroscope
		i2cMpuWrite(I2CMPU_ADRESS,GYRO_CONFIG, GYRO_FS_500dps);
		// INT Pin / Bypass Enable Configuration
		i2cMpuWrite(I2CMPU_ADRESS,INT_PIN_CFG, BYPASS_EN);

		ui8_mpuSetUpFlag = 1;

		// Init Magnetometer
		#if (MAGNET_INIT == 1)

		    // TODO read sensitivity test
            #if (MAGNET_READ_SENSITIVITY_VALUES == 1)
			// Perform a soft reset
			i2cMpuWrite(MAGNET_ADRESS, MAGNET_CNTL2, MAGNET_SRST);

			// Read adjustment values in Fuse ROM access mode
			i2cMpuWrite(MAGNET_ADRESS, MAGNET_CNTL1, MAGNET_FUSE_ROM_MODE | MAGNET_16_BIT_OUTPUT);

			// Read out the sensitivity adjustment values
            uint8_t ASAX_val[1] = {0};
            uint8_t ASAY_val[1] = {0};
            uint8_t ASAZ_val[1] = {0};


            // read Magned Sensor Data
            I2cBurstReadBlocking(READ_MAGNET,MAGNET_ADRESS,MAGNET_ASAX, ASAX_val, 1);
            I2cBurstReadBlocking(READ_MAGNET,MAGNET_ADRESS,MAGNET_ASAY, ASAY_val, 1);
            I2cBurstReadBlocking(READ_MAGNET,MAGNET_ADRESS,MAGNET_ASAZ, ASAZ_val, 1);


            #endif

            // TODO self test test
            #if (MAGNET_SELTEST_ON == 1)

            // Perform a soft reset
            i2cMpuWrite(MAGNET_ADRESS, MAGNET_CNTL2, MAGNET_SRST);


			i2cMpuWrite(MAGNET_ADRESS,MAGNET_ASTC, MAGNET_SELFTEST_BIT_ON);

			// Magnet self test mode and 16 bit mode
			i2cMpuWrite(MAGNET_ADRESS,MAGNET_CNTL1, MAGNET_SEFL_TEST_MODE | MAGNET_16_BIT_OUTPUT);

			uint8_t ui8_selftest_val[7];

			I2cBurstReadBlocking(READ_MAGNET,MAGNET_ADRESS,MAGNET_HXL, ui8_selftest_val, 7);

			s_rawData.x_magnet = (ui8_selftest_val[1]<<8) + ui8_selftest_val[0];
            s_rawData.y_magnet = (ui8_selftest_val[3]<<8) + ui8_selftest_val[2];
            s_rawData.z_magnet = (ui8_selftest_val[5]<<8) + ui8_selftest_val[4];

            e_sensorReadState=READY;

            i2cMpuWrite(MAGNET_ADRESS,MAGNET_ASTC, MAGNET_SELFTEST_BIT_OFF);


            #endif

        // Perform a soft reset
        i2cMpuWrite(MAGNET_ADRESS, MAGNET_CNTL2, MAGNET_SRST);

        // Continous mode and 16 bit mode
        i2cMpuWrite(MAGNET_ADRESS,MAGNET_CNTL1, MAGNET_CONTINUOUS_MODE2 | MAGNET_16_BIT_OUTPUT);

        ui8_magSetupFlag = 1;

		#endif

        // TODO test baro
        #if (BARO_INIT == 1)

            // perform reset of Barometer
            i2cBaroWrite(BARO_ADDRESS, BARO_RESET);

            // Delay after Reset before reading prom
            BusyDelay_Ms(15);

            // read calibration values from PROM 0xA0 -> 0xAE
            uint8_t i;
            uint8_t j = 0;

            for(i = 1; i <=6; i++)
            {
                I2cBurstReadBlocking(READ_BARO, BARO_ADDRESS, BARO_PROM_READ + i*2, ui8_i2cBufferRead, 2);

                ui8_i2cBufferRead[j] = ui8_i2cBufferRead[0];

                ui16_baro_calibration[i-1] = ui8_i2cBufferRead[j];
                ui16_baro_calibration[i-1] <<= 8;
                j++;

                ui8_i2cBufferRead[j] = ui8_i2cBufferRead[1];
                ui16_baro_calibration[i-1] |= ui8_i2cBufferRead[j];
                j++;

                e_sensorReadState=READY;

            }

        ui8_baroSetupFlag = 1;
        #endif

		// reset Sensor fault eventBit
		xEventGroupClearBits(gx_fault_EventGroup,fault_SENSOR);
	}

	/**
	 * \brief	collect data to calibrate the sensor
	 */
	static void SensorCalibrate(void)
	{
		// Read out sensor data and perform formatting
		float sensor_data[9];

		// Get MPU data and also barometer raw values and convert and collect correction data it
		i2cGetMpuData(sensor_data);
		IMUAxis2QCAxis(sensor_data);
		convertIMUData(sensor_data);
        correctIMUOffset(sensor_data, 1);

        // Get attitude quaternion via sensor fusion from MPU data
		MadgwickAHRSupdate(sensor_data[X_ACCEL], sensor_data[Y_ACCEL], sensor_data[Z_ACCEL],
		                   sensor_data[X_GYRO], sensor_data[Y_GYRO], sensor_data[Z_GYRO],
		                  sensor_data[X_MAGNET], sensor_data[Y_MAGNET], sensor_data[Z_MAGNET]);
		// Get Euler/Tait-Bryan angles from quaternion
		Math_QuatToEuler(q, gf_sensor_fusedAngles);

		// For calibration the barometer has to be read a few times so that the right temperature is measured (nothing special just reading)
		if(ui8_newPressValue == 1)
		{
		    ui8_newPressValue = 0;
		    getPressureData();

		}

        new_quaternion[0] = q[0];
        new_quaternion[1] = q[1];
        new_quaternion[2] = q[2];
        new_quaternion[3] = q[3];






	}

	/**
	 * \brief	read the sonsor and prepare data (sensor fusion).
	 *
	 *			fire eventBit fault_SENSOR if there is a fault,
	 *			else clear the bit.
	 * \note    Events: EventBit fault_SENSOR will be set or cleared in gx_fault_EventGroup
	 */
	void Sensor_ReadAndFusion(void)
	{
		// Read out sensor data and perform formatting
		float sensor_data[9];
		// TODO test baro


		// Get MPU data and convert and correct it
		i2cGetMpuData(sensor_data);
		IMUAxis2QCAxis(sensor_data);
		convertIMUData(sensor_data);
		correctIMUOffset(sensor_data, 0);


        // Get attitude quaternion via sensor fusion from MPU data
        MadgwickAHRSupdate(sensor_data[X_ACCEL], sensor_data[Y_ACCEL], sensor_data[Z_ACCEL],
                           sensor_data[X_GYRO],  sensor_data[Y_GYRO], sensor_data[Z_GYRO],
                           sensor_data[X_MAGNET], sensor_data[Y_MAGNET], sensor_data[Z_MAGNET]);
        // Get Euler/Tait-Bryan angles from quaternion
        Math_QuatToEuler(q, gf_sensor_fusedAngles);

        // calculate correct pressure for altitude control only every 10ms (5 loops) because conversion takes around >9ms
        if(ui8_newPressValue == 1)
        {
            ui8_newPressValue = 0;
            gf_sensor_pressure = getPressureData();

        }

        new_quaternion[0] = q[0];
        new_quaternion[1] = q[1];
        new_quaternion[2] = q[2];
        new_quaternion[3] = q[3];


	}


#elif ( setup_SENSOR_NONE == (setup_SENSOR&setup_MASK_OPT1) )

	void Sensor_InitPeriph(void){}
	void Sensor_InitSensor(void){}
	void SensorCalibrate(void){}
	void Sensor_ReadAndFusion(void){}


#elif ( setup_SENSOR_SPI == (setup_SENSOR&setup_MASK_OPT1) )

	#error ERROR: not portet yet, define a nother setup_SENSOR (in qc_setup.h) or port it from Eckstein
#else

	#error ERROR: define setup_SENSOR (in qc_setup.h)

#endif

/**
 * \brief	if calibration is required, start calibration.
 *
 *			calibrate for a minimum time of x ms.
 *			(x is defined in the driver)
 * \param	elapseTimeMS	elapsed time between last call
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
 * \brief	stopts the requirement of the sensor to calibrate
 */
void Sensor_CalibrateStop(void)
{
	i32_stateTime=CALIBRATE_STOP;
    // TODO delete HIDE_Debug_InterfaceSend("C finished", strlen("C finished")+1 );

}

/**
 * \brief	requires the sonsor to start calibration
 *
 *			(this does not calibrate, only requires
 *			use Sensor_Calibrate to calibrate)
 */
void Sensor_CalibrateRequire(void)
{
	if(!IS_CALIBRAE_REQUIRED(i32_stateTime))
		i32_stateTime=CALIBRATE_START;
}

/**
 * \brief	get the state of the sensor calibration
 * \return	true if calibration is not running or ( calibration is running but long enough )
 *			false else
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
 * \brief	return true if calibration is required
 * \return	true  if calibration is required,
 *			false else
 */
uint8_t Sensor_IsCalibrateRequired(void)
{

	return IS_CALIBRAE_REQUIRED(i32_stateTime);

}




#if (setup_DEV_DEBUG_USB)
    /**
     * \brief   send data of Sensor over USB to PC application
     */
    void HIDE_Sensor_SendDataOverUSB(void)
    {

        // TODO program default USB data send if possible choosable in qc_setup.h and helper function for size of array
        //float sensor_angles[3];

        //sensor_angles[0] = math_RAD2DEC(gf_sensor_fusedAngles[0]);
        //sensor_angles[1] = math_RAD2DEC(gf_sensor_fusedAngles[1]);
        //sensor_angles[2] = math_RAD2DEC(gf_sensor_fusedAngles[1]);
        //HIDE_Debug_USB_InterfaceSend(sensor_angles, sizeof(sensor_angles)/ sizeof(sensor_angles[0]), debug_FLOAT);

        gf_sensor_data[0] = gf_sensor_pressure;

        //HIDE_Debug_USB_InterfaceSend(gi16_sensor_data, sizeof(gi16_sensor_data)/ sizeof(gi16_sensor_data[0]), debug_INT16);
        HIDE_Debug_USB_InterfaceSend(gf_sensor_data, sizeof(gf_sensor_data)/ sizeof(gf_sensor_data[0]), debug_FLOAT);

        //HIDE_Debug_USB_InterfaceSend(new_quaternion, sizeof(new_quaternion)/sizeof(new_quaternion[0]), debug_FLOAT);

    }
#endif
