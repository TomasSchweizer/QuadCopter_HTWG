//=====================================================================================================
// @file motor_driver.c
//=====================================================================================================
//
// @brief API to interact with the motors.
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
#include <stdbool.h>

#include "motor_driver.h"

#include "peripheral_setup.h"

// drivers
#include "workload.h"
#include "debug_interface.h"
#include "display_driver.h"

#include "fault.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"


/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/
#define MOTOR_WRITE_TIMEOUT_MS              ( 0.5 )

#define event_MOTOR_WRITTEN                 ( 1 << 0 )


#define MOTOR_MAPPING			            {0,1,2,3}	// Order of the motors

enum motorStates_e{

    MOTOR0_WRITE,
    MOTOR1_WRITE,
    MOTOR2_WRITE,
    MOTOR3_WRITE,
    MOTOR_READY,
};


/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/
static uint8_t standardizeMotorDrawDisplay(uint16_t ui16);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/
extern volatile EventGroupHandle_t gx_fault_EventGroup;
extern volatile countEdges_handle_p gp_fault_coundEdges;

/**
 * \brief	Array to store information of all motors
 * \note	Write access:	flight_task
 */
volatile motor_Data_s gs_motor[motor_COUNT];


/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/
volatile EventGroupHandle_t gx_motor_EventGroup;

static workload_handle_p p_workHandle;
/** \brief	Order of the motors */
static uint8_t	ui8_motorMap[motor_COUNT] = MOTOR_MAPPING;
// USB variable
static uint16_t ui16_motorDataUsb[4];

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

/*
 * \ brief get the motor speed and map it between 0 -> 99 to display on display
 */
static uint8_t standardizeMotorDrawDisplay(uint16_t ui16)
{
	uint8_t result = (uint8_t)((((uint32_t)ui16)*100)/0xFFFF);
	if(result>=100)
		return 99;
	else
		return result;
}

/**
 * \brief	Draw info about Motor on the Display
 */
void Motor_DrawDisplay(void)
{
	//( M0 )		( M2 )
	//   -	   ^^   -
    //     -      -
	//   -          -
	//( M1 )		( M3 )

	const uint8_t xOffset 		= 107;
	const uint8_t yOffset 		= 52;
	const uint8_t dx 			= 11;
	const uint8_t dy 			= 5;

	u8g_SetFont(&gs_display, u8g_font_04b_03r);		// u8g_font_unifont
	u8g_DrawStr(&gs_display,xOffset + 0 ,yOffset + 0 ,u8g_u8toa(standardizeMotorDrawDisplay(gs_motor[ui8_motorMap[3]].ui16_setPoint),2));
	u8g_DrawStr(&gs_display,xOffset + 0 ,yOffset + dy,u8g_u8toa(standardizeMotorDrawDisplay(gs_motor[ui8_motorMap[2]].ui16_setPoint),2));
	u8g_DrawStr(&gs_display,xOffset + dx,yOffset + 0 ,u8g_u8toa(standardizeMotorDrawDisplay(gs_motor[ui8_motorMap[1]].ui16_setPoint),2));
	u8g_DrawStr(&gs_display,xOffset + dx,yOffset + dy,u8g_u8toa(standardizeMotorDrawDisplay(gs_motor[ui8_motorMap[0]].ui16_setPoint),2));
}

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Select Mode                                                   */
/* ---------------------------------------------------------------------------------------------------*/

#if ( setup_MOTOR_I2C == (setup_MOTOR&setup_MASK_OPT1) ) || DOXYGEN

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

    #include "sensorlib/i2cm_drv.h"

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local defines i2c Mode                                     */
    /* -----------------------------------------------------------------------------------------------*/

    //  I2C
    #define ESC_BASE_ADR            (0x29)      // simonk i2c base adr
    #define ESC_DATA_SIZE			(2047)		// esc daten  1 byte=255  2 byte=2047

	// define the desired peripheral setup (see peripheral_setup.h)
	#if ( periph_MOTOR_INT == INT_I2C1 )
		#define MOTOR_I2C_PERIPH		SYSCTL_PERIPH_I2C1
		#define MOTOR_I2C_PERIPH_BASE	I2C1_BASE
		#define MOTOR_I2C_INT_VEC 		INT_I2C1
		#define MOTOR_I2C_PORT			SYSCTL_PERIPH_GPIOA
		#define MOTOR_I2C_PORT_BASE		GPIO_PORTA_BASE
		#define MOTOR_I2C_SCL			GPIO_PA6_I2C1SCL
		#define MOTOR_I2C_SCL_PIN		GPIO_PIN_6
		#define MOTOR_I2C_SDA			GPIO_PA7_I2C1SDA
		#define MOTOR_I2C_SDA_PIN		GPIO_PIN_7
	#elif ( periph_MOTOR_INT == INT_I2C0 )
		#define MOTOR_I2C_PERIPH		SYSCTL_PERIPH_I2C0
		#define MOTOR_I2C_PERIPH_BASE	I2C0_BASE
		#define MOTOR_I2C_INT_VEC 		INT_I2C0
		#define MOTOR_I2C_PORT			SYSCTL_PERIPH_GPIOB
		#define MOTOR_I2C_PORT_BASE		GPIO_PORTB_BASE
		#define MOTOR_I2C_SCL			GPIO_PB2_I2C0SCL
		#define MOTOR_I2C_SCL_PIN		GPIO_PIN_2
		#define MOTOR_I2C_SDA			GPIO_PB3_I2C0SDA
		#define MOTOR_I2C_SDA_PIN		GPIO_PIN_3
	#else
		#error	ERROR implement the defines above here
	#endif

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local Type Definitions i2c Mode                            */
    /* -----------------------------------------------------------------------------------------------*/

    /* ------------------------------------------------------------------------------------------------*/
    /*                                      Forward Declarations i2c Mode                              */
    /* ------------------------------------------------------------------------------------------------*/

	void Motor_I2CIntHandler(void);  // ausprobieren, sich selber in Tabelle eintragen (macht das wirklich sinn)
	static void Motor_i2cCallback(void *pvData, uint_fast8_t ui8Status);
	static void Motor_i2cWrite(uint8_t ui8_motorNr);

    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Global Variables i2c Mode                                     */
    /* ---------------------------------------------------------------------------------------------------*/
    #if(periph_SENSOR_INT==periph_MOTOR_INT)
               tI2CMInstance i2cMastInst_s;             // I2C master instance
    #else
        static tI2CMInstance i2cMastInst_s;             // I2C master instance
    #endif

    /* ---------------------------------------------------------------------------------------------------*/
    /*                                      Local Variables i2c Mode                                      */
    /* ---------------------------------------------------------------------------------------------------*/

    static uint8_t ui8_i2cWriteBuffer[8];                    // message for I2C
    static uint8_t ui8_i2cMotorState = MOTOR_READY;                  // to store which Motor to write & for I2C_MODE_... defines
    static uint8_t ui8_i2cErrorFlag = 0;
    /* -----------------------------------------------------------------------------------------------*/
    /*                                      Procedure Definitions i2c mode                            */
    /* -----------------------------------------------------------------------------------------------*/

	/**
	 * \brief	Init the peripheral for the motor driver
	 */
	void Motor_InitPeriph(void)
	{
		ROM_IntPrioritySet(periph_MOTOR_INT, priority_MOTOR_ISR);

		// enable I2C peripheral
		ROM_SysCtlPeripheralEnable(MOTOR_I2C_PERIPH);

		// enable GPIO Port peripheral
		ROM_SysCtlPeripheralEnable(MOTOR_I2C_PORT);

		// multiplex I2C to pins
		ROM_GPIOPinConfigure(MOTOR_I2C_SCL);
		ROM_GPIOPinConfigure(MOTOR_I2C_SDA);

		// open drain with pullups
		GPIOPinTypeI2CSCL(MOTOR_I2C_PORT_BASE, MOTOR_I2C_SCL_PIN);
		ROM_GPIOPinTypeI2C(MOTOR_I2C_PORT_BASE, MOTOR_I2C_SDA_PIN);

		// i2c driver unit
		i2cMastInst_s.ui32Base = MOTOR_I2C_PERIPH_BASE;		// controller i2c modul adresse
		i2cMastInst_s.ui8Int = periph_MOTOR_INT;		// int vector
		i2cMastInst_s.ui8TxDMA = 0xff;			// no DMA
		i2cMastInst_s.ui8RxDMA = 0xff;			// no DMA
		i2cMastInst_s.ui8State = 0;				// beginning state idle
		i2cMastInst_s.ui8ReadPtr = 0;
		i2cMastInst_s.ui8WritePtr = 0;

        gx_motor_EventGroup = xEventGroupCreate();
        // Was the event group created successfully?
        if( gx_motor_EventGroup == 0 )
            while(1);

		// reset Motor motor fault eventBit
        xEventGroupClearBits(gx_fault_EventGroup,fault_MOTOR);
        xEventGroupClearBits(gx_motor_EventGroup, event_MOTOR_WRITTEN);

		// set optional eventBit name
		HIDE_Fault_SetEventName(fault_MOTOR,"Mot");

        // workload estimation for I2C
        HIDE_Workload_EstimateCreate(&p_workHandle, "mI2C");
	}

	/**
	 * \brief	Init through the peripheral all motors and stop them
	 */
	void Motor_InitMotor(void)
	{

        uint8_t i;
        for(i=0; i < motor_COUNT; i++)
        {
        	gs_motor[i].ui16_setPoint   = 0;
        	gs_motor[i].f_current    = 0.0;
        	gs_motor[i].ui8_state       = 0;
        	gs_motor[i].f_temperature = 0.0;
        	gs_motor[i].f_rpm         = 0.0;
        	gs_motor[i].f_voltage     = 0.0;
        }

		Motor_OutputAll();

	}

	/**
	 * \brief	output the set point for all motors (non blocking)
	 *
	 *			(set point has to be previously stored in gs_motor)
	 *			fire eventBit fault_MOTOR if there is a fault,
	 *			else clear the bit.
	 * \note    Events: EventBit fault_MOTOR will be set or cleared in gx_fault_EventGroup
	 */
	void Motor_OutputAll(void)
	{
	    if (ui8_i2cMotorState == MOTOR_READY)
        {
            HIDE_Workload_EstimateStart(p_workHandle);

            // TODO change back to MOTOR0_WRITE
            ui8_i2cMotorState = MOTOR0_WRITE;
            Motor_i2cWrite(MOTOR0_WRITE);   // write to the first motor
        }
	    else
	    {
	        // TODO uncomment after tests
	        //while(1); // Mistake
	    }

	    volatile EventBits_t x_motorEventBits;
        //  Wait for sensor received event
	    //xEventGroupClearBits(gx_motor_EventGroup, event_MOTOR_WRITTEN);
        x_motorEventBits = xEventGroupWaitBits(gx_motor_EventGroup,
                                              event_MOTOR_WRITTEN,
                                              pdTRUE,          // clear Bits before returning.
                                              pdTRUE,          // wait for all Bits
                                              MOTOR_WRITE_TIMEOUT_MS / portTICK_PERIOD_MS ); // maximum wait time

        // unblock because of timeout
        uint8_t ui8_error=(event_MOTOR_WRITTEN & x_motorEventBits == 0) || ui8_i2cErrorFlag;
        if( ui8_error )
        {
           // if any motor has a fault, set EventBit for motor fault
           xEventGroupSetBits(gx_fault_EventGroup,fault_MOTOR);
        }
        else
        {
           xEventGroupClearBits(gx_fault_EventGroup,fault_MOTOR);
        }
        HIDE_Fault_Increment(fault_MOTOR,ui8_error);

	}


	/**
	 * \brief	output for all motors to stop them (non blocking)
	 *
	 *			fire eventBit fault_MOTOR if there is a fault,
	 *			else clear the bit.
	 * \note    Events: EventBit fault_MOTOR will be set or cleared in gx_fault_EventGroup
	 */
	void Motor_StopAll(void)
	{
        uint8_t i;
        for(i=0; i < motor_COUNT; i++)
        	gs_motor[i].ui16_setPoint   = 0;
        Motor_OutputAll();
	}


	static void Motor_i2cCallback(void *pvData, uint_fast8_t ui8_status)
	{
		// is there  an error in I2C transmission?
		if(ui8_status != I2CM_STATUS_SUCCESS)
		{

		    //ui8_i2cMotorState = MOTOR_READY;
		    //while(1);
		    ui8_i2cErrorFlag = 1;
		}
		else
		{
		    ui8_i2cErrorFlag = 0;
		}

		switch(ui8_i2cMotorState)
		{
		    // TODO change back just for balancing motors and pid tuning always after motor back to ready state

            case MOTOR0_WRITE:
            {
                ui8_i2cMotorState = MOTOR1_WRITE;
                Motor_i2cWrite(MOTOR1_WRITE);
                break;
            }
            case MOTOR1_WRITE:
            {
                ui8_i2cMotorState = MOTOR2_WRITE;
                Motor_i2cWrite(ui8_motorMap[ui8_i2cMotorState]);
                break;
            }
            case MOTOR2_WRITE:
            {
               ui8_i2cMotorState = MOTOR3_WRITE;
               Motor_i2cWrite(ui8_motorMap[ui8_i2cMotorState]);
               break;
            }
            case MOTOR3_WRITE:
            {
                ui8_i2cMotorState = MOTOR_READY;
                HIDE_Workload_EstimateStop(p_workHandle);


                // fire eventBit to notify motor write has finished
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                if(pdFAIL==xEventGroupSetBitsFromISR(gx_motor_EventGroup, event_MOTOR_WRITTEN, &xHigherPriorityTaskWoken))
                    while(1);// you will come here, when configTIMER_QUEUE_LENGTH is full
                else
                    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

                break;


            }

		}

	}


	/**
	 * \brief	ISR for I2C
	 *
	 * 			Give the interrupt handle to the I2C Master instance
	 */
	void Motor_I2CIntHandler(void)
	{
		I2CMIntHandler(&i2cMastInst_s);
	}

	/**
	 * \brief	write the set point for the desired motor over I2C
	 * \param	ui8_motorNr		The number of the Motor (0,1,2,...)
	 */
	static void Motor_i2cWrite(uint8_t ui8_motorNr)
	{
		uint32_t ui32_speed = (uint32_t) gs_motor[ui8_motorNr].ui16_setPoint;	// speed in größere variiablen laden

		// TODO just for usb debugging
		//ui16_motorDataUsb[ui8_motorNr] = ui32_speed;

		// Wertebereich von 1000..2000us mappen auf
		// bei mode:2047   -> 0..2047
		// bei mode:255    -> 0..255
//		ui32_speed = ui32_speed - 1000;  // = 0...1000
//		ui32_speed = (ui32_speed * ESC_DATA_SIZE) / 1000;  //0..2047 oder 0..255


		ui32_speed = (ui32_speed * ESC_DATA_SIZE) / 0xFFFF;  //0..2047 oder 0..255

		// TODO delete later send motor data over USB
		//ui16_motorDataUsb[ui8_motorNr] = (uint16_t) ui32_speed;

		if(ESC_DATA_SIZE == 255)
		{
			// brushlesregler version 1
			ui8_i2cWriteBuffer[0] = (uint8_t)ui32_speed;
			I2CMWrite(	&i2cMastInst_s, 					// pointer to i2c master instance
						ESC_BASE_ADR + ui8_motorNr, 		// I2C adress
						ui8_i2cWriteBuffer, 							// I2C message
						1, 									// message length
						Motor_i2cCallback, 		// callback funktion (when message was send)
						0);									// callback data
		}
		else
		{
			// brushless version >= 2.0
			// Speed in 2 bytes (11bits) umwandeln
		    ui8_i2cWriteBuffer[0] = (uint8_t)(ui32_speed >> 3) & 0xff; // gets the high byte [10->3]
		    ui8_i2cWriteBuffer[1] = ((uint8_t)ui32_speed % 8) & 0x07;  // gets the low 3 bits [3->0]

			I2CMWrite(	&i2cMastInst_s, 					// pointer to i2c master instance
						ESC_BASE_ADR + ui8_motorNr, 		// I2C adress
						ui8_i2cWriteBuffer, 							// I2C message
						2,  								// message length
						Motor_i2cCallback, 		// callback funktion (when message was send)
						0);                             // callback data



		}
	}


	void HIDE_Motor_SendDataOverUSB(void)
	{
	    // Send Motor Data over USB
        HIDE_Debug_USB_InterfaceSend(ui16_motorDataUsb, sizeof(ui16_motorDataUsb)/sizeof(ui16_motorDataUsb[0]), debug_INT16);

	}



#elif ( setup_MOTOR_PWM == (setup_MOTOR&setup_MASK_OPT1) )

#elif ( setup_MOTOR_NONE == (setup_MOTOR&setup_MASK_OPT1) )

	void Motor_InitPeriph(void){}
	void Motor_InitMotor(void){}
	void Motor_OutputAll(void){}
	void Motor_StopAll(void)
	{
        uint8_t i;
        for(i=0; i < motor_COUNT; i++)
        	gs_motor[i].ui16_setPoint   = 0;
	}
	void Motor_Read(uint8_t ui8_motorNr){}
	void Motor_ReadAll(void){}

#else
	#error ERROR: define setup_MOTOR (in qc_setup.h)
#endif
