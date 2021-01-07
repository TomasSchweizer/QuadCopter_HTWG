/**
 * 		@file 	motor_driver.c
 * 		@brief	Funktions to init, output and read the motor.
 *//*	@author Tobias Grimm
 * 		@date 	30.05.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>

// TODO test math.h
#include <math.h>

//  Hardware Specific
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "peripheral_setup.h"

// drivers
#include "workload.h"
#include "debug_interface.h"
#include "display_driver.h"
#include "motor_driver.h"
#include "fault.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"
#include "motor_driver.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

#define MOTOR_OVER_CURRENT		0xFF		// 16Bit
#define MOTOR_MAPPING			{0,1,2,3}	// Order of the motors

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void Motor_InitPeriph(void);
void Motor_OutputAll(void);
void Motor_StopAll(void);
void Motor_Read(uint8_t ui8_motorNr);
void Motor_ReadAll(void);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/**
 * \brief	Array to store information of all motors
 * \note	Write access:	flight_task
 */
volatile motor_Data_s gs_motor[motor_COUNT];

/**
 * \brief	fault information for all Motors
 *
 * 			every motor has one bit. It will be set, if there
 *			is a fault, and cleared when the fault is gone.
 *			(see MOTOR_MAPPING for bit-Order)
 * \note	Write access:	write-ISR of the motor driver
 */
volatile uint32_t 	  gui32_motor_fault=0;

/**
 * \brief	over current information for all Motors
 *
 * 			every motor has one bit. It will be set, if there
 *			is a over current, and cleared when the over current is gone.
 *			(see MOTOR_MAPPING for bit-Order)
 * \note	Write access:	read-ISR of the motor driver (### not implemented yet)
 */
 volatile uint32_t 	  gui32_motor_overCurrent=0;

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/** \brief	Order of the motors */
static uint8_t	ui8_motorMap[motor_COUNT] = MOTOR_MAPPING;

// TODO delete later test variable to send motor data over usb
uint16_t ui16_motorDataUsb[4];
/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

static uint8_t Standardize(uint16_t ui16)
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
	u8g_DrawStr(&gs_display,xOffset + 0 ,yOffset + 0 ,u8g_u8toa(Standardize(gs_motor[ui8_motorMap[3]].ui16_setPoint),2));
	u8g_DrawStr(&gs_display,xOffset + 0 ,yOffset + dy,u8g_u8toa(Standardize(gs_motor[ui8_motorMap[2]].ui16_setPoint),2));
	u8g_DrawStr(&gs_display,xOffset + dx,yOffset + 0 ,u8g_u8toa(Standardize(gs_motor[ui8_motorMap[1]].ui16_setPoint),2));
	u8g_DrawStr(&gs_display,xOffset + dx,yOffset + dy,u8g_u8toa(Standardize(gs_motor[ui8_motorMap[0]].ui16_setPoint),2));
}

/* ------------------------------------------------------------ */
/*				Select the Mode									*/
/* ------------------------------------------------------------ */

#if   ( setup_MOTOR_I2C == (setup_MOTOR&setup_MASK_OPT1) ) || DOXYGEN

	/* ------------------------------------------------------------ */
	/*				Include File Definitions						*/
	/* ------------------------------------------------------------ */

	//  Hardware Specific
	#include "driverlib/i2c.h"
	#include "sensorlib/i2cm_drv.h"

	/* ------------------------------------------------------------ */
	/*				Local Defines									*/
	/* ------------------------------------------------------------ */

	//  I2C
	#define I2C_INCREMENTAL_MODE	(1)			// 1 -> start next I2C command in I2C ISR, 0 -> busy waiting between I2C commands
	#define ESC_DATA_SIZE			(2047)		// esc daten  1 byte=255  2 byte=2047
	#define ESC_I2C_SPEED			(1)			// 0 = 100kbit  1=400kbit
	#define ESC_BASE_ADR			(0x29)		// simonk i2c base adr

	/*  I2C Adress (Ist gerade noch nicht so!)
	 * 	Write-Adress = I2C_BASE_ADR + MOTOR_NR * 2
	 * 	Read -Adress = I2C_BASE_ADR + MOTOR_NR * 2 + 1
	 * 	To init the ESCs sent 0 as set-point */

	#define I2C_MODE_READY			(-1)
	#define I2C_MODE_BUSY			(0)

	//  I2C read: byte Order
	#define I2C_READ_CURRENT		(1 << 0)
	#define I2C_READ_STATE			(1 << 1)
	#define I2C_READ_TEMP			(1 << 2)
	#define I2C_READ_RPM			(1 << 3)
	#define I2C_READ_VOLT			(1 << 4)
	//  I2C read: pack & unpack messages
	#define SET_POINT2HIGH_BYTE(ui16_setPoint)	  ( (uint8_t)(ui16_setPoint >> 3) & 0xff )    				// first byte
	#define SET_POINT2LOW_BYTE(ui16_setPoint)     ( ((uint8_t)(ui32_speed & 0x07 )  		  				// second byte
	#define BYTES2SET_POINT(ui8_highB,ui8_lowB)   ( ((uint16_t)((ui8_highB << 3) | (ui8_lowB & 0x07)  )  	// low byte
	#define LOW_BYTE_GET_MODE(ui8_lowB) 		  ( ui8_lowB & 0x80  )  									//  If highest bit = 0 -> normal mode, else esc-confic-mode

	//
	// define the desired peripheral setup (see peripheral_setup.h)
	//
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
	/* ------------------------------------------------------------ */
	/*				Local Type Definitions							*/
	/* ------------------------------------------------------------ */

	#if(periph_SENSOR_INT==periph_MOTOR_INT)
			   tI2CMInstance i2cMastInst_s;				// I2C master instance
	#else
		static tI2CMInstance i2cMastInst_s;				// I2C master instance
	#endif

	static uint8_t ui8_msg[8];						// message for I2C

	static volatile int8_t i8_i2cWriteNum;					// to store which Motor to write & for I2C_MODE_... defines
	static volatile int8_t i8_i2cReadNum;			// to store which Motor to read & for I2C_MODE_... defines
	static workload_handle_p p_workHandle;
	/* ------------------------------------------------------------ */
	/*				Forward Declarations							*/
	/* ------------------------------------------------------------ */

	void Motor_I2CIntHandler(void);  // ausprobieren, sich selber in Tabelle eintragen (macht das wirklich sinn)
	static void I2CWriteFinishCallback(void *pvData, uint_fast8_t ui8Status);
	static void I2CReadFinishCallback(void *pvData, uint_fast8_t ui8Status);
	static void I2CWriteSpeed(uint8_t ui8_motorNr);


	/* ------------------------------------------------------------ */
	/*				Procedure Definitions							*/
	/* ------------------------------------------------------------ */

	/**
	 * \brief	Init the peripheral for the motor driver
	 */
	void Motor_InitPeriph(void)
	{
		ROM_IntPrioritySet(periph_MOTOR_INT,    priority_MOTOR_ISR);

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

		// init i2c master module
		// und den i2c speed
		ROM_I2CMasterInitExpClk(i2cMastInst_s.ui32Base, ROM_SysCtlClockGet(), ESC_I2C_SPEED);

		// enable interrupts
		ROM_IntEnable(i2cMastInst_s.ui8Int);
		ROM_I2CMasterIntEnableEx(i2cMastInst_s.ui32Base, I2C_MASTER_INT_DATA);

        // reset Motor Overcurrent eventBit
        xEventGroupClearBits(gx_fault_EventGroup,fault_MOTOR_OVER_CURRENT);

		// set optional eventBit name
		HIDE_Fault_SetEventName(fault_MOTOR,"Mot");
		HIDE_Fault_SetEventName(fault_MOTOR_OVER_CURRENT,"MOC");

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
		i8_i2cWriteNum = I2C_MODE_READY;
		//i8_i2cReadNum = I2C_MODE_READY;
		Motor_OutputAll();						// write 0 speed to all motors
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
		if (i8_i2cWriteNum != I2C_MODE_READY)
		{
			// this should never happen!
			// you come here, when you want to OutputAll Motors and the last OutputAll Command is not finished.
			//while(1);
		}
		else
		{
			// if any motor has a fault, set EventBit for motor fault
			if(gui32_motor_fault)
				xEventGroupSetBits(gx_fault_EventGroup,fault_MOTOR);
			else
				xEventGroupClearBits(gx_fault_EventGroup,fault_MOTOR);
			HIDE_Fault_Increment(fault_MOTOR,gui32_motor_fault);

			HIDE_Workload_EstimateStart(p_workHandle);
			#if(I2C_INCREMENTAL_MODE)
				i8_i2cWriteNum = I2C_MODE_BUSY;		// start next i2c write command in i2c callback
				I2CWriteSpeed(ui8_motorMap[0]); 	// write to the first motor
			#else
				while(I2CMasterBusy(MOTOR_I2C_PERIPH_BASE));
				uint8_t i;
				for(i=0;i<motor_COUNT;++i)
				{
					I2CWriteSpeed(ui8_motorMap[i]);
					while(I2CMasterBusy(MOTOR_I2C_PERIPH_BASE));
				}
				HIDE_Workload_EstimateStop(p_workHandle);
			#endif
		}
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

	// TODO motor diagnostics are working only if other firmware is flashed. in the current version of the firmware no i2c read connection feasible
	/**
	 * \brief	read one motor (non blocking)
	 *
	 *			(informations will be stored in gs_motor)
	 *			fire eventBit fault_MOTOR_OVER_CURRENT if there is a fault,
	 *			else clear the bit.
	 * \param	ui8_motorNr	   The number of the Motor (0,1,2,...)
	 * \note    NOT TESTED AND IMPLEMENTED FINISHED
	 * 			Events: EventBit fault_MOTOR_OVER_CURRENT will be set or cleared in gx_fault_EventGroup
	 */
	void Motor_Read(uint8_t ui8_motorNr)
	{
		if (i8_i2cReadNum != I2C_MODE_READY)
		{
			// this should never happen!
			// you come here, when you want to read a Motor and the last read command is not finished.
			while(1);
		}
		else
		{
			// if any motor has over current, set EventBit for over current
			if(gui32_motor_overCurrent)
				xEventGroupSetBits(gx_fault_EventGroup,fault_MOTOR_OVER_CURRENT);
			else
				xEventGroupClearBits(gx_fault_EventGroup,fault_MOTOR_OVER_CURRENT);

//			I2CMRead(&i2cMastInst_s, 										// pointer to i2c master instance
//								ESC_BASE_ADR + ui8_motorMap[ui8_motorNr], 				// I2C adress ### Adresse zum lesen ist eine andere wie zum schreiben!
//
//								(uint8_t *)& gs_motor[ui8_motorMap[i8_i2cWriteNum]].ui8_current, 	// I2C message
//								2, 														// message length
//								I2CReadFinishCallback, 										// callback funktion (when message was sent)
//								0);
																				// callback data
		}
	}

	/**
	 * \brief	read all motors (non blocking)
	 *
	 *			(informations will be stored in gs_motor)
	 *			fire eventBit fault_MOTOR_OVER_CURRENT if there is a fault,
	 *			else clear the bit.
	 * \note    NOT TESTED AND NOT IMPLEMENTED
	 * 			Events: EventBit fault_MOTOR_OVER_CURRENT will be set or cleared in gx_fault_EventGroup
	 */
	void Motor_ReadAll(void)
	{
		// vermutlich besser nicht alle zu machen, sondert Task fragt in jeder Runde nur einen ab (I2C entlasten, und Info muss nicht sofort da sein)
	}

	/**
	 * \brief	when I2C trnsactions have completed, this funktion is called
	 *			in the context of the I2C master interrupthandler.
	 *
	 *			when it es commanded, a new I2C message will be started.
	 *			store in gui32_motor_fault which motor has a fault
	 * \param	pvData		not used
	 * \param	ui8_status	status to find error in I2C transmission
	 */
	static void I2CWriteFinishCallback(void *pvData, uint_fast8_t ui8_status)
	{
		// is there  an error in I2C transmission?
		if(ui8_status != I2CM_STATUS_SUCCESS)
		{
			// store which motor has a i2c fault
			gui32_motor_fault |= ( 1 << ui8_motorMap[i8_i2cWriteNum] );
		}
		else
		{
			// store which motor hasn't a i2c fault
			gui32_motor_fault &= ~( 1 << ui8_motorMap[i8_i2cWriteNum] );
		}

		// TODO change back to right motor control
//		i8_i2cWriteNum++;  // increment motor Number to adress the next motor
//		if(i8_i2cWriteNum<motor_COUNT)
//		{
//			I2CWriteSpeed(ui8_motorMap[i8_i2cWriteNum]);
//		}
		i8_i2cWriteNum += 3;
		if(i8_i2cWriteNum<motor_COUNT)
		{
		    I2CWriteSpeed(ui8_motorMap[i8_i2cWriteNum]);
		}
		else // all motors have been updated
		{
			// I2C finish
			i8_i2cWriteNum=I2C_MODE_READY;

			HIDE_Workload_EstimateStop(p_workHandle);
		}
	}

	/**
	 * \brief	when I2C trnsactions have completed, this funktion is called
	 *			in the context of the I2C master interrupthandler.
	 *
	 *			when it es commanded, a new I2C read will be started.
	 * \param	pvData		not used
	 * \param	ui8_status	status to find error in I2C transmission
	 * \note    NOT TESTED AND NOT IMPLEMENTED
	 */
	static void I2CReadFinishCallback(void *pvData, uint_fast8_t ui8_status)
	{
		// over current at the motor?
		if( gs_motor[ui8_motorMap[i8_i2cReadNum]].f_current >= MOTOR_OVER_CURRENT)
		{
			// store motor has a over current
			gui32_motor_overCurrent |= ( 1 << ui8_motorMap[i8_i2cReadNum] );
		}
		else
		{
			// store motor hasn't a over current
			gui32_motor_overCurrent &= ~( 1 << ui8_motorMap[i8_i2cReadNum] );
		}

		i8_i2cReadNum=I2C_MODE_READY;
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
	static void I2CWriteSpeed(uint8_t ui8_motorNr)
	{
		uint32_t ui32_speed = (uint32_t) gs_motor[ui8_motorNr].ui16_setPoint;	// speed in größere variiablen laden

		ui16_motorDataUsb[ui8_motorNr] = ui32_speed;

		// Wertebereich von 1000..2000us mappen auf
		// bei mode:2047   -> 0..2047
		// bei mode:255    -> 0..255
//		ui32_speed = ui32_speed - 1000;  // = 0...1000
//		ui32_speed = (ui32_speed * ESC_DATA_SIZE) / 1000;  //0..2047 oder 0..255


		ui32_speed = (ui32_speed * ESC_DATA_SIZE) / 0xFFFF;  //0..2047 oder 0..255

		// TODO delete later send motor data over USB
		ui16_motorDataUsb[ui8_motorNr] = (uint16_t) ui32_speed;

		if(ESC_DATA_SIZE == 255)
		{
			// brushlesregler version 1
			ui8_msg[0] = (uint8_t)ui32_speed;
			I2CMWrite(	&i2cMastInst_s, 					// pointer to i2c master instance
						ESC_BASE_ADR + ui8_motorNr, 		// I2C adress
						ui8_msg, 							// I2C message
						1, 									// message length
						#if(I2C_INCREMENTAL_MODE)
							I2CWriteFinishCallback, 		// callback funktion (when message was send)
						#else
							0,								// no callback funktion
						#endif
						0);									// callback data
		}
		else
		{
			// brushless version >= 2.0
			// Speed in 2 bytes (11bits) umwandeln
			ui8_msg[0] = (uint8_t)(ui32_speed >> 3) & 0xff; // gets the high byte [10->3]
			ui8_msg[1] = ((uint8_t)ui32_speed % 8) & 0x07;  // gets the low 3 bits [3->0]

			I2CMWrite(	&i2cMastInst_s, 					// pointer to i2c master instance
						ESC_BASE_ADR + ui8_motorNr, 		// I2C adress
						ui8_msg, 							// I2C message
						2, 									// message length
						#if(I2C_INCREMENTAL_MODE)
							I2CWriteFinishCallback, 		// callback funktion (when message was send)
						#else
							0,								// no callback funktion
						#endif
						0);

												// callback data

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
