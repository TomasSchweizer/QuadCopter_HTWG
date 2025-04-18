/**
 * 		@file 	qc_setup.h
 * 		@brief	include and exclude the desired hardware setup
 *  			and software modules for the QuadCopter.
 *//*	@author Tobias Grimm
 * 		@date 	17.06.2016	(last modified)
 */

#ifndef __QC_SETUP_H__
#define	__QC_SETUP_H__

/* ------------------------------------------------------------ */
/*			Include/Exclude Software Addons 			    		*/
/* ------------------------------------------------------------ */

// Select Addons for the Quadcopter (1 -> include, 0 -> exclude)
#define setup_QC_BASIC  				( 1 )					// basic setup to fly with remote control
#define setup_QC_DEVELOPMENT  			( 1 )					// enable development addons (see below)
#define setup_QC_AUTONOMOUS_NAVIGATION  ( 0 )					// not implemented yet
#define setup_QC_KAMERA_ADDON			( 0 )					// not implemented yet

// Select the desired development addons (1 -> include, 0 -> exclude) ( all development addons use HIDE! )
#if setup_QC_DEVELOPMENT

	#define setup_DEV_STACK_OVERFLOW	( 1 )					// check all tasks for stack overflow
	#define setup_DEV_WORKLOAD_CALC		( 1 )					// calc workload for all tasks & enable workload estimation instances
	#define setup_DEV_WORKLOAD_LED		( 1 )					// enable workload LED
	#define setup_DEV_SUM_FAULTS		( 1 )					// every fault   eventBit gets a name & every time a fault   event happens, increment a counter (gp_fault_coundEdges)
	#define setup_DEV_SUM_RECEIVS		( 1 )					// every receive eventBit gets a name & every time a receive event happens, increment a counter
	#define setup_DEV_DISPLAY			( 1 )					// show workloads, sumFaults, sumReceivs on Display (but only when the desired setup_DEV=1)
	#define setup_DEV_PID_TUNE			( 1 )					// read debug interface for PID values (this can crash with write commands on the trace interface)
	#define setup_DEV_DEBUG_PINS		( 1 )					// enable debug pins
	#define setup_DEV_COMMANDS			( 0 )					// enable HIDE-function macros to insert into the command Queue Task

#endif

/* ------------------------------------------------------------ */
/*				Select hardware setup			    			*/
/* ------------------------------------------------------------ */

// Masks to unpack options (if more then one option is used for this hardware)
#define setup_MASK_OPT1					( 0x000000FF )
#define setup_MASK_OPT2					( 0x0000FF00 )
#define setup_MASK_OPT3					( 0x00FF0000 )
#define setup_MASK_OPT4					( 0xFF000000 )

// Select the Mode to control the Motors (others will be exclude from build)
#define setup_MOTOR_NONE  				( 0x00000001 )			// all motor functions will do nothing
#define setup_MOTOR_PWM  				( 0x00000002 )			// port it from RiestererN
#define setup_MOTOR_I2C  				( 0x00000003 )			// tested
#define setup_MOTOR     				( setup_MOTOR_I2C )

// Select the Mode for the remote control (others will be exclude from build)
#define setup_REMOTE_NONE				( 0x00000001 )			// all remote control functions will do nothing
#define setup_REMODE_CPPM  				( 0x00000002 )			// all Channels are in 1 wire
#define setup_REMOTE_SIMULATE			( 0x00000003 )			// remote control get funktions will output const values and every x ms a receiver eventBit will be fired
#define setup_REMOTE_RIGHT_HAND			( 0x00000100 )			// throttle on right hand
#define setup_REMOTE_LEFT_HAND			( 0x00000200 )			// throttle on left hand
#define setup_REMOTE      				( setup_REMODE_CPPM | setup_REMOTE_LEFT_HAND )

// Select the Mode for the sensor (others will be exclude from build)
#define setup_SENSOR_NONE  				( 0x00000001 )			// all sensor functions will do nothing
#define setup_SENSOR_SPI  				( 0x00000002 )			// not portet from Eckstein
#define setup_SENSOR_I2C  				( 0x00000003 )			// tested
#define setup_SENSOR_INV_ROLL			( 0x00000100 )			// invert roll  signal
#define setup_SENSOR_INV_PITCH			( 0x00000200 )			// invert pitch signal
#define setup_SENSOR_INV_YAW			( 0x00000400 )			// invert yaw 	signal
#define setup_SENSOR_SWOP_ROLL_PITCH	( 0x00000800 )			// swop roll and pitch signals
#define setup_SENSOR      				( setup_SENSOR_I2C | setup_SENSOR_INV_ROLL | setup_SENSOR_SWOP_ROLL_PITCH )

// Select the Mode for the display (others will be exclude from build, & all display functions use HIDE!)
#define setup_DISPLAY_NONE  			( 0x00000001 )			// all display funktions will do nothing and consume no memmory (HIDE)
#define setup_DISPLAY_SPI  				( 0x00000002 )			// tested
#define setup_DISPLAY_I2C  				( 0x00000003 )			// not implemented
#define setup_DISPLAY      				( setup_DISPLAY_SPI )

// Select the Mode for the debug interface (others will be exclude from build, & all debug functions use HIDE!)
#define setup_DEBUG_NONE  				( 0x00000001 )					// all debug interface functions will do nothing and consume no memmory (HIDE)
#define setup_DEBUG_UART  				( 0x00000002 )					// tested
#define setup_DEBUG_USB  				( 0x00000003 )					// not implemented
#define setup_DEBUG      				( setup_DEBUG_UART )

#endif // __QC_SETUP_H__
