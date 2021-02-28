/*===================================================================================================*/
/*  qc_setup.h                                                                                      */
/*===================================================================================================*/

/**
*   @file   qc_setup.h
*
*   @brief  Include and exclude the desired hardware setup and software modules for the QuadCopter.
*
*   @details
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>17/06/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>14/12/2020      <td>Tomas Schweizer     <td>Overall changes / added USB, Watchdog...
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*/

/*====================================================================================================*/

#ifndef __QC_SETUP_H__
#define	__QC_SETUP_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include/Exclude Software Add-ons                               */
/* ---------------------------------------------------------------------------------------------------*/

// Select Add-ons for the Quadcopter (1 -> include, 0 -> exclude)
#define setup_QC_BASIC  				( 1 )					///< Basic setup to fly with remote control
#define setup_QC_DEVELOPMENT  			( 1 )					///< Enable development Add-ons
#define setup_QC_ALTITUDE_HOLD          ( 1 )                   ///< Enable Altitude hold mode
#define setup_QC_AUTONOMOUS_NAVIGATION  ( 0 )					///< not implemented yet
#define setup_QC_KAMERA_ADDON			( 0 )					///< not implemented yet

// Select the desired development add-ons (1 -> include, 0 -> exclude) ( all development add-ons use HIDE! )
#if setup_QC_DEVELOPMENT

	#define setup_DEV_STACK_OVERFLOW	( 1 )					///< Check all tasks for stack overflow
	#define setup_DEV_WORKLOAD_CALC		( 1 )					///< Calc workload for all tasks & enable workload estimation instances
	#define setup_DEV_WORKLOAD_LED		( 1 )					///< Enable workload LED
	#define setup_DEV_SUM_FAULTS		( 1 )					///< Every fault eventBit gets a name & every time a fault   event happens, increment a counter (gp_fault_coundEdges)
	#define setup_DEV_SUM_RECEIVS		( 1 )					///< Every receive eventBit gets a name & every time a receive event happens, increment a counter
	#define setup_DEV_DISPLAY			( 1 )					///< Show workloads, sumFaults, sumReceivs on Display (but only when the desired setup_DEV=1)
	#define setup_DEV_PID_TUNE			( 1 )					///< Read debug interface for PID values (this can crash with write commands on the trace interface)
	#define setup_DEV_DEBUG_PINS		( 1 )					///< Enable debug pins
    #define setup_DEV_DEBUG_USB         ( 1 )                   ///< Enable debug options over USB
	#define setup_DEV_COMMANDS			( 0 )					///< Enable HIDE-function macros to insert into the command Queue Task

#endif

// Select the desired sensors for the altitude hold add-on
#if setup_QC_ALTITUDE_HOLD

    #define setup_ALT_BARO              ( 1 )                   ///< Barometer is used to measure altitude
    #define setup_ALT_LIDAR             ( 1 )                   ///< Lidar is used to measure altitude

#endif

/* ---------------------------------------------------------------------------------------------------*/
/*                                    Select Hardware                                                 */
/* ---------------------------------------------------------------------------------------------------*/

// Masks to unpack options (if more then one option is used for this hardware)
#define setup_MASK_OPT1					( 0x000000FF )              ///< Mask 1 for bytes 1/2
#define setup_MASK_OPT2					( 0x0000FF00 )              ///< Mask 2 for bytes 3/4
#define setup_MASK_OPT3					( 0x00FF0000 )              ///< Mask 3 for bytes 5/6
#define setup_MASK_OPT4					( 0xFF000000 )              ///< Mask 4 for bytes 7/8

// Select if watchdog should be enabled or not. !!! Should be disabled in development/Debugging !!!
#define setup_WATCHDOG_NONE             ( 0x00000001 )              ///< Watchdog is disabled
#define setup_WATCHDOG_ACTIVE           ( 0x00000002 )              ///< Watchdog is activated
#define setup_WATCHDOG                  ( setup_WATCHDOG_ACTIVE )   ///< Macro for preprocessor commands in other files


// Select the mode to control the motors (others will be exclude from build)
#define setup_MOTOR_NONE  				( 0x00000001 )			    ///< No motor commands to ESCs
#define setup_MOTOR_PWM  				( 0x00000002 )			    ///< PWM motor commands !!!ported from RiestererN NOT TESTED!!!
#define setup_MOTOR_I2C  				( 0x00000003 )			    ///< I2C motor commands
#define setup_MOTOR     				( setup_MOTOR_I2C )         ///< Macro for preprocessor commands in other files

// Select the mode for the remote control (others will be exclude from build)
#define setup_REMOTE_NONE				( 0x00000001 )			    ///< No remote control functions
#define setup_REMODE_CPPM  				( 0x00000002 )			    ///< CPPM signal for remote control
#define setup_REMOTE_SIMULATE			( 0x00000003 )			    ///< Remote control simulates constant values and every x ms a receiver eventBit will be fired
#define setup_REMOTE_RIGHT_HAND			( 0x00000100 )			    ///< Right hand receiver
#define setup_REMOTE_LEFT_HAND			( 0x00000200 )			    ///< Left hand receiver
#define setup_REMOTE_FIT_TO_QC          ( 0x00010000 )              ///< Fit the remote control input to quadcopter rotational axes (inverse yaw, roll and pitch)
#define setup_REMOTE      				( setup_REMODE_CPPM | setup_REMOTE_RIGHT_HAND | setup_REMOTE_FIT_TO_QC ) ///< Macro for preprocessor commands in other files

// Select the mode for the sensor (others will be exclude from build)
#define setup_SENSOR_NONE  				( 0x00000001 )			    ///< Sensor functions will do nothing
#define setup_SENSOR_I2C  				( 0x00000002 )			    ///< I2C communication with sensors
#define setup_SENSOR      				( setup_SENSOR_I2C)

// Select the mode for the display (others will be exclude from build, & all display functions use HIDE!)
#define setup_DISPLAY_NONE  			( 0x00000001 )			    ///< All display functions will do nothing and consume no memory (HIDE)
#define setup_DISPLAY_SPI  				( 0x00000002 )			    ///< Display over SPI
#define setup_DISPLAY_I2C  				( 0x00000003 )			    ///< Display over I2C !!!NOT IMPLEMENTED!!!
#define setup_DISPLAY      				( setup_DISPLAY_SPI )       ///< Macro for preprocessor commands in other files

// Select the mode for the debug interface (others will be exclude from build, & all debug functions use HIDE!)
#define setup_DEBUG_NONE  				( 0x00000001 )			    ///< Debug interface functions will do nothing and consume no memory (HIDE)
#define setup_DEBUG_UART  				( 0x00000002 )			    ///< Debug interface over UART !!!Tested, but normally use USB!!!
#define setup_DEBUG_USB  				( 0x00000003 )			    ///< Debug interface over USB
#define setup_DEBUG      				( setup_DEBUG_USB )         ///< Macro for preprocessor commands in other files

#endif // __QC_SETUP_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
