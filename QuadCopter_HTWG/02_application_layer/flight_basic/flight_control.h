/*===================================================================================================*/
/*  flight_control.h                                                                               */
/*===================================================================================================*/

/**
*   @file flight_control.h
*
*   @brief API for flight control
*
*   @details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>13/05/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>18/12/2020      <td>Tomas Schweizer     <td>Added functions USB and PID tune
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   -
*/
/*====================================================================================================*/

#ifndef __FLIGHT_CONTROL_H__
#define	__FLIGHT_CONTROL_H__

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>

// Setup
#include "qc_setup.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Defines                                                       */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Type Definitions                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      API Procedure Definitions                                     */
/* ---------------------------------------------------------------------------------------------------*/

extern void Control_FlightStabilisation(void);
extern void Control_Reset(void);
extern void Control_Mixer(void);
extern void Control_MixerPassThrottle(void);
extern void Control_MotorSameSetPoint(uint16_t ui16_setPoint);


#if	( setup_DEV_PID_TUNE ) || DOXYGEN
	extern void HIDE_Control_DebugGetPid(void);
	extern void HIDE_Control_Debug_USB_GetPID(void);
    extern void HIDE_Control_PID_TUNE_DrawDisplay(void);
    extern void HIDE_Control_SendDataOverUSB(void);
#else
	#define HIDE_Control_DebugGetPid()					// this define will be kicked off from the preprocessor
    #define HIDE_Control_Debug_USB_GetPID()             // this define will be kicked off from the preprocessor
    #define HIDE_Control_PID_TUNE_DrawDisplay()         // this define will be kicked off from the preprocessor
    #define HIDE_Control_SendDataOverUSB()              // this define will be kicked off from the preprocessor
#endif


#endif // __FLIGHT_CONTROL_H__

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
