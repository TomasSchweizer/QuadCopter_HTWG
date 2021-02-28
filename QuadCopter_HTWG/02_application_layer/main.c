/*===================================================================================================*/
/*  main.c                                                                                           */
/*===================================================================================================*/

/**
*   @file main.c
*
*   @brief Initializes the system, the tasks, the watchdog and starts the FreeRTOS scheduler.
*
*   @details
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>21/03/2016      <td>Tobias Grimm        <td>Implementation & Last modification of MAs
*   <tr><td>14/12/2020      <td>Tomas Schweizer     <td>Added Watchdog
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
#include <stdbool.h>

// Hardware specific libraries
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

// Setup
#include "qc_setup.h"

// Application
#include "flight_task.h"
#include "receiver_task.h"
#include "command_task.h"
#include "watchdog.h"

// Driver
#include "debug_interface.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// Utilities
#include "workload.h"
#include "fault.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/

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
 * @brief   Initializes the system, the tasks, the watchdog and starts the FreeRTOS scheduler.
 *
 * @return  exit --> returns the exit code of the operating system
 */
int main(void)
{
	// Set internal Clock to 80MHz
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// Wait for clock to stabilize
	ROM_SysCtlDelay(ROM_SysCtlClockGet() / 12);

	// Default applications
	if(Fault_Init())
		while(1);
	if(ReceiverTask_Init())
		while(1);
	if(CommandTask_Init())
		while(1);

	// DEV INITs
	HIDE_Workload_Init();
	HIDE_Debug_PinsInit();

	// QC_BASIC
	#if ( setup_QC_BASIC )

		if(FlightTask_Init())
			while(1);
	#endif

    // Enable all interrupts
    ROM_IntMasterEnable();

    // Initialize and start watchdog !!! COMMENT OUT FOR DEBUGGING !!!
    //Watchdog_Init();

    // Start the scheduler and runs in a endless loop
    vTaskStartScheduler();

    // In case the scheduler returns for some reason, loop forever.
    while(1);
}

/**
 * @brief   This hook is called by FreeRTOS when an stack overflow error is detected.
 *
 * @param	pxTask		The task where the stack overflow occurred
 * @param 	pcTaskName	The name of the task where the stack overflow occurred
 *
 * @return  void
 *
 */
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    /*
     *  This function can not return, so loop forever.  Interrupts are disabled
     *  on entry to this function, so no processor interrupts will interrupt
     *  this loop.
     */
    while(1)
    {
    }
}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
