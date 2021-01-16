/**
 * 		@file 	main.c
 * 		@brief	initializes the System and start all Tasks
 *//*	@author Tobias Grimm
 * 		@date 	21.03.2016	(last modified)
 */



/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>

// Hardware Specific
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "debug_interface.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// application
#include "flight_task.h"
#include "receiver_task.h"
#include "command_task.h"

// utils
#include "workload.h"
#include "fault.h"

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/**
 * \brief	initializes the System and start all Tasks
 * \return	this should never happen
 */
int main(void)
{
	// Set internal Clock to 80MHz
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// Wait for clock to stabilize
	ROM_SysCtlDelay(ROM_SysCtlClockGet() / 12);

	//TODO just debugging


	//
	//	Default Applications
	//
	if(Fault_Init())
		while(1);
	if(ReceiverTask_Init())
		while(1);
	if(CommandTask_Init())
		while(1);

	//  DEV INITs
	HIDE_Workload_Init();
	HIDE_Debug_PinsInit();

	//
	//  QC_BASIC
	//
	#if ( setup_QC_BASIC )

		if(FlightTask_Init())
			while(1);

	#endif

    // Enable All Interrupts
    ROM_IntMasterEnable();

    // Start the scheduler.  This should not return.
    vTaskStartScheduler();

    // In case the scheduler returns for some reason, loop forever.
    while(1);
}

/**
 * \brief	This hook is called by FreeRTOS when an stack overflow error is detected.
 * \param	pxTask		the task where the stack overflow happened
 * \param 	pcTaskName	the name of the task where the stack overflow happened
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
