/*===================================================================================================*/
/*  watchdog.c                                                                                       */
/*===================================================================================================*/

/*
*   file   watchdog.c
*
*   brief  Implementation of a watchdog timer to reset the system in dangerous situations
*
*   details
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>15/01/2021      <td>Tomas Schweizer     <td>Implementation
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*   TivaWare Examples
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdint.h>
#include <stdbool.h>

// Hardware specific libraries
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/watchdog.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"

// Setup
#include "prioritys.h"
#include "peripheral_setup.h"

// Application
#include "watchdog.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/

void Watchdog_IntHandler(void);

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Global Variables                                              */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Variables                                               */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Procedure Definitions                                         */
/* ---------------------------------------------------------------------------------------------------*/

// If watchdog is activated in qc_setup
#if (setup_WATCHDOG == (setup_WATCHDOG_ACTIVE&setup_MASK_OPT1) )


    #define WATCHDOG_SYSCTL_PERIPH              SYSCTL_PERIPH_WDOG0     ///<watchdog peripheral
    #define WATCHDOG_BASE                       WATCHDOG0_BASE          ///< watchdog module base


    /**
     * @brief   Initializes the watchdog timer to 2ms
     *
     * @return  void
     */
    void Watchdog_Init(void)
    {
        // Enable Watchdog peripheral
        ROM_SysCtlPeripheralEnable(WATCHDOG_SYSCTL_PERIPH);

        while(!ROM_SysCtlPeripheralReady(WATCHDOG_SYSCTL_PERIPH))
        {

        }

        // Set interrupt priority of watchdog to highest value 0
        ROM_IntPrioritySet(periph_WATCHDOG_INT, priority_WATCHDOG_ISR);

        // Check if watchdog registers are locked, and if so unlock to write
        if(ROM_WatchdogLockState(WATCHDOG_BASE) == true)
        {
            ROM_WatchdogUnlock(WATCHDOG_BASE);
        }

        // Enable the watchdog interrupt
        ROM_IntEnable(periph_WATCHDOG_INT);

        // Set period of Watchdog timer to 2ms which equals one loop of the flight task
        ROM_WatchdogReloadSet(WATCHDOG_BASE, ROM_SysCtlClockGet()/500);

        // Enable Reset generation for watchdog timer
        ROM_WatchdogResetEnable(WATCHDOG_BASE);

        ROM_WatchdogEnable(WATCHDOG_BASE);

        ROM_WatchdogIntClear(WATCHDOG_BASE);
    }


    /**
     * @brief   Watchdog interrupt handler, clears the watchdog interrupt.
     *          If two times not cleared microcontroller resets.
     *
     * @return  void
     *
     * @note    Should be disabled for development/debugging
     */
    void Watchdog_IntHandler(void)
    {


        if(ROM_WatchdogIntStatus(WATCHDOG_BASE, true))
        {
            ROM_WatchdogIntClear(WATCHDOG_BASE);
        }

    }



#elif ( setup_WATCHDOG == (setup_WATCHDOG_NONE&setup_MASK_OPT1) )

    void Watchdog_Init(void){}
    void Watchdog_Start(void){}

#else
    #error ERROR: define setup_WATCHDOG in qc_setup.h
#endif

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
