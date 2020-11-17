/**
 * 		@file 	busy_delay.c
 * 		@brief	simple busy waiting delays
 *
 * 				implemented with a timer
 *//* 	@author Tobias Grimm
 * 		@date 	25.05.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */


#include <stdint.h>
#include <stdbool.h>

//  Hardware Specific
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

// utils
#include "busy_delay.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

#define TIMER_BASE				TIMER2_BASE			   // free running Timer 16 Bit 1us ticks
#define TIMER_MODULE 			TIMER_A
#define SYSCTL_PERIPH_TIMER		SYSCTL_PERIPH_TIMER2

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void BusyDelay_Init();
void BusyDelay_Us(uint16_t ui16_us);
void BusyDelay_Ms(uint32_t ui32_ms);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/**
 * \brief	Init needed peripheral for busy waiting delays
 */
void BusyDelay_Init()
{
	// timer 1 aktivieren
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER);
	// zuerst Timer disable
	ROM_TimerDisable(TIMER_BASE,TIMER_MODULE);

	// timer 1A zählt hoch von 0xffff bis 0x000 und fängt von vorne an
	ROM_TimerConfigure(TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

	// von 0xffff  bis 0 zählen und dann von vorne
	// prescaler 80 bei 80Mhz -> 1us
	// prescaler 50 bei 50Mhz  -> 1us
	ROM_TimerPrescaleSet(TIMER_BASE,TIMER_MODULE,80);

	// timer starten
	ROM_TimerEnable(TIMER_BASE, TIMER_MODULE);
}


/**
 * \brief	busy waiting us
 *
 * \param	ui16_us   delay value [us]
 */
void BusyDelay_Us(uint16_t ui16_us)
{
	uint16_t ui16_startTime = ROM_TimerValueGet(TIMER_BASE, TIMER_MODULE);
	while ( (ui16_startTime-ROM_TimerValueGet(TIMER_BASE, TIMER_MODULE)) < ui16_us);
}

/**
 * \brief	busy waiting for ms
 *
 * \param	ui32_ms   delay value [ms]
 */
void BusyDelay_Ms(uint32_t ui32_ms)
{
	uint32_t i;
	for(i=0;i<ui32_ms;++i)
		BusyDelay_Us(1000);
}
