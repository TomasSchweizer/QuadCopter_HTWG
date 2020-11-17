/**
 * 		@file 	dma_driver.c
 * 		@brief	Functions to coordinate commands for the DMA controller
 *//*	@author Tobias Grimm
 * 		@date 	25.04.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>

//  FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"

//  Hardware Specific
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/udma.h"
#include "driverlib/sysctl.h"

// application
#include "receiver_task.h"	// for eventBits

// driver
#include "dma_driver.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void Dma_Init();
void Dma_ChannelLock(uint32_t ui32_channelEventBit);
void Dma_ChannelUnlock(uint32_t ui32_channelEventBit);
void Dma_ChannelUnlockFromISR(uint32_t ui32_channelEventBit);
void Dma_ChannelWaitBlocked(uint32_t ui32_channelEventBit);
void Dma_ChannelWaitBusy(uint8_t ui8_channelNum);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

#pragma DATA_ALIGN(ui8_dma_controlTable, 1024)
static uint8_t ui8_dma_controlTable[1024];

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/**
 * \brief	ErrorISR of DMA
 */
void Dma_ErrorISR()
{
	while(1);
}

/**
 * \brief	Init DMA
 */
void Dma_Init()
{
    // Enable the uDMA controller at the system level.  Enable it to continue
    // to run while the processor is in sleep.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    // Enable the uDMA controller error interrupt.  This interrupt will occur
    // if there is a bus error during a transfer.
    ROM_IntEnable(INT_UDMAERR);

    // Enable the uDMA controller.
    ROM_uDMAEnable();

    // Point at the control table to use for channel control structures.
    ROM_uDMAControlBaseSet(ui8_dma_controlTable);
}

/**
 * \brief	Try to Lock the desired channel
 *
 * 			if the channel is allready locked, block until it es unlocked.
 * \param	ui32_channelEventBit	the Channel eventBit
 */
void Dma_ChannelLock(uint32_t ui32_channelEventBit)
{
	xEventGroupWaitBits(gx_receiver_eventGroup,
			ui32_channelEventBit,
			pdTRUE,          // clear Bits before returning.
			pdFALSE,         // wait for any Bits
			portMAX_DELAY ); // Wait the maximum time.
}

/**
 * \brief	Unlock the desired channel.
 * \param	ui32_channelEventBit	the Channel eventBit
 */
void Dma_ChannelUnlock(uint32_t ui32_channelEventBit)
{
	xEventGroupSetBits(gx_receiver_eventGroup, ui32_channelEventBit );
}

/**
 * \brief	Unlock the desired channel from ISR
 * \param	ui32_channelEventBit	the Channel eventBit
 */
void Dma_ChannelUnlockFromISR(uint32_t ui32_channelEventBit)
{
	// Send a notification to ReceiverTask, that frame is ready
	// and can be read during sync time of the frame
	BaseType_t x_higherPriorityTaskWoken = pdFALSE;
	if(pdFAIL==xEventGroupSetBitsFromISR(gx_receiver_eventGroup,ui32_channelEventBit, &x_higherPriorityTaskWoken))
	{
		// this should never happen!
		// you will come here, when configTIMER_QUEUE_LENGTH is full
		while(1);
	}
	else
	{
		portYIELD_FROM_ISR( x_higherPriorityTaskWoken );
	}
}

/**
 * \brief	wait blocking until the desired channel is unlocked.
 * \param	ui32_channelEventBit	the Channel eventBit
 */
void Dma_ChannelWaitBlocked(uint32_t ui32_channelEventBit)
{
    //  Wait for any receiver event
	xEventGroupWaitBits(gx_receiver_eventGroup,
			ui32_channelEventBit,
			pdFALSE,         // don't clear Bits before returning.
			pdFALSE,         // wait for any Bits
			portMAX_DELAY ); // Wait the maximum time.

	HIDE_Receive_Increment(ui32_channelEventBit);
}

/**
 * \brief	wait busy until the desired channel is ready.
 * \param	ui8_channelNum	the Channel Number
 */
void Dma_ChannelWaitBusy(uint8_t ui8_channelNum)
{
	while(	UDMA_MODE_STOP!=ROM_uDMAChannelModeGet(ui8_channelNum | UDMA_PRI_SELECT)   ||
			UDMA_MODE_STOP!=ROM_uDMAChannelModeGet(ui8_channelNum | UDMA_ALT_SELECT));
}
