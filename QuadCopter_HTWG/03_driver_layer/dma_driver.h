/**
 * 		@file 	dma_driver.h
 * 		@brief	Functions to coordinate commands for the DMA controller
 *//*	@author Tobias Grimm
 * 		@date 	25.04.2016	(last modified)
 */

#ifndef __DMA_DRIVER_H__
#define	__DMA_DRIVER_H__

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
//#include "receiver_task.h"	// for eventBits

/* ------------------------------------------------------------ */
/*				   	Defines			    						*/
/* ------------------------------------------------------------ */

// DMA Channel numbers to be passed to API functions that require a ui8_channelNum
/*#define dma_CH0   		0
#define dma_CH1   		1
#define dma_CH2   		2
#define dma_CH3   		3
#define dma_CH4   		4
#define dma_CH5   		5
#define dma_CH6     	6
#define dma_CH7     	7
#define dma_CH8    		8
#define dma_CH9    		9
#define dma_CH10    	10
#define dma_CH11     	11
#define dma_CH14       	14
#define dma_CH15       	15
#define dma_CH16       	16
#define dma_CH17       	17
#define dma_CH18      	18
#define dma_CH19      	19
#define dma_CH20      	20
#define dma_CH21      	21
#define dma_CH22    	22
#define dma_CH23    	23
#define dma_CH24     	24
#define dma_CH25     	25
#define dma_CH28     	28
#define dma_CH29     	29
#define dma_CH30        30
*/
// DMA Channel eventBit numbers to be passed to API functions that require a ui32_channelEventBit
// (if event Bits are needed, reserve them in receiver_task.h
//  and unlock them before first usage with Dma_ChannelUnlock)
// e.g #define dma_CH15_EB		receiver_DMA_CH15

/* ------------------------------------------------------------ */
/*				   Type Definitions			    				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

extern void Dma_Init();
extern void Dma_ChannelLock(uint32_t ui32_channelEventBit);
extern void Dma_ChannelUnlock(uint32_t ui32_channelEventBit);
extern void Dma_ChannelUnlockFromISR(uint32_t ui32_channelEventBit);
extern void Dma_ChannelWaitBlocked(uint32_t ui32_channelEventBit);
extern void Dma_ChannelWaitBusy(uint8_t ui8_channelNum);

/* ------------------------------------------------------------ */
/*					Global Variables							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */

#endif // __DMA_DRIVER_H__
