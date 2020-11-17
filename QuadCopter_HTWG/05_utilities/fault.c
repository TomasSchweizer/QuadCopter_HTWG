/**
 * 		@file 	fault.c
 * 		@brief	central place for all fault Events.
 *
 * 				Fault Events can get Names and counted how often they happened.
 *//*	@author Tobias Grimm
 * 		@date 	17.04.2016	(last modified)
 */

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <stdint.h>
#include <stdbool.h>

// freeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

// application
#include "fault.h"

// utils
#include "count_edges.h"
#include "qc_math.h"

// driver
#include "display_driver.h"

// setup
#include "qc_setup.h"

/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

uint32_t Fault_Init(void);

/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/**
 * \brief	event bits to indicate if a fault appears
 *
 *			(see fault.h for bit order)
 */
volatile EventGroupHandle_t gx_fault_EventGroup;

/**
 * \brief	every event bit has a field to sum up
 *			how often it was fired (rising edge)
 *			(see fault.h & cound_edges.h)
 * \note	Write access:	a lot of tasks &| ISRs -> (this is only for development, race conditions can happen, but it is not critical.)
 */
volatile countEdges_handle_p gp_fault_coundEdges;

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/**
 * \brief	initializes fault EventGroup and set all bits by default
 * \return	false if creation was successful,
 *			true  else
 */
uint32_t Fault_Init(void)
{

    // Attempt to create the event groups.
	gx_fault_EventGroup = xEventGroupCreate();

    // Was the event group created not successfully?
    if( gx_fault_EventGroup == math_NULL )
    	return(true);
    else	// it was successfully
    	xEventGroupSetBits(gx_fault_EventGroup, 0xFFFFFF );		// all fault bits are set by default

	#if	( setup_DEV_SUM_FAULTS )
		gp_fault_coundEdges=CountEdges_Create(fault_COUNT);
		if( gp_fault_coundEdges == math_NULL )
			return(true);
	#endif

    // Success
    return(false);
}

#if ( setup_DEV_SUM_FAULTS ) || DOXYGEN

	/**
	 * \brief	Draw info about faults on the Display
	 * \note	to enable this HIDE function set setup_DEV_SUM_FAULTS in qc_setup.h
	 */
	void HIDE_Fault_DrawDisplay(void)
	{
		EventBits_t x_faultEventBits = xEventGroupGetBits( gx_fault_EventGroup );
		const uint8_t drawHight 	= 6;
		const uint8_t drawLengthNum = 5;
		const uint8_t xOffset 		= 92;
		const uint8_t xOffsetNum	= 15+1;
		u8g_SetFont(&gs_display, u8g_font_04b_03r);		// u8g_font_unifont
		uint8_t i;
		uint16_t hight=drawHight-1;
		const char* p_name;
		for(i=0;i<fault_COUNT;++i)
		{
			p_name=CountEdges_GetName(gp_fault_coundEdges,i);
			if(p_name!=0)
			{
				u8g_DrawStr(&gs_display, xOffset + 0,  hight,p_name);						// Name
				u8g_DrawStr(&gs_display, xOffset + xOffsetNum, hight, u8g_u16toa(CountEdges_Get(gp_fault_coundEdges,i),3));		// How often fired
				if(x_faultEventBits&(1<<i))
					u8g_DrawStr(&gs_display, xOffset + xOffsetNum + drawLengthNum*3,  hight,  "!!!");		// Bit set
				else
					u8g_DrawStr(&gs_display, xOffset + xOffsetNum + drawLengthNum*3,  hight,  " ");		// Bit not set
				hight+=drawHight;
			}
		}
	}

	/**
	 * \brief	increment counter for rising edges for a fault bit.
	 *
	 * 			increment in gp_fault_coundEdges at the desired bit
	 *			if there is a rising edge in ui32_faultBitValue (0 old 1 currend)
	 * \param	ui32_faultEventBit	see fault.h for bit defines
	 * \param	ui32_faultBitValue	0 when bit is cleared, !=0 when bit is fired
	 * \note	to enable this HIDE function set setup_DEV_SUM_FAULTS in qc_setup.h
	 */
	void HIDE_Fault_Increment(uint32_t ui32_faultEventBit,uint32_t ui32_faultBitValue)
	{
		CountEdges_Update(gp_fault_coundEdges,CountEdges_Bit2Num(ui32_faultEventBit),ui32_faultBitValue!=0,countEdges_METHOD_RISING);
	}

	/**
	 * \brief	store the name of the fault eventBit into gp_fault_coundEdges
	 *
	 *			(this should be performed before scheduler starts)
	 *			e.g. in the init of the driver who fires the eventBit
	 * \param	ui32_eventBit	the fault eventBit (see fault.h)
	 * \param	pc_name			const string name
	 * \note	to enable this HIDE function set setup_DEV_SUM_FAULTS in qc_setup.h
	 */
	void HIDE_Fault_SetEventName(uint32_t ui32_eventBit,const char* pc_name)
	{
		CountEdges_SetName(gp_fault_coundEdges,CountEdges_Bit2Num(ui32_eventBit),pc_name);
	}

#endif
