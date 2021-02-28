/*===================================================================================================*/
/*  count_edges.c                                                                                    */
/*===================================================================================================*/

/*
*   file   count_edges.c
*
*   brief  Implementation of functions to count edges of different signals.
*
*   @details
*   Counting method can be rising, falling, or both edges.
*   Every signal can have a name.\n
*
*   sig0:   0|1 1 1|0 0 0       countEdges_METHOD_RISING:  1\n
*
*   sig1:   0|1 1 1|0|1 1       countEdges_METHOD_FALLING: 1\n

*   sig2:   0|1 1 1|0|1|0       countEdges_METHOD_BOTH:    4\n
*
*   \n
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>23/04/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
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

// Standard library
#include <stdint.h>

// FreeRTOS
#include "FreeRTOS.h"		// for pvPortMalloc

// Utilities
#include "qc_math.h"
#include "count_edges.h"

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Defines                                                 */
/* ---------------------------------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Local Type Definitions                                        */
/* ---------------------------------------------------------------------------------------------------*/

typedef struct countEdges_s{
	uint8_t   		ui8_length;		/**< length of the arrays */
	uint8_t*  		pui8_edgeSum;	/**< Array with the sum of edges */
	uint8_t*  		pui8_oldSig;	/**< Array for the old sig values */
	const char** 	pc_name;		/**< Array for names of the signals */
}countEdges_s;

/* ---------------------------------------------------------------------------------------------------*/
/*                                      Forward Declarations                                          */
/* ---------------------------------------------------------------------------------------------------*/

countEdges_handle_p CountEdges_Create(uint8_t ui8_length);
void CountEdges_Reset(countEdges_handle_p p_handle);
void CountEdges_Update(countEdges_handle_p p_handle, uint8_t ui8_sigNum, uint8_t ui8_sigValue, uint8_t ui8_method);
void CountEdges_Increment(countEdges_handle_p p_handle, uint8_t ui8_sigNum);
uint8_t CountEdges_Get(countEdges_handle_p p_handle, uint8_t ui8_sigNum);
uint8_t CountEdges_Bit2Num(uint32_t ui32_bit);

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
 * @brief	Create an instance of coundEdges
 *
 * @param	ui8_length --> Max. Count of the Signals to observe
 *
 * @return	countEdges_handle_p --> Handle to the coundEdges instance
 */
countEdges_handle_p CountEdges_Create(uint8_t ui8_length)
{
	countEdges_s* ps_coundEdges = (countEdges_s*) pvPortMalloc(sizeof(countEdges_s));
	if( ps_coundEdges != math_NULL )
	{
		ps_coundEdges->ui8_length   = ui8_length;
		ps_coundEdges->pui8_edgeSum = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*ui8_length);
		ps_coundEdges->pui8_oldSig  = (uint8_t*) pvPortMalloc(sizeof(uint8_t)*ui8_length);
		ps_coundEdges->pc_name  	= (const char**) pvPortMalloc(sizeof(const char*)*ui8_length);

		if((ps_coundEdges->pui8_edgeSum==math_NULL)||(ps_coundEdges->pui8_oldSig==math_NULL)||(ps_coundEdges->pc_name==math_NULL))
			return math_NULL;

		uint8_t i;
		for(i=0;i<ui8_length;++i)
		{
			ps_coundEdges->pui8_edgeSum[i]=0;
			ps_coundEdges->pui8_oldSig[i]=0;
			ps_coundEdges->pc_name[i]=0;
		}
	}
	return (countEdges_handle_p) ps_coundEdges;
}

/**
 * @brief	Store a constant name for a desired signal
 *
 * @param	p_handle --> Handle to the coundEdges instance
 * @param	ui8_sigNum --> Signal Number (from 0 to length-1)
 * @param	pc_name -->	Constant name for the Signal
 *
 * @return  void
 */
void CountEdges_SetName(countEdges_handle_p p_handle, uint8_t ui8_sigNum,const char* pc_name)
{
	countEdges_s* ps_coundEdges = (countEdges_s*) p_handle;
	ps_coundEdges->pc_name[ui8_sigNum]=pc_name;
}

/**
 * @brief	Return the desired constant name of a signal
 *
 * @param	p_handle --> Handle to the coundEdges instance
 * @param	ui8_sigNum --> Signal Number (from 0 to length-1)
 *
 * @return	const char* --> The desired constant name of a signal
 */
const char* CountEdges_GetName(countEdges_handle_p p_handle, uint8_t ui8_sigNum)
{
	countEdges_s* ps_coundEdges = (countEdges_s*) p_handle;
	return ps_coundEdges->pc_name[ui8_sigNum];
}

/**
 * @brief	Reset the edge count of all signals the stored old value will be reset, too.
 *
 * @param	p_handle --> Handle to the coundEdges instance
 *
 * @return  void
 */
void CountEdges_Reset(countEdges_handle_p p_handle)
{
	countEdges_s* ps_coundEdges = (countEdges_s*) p_handle;
	uint8_t i;
	for(i=0;i<ps_coundEdges->ui8_length;++i)
	{
		ps_coundEdges->pui8_edgeSum[i]=0;
		ps_coundEdges->pui8_oldSig[i]=0;
	}
}

/**
 * @brief	Increment the counter if the desired event happens.
 *
 * @details
 * This method should be called periodically to update the count of a desired edge.
 * If the ui8_method and the ui8_sigValue value match to each other,
 * the desired ui8_sigNum will be incremented. e.g. if there is a rising edge -> increment, else do nothing
 *
 * @param	p_handle --> Handle to the coundEdges instance
 * @param	ui8_sigNum --> Signal number (from 0 to length-1)
 * @param	ui8_sigValue --> Signal value  (0 or 1)
 * @param	ui8_method --> See defines in cound_edges.h
 *
 * @return  void
 */
void CountEdges_Update(countEdges_handle_p p_handle, uint8_t ui8_sigNum, uint8_t ui8_sigValue, uint8_t ui8_method)
{
	countEdges_s* ps_edgeCount = (countEdges_s*) p_handle;

	if( countEdges_METHOD_RISING & ui8_method )
		if((ps_edgeCount->pui8_oldSig[ui8_sigNum]==0) && (ui8_sigValue==1))
			ps_edgeCount->pui8_edgeSum[ui8_sigNum]++;

	if( countEdges_METHOD_FALLING & ui8_method )
		if((ps_edgeCount->pui8_oldSig[ui8_sigNum]==1) && (ui8_sigValue==0))
			ps_edgeCount->pui8_edgeSum[ui8_sigNum]++;

	ps_edgeCount->pui8_oldSig[ui8_sigNum]=ui8_sigValue;
}

/**
 * @brief	Increment the desired edge counter
 *
 * @param	p_handle --> Handle to the coundEdges instance
 * @param	ui8_sigNum --> Signal Number (from 0 to length-1)
 *
 * @return  void
 */
void CountEdges_Increment(countEdges_handle_p p_handle, uint8_t ui8_sigNum)
{
	countEdges_s* ps_edgeCount = (countEdges_s*) p_handle;
	ps_edgeCount->pui8_edgeSum[ui8_sigNum]++;
}


/**
 * @brief	Get the edge count of the desired signal
 *
 * @param	p_handle --> Handle to the coundEdges instance
 * @param	ui8_bitNum --> The number of the bit, which is set (see CountEdges_Bit2Num)
 *
 * @return  uint8_t --> Sum of count edges for the desired signal
 */
uint8_t CountEdges_Get(countEdges_handle_p p_handle, uint8_t ui8_bitNum)
{
	countEdges_s* ps_bitSum = (countEdges_s*) p_handle;
	return ps_bitSum->pui8_edgeSum[ui8_bitNum];
}

/**
 * @brief	Convert bit to number
 *
 * @details
 * bitDefine: 0100 -> return 2\n
 * bitDefine: 0001 -> return 0
 *
 * @param	ui32_bit --> A bit define
 *
 * @return	uint8_t --> The number of the bit, which is set
 */
uint8_t CountEdges_Bit2Num(uint32_t ui32_bit)
{
	uint8_t i=0;
	while(ui32_bit!=(1 << i))
		i++;
	return i;
}

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
