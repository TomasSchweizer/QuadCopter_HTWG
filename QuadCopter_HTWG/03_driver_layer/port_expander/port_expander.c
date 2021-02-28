/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/* NOT USED AT THE MOMENT, THEREFORE EXCLUDED FROM BUILD. TO USE INCLUDE
 * BACK INTO BUILD
 */
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/************************************************************************
 *
 *	port_expander.c	  	Functions for Filters and Controllers
 *
 ************************************************************************
 *	Author: 			Tobias Grimm
 *
 ************************************************************************
 *  Module Description:	 Funktions for Filters and Controllers.
 *  					 (all implemented with floating point)
 *
 ************************************************************************
 *  Revision History:
 *
 *	10/10/2015(GrimmT): created
 *	25/02/2016(GrimmT): adapted to Coding Guidlines
 *
 ************************************************************************/


/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */
#include <stdio.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

//  Hardware Specific
#include "driverlib/gpio.h"

//  used drivers
#include "port_expander.h"


/* ------------------------------------------------------------ */
/*				Local Defines									*/
/* ------------------------------------------------------------ */

#define SER_SHIFT_REGISTER_PORT          0x00000001             // Serial Input shift register.
#define SER_SHIFT_REGISTER_PIN           0x00000001

#define SCK_SHIFT_REGISTER_PORT          0x00000001             // Input to push the conten of the shift register.
#define SCK_SHIFT_REGISTER_PIN           0x00000001

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */
typedef void (*hardwareType_fp)(uint32_t ui32_port);
typedef struct hardwareAssignment_s
{
	hardwareType_fp fp_hardwarePortsUpdate;
	uint8_t ui8_portNumber;
} hardwareAssignment_s;

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void Expander_Init();
void Expander_PinWrite(uint32_t ui32_port, uint8_t ui8_pins, uint8_t ui8_value);
void Expander_PinToggel(uint32_t ui32_port, uint8_t ui8_pins);
int32_t Expander_PinRead(uint32_t ui32_port, uint8_t ui8_pins);

void ExpanderPortsUpdate(uint32_t ui32_port);

static void ShiftRegisterPortsUpdate(uint32_t ui32_port);

static uint8_t PortCounter(hardwareType_fp fp_portType);
static uint8_t* PortOrderAssignment(hardwareType_fp fp_hardwareType);


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

#if expander_PORT_COUNT != 0
	static uint8_t ui8_ports[expander_PORT_COUNT];
#endif

hardwareAssignment_s hardwareAssignment[expander_PORT_COUNT];

uint8_t* pui8_shiftRegisterPortAssignment;




/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/***	ExpanderInit
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Expander Main Init function
**		Assigns the in port_expander.h defined expander Ports to the Hardware realization.(hardwareAssignment is an array of function pointers)
*/

void Expander_Init()
{
	// Hardware Assignment Table---------------------------------------------------------------------------------

	// Table to assign the hardware ports to digital ports. (Diffrent hardware realizations possible)
	// hardwareAssignment is an array conisting of structures that contain a functionhandle(to use the desired hardware)
	// and a Number(Portnumber of respective hardware realization).Portnumbers are starting whith 1.

	hardwareAssignment[expander_PORT_A] = (hardwareAssignment_s){ ShiftRegisterPortsUpdate, 1 };

	//-----------------------------------------------------------------------------------------------------------

	// Lists the digital ports in arrays according to there hardware realization and in order of the above defined Portnumbers.

	// For each hardware realization one array needs to be defined as local variable of type uint8_t*.
	// The array is then filled with the function PortOrderAssignment which needs the functionhandle of the corresponding hardware realization.

	pui8_shiftRegisterPortAssignment = PortOrderAssignment(ShiftRegisterPortsUpdate);

	//-----------------------------------------------------------------------------------------------------------
}

/***	PortOrderAssignment
**
**	Parameters:
**		hardwareType_fp  	The function pointer that handles the harware and indicates which ports are part of one hardware.
**
**	Return Value:
**		portAssignment      An array that contains the digital ports of the desierd hardware realization in order of the in Expander_Init defiend Portnumbers.
**
**	Errors:
**		none
**
**	Description:
**		Takes all ports of the hardware realization corresponding to the input pointer and puts them in an array.
**		The order of the ports is given by the Portnumbers deffined in the Hardware Assignment Table in Expander_Init.
*/

//------- Help function ------------------------------------------------------------------------

static uint8_t* PortOrderAssignment(hardwareType_fp fp_hardwareType)
{
	uint8_t ui8_portCount = PortCounter(fp_hardwareType);
	uint8_t *portAssignment = (uint8_t*)malloc(( 1 + ui8_portCount )*sizeof(uint8_t)); // reserves memory for array (first element is number of ports)
	if(portAssignment == NULL)
	{
		// Fehler !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// printf("Fehler !!!!!!");
	}
	else
	{
		portAssignment[0] = ui8_portCount; // First element tells how many elements are in the array after.

		uint8_t ui8_i;
		uint8_t ui8_j;

		for(ui8_i = 1; ui8_i <= ui8_portCount; ui8_i++)
		{
			for(ui8_j = 0; ui8_j <= expander_PORT_COUNT; ui8_j++)
			{
				if( ui8_i == hardwareAssignment[ui8_j].ui8_portNumber )
				{
					portAssignment[ui8_i] = ui8_j;
					break;
				}
			}
		}
	}
	return portAssignment;
}

/***	PortCounter
**
**	Parameters:
**		fp_portType      For every diffrent Harware realization of the ports the funcion pointer points to an diffrent function.
**		                 Therfore the pointer can be used to check wich port was deffined for wich hardware realization.
**
**	Return Value:
**		ui8_portCount    ui8_portCount is the number of defined ports that are realized with the type of the handed over function pointer.
**
**	Errors:
**		none
**
**	Description:
**		Takes a function pointer and counts the amount of the defined ports that are using that function pointer to handle the hardware.
*/

static uint8_t PortCounter(hardwareType_fp fp_portType)
{
	uint8_t ui8_portCount = 0;
	uint8_t ui8_i;
	for(ui8_i = 0; ui8_i > expander_PORT_COUNT; ui8_i++)
	{
		if(hardwareAssignment[ui8_i].fp_hardwarePortsUpdate == fp_portType)
			ui8_portCount++;
	}
	return ui8_portCount;
}

/***	ExpanderPinWrite
**
**	Parameters:
**		ui32_port  		8 bit digital Port to write on.
**		ui8_pins        Bitmask who defines wich bits to write on (1-> write ; 0 -> do nothing).
**		ui8_value       the value of the Bits that should be writen.
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Writes on the choosen 8 bit Port of the port expander.
**		The bits to writte on are decided through a 1 in the 8 bit paramter ui8_pins.
**		The 8 bit parameter ui8_value defines if 1 or 0 is written.
*/

//------- Hardware functions -------------------------------------------------------------------

void Expander_PinWrite(uint32_t ui32_port, uint8_t ui8_pins, uint8_t ui8_value)
{
	ui8_ports[ui32_port] = (ui8_pins & ui8_value) | (~ui8_pins & ui8_ports[ui32_port]);
}

/***	ExpanderPinToggel
**
**	Parameters:
**      ui32_port  	    8 bit digital Port to toggle
**		ui8_pins        Bitmask who defines wich bits to toggle (1-> toggle ; 0 -> do nothing)
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Toggles the choosen bits in a choosen 8 bit Port of the Portexpander.
**		The bits to toggle are given by the parameter ui8_pins.
**		
*/

void Expander_PinToggel(uint32_t ui32_port, uint8_t ui8_pins)
{
	ui8_ports[ui32_port] = (ui8_pins & (ui8_ports[ui32_port] ^ ui8_pins)) | (~ui8_pins & ui8_ports[ui32_port]);
}

/***	ExpanderPinToggel
**
**	Parameters:
**      ui32_port  	    8 bit digital Port to read
**		ui8_pins        Bitmask who defines wich bits to read (1-> read ; 0 -> 0)
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Reads the choosen bitsof a choosen 8 bit Port of the port expander.
**		The bits to read are given by the parameter ui8_pins.
**
*/

int32_t Expander_PinRead(uint32_t ui32_port, uint8_t ui8_pins)
{
	return ui8_ports[ui32_port] & ui8_pins;
}

/***	ExpanderPortsUpdate
**
**	Parameters:
**		ui32_port  	    8 bit hardware Port to update.
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Transfers the data of the choosen digital Port to its hardware counter-part.
**		For each diffrent hardware realization there is a function to handel the Task.
**		The Assignment to this functions is implementet with an array of function pointers(hardwareAssignment).
*/

void Expander_PortsUpdate(uint32_t ui32_port)
{
	hardwareAssignment[ui32_port].fp_hardwarePortsUpdate(ui32_port);
}


/***	ShiftRegisterPortsUpdate
**
**	Parameters:
**		ui32_port  		8 bit hardware Port to update.

**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Takes all digital ports what are associated whith the same hardeware and writtes the contens to the hardware counterparts.
*/

static void ShiftRegisterPortsUpdate(uint32_t ui32_port)
{

	uint8_t ui8_shiftRegisterPortCount = pui8_shiftRegisterPortAssignment[0]; // First element is length of folowing array.

	uint8_t ui8_targetPort ;
	int8_t i8_targetBit;

	int8_t i8_i;
	for(i8_i = (8 * ui8_shiftRegisterPortCount) - 1; i8_i >= 0 ; i8_i-- ) //------------------------------------------------ for---------------
	{
		ui8_targetPort = i8_i / 8 + 1; // shifted by one because pui8_shiftRegisterPortAssignment begins by 1 not 0.
		i8_targetBit   = i8_i % 8;

		uint8_t ui8_shiftedPort;

		uint8_t ui8_bitPositionCheck = SER_SHIFT_REGISTER_PIN;
		int8_t i8_serBitPosition = 0;

		while(ui8_bitPositionCheck > 1);
		{
			if(ui8_bitPositionCheck % 2 != 0) // Bitmaske darf nur ein bit mit 1 haben (SER_Port).
			{
				// Fehler !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			}
			ui8_bitPositionCheck = ui8_bitPositionCheck / 2;
			i8_serBitPosition++; // counts the x of the input 2^(x) (x = bit position).
		}
		int8_t i8_shift = i8_serBitPosition - i8_targetBit; // i8_serBitPosition is the position the i8_targetBit(bit to write) has to be schifted to.
		                                                    // i8_serBitPosition(soll) = i8_targetBit(ist) + i8_shift(correction)
		if(i8_shift < 0)
			ui8_shiftedPort = ui8_ports[pui8_shiftRegisterPortAssignment[ui8_targetPort]] << -i8_shift; // shift to left  (- * - = +)
		else
			ui8_shiftedPort = ui8_ports[pui8_shiftRegisterPortAssignment[ui8_targetPort]] >>  i8_shift; // shift to right

		GPIOPinWrite(SER_SHIFT_REGISTER_PORT, SER_SHIFT_REGISTER_PIN, ui8_shiftedPort); // Writtes all bits of the digital shift register ports to the harware ports begining with the last.

		GPIOPinWrite(SCK_SHIFT_REGISTER_PORT, SCK_SHIFT_REGISTER_PIN, 255); //  set  SCK = 1
		GPIOPinWrite(SCK_SHIFT_REGISTER_PORT, SCK_SHIFT_REGISTER_PIN, 0);   // clear SCK = 0   =>  0-1-0 pulse to shift the shift register.

	}  // --------------------------------------------------------------------------------------------------------------- end --------------------------

}
