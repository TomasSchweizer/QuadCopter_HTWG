/*===================================================================================================*/
/*  workload.c                                                                                       */
/*===================================================================================================*/

/*
*   file   workload.c
*
*   brief  Implementation of functions to estimate workload for desired tasks/transactions
*
*   @details
*   Used to create workload instances to estimate desired workloads
*
*   <table>
*   <tr><th>Date            <th>Author              <th>Notes
*   <tr><td>21/05/2016      <td>Tobias Grimm        <td>Implementation & last modifications through MAs
*   <tr><td>31/01/2021      <td>Tomas Schweizer     <td>Code clean up & Doxygen
*   </table>
*   \n
*
*   Sources:
*
*/
/*====================================================================================================*/

/* ---------------------------------------------------------------------------------------------------*/
/*                                     Include File Definitions                                       */
/* ---------------------------------------------------------------------------------------------------*/

// Standard libraries
#include <stdbool.h>
#include <stdint.h>

//  Hardware specific libraries
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"

// Setup
#include "peripheral_setup.h"
#include "prioritys.h"

// Drivers
#include  "workload.h"
#include "qc_math.h"
#include "display_driver.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"


#if ( setup_DEV_WORKLOAD_CALC ) || DOXYGEN
    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local defines                                              */
    /* -----------------------------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------------------------------*/
    /*                                     Local Type Definitions                                     */
    /* -----------------------------------------------------------------------------------------------*/

    /// Struct for a workload instance
	typedef struct workload_s{
		uint16_t  		ui16_start;		/**< time start [us] */
		uint16_t  		ui16_storage;	/**< time storage [us] */
		uint16_t  		ui16_workload;	/**< average workload [us] */
		const char*		pc_name;		/**< name */
		struct workload_s*  ps_last; 	/**< pointer to the next element (linked List) */
	}workload_s;

	/* ------------------------------------------------------------------------------------------------*/
    /*                                      Forward Declarations                                       */
    /* ------------------------------------------------------------------------------------------------*/

    void vApplicationIdleHook( void );
    void Workload_TaskSwitchNotify( void );
    void Workload_TimeSampelIntHandler( void );

	/* ---------------------------------------------------------------------------------------------------*/
	/*                                      Global Variables                                              */
	/* ---------------------------------------------------------------------------------------------------*/


	/**
	 * @brief	Every Task has a field index (see priority.h)
	 */
	volatile uint16_t gui16_tasksTimeShareMemory[priority_NUM_COUNT];

	/**
	 * @brief	Array with handles to the different tasks.
	 *
	 * @details
	 * Every Task stores his handle under the right NUM (with the HIDE_Prio_StoreTaskHandle function)
	 * make sure, that the Idle Task and Demon Task do that too (in FreeRTOS source files)
	 */
	TaskHandle_t  g_workload_taskHandles[priority_NUM_COUNT];

	/* ---------------------------------------------------------------------------------------------------*/
    /*                                      Local Variables                                               */
    /* ---------------------------------------------------------------------------------------------------*/

	static uint16_t ui16_taskStartTime;
	static volatile uint16_t aui16_tasksTimeShareCalc[priority_NUM_COUNT];
	static workload_s* ps_last = 0;		// linked list to walk backwards through all workload_s instances


	/* -----------------------------------------------------------------------------------------------*/
    /*                                      Procedure Definitions                                     */
    /* -----------------------------------------------------------------------------------------------*/

	/**
	 * @brief	Create an instance of a workload estimator.
	 *
	 * @details
	 * This should be performed before scheduler starts, e.g. in the init of a driver or task
	 *
	 * @param	p_handle --> Handle to the workload instance to be created
	 * @param	pc_name --> Name for the workload estimation instance
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function set setup_DEV_WORKLOAD_CALC in qc_setup.h
	 */
	void HIDE_Workload_EstimateCreate(workload_handle_p* p_handle, const char* pc_name)
	{
		workload_s* ps_work = (workload_s*) pvPortMalloc(sizeof(workload_s));
		if( ps_work != math_NULL )
		{
			ps_work->ui16_start   	= 0;
			ps_work->ui16_storage	= 0;
			ps_work->ui16_workload  = 0;
			ps_work->ps_last		= ps_last;
			ps_work->pc_name		= pc_name;
			ps_last=ps_work;					// linked list
		}
		*p_handle = (workload_handle_p) ps_work;
	}

	/**
	 * @brief	Start time measurement for a workload estimation
	 *
	 * @details
	 * Keep in mind, that the measurement estimation can become bad,
	 * if the scheduler switches to another task or ISR happens, while the measurement is running.
	 *
	 *
	 * @param	p_handle --> Pointer to handle of the workload instance
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function set setup_DEV_WORKLOAD_CALC in qc_setup.h
	 */
	void HIDE_Workload_EstimateStart(workload_handle_p p_handle)
	{
		workload_s* ps_work = (workload_s*) p_handle;
		ps_work->ui16_start=ROM_TimerValueGet(periph_WORKLOAD_TIMER_BASE, periph_WORKLOAD_TIMER_MODULE);
	}

	/**
	 * @brief	Stop time measurement for a workload estimation
	 *
	 * @details
	 * Keep in mind, that the measurement estimation can become bad,
	 * if scheduler switches to another task or ISR happens, while the measurement is running
	 *
	 * @param	p_handle --> Pointer to handle of the workload instance
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function set setup_DEV_WORKLOAD_CALC in qc_setup.h
	 */
	void HIDE_Workload_EstimateStop(workload_handle_p p_handle)
	{
		workload_s* ps_work = (workload_s*) p_handle;
		ps_work->ui16_storage+=ps_work->ui16_start-ROM_TimerValueGet(periph_WORKLOAD_TIMER_BASE, periph_WORKLOAD_TIMER_MODULE);
	}

	/**
	 * @brief	Store all workloads and reset the storages of all workload instances
	 *
	 * @return  void
	 */
	static void EstimateEndAll(void)
	{
		if(ps_last!=0)
		{
			workload_s* ps_work=ps_last;
			uint8_t flag=1;
			while(flag)
			{
				ps_work->ui16_workload=ps_work->ui16_storage;
				ps_work->ui16_storage=0;
				if(ps_work->ps_last==0)		// end of linked list?
					flag=0;
				else
					ps_work=ps_work->ps_last;
			}
		}

	}

	/**
	 * @brief	Store the taskHandle under the TaskNumber
	 *
	 * @param	ui8_numTask --> The unique Task Number (see prioritys.h)
	 * @param	x_taskHandle --> The taskHandle to store
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function set setup_DEV_WORKLOAD_CALC in qc_setup.h
	 */
	void HIDE_Workload_StoreTaskHandle(uint8_t ui8_numTask,TaskHandle_t x_taskHandle)
	{
		g_workload_taskHandles[ui8_numTask] = x_taskHandle;
		vTaskSetTaskNumber(x_taskHandle,ui8_numTask);
	}

	/**
	 * @brief	Draw info about Workload on the display
	 *
	 * @return  void
	 *
	 * @note	To enable this HIDE function set setup_DEV_WORKLOAD_CALC in qc_setup.h
	 */
	void HIDE_Workload_DrawDisplay(void)
	{
		const uint8_t drawHight = 6;
		u8g_SetFont(&gs_display, u8g_font_04b_03r);		// u8g_font_unifont
		int8_t i;
		uint16_t hight=drawHight-1;
		for(i=priority_NUM_COUNT-1;i>=0;--i)
			if(g_workload_taskHandles[i]!=0)
			{
				u8g_DrawStr(&gs_display,  0, hight, u8g_u16toa(uxTaskPriorityGet(g_workload_taskHandles[i]),1));
				u8g_DrawStr(&gs_display,  7, hight,  pcTaskGetTaskName(g_workload_taskHandles[i]));
				u8g_DrawStr(&gs_display,  24, hight, u8g_u16toa(gui16_tasksTimeShareMemory[i],5));
				hight+=drawHight;
			}

		// draw workload instances information
		if(ps_last!=0)
		{
			workload_s* ps_work=ps_last;
			uint8_t flag=1;
			while(flag)
			{
				u8g_DrawStr(&gs_display,  0, hight,  ps_work->pc_name);
				u8g_DrawStr(&gs_display,  24, hight, u8g_u16toa(ps_work->ui16_workload,5));
				hight+=drawHight;

				if(ps_work->ps_last==0)		// end of linked list?
					flag=0;
				else
					ps_work=ps_work->ps_last;
			}
		}
	}

	/**
	 * @brief	Saves the time share of the Tasks every time the alive timer flows over in an separate array.
	 *
	 * @return  void
	 */
	void Workload_TimeSampelIntHandler( void )
	{
		// Clear the timer interrupt flag.
		ROM_TimerIntClear(periph_WORKLOAD_TIMER_BASE, TIMER_TIMA_TIMEOUT);

		//Workload_TaskSwitchNotify();

		int8_t i8_i;
		for(i8_i = 0; i8_i < priority_NUM_COUNT; i8_i++ )
		{
			gui16_tasksTimeShareMemory[i8_i] = aui16_tasksTimeShareCalc[i8_i];
			aui16_tasksTimeShareCalc[i8_i] = 0;
		}
		EstimateEndAll();
	}
#endif

#if ( setup_DEV_WORKLOAD_CALC || setup_DEV_WORKLOAD_LED ) || DOXYGEN

	/* ---------------------------------------------------------------------------------------------------*/
	/*                                      Defines                                                       */
	/* ---------------------------------------------------------------------------------------------------*/

	// Define the desired peripheral setup (see peripheral_setup.h)
	#if ( GPIO_PORTF_BASE == (periph_WORKLOAD_ALIVE_LED&periph_MASK_PORT) )
		#define ALIVE_LED_PORT_SYSCTL		SYSCTL_PERIPH_GPIOF
	#else
		#error	ERROR implement the defines above here
	#endif
	#define ALIVE_LED_PORT				(periph_WORKLOAD_ALIVE_LED&periph_MASK_PORT)
	#define ALIVE_LED_PIN				(periph_WORKLOAD_ALIVE_LED&periph_MASK_PIN)

	#if ( periph_WORKLOAD_TIMER_BASE == TIMER2_BASE )
		#define WORKLOAD_SYSCTL_PERIPH_TIMER		SYSCTL_PERIPH_TIMER2
	#else
		#error	ERROR implement the defines above here
	#endif

	/**
	 * @brief	Does work with the smallest priority
	 */
	void vApplicationIdleHook( void )
	{
		#if ( setup_DEV_WORKLOAD_LED )
			ROM_GPIOPinWrite(ALIVE_LED_PORT, ALIVE_LED_PIN, ALIVE_LED_PIN);
		#endif
	}

	/**
	 * @brief	Initializes peripherals for workload calculation
	 *
	 * @note	To enable this HIDE function set setup_DEV_WORKLOAD_CALC or setup_DEV_WORKLOAD_LED in qc_setup.h
	 */
	void HIDE_Workload_Init( void )
	{
		// Activates Timer
		ROM_SysCtlPeripheralEnable(WORKLOAD_SYSCTL_PERIPH_TIMER);
		// Disable Timer for configuration.
		ROM_TimerDisable(periph_WORKLOAD_TIMER_BASE,periph_WORKLOAD_TIMER_MODULE);

		// Timer2A counts from 0xffff to 0x000 down and starts again.
		ROM_TimerConfigure(periph_WORKLOAD_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);

		// prescaler 80 for 80Mhz -> 1us

		ROM_TimerPrescaleSet(periph_WORKLOAD_TIMER_BASE,periph_WORKLOAD_TIMER_MODULE,80);

		// If workload calculation is set then interruts for timer are activated
		#if ( setup_DEV_WORKLOAD_CALC )
			ROM_TimerIntEnable(periph_WORKLOAD_TIMER_BASE, TIMER_TIMA_TIMEOUT );
			ROM_IntEnable(periph_WORKLOAD_TIMER_INT);
		#endif

		ROM_TimerEnable(periph_WORKLOAD_TIMER_BASE, periph_WORKLOAD_TIMER_MODULE);

		// Init Alive LED
		#if ( setup_DEV_WORKLOAD_LED )
			ROM_SysCtlPeripheralEnable(ALIVE_LED_PORT_SYSCTL);
			ROM_GPIOPinTypeGPIOOutput(ALIVE_LED_PORT, ALIVE_LED_PIN);
		#endif
	}

	/**
	 * @brief	Measures the task time shares and adds them up (for each task separately) until the alive time interrupt is triggered.
	 */
	void Workload_TaskSwitchNotify( void )
	{
		#if ( setup_DEV_WORKLOAD_LED )
			ROM_GPIOPinWrite(ALIVE_LED_PORT, ALIVE_LED_PIN, 0);
		#endif

		#if ( setup_DEV_WORKLOAD_CALC )
			uint16_t ui16_taskEndTime = ROM_TimerValueGet(periph_WORKLOAD_TIMER_BASE, periph_WORKLOAD_TIMER_MODULE);
			aui16_tasksTimeShareCalc[uxTaskGetCurrentTaskNumber()] += (ui16_taskStartTime - ui16_taskEndTime); // Timer Counts down.

			//ui16_lastTaskNumber = uxTaskGetCurrentTaskNumber();
			ui16_taskStartTime = ui16_taskEndTime;
		#endif
	}


#endif

/*====================================================================================================*/
/* End of file                                                                                        */
/*====================================================================================================*/
