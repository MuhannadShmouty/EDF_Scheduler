/******************************************************************************************************
 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------*/
/*      
 *  \file		main.c
 *  \brief 
 *				 	RTOS Application to test the EDF Scheduler based on FreeRTOS
 * 
 *  \details
 *					The application consists of six tasks, each has different deadline and execution time.
 *****************************************************************************************************/
 
/******************************************************************************************************
 *  INCLUDES
 *****************************************************************************************************/
#include <LPC21xx.H>     
#include <stdlib.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "serial.h"
#include "GPIO.h"
#include "GPIO_cfg.h"
              

/******************************************************************************************************
 *  LOCAL MACROS CONSTANT\FUNCTIONS
 *****************************************************************************************************/
/* Constants to setup I/O and processor. */
#define mainTX_ENABLE		( ( unsigned long ) 0x00010000 )	/* UART1. */
#define mainRX_ENABLE		( ( unsigned long ) 0x00040000 ) 	/* UART1. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )
#define mainLED_TO_OUTPUT	( ( unsigned long ) 0xff0000 )

/*UART*/
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

/* Tasks Periods */
#define Button_1_Monitor_TaskPeriod 		50
#define Button_2_Monitor_TaskPeriod 		50
#define Periodic_Transmitter_TaskPeriod 100
#define Uart_Receiver_TaskPeriod 				20
#define Load_1_Simulation_TaskPeriod 		10
#define Load_2_Simulation_TaskPeriod 		100

/******************************************************************************************************
 *  LOCAL DATA
 *****************************************************************************************************/
/* Total CPU Load Variables' Calculation */
int tasks_total_time=0, task_in_time=0, task_out_time=0;
int system_time=0;
float cpu_load=0;

/* Tasks variables to be used as indicators in the Logic Analyzer (LA prefix) */
volatile bool LA_Button1_Task = false;
volatile bool LA_Button2_Task = false;
volatile bool LA_PeriodicTask = false;
volatile bool LA_UART_Task = false;
volatile bool LA_Load1_Task = false;
volatile bool LA_Load2_Task = false;
volatile bool LA_Tick = false;
volatile bool LA_IDLE = false;

/* Queues for inter-task communication */
QueueHandle_t msgQueue;
char TxBuff[40];


/* Task Handlers */
TaskHandle_t Button_1_Monitor_Handler;
TaskHandle_t Button_2_Monitor_Handler;
TaskHandle_t Periodic_Transmitter_Handler;
TaskHandle_t Uart_Receiver_Handler;
TaskHandle_t Load_1_Simulation_Handler;
TaskHandle_t Load_2_Simulation_Handler;

/******************************************************************************************************
 *  LOCAL FUNCTION PROTOTYPES
 *****************************************************************************************************/
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

/*Tasks' prototypes*/
void Button_1_Monitor (void * pvParameters);
void Button_2_Monitor (void * pvParameters);
void Periodic_Transmitter (void * pvParameters);
void Uart_Receiver (void * pvParameters);
void Load_1_Simulation (void * pvParameters);
void Load_2_Simulation (void * pvParameters);

/******************************************************************************************************
 *  GLOBAL FUNCTIONS
 *****************************************************************************************************/
int main( void )

{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();		
	msgQueue = xQueueCreate(5, sizeof(TxBuff));

	
	xTaskPeriodicCreate(
                    Button_1_Monitor,         /* Function that implements the task. */
                    "Button_1_Monitor",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1,
                    Button_1_Monitor_Handler,
										Button_1_Monitor_TaskPeriod);

	xTaskPeriodicCreate(
                    Button_2_Monitor,         /* Function that implements the task. */
                    "Button_2_Monitor",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1,
                    Button_2_Monitor_Handler,
										Button_2_Monitor_TaskPeriod); 
	
	xTaskPeriodicCreate(
                    Periodic_Transmitter,         /* Function that implements the task. */
                    "Periodic_Transmitter",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1,
                    Periodic_Transmitter_Handler,
										Periodic_Transmitter_TaskPeriod);
										
  xTaskPeriodicCreate(
                    Uart_Receiver,         /* Function that implements the task. */
                    "Uart_Receiver",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1,
                    Uart_Receiver_Handler,
										Uart_Receiver_TaskPeriod);
										
	xTaskPeriodicCreate(
                    Load_1_Simulation,         /* Function that implements the task. */
                    "Load_1_Simulation",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1,
                    Load_1_Simulation_Handler,
										Load_1_Simulation_TaskPeriod);
										
	xTaskPeriodicCreate(
                    Load_2_Simulation,         /* Function that implements the task. */
                    "Load_2_Simulation",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1,
                    Load_2_Simulation_Handler,
										Load_2_Simulation_TaskPeriod);	


	/* Now all the tasks have been started - start the scheduler.
	
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/******************************************************************************************************
 *  LOCAL FUNCTIONS
 *****************************************************************************************************/

void Button_1_Monitor (void * pvParameters)
{
	LA_Button1_Task = true;
	for(;;)
	{
		TickType_t lastWakeTime = xTaskGetTickCount();
		
		if(GPIO_read(PORT_0, PIN0) == PIN_IS_HIGH)
		{
			memset(TxBuff, '\0', sizeof(TxBuff));		// Clear the Tx Buffer
			sprintf(TxBuff, "Falling edge on Button 1\n");
			xQueueSend(msgQueue, (void *)TxBuff, (TickType_t) 0);
		}
		else
		{
			memset(TxBuff, '\0', sizeof(TxBuff));		// Clear the Tx Buffer
			sprintf(TxBuff, "Rising Edge on button 1\n");
			xQueueSend(msgQueue, (void *)TxBuff, (TickType_t) 0);
			
		}
		
		vTaskDelayUntil(&lastWakeTime, Button_1_Monitor_TaskPeriod);
		LA_Button1_Task = false;
	}
}

void Button_2_Monitor (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t) 2 );
	for(;;)
	{
		TickType_t lastWakeTime = xTaskGetTickCount();
		
		
		if(GPIO_read(PORT_1, PIN5) == PIN_IS_HIGH)
		{
			memset(TxBuff, '\0', sizeof(TxBuff));		// Clear the Tx Buffer
			sprintf(TxBuff, "Falling edge on Button 2\n");
			xQueueSend(msgQueue, (void *)TxBuff, (TickType_t) 0);
		}
		else
		{
			memset(TxBuff, '\0', sizeof(TxBuff));		// Clear the Tx Buffer
			sprintf(TxBuff, "Rising edge on Button 2\n");
			xQueueSend(msgQueue, (void *)TxBuff, (TickType_t) 0);
		}
		
		vTaskDelayUntil(&lastWakeTime, Button_2_Monitor_TaskPeriod);
		
	}
}

void Periodic_Transmitter (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t) 3 );
	for(;;)
	{
		TickType_t lastWakeTime = xTaskGetTickCount();
		
		//Sending a random string every 100ms as stated in the project
		memset(TxBuff, '\0', sizeof(TxBuff));
		sprintf(TxBuff, "Periodic Message\n");
		xQueueSend(msgQueue, (void *)TxBuff, (TickType_t) 0);
		
		
		vTaskDelayUntil(&lastWakeTime, Periodic_Transmitter_TaskPeriod);
		
	}
}

void Uart_Receiver (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t) 4 );
	char RxBuff[40];
	
	for(;;)
	{
		TickType_t lastWakeTime = xTaskGetTickCount();
		
		if (msgQueue != 0)
		{
			xQueueReceive(msgQueue, (void *)RxBuff, (TickType_t) 5);
			vSerialPutString((const signed char *)RxBuff, 40);
		}
		
		vTaskDelayUntil(&lastWakeTime, Uart_Receiver_TaskPeriod);
		
	}
}

void Load_1_Simulation (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t) 5 );
	for(;;)
	{	
		TickType_t lastWakeTime = xTaskGetTickCount();
		int i;
		
		for(i=0; i<33500; i++)
		{
			i=i;
		}
		
		vTaskDelayUntil(&lastWakeTime, Load_1_Simulation_TaskPeriod);
		
	}
}

void Load_2_Simulation (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, (TaskHookFunction_t) 6 );
	for(;;)
	{	
		TickType_t lastWakeTime = xTaskGetTickCount();
		int i;
		
		
		for(i=0; i<78950; i++)
		{
			i=i;
		}
		
		vTaskDelayUntil(&lastWakeTime, Load_2_Simulation_TaskPeriod);
		
		
	}
}

/*-----------------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	
	LA_IDLE = true;
	
}

void vApplicationTickHook( void )
{
	LA_Tick = true;
	LA_Tick = false;
}

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */
	
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);
	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
	
	configTimer1();
	GPIO_init();
}
/*-----------------------------------------------------------*/
/******************************************************************************************************
 *  END OF FILE:    main.c
 *****************************************************************************************************/