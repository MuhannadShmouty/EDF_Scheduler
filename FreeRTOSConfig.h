/*
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <lpc21xx.h>
#include "GPIO.h"
#include <stdbool.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/*
 * Configuration Usage Macros
*/
#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			1
#define configUSE_TRACE_FACILITY	0
#define configUSE_16_BIT_TICKS		0
#define configUSE_EDF_SCHEDULER   1
#define configUSE_APPLICATION_TASK_TAG   1
#define configUSE_TIME_SLICING           0
/*------------------------------------------------------------------*/

/*
 * Configuration Setters Macros
*/
#define configCPU_CLOCK_HZ			( ( unsigned long ) 60000000 )	/* =12.0MHz xtal multiplied by 5 using the PLL. */
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES		( 4 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 90 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) 13 * 1024 )
#define configMAX_TASK_NAME_LEN		( 8 )
#define configIDLE_SHOULD_YIELD		1
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configQUEUE_REGISTRY_SIZE 	0
#define IdleTaskMaxPeriod              150
/*------------------------------------------------------------------*/

/*
 * 	Trace Hooks
*/
extern int tasks_total_time, task_in_time, task_out_time;
extern int system_time;
extern float cpu_load;

extern volatile bool LA_Button1_Task;
extern volatile bool LA_Button2_Task;
extern volatile bool LA_PeriodicTask;
extern volatile bool LA_UART_Task;
extern volatile bool LA_Load1_Task;
extern volatile bool LA_Load2_Task;
extern volatile bool LA_Tick;
extern volatile bool LA_IDLE;

#define traceTASK_SWITCHED_OUT()   do\
																	{\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) != 0 )\
																	{\
																		task_out_time = T1TC;\
																		tasks_total_time += (task_out_time - task_in_time);\
																	}\
																	system_time = T1TC;\
																	cpu_load = (tasks_total_time/ (float)system_time)*100;\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 1 ) {LA_Button1_Task = false;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 2 ) {LA_Button2_Task = false;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 3 ) {LA_PeriodicTask = false;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 4 ) {LA_UART_Task    = false;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 5 ) {LA_Load1_Task   = false;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 6 ) {LA_Load2_Task   = false;}\
																	}while(0);
#define traceTASK_SWITCHED_IN()	do\
																	{\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) != 0 )\
																	{\
																		task_in_time = T1TC;\
																		LA_IDLE = false;\
																	}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 1 ) {LA_Button1_Task = true;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 2 ) {LA_Button2_Task = true;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 3 ) {LA_PeriodicTask = true;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 4 ) {LA_UART_Task    = true;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 5 ) {LA_Load1_Task   = true;}\
																	if( ((int)(pxCurrentTCB->pxTaskTag)) == 6 ) {LA_Load2_Task   = true;}\
																	}while(0);
/*------------------------------------------------------------------*/


/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1
/*------------------------------------------------------------------*/



#endif /* FREERTOS_CONFIG_H */