/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "queue.h"
/* Peripheral includes. */
#include "serial_new.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

extern BaseType_t xTaskPeriodicCreate( TaskFunction_t pxTaskCode,
                            const char * const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                            const configSTACK_DEPTH_TYPE usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pxCreatedTask , TickType_t period);
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
					
QueueHandle_t xQueue ;														
														

void Button1 ( void *pvParameter){
	static int CurrentState ;
	static int PrevState    ;
	const char * h = "b1high";
	const char * l = "b1low";
	for(;;){
	CurrentState = GPIO_read(PORT_0,PIN0);
	if (CurrentState != PrevState) 
		{
			PrevState = CurrentState ;
			if (CurrentState) xQueueSend( xQueue,( void * ) &h,( TickType_t ) 100 );
			else xQueueSend( xQueue,( void * ) &l,( TickType_t ) 100 );
		}
	else 
		{
			/** **/
		}}
	
}

void Button2 ( void *pvParameter){
	static int CurrentState ;
	static int PrevState    ;
	const char * h = "b2high";
	const char * l = "b2low";
	for(;;){
	CurrentState = GPIO_read(PORT_0,PIN1);
	if (CurrentState != PrevState) 
		{
			PrevState = CurrentState ;
			if (CurrentState) xQueueSend( xQueue,( void * ) &h,( TickType_t ) 100 );
			else xQueueSend( xQueue,( void * ) &l,( TickType_t ) 100 );
		}
	else 
		{
			/** **/
		}
	}
	
}

void transmitter (void *pvParameter)
{
	const char * s = "string"; 
	for(;;){
	xQueueSend( xQueue,( void * ) &s,( TickType_t ) 100 );

	}
	}



void receiver (void *pvParameter)
{
	
	char string[10];
	for(;;){
	xQueueReceive( xQueue,&string,( TickType_t ) 10 );
	vSerialPutString((const signed char *)&string,10);
	}

}



void TestGPIO(void *pvParameter){
	for(;;){
	GPIO_write(PORT_0,PIN2,!GPIO_read(PORT_0,PIN2));
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
  xSerialPortInitMinimal(ser9600);
	

	//xQueue=xQueueCreate( 10, sizeof( char ) ) ;
	//xTaskPeriodicCreate( Button1, ( const char * ) "Button 1", configMINIMAL_STACK_SIZE, NULL,1, NULL, 50 );
	//xTaskPeriodicCreate( Button2, ( const char * ) "Button 2", configMINIMAL_STACK_SIZE, NULL,1, NULL, 50 );
	//xTaskPeriodicCreate( transmitter, ( const char * ) "transmitter", configMINIMAL_STACK_SIZE, NULL,1, NULL, 100 );
	//xTaskPeriodicCreate( receiver , ( const char * ) "receiver", configMINIMAL_STACK_SIZE, NULL,1, NULL, 20 );
	
	#if(configUSE_EDF_SCHEDULER == 1)
	xTaskPeriodicCreate( TestGPIO , ( const char * ) "Test", configMINIMAL_STACK_SIZE, NULL,1, NULL, 2 );
	#else
	xTaskCreate( TestGPIO , ( const char * ) "Test", configMINIMAL_STACK_SIZE, NULL,1, NULL );
	#endif
	
    /* Create Tasks here */
	

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

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/
#if (configUSE_TICK_HOOK == 1) 
void vApplicationTickHook( void ){
		GPIO_write(PORT_0,PIN3,!GPIO_read(PORT_0,PIN3));
	//GPIO_write(PORT_0,PIN2,!GPIO_read(PORT_0,PIN3));
}

#endif


#if (configUSE_IDLE_HOOK == 1) 
void vApplicationIdleHook( void ){
GPIO_write(PORT_0,PIN4,!GPIO_read(PORT_0,PIN4));
}

#endif


