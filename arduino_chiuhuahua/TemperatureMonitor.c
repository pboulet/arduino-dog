/*!	\file TemperatureMonitor.c
 * \author Patrice Boulet, Ladan Maxamud
 * \date February 2nd, 2016
 * \brief Temperature Monitor Module
 * \details Defines and manage task execution to read temperatures
 * \from the TemperatureReader module and to display them with
 * \the LED and LCD modules.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <avr/io.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* serial interface include file. */
#include "usartserial.h"

/* Temperature Reader Module include file. */
#include "TemperatureReader.h"


/*-----------------------------------------------------------*/
static void TaskScheduler(void*);
void ReadTemperatures(uint8_t*);
/*-----------------------------------------------------------*/

/* Main program loop */
int usartfd;

/** Main loop.  Creates tasks, set their order and start the FreeRTOS scheduler.
 * @return the status of the program when it ends
 */
int main(void)
{
    // turn on the serial port for debugging or for other USART reasons.
	usartfd = usartOpen( USART0_ID, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	usart_print_P(PSTR("\r\n\n\nChico: Initializing...\r\n")); // Ok, so we're alive...

	/* Initialize all modules before enabling interrupts. */
	InitTemperatureReader();
	//InitLED();
	//InitLCD();

	xTaskCreate(TaskScheduler,  (const portCHAR *)"Scheduler" , 256, NULL, 3,  NULL );

	usart_printf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

	vTaskStartScheduler();

	usart_print_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

/** Reads the temperatures from the robot's thermal array
 * sensor each 100ms, then displays the average temperature in
 * front of the robot on the LED and all temperatures readings on
 * the LCD.
 */
static void TaskScheduler(void* gvParameters) {
    TickType_t xLastWakeTime;							// keeps track of timing

	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	uint8_t *temperatures = malloc(sizeof(uint8_t)*9);

    while(1)
    {
    	ReadTemperatures(temperatures);
    	// displayLED();
    	// displayLCD();
		vTaskDelayUntil( &xLastWakeTime, ( 300 / portTICK_PERIOD_MS ) );
    }
}

/** Gets temperature readings from the TPA81 thermal array sensor.
 *  @param[temperatures] an array to hold the 9 temperatures to be read
 */
void ReadTemperatures(uint8_t *temperatures) {
	getTemperatureFromSensor(temperatures);
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    portCHAR *pcTaskName )
{
	while(1);
}

