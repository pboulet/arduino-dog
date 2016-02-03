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

static void TaskGetTemperatures(uint8_t*);
/*-----------------------------------------------------------*/

/* Main program loop */
int usartfd;

/** Main loop.  Creates tasks, set their order and start the FreeRTOS scheduler.
 * @return the status of the program when it ends
 */
int main(void)
{
	uint8_t *temperatures = malloc(sizeof(uint8_t)*10);

    // turn on the serial port for debugging or for other USART reasons.
	usartfd = usartOpen( USART0_ID, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	usart_print_P(PSTR("\r\n\n\nChico: Initializing...\r\n")); // Ok, so we're alive...

	xTaskCreate(TaskGetTemperatures,  (const portCHAR *)"ReadTemperature" , 256, temperatures, 3,  NULL );

	usart_printf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

	vTaskStartScheduler();

	usart_print_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

/** Gets temperature readings from the TPA81 thermal array sensor
 *  each 400 ticks.
 *  @param[temperatures] an array to hold the 9 temperatures to be read
 */
static void TaskGetTemperatures(uint8_t *temperatures){
	    TickType_t xLastWakeTime;							// keeps track of timing

		/* The xLastWakeTime variable needs to be initialised with the current tick
		count.  Note that this is the only time we access this variable.  From this
		point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
		API function. */
		xLastWakeTime = xTaskGetTickCount();

	    while(1)
	    {
	    	getTemperatureFromSensor(temperatures);
			vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_PERIOD_MS ) );
	    }
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    portCHAR *pcTaskName )
{

	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // main (red PB7) LED on. Mega main LED on and die.
	while(1);
}

