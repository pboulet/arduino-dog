////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

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
/* Create a handle for the serial port. */
//extern xComPortHandle xSerialPort;

static void TaskGetTemperatures(void *pvParameters);

/*-----------------------------------------------------------*/

/* Main program loop */
//int main(void) __attribute__((OS_main));
int usartfd;
uint8_t *temperatures;


int main(void)
{

    // turn on the serial port for debugging or for other USART reasons.
	usartfd = usartOpen( USART0_ID, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	usart_print_P(PSTR("\r\n\n\nTesting!!!!!\r\n")); // Ok, so we're alive...

	 xTaskCreate(
	    	TaskGetTemperatures
			,  (const portCHAR *)"ReadTemperature" // Temperature reading task
			,  256				// Tested 9 free @ 208
			,  NULL
			,  3
			,  NULL ); // */

	usart_printf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

	vTaskStartScheduler();

	usart_print_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

static void TaskGetTemperatures(void *pvParameters){
	(void) pvParameters;;
	    TickType_t xLastWakeTime;

		/* The xLastWakeTime variable needs to be initialised with the current tick
		count.  Note that this is the only time we access this variable.  From this
		point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
		API function. */
		xLastWakeTime = xTaskGetTickCount();


	    while(1)
	    {
	    	temperatures = getTemperatureFromSensor();
	    	usart_printf_P(PSTR("Temperature 1 @ %u\r\n"), temperatures[1]);
	    	usart_printf_P(PSTR("Temperature 2 @ %u\r\n"), temperatures[2]);
	    	usart_printf_P(PSTR("Temperature 3 @ %u\r\n"), temperatures[3]);
	    	usart_printf_P(PSTR("Temperature 4 @ %u\r\n"), temperatures[4]);
	    	usart_printf_P(PSTR("Temperature 5 @ %u\r\n"), temperatures[5]);
	    	usart_printf_P(PSTR("Temperature 6 @ %u\r\n"), temperatures[6]);
	    	usart_printf_P(PSTR("Temperature 7 @ %u\r\n"), temperatures[7]);
	    	usart_printf_P(PSTR("Temperature 8 @ %u\r\n"), temperatures[8]);
	    	usart_printf_P(PSTR("Temperature 9 @ %u\r\n"), temperatures[9]);
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

/*-----------------------------------------------------------*/

