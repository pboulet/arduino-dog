/*
 * ThermalSensor.c
 *
 */

/*(Doxygen help: use \brief to provide short summary and \details command can be used)*/

/*!	\file template.c
 * \author Ladan Maxamud and Patrice Boulet
 * \date January 27th 2016
 * \brief Thermal Sensor reader
 * \details This module communicates with the thermal sensor TPA81 using the I2C bus protocol.
 *
 *
 *
 */

/* --Includes-- */
#include <stdio.h>
#include <stdlib.h>

/*AVR includes*/
#include "avr/io.h"
#include "avr/interrupt.h"


/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "usartserial.h"
#include "i2cMultiMaster.h"

/*include your header files here*/
#include "ThermalSensor.h"

/*---------------------------------------  Function Declarations  -------------------------------------------------*/
void TemperatureSensor(void);
static void TaskGetTemperature(void *pvParameters);
uint8_t *getTempartureFromSensor(void);



/*! \brief Main entry point for temperature module
 *
 * \details This function
 * \note main() function can be only appear in one module, which is the main entry point of program;
 * hence remove it while coding a supporting module, which itself is not suppose to execute.
 *
 *
 * @return void
 */
void TemperatureSensor(void){
	I2C_Master_Initialise(0xC0);

	 xTaskCreate(
	    	TaskGetTemperature
			,  (const portCHAR *)"ReadTemperature" // Temperature reading task
			,  256				// Tested 9 free @ 208
			,  NULL
			,  3
			,  NULL ); // */
}

/*---------------------------------------  LOCAL FUNCTIONS  ------------------------------------------------------*/
uint8_t *getTempartureFromSensor(void){

	/* 1. Send a start sequence
	 * 2. Send 0xD0 ( I2C address of the thermal sensor with the R/W bit low (even address)
	 * 3. Send 0x01 (Internal address of the bearing register - 0x01 == Ambient)
	 * 4. Send a start sequence again (repeated start)
	 * 5. Send 0xD1 ( I2C address of the thermal sensor with the R/W bit high (odd address)
	 * 6. Read data byte from thermal sensor
	 * 7. Send the stop sequence.
	 * */

	uint8_t writeCommand[2] = {0xD0, 0x01};
	uint8_t * readCommand = malloc(sizeof(uint8_t)*10);
	readCommand[0] = 0xD1;

	I2C_Master_Start_Transceiver_With_Data(writeCommand,2);
	I2C_Master_Start_Transceiver_With_Data(readCommand,10);
	I2C_Master_Get_Data_From_Transceiver(readCommand,10);

	return readCommand;
}

static void TaskGetTemperature(void *pvParameters){
	(void) pvParameters;;
	    TickType_t xLastWakeTime;

		/* The xLastWakeTime variable needs to be initialised with the current tick
		count.  Note that this is the only time we access this variable.  From this
		point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
		API function. */
		xLastWakeTime = xTaskGetTickCount();


	    while(1)
	    {
	    	uint8_t *Temperature = getTempartureFromSensor();
	    	usart_printf_P(PSTR("Temperature 1 @ %u\r\n"), Temperature[1]);
	    	usart_printf_P(PSTR("Temperature 2 @ %u\r\n"), Temperature[2]);
	    	usart_printf_P(PSTR("Temperature 3 @ %u\r\n"), Temperature[3]);
	    	usart_printf_P(PSTR("Temperature 4 @ %u\r\n"), Temperature[4]);
	    	usart_printf_P(PSTR("Temperature 5 @ %u\r\n"), Temperature[5]);
	    	usart_printf_P(PSTR("Temperature 6 @ %u\r\n"), Temperature[6]);
	    	usart_printf_P(PSTR("Temperature 7 @ %u\r\n"), Temperature[7]);
	    	usart_printf_P(PSTR("Temperature 8 @ %u\r\n"), Temperature[8]);
	    	usart_printf_P(PSTR("Temperature 9 @ %u\r\n"), Temperature[9]);
			vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_PERIOD_MS ) );
	    }
}
