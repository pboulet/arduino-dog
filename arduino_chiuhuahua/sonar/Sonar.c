/*
 * sonar.c
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file sonar.c
 *
 * \author
 *
 * \date
 *
 * \brief
 *
 */

/******************************************************************************************************************/

/***** Include *******/

#include "Sonar.h"
#include "./custom_timer.h"
#include "avr/io.h"

/******************************************************************************************************************/

/***************************************  Function Declarations  **************************************************/


/******************************************************************************************************************/

/********************************************* Entry Points  ******************************************************/

/******************************************************************************************************************/

/******************************************* Local functions  *****************************************************/

void InitSonarModule(void){
	initialize_module_timer0();
}

/*!\brief
 *
 * @returns
 */
float getDistance(void){

	/* Set the data direction register to write
	 * and set the sonar's input register to high. */
	DDRA |= _BV(DDA0);
	PORTA |= _BV(PA0);

	/* Send a high pulse for 5 microseconds. */
    delay_milliseconds(0.005);

    /* Set the data direction register to read
     * and set the sonar's input register to low. */
	DDRA &= ~_BV(DDA0);
	PORTA &= ~_BV(PA0);

	/* Start measuring the time after sending the whole pulse. */
	unsigned long timeBefore = time_in_microseconds();

	/* Wait until the whole pulse has been received by the sonar. */
	loop_until_bit_is_set(PINA, PA0);
	loop_until_bit_is_clear(PINA, PA0);

	/* Measure the time it took for the pulse to bounce back. */
	unsigned long timeAfter = time_in_microseconds();
	unsigned long timeDifference = timeAfter - timeBefore;

	/* Convert with respect to the speed of sound in air to \
	 * get the distance traveled. */
	float distance = 343.2F * timeDifference * 0.000001 / 2;

	return distance;
}

/******************************************************************************************************************/
