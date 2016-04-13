/*
 * Sonar.c
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Sonar.c
 *
 * \author Patrice Boulet, Ladan Maxamud, Alexander Teske,
 * Adnane Gasmi, Nick Dubus, Justin Langis
 *
 * \date 2016-04-12
 *
 * \brief Module that detects the distance between the robot and
 * objects in front of it by using a sonar.
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


/*!\fn InitSonarModule(void)
 * \brief Module initializer.
 *
 *\details Initializes the timer module
 *\details that is used in the sonar module.
 *
 * @returns none
 */
void InitSonarModule(void){
	initialize_module_timer0();
}


/*!\fn getDistance(float* objectDistance)
 * \brief Gets the distance of the object
 * in front of the robot.  Works for objects
 * ranging from 0.3m to 3m in distance.
 *
 * @param objectDistance distance between the robot
 * and the object in front of it.
 *
 * @returns none
 */
void getDistance(float* objectDistance){

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
	*objectDistance = 343.2F * timeDifference * 0.000001 / 2;
}

/******************************************************************************************************************/

/******************************************* Local functions  *****************************************************/

/******************************************************************************************************************/
