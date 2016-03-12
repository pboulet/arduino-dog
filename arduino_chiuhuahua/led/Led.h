/*
 * Led.h
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Led.h
 *
 * \author Alexander Teske & Adnane Gasmi, Patrice Boulet
 *
 * \date March 13th, 2016
 *
 * \brief Module that provides a wrapper around the LED hardware of the robot.
 *
 */

/******************************************************************************************************************/

#ifndef LED_H_
#define LED_H_

typedef enum {RED, GREEN, BLUE, WHITE, OFF} LEDState;


/******************************************************************************************************************/


/********************************************* Entry Points  ******************************************************/

/*!\brief Turn on the proper LED depending on passed parameter
 *
 * @param on enumeration value that dictates the LED color to show
 * @returns none
 */
void lightLED(LEDState);

#endif

/******************************************************************************************************************/
