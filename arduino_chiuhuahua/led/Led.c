/*
 * Led.c
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Led.c
 *
 * \author Alexander Teske & Adnane Gasmi, Patrice Boulet
 *
 * \date March 13th, 2016
 *
 * \brief Module that provides a wrapper around the LED hardware of the robot.
 *
 */

/******************************************************************************************************************/

/***** Include *******/

#include "avr/io.h"
#include "Led.h"

/******************************************************************************************************************/

/***************************************  Function Declarations  **************************************************/

void redLED(int);

void greenLED(int);

void blueLED(int);

/******************************************************************************************************************/

/********************************************* Entry Points  ******************************************************/

/*!\brief Turn on the proper LED depending on passed parameter
 *
 * @param on enumeration value that dictates the LED color to show
 * @returns none
 */
void lightLED(LEDState LED){
    switch(LED)
    {
        case RED:
            redLED(1);
            greenLED(0);
            blueLED(0);
            break;
        case GREEN:
            greenLED(1);
            blueLED(0);
            redLED(0);
            break;
        case BLUE:
            greenLED(0);
            blueLED(1);
            redLED(0);
            break;
        case WHITE:
            redLED(1);
            greenLED(1);
            blueLED(1);
            break;
        case OFF:
            redLED(0);
            greenLED(0);
            blueLED(0);
            break;
    }
}


/******************************************************************************************************************/

/******************************************* Local functions  *****************************************************/

/*!\brief Turn on or off the red LED depending on passed parameter
 *
 * @param on Set to 1 when we want to turn this LED on and to 0 otherwise.
 * @returns none
 */
void redLED(int on){
    if(on == 1){
        DDRH |= _BV(DDH3); /**< set proper pin in data register */
        PORTH &= ~_BV(PORTH3); /**< turn on LED by setting proper pin to 0 */
    }else{
        if (on == 0){
            PORTH |= _BV(PORTH3); /**< set proper pin to 1 to turn off LED */
        }
    }
}


/*!\brief Turn on or off the green LED depending on passed parameter
 *
 * @param on Set to 1 when we want to turn this LED on and to 0 otherwise.
 * @returns none
 */
void greenLED(int on) {
    if(on == 1){
        DDRE |= _BV(DDE3); /**< set proper pin in data register */
        PORTE &= ~_BV(PORTE3); /**< turn on LED by setting proper pin to 0 */
    }else{
        if(on == 0){
            PORTE |= _BV(PORTE3); /**< set proper pin to 1 to turn off LED */
        }
    }
}

/*!\brief Turn on or off the blue LED depending on passed parameter
 *
 * @param on Set to 1 when we want to turn this LED on and to 0 otherwise.
 * @returns none
 */
void blueLED(int on) {
    if(on == 1){
        DDRE |= _BV(DDE5); /**< set proper pin in data register */
        PORTE &= ~_BV(PORTE5); /**< turn on LED by setting proper pin to 0 */
    }else{
        if(on == 0){
            PORTE |= _BV(PORTE5); /**< set proper pin to 1 to turn off LED */
        }
    }
}

/******************************************************************************************************************/
