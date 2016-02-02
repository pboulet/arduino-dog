#ifndef LED_H_
#define LED_H_

#include "avr/io.h"

/**Turns the red LED on or off
 * @param[in] TRUE to turn the LED on. FALSE to turn it off.
 */
void redLED(int on);

/**Turns the green LED on or off
 * @param[in] TRUE to turn the LED on. FALSE to turn it off.
 */
void greenLED(int on);

/**Turns the blue LED on or off
 * @param[in] TRUE to turn the LED on. FALSE to turn it off.
 */
void blueLED(int on);

#endif
