#ifndef LED_H_
#define LED_H_

typedef enum {RED, GREEN, BLUE, WHITE, OFF} LEDState;


/**Turns on the LED passed as a parameter, or turns the LED off
 * @param[in] LED to turn on, or parameter to turn off LED.
 */
void lightLED(LEDState);

#endif
