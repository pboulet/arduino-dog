#include "LED.h"

/**
 * Function: redLED
 * Returns: None
 * Desc: Turn on or off the red LED depending on passed parameter  
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

/**
 * Function: greenLED
 * Returns: None
 * Desc: Turn on or off the green LED depending on passed parameter  
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


/**
 * Function: blueLED
 * Returns: None
 * Desc: Turn on or off the blue LED depending on passed parameter  
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
