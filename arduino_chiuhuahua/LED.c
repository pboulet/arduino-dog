#include "include/LED.h"


/**-----------------------------------------------------------------*
 * Function: redLED                                                *
 * Returns: None                                                   *
 * Desc: Turn on or off the red LED depending on passed parameter  *
 *-----------------------------------------------------------------*/
void redLED(int on){
	if(on == 1){
		DDRH |= _BV(DDH3);
		 PORTH &= ~_BV(PORTH3);
	  }else{
		  if (on == 0){
		  PORTH |= _BV(PORTH3);
		  }
	  }
}

/**-----------------------------------------------------------------*
 * Function: greenLED                                              *
 * Returns: None                                                   *
 * Desc: Turn on or off the green LED depending on passed parameter*
 *-----------------------------------------------------------------*/
void greenLED(int on) {
  if(on == 1){
	  DDRE |= _BV(DDE3);
	  PORTE &= ~_BV(PORTE3);
   }else{
	  if(on == 0){
	  PORTE |= _BV(PORTE3);
	  }
  }
}


/**-----------------------------------------------------------------*
 * Function: blueLED                                              *
 * Returns: None                                                   *
 * Desc: Turn on or off the blue LED depending on passed parameter *
 *-----------------------------------------------------------------*/
void blueLED(int on) {
  if(on == 1){
	  DDRE |= _BV(DDE5);
	  PORTE &= ~_BV(PORTE5);
  }else{
	  if(on == 0){
	  PORTE |= _BV(PORTE5);
	  }
  }
}
