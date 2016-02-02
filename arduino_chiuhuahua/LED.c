#include "include/LED.h"

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