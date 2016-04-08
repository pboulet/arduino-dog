/* Modules includes. */
#include "../attachment/Attachment.h"

#include "../temperatureReader/TemperatureReader.h"
#include "../motion/MotionControl.h"
#include "../motion/motion.h"
#include "../sonar/custom_timer.h"

/******************************************************************************************************************/

/***************************************  Function Declarations  **************************************************/



/*************************************  Type definitions & Macros  ************************************************/

/******************************************************************************************************************/

/******************************************* Global variables *****************************************************/

/*!
 *  \var const int HUMAN_TEMP
 *  \brief
 */
const uint8_t HUMAN_TEMP = 30;

/*!
 *  \var  int HUMAN
 *  \brief
 */
uint8_t HUMAN = 0;


int panicCounter = 0;
unsigned long panicTimer;

/******************************************************************************************************************/

/****************************************  ENTRY POINTS  **********************************************************/

void FindHuman(uint8_t* temperatures, AttachmentState* state) {
	if ( (*state) != PANIC) {
		for (int i = 4; i < 6; i++) {
			if (temperatures[i] >= HUMAN_TEMP
					&& temperatures[i] <= HUMAN_TEMP + 7) {
				HUMAN = 1;
				*state = LOCKED_ON_TARGET;
				break;
			}
		}
	}
	if (HUMAN == 0) {
		*state = SEARCHING;
		setMotionMode(SPINRIGHT);
		panicCounter++;
	}
}

void PanicNoHuman(AttachmentState *state) {
	if (panicCounter > 100 && HUMAN == 0) {
		HUMAN = 2;
		*state = PANIC;
		setMotionMode(SPINLEFT);
		panicTimer = time_in_microseconds();
	} else if (HUMAN == 2) {
		unsigned long panicTimeBefore = panicTimer;
		unsigned long panicTimeAfter = time_in_microseconds();
		unsigned long panicTimeBetween = panicTimeAfter - panicTimeBefore;

		if ( panicTimeBetween > 60000000){
			HUMAN = 0;
			*state = SEARCHING;
			panicCounter = 0;
			setMotionMode(STOP);
			panicTimer = 0;
		}
	}

}

void FollowHuman(uint8_t* temperatures, AttachmentState *state) {
	uint8_t avgtempLeft = 0;
	uint8_t avgtempRight = 0;
	uint8_t avgtempCenter = 0;

	avgtempLeft = (temperatures[1] + temperatures[2] + temperatures[3]) / 3;
	avgtempRight = (temperatures[6] + temperatures[7] + temperatures[8]) / 3;
	avgtempCenter = (temperatures[4] + temperatures[5]) / 2;

	if (HUMAN == 1 && *state != TARGET_HIT) {
		*state = LOCKED_ON_TARGET;
		if (avgtempLeft > avgtempCenter) {
			//turn left
			setMotionMode(SPINRIGHT);
		    delay_milliseconds(50);
		    setMotionMode(FORWARD);
		} else if (avgtempRight > avgtempCenter) {
			//turn right
			setMotionMode(SPINLEFT);
		    delay_milliseconds(50);
		    setMotionMode(FORWARD);
		} else {
			setMotionMode(FORWARD);
		}

		if (avgtempLeft < HUMAN_TEMP && avgtempRight < HUMAN_TEMP
				&& avgtempCenter < HUMAN_TEMP) {
			HUMAN = 0;
			*state = SEARCHING;
		}
	}
}
