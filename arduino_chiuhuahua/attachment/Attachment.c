/*
 * Attachment.c
 * Chico The Robot
 */

/*!\file Attachment.c
 * \brief Module that handles the attachment
 * of the robot to humans.
 *
 * \details This modules provides functionality
 * to lock a heat source that is close to human
 * temperature, follow it until it attaches to it,
 * or panic if it doesn't find anything to follow
 * for a while.
 *
 * \author Ladan Maxamud, Adnane Gasmi, Patrice Boulet and Alex Teske
 */


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
 *  \brief	Constant representing the average human temperature
 *  (lower than 36 because most of the time the robot
 *  captures lower temperatures for hands and feet).
 */
const uint8_t HUMAN_TEMP = 30;

/*!
 *  \var  int HUMAN
 *  \brief Holds 1 if the robot has locked to follow
 *  a human, 0 otherwise.
 */
uint8_t HUMAN = 0;


/*!
 *  \var  int panicCounter
 *  \brief Counter that is incremented when the robot doesn't
 *  find any heat source to follow.  It is reset when
 *  a heat source is locked.
 */
int panicCounter = 0;

/*!
 * \var unsigned long panicTimer
 * \brief Timer used to time the amount of time
 * for which the robot has been in panic mode.
 */
unsigned long panicTimer;

/******************************************************************************************************************/

/****************************************  ENTRY POINTS  **********************************************************/

/*! \fn FindHuman(uint8_t* temperatures, AttachmentState* state)
 *  \brief Checks if the heat of a human is found.
 *
 *  \details  Checks if the heat of a human is found in the
 *  thermal array sensor's range and updates the "found human"
 *  state and attachment state.
 *
 *  @param temperatures latest temperatures captures by the thermal array sensors
 *  @param state the attachment state of the robot
*/
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

/*! \fn PanicNoHuman(AttachmentState *state)
 *  \brief Panic because there hasn't been any human
 *  found recently.
 *
 *  \details  If no human has been found for quite some
 *  time then it sets the robot to panic mode (spin left
 *  without searching for any human) for a minute
 *  before coming back to search mode.
 *
 *  @param state attachment state of the robot
*/
void PanicNoHuman(AttachmentState *state) {
	/* No human has been found for a while, go to panic mode. */
	if (panicCounter > 100 && HUMAN == 0) {
		HUMAN = 2;
		*state = PANIC;
		setMotionMode(SPINLEFT);
		panicTimer = time_in_microseconds();

    /* Already in panic mode, check if we should get out of
     * this mode. */
	} else if (HUMAN == 2) {
		unsigned long panicTimeBefore = panicTimer;
		unsigned long panicTimeAfter = time_in_microseconds();
		unsigned long panicTimeBetween = panicTimeAfter - panicTimeBefore;

		/* Get out of panic mode if a minute has passed since the entry
		 * in the mode.
		 */
		if ( panicTimeBetween > 60000000){
			HUMAN = 0;
			*state = SEARCHING;
			panicCounter = 0;
			setMotionMode(STOP);
			panicTimer = 0;
		}
	}

}

/*! \fn FollowHuman(uint8_t* temperatures, AttachmentState *state)
 *  \brief Makes the robot follow the heat source of a human.
 *
 *  \details  Moves the robot towards the human target locked in to
 *  be followed if the robot hasn't attached to it.  If the average
 *  temperature of captured by the sensors is higher on the left, turn left.
 *  Similarly, if the average temperature captured by the sensors is higher on the
 *  right, turn right.  Finally, if the average temperature is higher right in
 *  front of the robot, just go forward.
 *
 *  @param temperatures latest temperatures captures by the thermal array sensors
 *  @param state the attachment state of the robot
*/
void FollowHuman(uint8_t* temperatures, AttachmentState *state) {

	/* Compute the average temperature on the left, right and center
	 * of the thermal array sensor range. */
	uint8_t avgtempLeft = (temperatures[1] + temperatures[2] + temperatures[3]) / 3;
	uint8_t avgtempRight = (temperatures[6] + temperatures[7] + temperatures[8]) / 3;
	uint8_t avgtempCenter = (temperatures[4] + temperatures[5]) / 2;

	if (HUMAN == 1 && *state != TARGET_HIT) {
		*state = LOCKED_ON_TARGET;

		/* Average temperature is higher on the left of the robot. */
		if (avgtempLeft > avgtempCenter) {
			//turn left
			setMotionMode(SPINRIGHT);
		    delay_milliseconds(50);
		    setMotionMode(FORWARD);

		/* Average temperature is higher on the right of the robot. */
		} else if (avgtempRight > avgtempCenter) {
			//turn right
			setMotionMode(SPINLEFT);
		    delay_milliseconds(50);
		    setMotionMode(FORWARD);

		/* Average temperature is higher right in front of the robot. */
		} else {
			setMotionMode(FORWARD);
		}

		/* If the average temperatures are not near the human temperature
		 * reference, then the robot has lost the human target to follow. */
		if (avgtempLeft < HUMAN_TEMP && avgtempRight < HUMAN_TEMP
				&& avgtempCenter < HUMAN_TEMP) {
			HUMAN = 0;
			*state = SEARCHING;
		}
	}
}
