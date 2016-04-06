/* Modules includes. */
#include "../attachment/Attachment.h"

#include "../temperatureReader/TemperatureReader.h"
#include "../motion/MotionControl.h"
#include "../motion/motion.h"

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

/******************************************************************************************************************/

/****************************************  ENTRY POINTS  **********************************************************/


/*!\brief
 *
 *\details
 *
 * @param
 * @returns none
 */
void Sweep(void) {
	if (HUMAN == 0) {
		setMotionMode(SPINRIGHT);
	}
}

void FindHuman(uint8_t* temperatures) {
	for (int i = 4; i < 6; i++) {
		if (temperatures[i] >= HUMAN_TEMP
				&& temperatures[i] <= HUMAN_TEMP + 7) {
			HUMAN = 1;
			setMotionMode(STOP);
			break;
		}
	}
	if (HUMAN == 0) {
		panicCounter++;
	}
}

void PanicNoHuman(void) {
	if (panicCounter > 100 && HUMAN==0) {
		HUMAN = 2;
		setMotionMode(SPINLEFT);
	} else if (HUMAN == 2) {
		HUMAN = 0;
		panicCounter = 0;
		setMotionMode(STOP);
	}

}

void FollowHuman(uint8_t* temperatures) {
	uint8_t avgtempLeft = 0;
	uint8_t avgtempRight = 0;
	uint8_t avgtempCenter = 0;
	int risingEdgeL = 0;
	int risingEdgeR = 0;

	avgtempLeft = (temperatures[1] + temperatures[2] + temperatures[3]) / 3;
	avgtempRight = (temperatures[6] + temperatures[7] + temperatures[8]) / 3;
	avgtempCenter = (temperatures[4] + temperatures[5]) / 2;

	if (HUMAN == 1) {
		if (avgtempLeft > avgtempCenter) {
			//turn left
			setMotionMode(SPINRIGHT);
			while (risingEdgeL == 0 && risingEdgeL == 0) {
				risingEdgeL = motion_enc_read(MOTION_WHEEL_LEFT, NULL);
				risingEdgeR = motion_enc_read(MOTION_WHEEL_RIGHT, NULL);
			}
			setMotionMode(STOP);

		} else if (avgtempRight > avgtempCenter) {
			//turn right
			setMotionMode(SPINLEFT);
			while (risingEdgeL == 0 && risingEdgeL == 0) {
				risingEdgeL = motion_enc_read(MOTION_WHEEL_LEFT, NULL);
				risingEdgeR = motion_enc_read(MOTION_WHEEL_RIGHT, NULL);
			}
			setMotionMode(STOP);
		}
		if (avgtempLeft < HUMAN_TEMP && avgtempRight < HUMAN_TEMP
				&& avgtempCenter < HUMAN_TEMP) {
			HUMAN = 0;
		}
	}

}
