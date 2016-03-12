/*
 * MotionControl.c
 * Chico The Robot
 *
 * Authors: Ladan Maxamud, Patrick Boulet, Nick Dubus,
 * 			Justin Langis, Alexander Teske & Adnane Gasmi.
 */

/******************************************************************************************************************/

/*!	\file MotionControl.h
 *
 * \author Ladan Maxamud, Patrick Boulet, Nick Dubus,
 * 			Justin Langis, Alexander Teske & Adnane Gasmi.
 *
 * \date March 13th, 2016
 *
 * \brief Module that provides motor functionalities to the robot.
 *
 * \details This module provides functionalities to control the servo
 * 	motors of the wheels of the robot as well as the one on the
 * 	temperature array sensor.
 *
 *
 */

/******************************************************************************************************************/

/******************************************************************************************************************/

/* --Includes-- */
#include "MotionControl.h"
#include "Motion.h"

/******************************************************************************************************************/

MotionMode motionMode;											/* Current motion mode of the robot (e.g. FORWARD) */
int rightWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;			/* Current pulse with of the signal sent to the right wheel servo motor. */
int leftWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;			/* Current pulse with of the signal sent to the left wheel servo motor. */
int moving = 0;													/* Holds 1 if the robot is currently in motion, 0 otherwise. */
const uint8_t numRisingEdgesForAvg = 32;						/* Number of rising edges to use in the speed average computation. */
uint32_t observationLeftCtr = 0;								/* Holds the count of encoder observations gathered for the left wheel. */
uint32_t observationRightCtr = 0;								/* Holds the count of encoder observations gathered for the right wheel. */
uint32_t leftRotationTicks[32] = {[0 ... 31] = 45000};			/* Holds tick difference values gathered from the encoder of the left wheel. */
uint32_t rightRotationTicks[32] = {[0 ... 31] = 45000};			/* Holds tick difference values gathered from the encoder of the right wheel. */
uint16_t clockwise = 1;											/* Holds 1 if the robot is in a state to turn clockwise, 0 otherwise. */

/******************************************************************************************************************/

/***************************************** ENTRY POINTS  **********************************************************/


/*!\brief Module initializer.
 *
 *\details  Sets the initial position of the thermal array's sensor's servo
 *			to the center and enables its motion.
 *
 * @param mode Current motion mode of the robot.
 * @param servoPosition Current position of the thermal array sensor in ticks.
 * @returns
 */
void initMotionControl(uint16_t* servoPosition) {
	*servoPosition = INITIAL_PULSE_WIDTH_TICKS;
	motion_servo_set_pulse_width(MOTION_SERVO_CENTER, *servoPosition);
	motion_servo_start(MOTION_SERVO_CENTER);
}

/*!\brief Rotates the thermal array servo moto in a sweep movement from left to right and inversely.
 *
 *\details  Provides the following functions:
 *			- Sweeps the thermal array sensor's servo motor from left to right when its current
 *				position is on the left of the center position.
 *			- Sweeps the thermal array sensor's
 *				servo motor from right to left when its current position is on the right of the center
 *				position.
 *			- Stops the thermal array sensor's servo motor motion when the motion mode of
 *				the robot is STOP.
 *
 * @param mode Current motion mode of the robot.
 * @param servoPosition Current position of the thermal array sensor in ticks.
 * @returns
 */
void temperatureSweep(MotionMode mode, uint16_t* servoPosition) {
		if (mode == STOP){
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER, INITIAL_PULSE_WIDTH_TICKS);
		} else if(clockwise == 1){
            /* Increment servo position to create rotation of motor. */
			*servoPosition += 250;
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER,*servoPosition);

			/* Change direction when we're at the maximum pulse width. */
			if ( *servoPosition >= MAX_PULSE_WIDTH_TICKS)
				clockwise = 0;
		} else {
			/* Decrement servo position to create rotation of motor in opposite direction */
			*servoPosition -= 250;
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER, *servoPosition);

			/* Change direction when we're at the mimimum pulse width. */
			if ( *servoPosition <= MIN_PULSE_WIDTH_TICKS)
				clockwise = 1;
		}
}

/*!\brief Sets the current motion mode of the robot and enables/disables its motion.
 *
 *\details  Provides the following functions:
 *			- Depending on the motion mode parameter provided, sets the
 *			current motion mode to forward, backwards, spinning clockwise/counter-clockwise
 *			or stop.
 *			- Sets the pulse width for the servo motors of both wheels of the robot.
 *
 * @param _motionMode Current motion mode to be set.
 * @returns
 */
void setMotionMode(MotionMode _motionMode)
{
	motionMode = _motionMode;

	if (!moving && motionMode != STOP)
	{
		motion_servo_start(MOTION_WHEEL_LEFT);
		motion_servo_start(MOTION_WHEEL_RIGHT);
		moving = 1;
	}

	switch(motionMode)
	{
		case FORWARD:
			leftWheelPulseWidth = MAX_PULSE_WIDTH_TICKS;
			rightWheelPulseWidth = MIN_PULSE_WIDTH_TICKS;
			break;
		case BACKWARD:
			leftWheelPulseWidth = MIN_PULSE_WIDTH_TICKS;
			rightWheelPulseWidth = MAX_PULSE_WIDTH_TICKS;
			break;
		case SPINRIGHT:
			leftWheelPulseWidth = MAX_PULSE_WIDTH_TICKS;
			rightWheelPulseWidth = MAX_PULSE_WIDTH_TICKS;
			break;
		case SPINLEFT:
			leftWheelPulseWidth = MIN_PULSE_WIDTH_TICKS;
			rightWheelPulseWidth = MIN_PULSE_WIDTH_TICKS;
			break;
		case STOP:
			motion_servo_stop(MOTION_WHEEL_LEFT);
			motion_servo_stop(MOTION_WHEEL_RIGHT);
			moving = 0;
			return;
	}

	motion_servo_set_pulse_width(MOTION_WHEEL_LEFT, leftWheelPulseWidth);
	motion_servo_set_pulse_width(MOTION_WHEEL_RIGHT, rightWheelPulseWidth);

}

/*!\brief Wheels servo motors synchronization controller function.
 *
 *\details  Provides the following functions:
 *			- Adjusts the speed of the right wheel to match the one of the left one
 *			by adjusting the pulse width of the right wheel's servo motor proportionally
 *			to the ratio of the speed between wheels.
 *
 * @param currentSpeedLeftWheel Current speed of the left wheel. (m/s)
 * @param currentSpeedRightWheel Current speed of the right wheel
 * @returns
 */
void updateRobotMotion(double currentSpeedLeftWheel, double currentSpeedRightWheel) {

	/* Left wheel is master, right wheel is slave */
	double rightWheelDiff = currentSpeedRightWheel / currentSpeedLeftWheel;
	if (rightWheelPulseWidth < 2850)
	{
		rightWheelPulseWidth = rightWheelDiff * rightWheelPulseWidth;
	}
	else
	{
		rightWheelPulseWidth = rightWheelPulseWidth/rightWheelDiff;
	}

	motion_servo_set_pulse_width(MOTION_WHEEL_RIGHT, rightWheelPulseWidth);
}

/*!\brief Reads the speed of both wheels, computes and update the distance traveled.
 *
 *\details  Provides the following functions:
 *			- Waits until it gets a successful reading for both wheels first.  Then
 *			it stores the values in buffer arrays where we keep the 32 most recent
 *			speed readings.  We then make an average of the values that are not
 *			zero inside these array and sets the wheels speed to the computed averages.  Finally,
 *			we calculate the current distance traveled by the distance traveled since the
 *			last execution of this task.
 *
 * @param speedLeft Reference to the memory location for current speed of the left wheel (m/s).
 * @param speedRight Reference to the memory location for current speed of the right wheel (m/s).
 * @param distance Reference to the memory location for the distance traveled (m).
 * @returns
 */
void readSpeed(double *speedLeft, double *speedRight, double* distance) {
	uint32_t 	ticCountLeft = 0,
				ticCountRight = 0;

	/* Reads until it gets a successful reading from the left wheel's encoder. */
	int leftReadSuccessful = 0;
	while(leftReadSuccessful != 1)
		leftReadSuccessful = motion_enc_read(MOTION_WHEEL_LEFT, &ticCountLeft);

	/* Reads until it gets a successful reading from the right wheel's encoder. */
	int rightReadSuccessful = 0;
	while(rightReadSuccessful != 1)
		rightReadSuccessful = motion_enc_read(MOTION_WHEEL_RIGHT, &ticCountRight);

	/* Stores the tick count reading in the left wheel most recent observations buffer. */
	leftRotationTicks[observationLeftCtr] = ticCountLeft;
	observationLeftCtr = (observationLeftCtr + 1 ) % (numRisingEdgesForAvg - 1);

	/* Stores the tick count reading in the right wheel most recent observations buffer. */
	rightRotationTicks[observationRightCtr] = ticCountRight;
	observationRightCtr = (observationRightCtr + 1) % (numRisingEdgesForAvg - 1);

	uint32_t rotationTicksCountLeft = 0;			/* Summation of the tick counts of the 32 most recent speed readings of the left wheel. */
	uint32_t rotationTicksCountRight = 0;			/* Summation of the tick counts of the 32 most recent speed readings of the right wheel. */
	uint8_t rotationLeftTicksCountForAvg = 0;		/* Number of successful most recent speed readings up to now for the left wheel (max 32). */
	uint8_t rotationRightTicksCountForAvg = 0;		/* Number of successful most recent speed readings up to now for the right wheel (max 32). */

	/* Computes the summation of tick count and updates the count of relevant observations for the left wheel's encoder up to now. */
	for(int i = 0; i < numRisingEdgesForAvg; i++){
		if (leftRotationTicks[i] != 0) {
			rotationTicksCountLeft += leftRotationTicks[i];
			rotationLeftTicksCountForAvg++;
		}
	}

	/* Computes the summation of tick count and updates the count of relevant observations for the right wheel's encoder up to now. */
	for(int i = 0; i < numRisingEdgesForAvg; i++){
		if (rightRotationTicks[i] != 0) {
			rotationTicksCountRight += rightRotationTicks[i];
			rotationRightTicksCountForAvg++;
		}
	}

	/* Computes the speed of both wheels. */
	/* Formula used is the following:
	 *
	 *		(c/n)/(t/n)*l
	 *
	 *		where 	c is the circumference of a wheel,
	 *				n the number of relevant observation to take into account in the average,
	 *				t the summations of relevant tick count observations for the average,
	 *				l the number of seconds in a tick
	 */
	*speedLeft = (0.1728/(double)rotationLeftTicksCountForAvg) / (((double)rotationTicksCountLeft / (double)rotationLeftTicksCountForAvg) * 0.0000005);
	*speedRight = (0.1728/(double)rotationRightTicksCountForAvg) / (((double)rotationTicksCountRight /(double)rotationRightTicksCountForAvg) * 0.0000005);

	/* Incremments the distance traveled by the distance traveled since the last execution of this task. */
	/* Forumla used is the following:
	 *
	 * 		(v1+v2)/2 * t
	 *
	 * 		where	v1 is the speed of the left wheel
	 * 				v2 is the speed of the right wheel
	 * 				t is the number of seconds elapsed since the last execution of this task
	 */
	*distance += (*speedLeft + *speedRight)/2.0 * 0.100;
}

/******************************************************************************************************************/
