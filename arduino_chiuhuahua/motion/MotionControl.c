/*
 * MotionControl.c
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file MotionControl.c
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
 */

/******************************************************************************************************************/

/******************************************************************************************************************/

/* --Includes-- */
#include "MotionControl.h"
#include "Motion.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "usartserial.h"

/******************************************************************************************************************/

MotionMode motionMode;											/* Current motion mode of the robot (e.g. FORWARD) */
int rightWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;			/* Current pulse with of the signal sent to the right wheel servo motor. */
int leftWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;			/* Current pulse with of the signal sent to the left wheel servo motor. */
int moving = 0;													/* Holds 1 if the robot is currently in motion, 0 otherwise. */
int clockwise = 0;

int IC_RIGHT_WHEEL = MOTION_WHEEL_LEFT;
int IC_LEFT_WHEEL = MOTION_WHEEL_RIGHT;

float DIST_BETWEEN_ENCODER_REFLECTIVE = 0.0054; //measured in meters
float DIST_BETWEEN_ENCODER_REFLECTIVE_SPEED_CSTE = 10800;// = 0.0054/500ns measured in m/s

int move_init = 0;

float leftAvg = 0.0;
int leftReadCnt = 0;
float rightAvg = 0.0;
int rightReadCnt = 0;

int legDistance = 0;
int totalDistance = 0;

/******************************************************************************************************************/

/***************************************** ENTRY POINTS  **********************************************************/


/*!\brief Module initializer.
 *
 *\details  Sets the initial position of the thermal array's sensor's servo
 *			to the center and enables its motion.
 *
 * @param mode Current motion mode of the robot.
 * @param servoPosition Current position of the thermal array sensor in ticks.
 * @returns none
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
 * @returns none
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
 * @returns none
 */
void setMotionMode(MotionMode _motionMode)
{
	if (_motionMode != motionMode)
	{
		motionMode = _motionMode;
		leftAvg = 0.0;
		leftReadCnt = 0;
		rightAvg = 0.0;
		rightReadCnt = 0;

		totalDistance += legDistance;
	}

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
 * @returns none
 */
void updateRobotMotion(float currentSpeedLeftWheel, float currentSpeedRightWheel) {

	/* Left wheel is master, right wheel is slave */
	float rightWheelDiff = currentSpeedRightWheel / currentSpeedLeftWheel;
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
 * @returns none
 */
void readSpeed(float *leftWheelSpeed, float *rightWheelSpeed, float* distanceTravelled) {
    //!!!
    //!!!
    //!!! The Left Encoder reads from the Right Wheel!
    //!!!
    //!!!

    uint32_t leftWheelNewReading = 0;
    uint32_t rightWheelNewReading = 0;
    int isNewReading = 0;

    //LEFT WHEEL
    if(moving == 1){
        //get new reading if available
        isNewReading = motion_enc_read(IC_LEFT_WHEEL, &leftWheelNewReading);
        if(isNewReading == 1){
            leftReadCnt += 1;
            leftAvg = (leftAvg * (leftReadCnt-1) + leftWheelNewReading) / leftReadCnt;
            *leftWheelSpeed = DIST_BETWEEN_ENCODER_REFLECTIVE_SPEED_CSTE / leftAvg;
            usart_printf_P(PSTR("LEFT WHEEL READING: %.4f \r\n"), *leftWheelSpeed);

        }
    }
    //RIGHT WHEEL
    if(moving == 1){
        //get new reading if available
        isNewReading = motion_enc_read(IC_RIGHT_WHEEL, &rightWheelNewReading);
        if(isNewReading == 1){
            rightReadCnt += 1;
            rightAvg = (rightAvg * (rightReadCnt-1) + rightWheelNewReading) / rightReadCnt;
            *rightWheelSpeed = DIST_BETWEEN_ENCODER_REFLECTIVE_SPEED_CSTE / rightAvg;
        }
    }
}

/*
assuming the accuracy of two significant digits (e.g. 0.xyz m/s) then
0.54cm / (80,000 * 500ns) = 0.135 ms/s
0.54cm / (81,000 * 500ns) = 0.133 ms/s
0.54cm / (82,000 * 500ns) = 0.131 ms/s
0.54cm / (83,000 * 500ns) = 0.130 ms/s
you can be off by +/- 3,000 tics and values xy are not affected..

in looking at the online doc chart, the values vary between 80k and 100k.  hence at the other extreme
we see that the speed can vary alot.
0.54cm / (100,000 * 500ns) = 0.108 ms/s
It would be a good idea to "reproduce the prof's chart" and measure the variation between measurments for our robot
at max speed (and other?).
When doing the experiment, we know the register value we are setting in tics (say x) and the tic time elapsed
between each of the 32 rotations (like what the prof has done). we average those values out and get a y
so we know when we set x we expect y (or a certain amount of error).
using this "custom formla" we can adjust readings as to avoid calculating strange averages in real time
*/
void calcDistance(float* distanceTravelled){
    if (leftAvg > 0.0 && rightAvg > 0.0){
        //calculate the average number of encoders read (e.g. using count) and multiply by the distance separating them.
        legDistance = ( (rightReadCnt + leftReadCnt)/2 ) * DIST_BETWEEN_ENCODER_REFLECTIVE;
        *distanceTravelled = legDistance + totalDistance;
        //usart_printf_P(PSTR("AVG SPEED %.4f m.s(LW:%.4f m.s, RW:%.4f m.s)    DIST: %.4f m\r\n"), avgSpeed, leftWheelSpeed, rightWheelSpeed, distanceTraveled);
    }
}

/******************************************************************************************************************/
