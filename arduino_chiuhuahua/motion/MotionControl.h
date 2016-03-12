/*
 * MotionControl.h
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

#ifndef INCLUDE_MOTION_CONTROL_H_
#define INCLUDE_MOTION_CONTROL_H_

// we need to expose this dependency here otherwise our
// API cannot compile some of its type definitions
#include <stdint.h>

typedef enum {FORWARD, BACKWARD, SPINLEFT, SPINRIGHT, STOP} MotionMode;

/*---------------------------------------  ENTRY POINTS  ---------------------------------------------------------*/

/*!\brief Module initializer.
 *
 *\details  Sets the initial position of the thermal array's sensor's servo
 *			to the center and enables its motion.
 *
 * @param mode Current motion mode of the robot.
 * @param servoPosition Current position of the thermal array sensor in ticks.
 * @returns
 */
void initMotionControl(uint16_t*);

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
void setMotionMode(MotionMode);

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
void updateRobotMotion(double, double);

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
void readSpeed(double*, double*, double*);

/*!\brief Rotates the thermal array servo moto in a sweep movement from left to right and inversely.
 *
 *\details  Provides functions to
 *\details  - Sweeps the thermal array sensor's servo motor from left to right when its current
 *				position is on the left of the center position.
 *			- Sweeps the thermal array sensor's
 *				servo motor from right to left when its current position is on the right of the center
 *				position.
 *			- Stops the thermal array sensor's servo motor motion when the motion mode of
 *				the robot is STOP.
 *
 * @param mode current motion mode of the robot
 * @param servoPosition current position of the thermal array sensor (in ticks)
 */
void temperatureSweep(MotionMode, uint16_t*);

#endif

