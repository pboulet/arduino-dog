#include "include/MotionControl.h"
#include "include/motion.h"

MotionMode motionMode;
int rightWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;
int leftWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;

void setTargetSpeed(int _targetSpeed, MotionMode _motionMode)
{
	if (_targetSpeed == 0)
	{
		motion_servo_stop(MOTION_WHEEL_LEFT);
		motion_servo_stop(MOTION_WHEEL_RIGHT);
		return;
	}
	motionMode = _motionMode;

	motion_servo_start(MOTION_WHEEL_LEFT);
	motion_servo_start(MOTION_WHEEL_RIGHT);

	switch(motionMode)
	{
		case FORWARD:
			leftWheelPulseWidth = MAX_PULSE_WIDTH_TICKS;
			rightWheelPulseWidth = MIN_PULSE_WIDTH_TICKS;
			break;
		case BACKWARDS:
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
	}

	motion_servo_set_pulse_width(MOTION_WHEEL_LEFT, leftWheelPulseWidth);
	motion_servo_set_pulse_width(MOTION_WHEEL_RIGHT, rightWheelPulseWidth);

}

/**
 * Function: greenLED
 * Returns: None
 * Desc: Turn on or off the green LED depending on passed parameter
 */
void updateRobotMotion(int currentSpeedLeftWheel, int currentSpeedRightWheel) {

	//left wheel master, right wheel slave
	double rightWheelDiff = (double)currentSpeedRightWheel / currentSpeedLeftWheel;
	if (rightWheelPulseWidth < 2850)
	{
		//
		rightWheelPulseWidth = rightWheelDiff * rightWheelPulseWidth;
	}
	else
	{
		rightWheelPulseWidth = rightWheelPulseWidth/rightWheelDiff;
	}

	//motion_servo_set_pulse_width(MOTION_WHEEL_LEFT, 2500);
	motion_servo_set_pulse_width(MOTION_WHEEL_RIGHT, rightWheelPulseWidth);
}
