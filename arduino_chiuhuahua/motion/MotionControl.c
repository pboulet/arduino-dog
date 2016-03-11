#include "MotionControl.h"

#include "Motion.h"

MotionMode motionMode;
int rightWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;
int leftWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;
int moving = 0;
const uint8_t numRisingEdgesForAvg = 32;

uint16_t clockwise = 1;

void initMotionControl(uint16_t* servoPosition) {
	*servoPosition = INITIAL_PULSE_WIDTH_TICKS;
	motion_servo_set_pulse_width(MOTION_SERVO_CENTER, *servoPosition);
	motion_servo_start(MOTION_SERVO_CENTER);
}

void temperatureSweep(MotionMode mode, uint16_t* servoPosition) {
		if (mode == STOP){
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER, INITIAL_PULSE_WIDTH_TICKS);
		} else if(clockwise == 1){
			*servoPosition += 100;
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER,*servoPosition);
			if ( *servoPosition >= MAX_PULSE_WIDTH_TICKS)
				clockwise = 0;
		} else {
			*servoPosition -= 100;
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER, *servoPosition);
			if ( *servoPosition <= MIN_PULSE_WIDTH_TICKS)
				clockwise = 1;
		}
}

void setMotionMode(MotionMode _motionMode)
{
	motionMode = _motionMode;

	if (!moving)
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

/**
 * Function: greenLED
 * Returns: None
 * Desc: Turn on or off the green LED depending on passed parameter
 */
void updateRobotMotion(double currentSpeedLeftWheel, double currentSpeedRightWheel) {

	//left wheel master, right wheel slave
	double rightWheelDiff = currentSpeedRightWheel / currentSpeedLeftWheel;
	if (rightWheelPulseWidth < 2850)
	{
		//
		rightWheelPulseWidth = rightWheelDiff * rightWheelPulseWidth;
	}
	else
	{
		rightWheelPulseWidth = rightWheelPulseWidth/rightWheelDiff;
	}

	motion_servo_set_pulse_width(MOTION_WHEEL_RIGHT, rightWheelPulseWidth);
}

void readSpeed(double *speedLeft, double *speedRight, double* distance) {
	uint32_t 	ticCountLeft = 0,
				ticCountRight = 0,
				oneRotLeft = 0,
				oneRotRight = 0,
				observationLeftCtr = 0,
				observationRightCtr = 0;

	while(observationLeftCtr < numRisingEdgesForAvg || observationRightCtr < numRisingEdgesForAvg) {
		int leftReadSuccessful = motion_enc_read(MOTION_WHEEL_LEFT, &ticCountLeft);
		int rightReadSuccessful = motion_enc_read(MOTION_WHEEL_RIGHT, &ticCountRight);

		if(leftReadSuccessful > 0 && observationLeftCtr < numRisingEdgesForAvg){
			oneRotLeft += ticCountLeft;
			observationLeftCtr += 1;
		}

		if(rightReadSuccessful > 0 && observationRightCtr < numRisingEdgesForAvg){
			oneRotRight += ticCountRight;
			observationRightCtr += 1;
		}
	}

	*speedLeft = (0.1728/(double)numRisingEdgesForAvg) / (((double)oneRotLeft / (double)numRisingEdgesForAvg) * 0.0000005);
	*speedRight = (0.1728/(double)numRisingEdgesForAvg) / (((double)oneRotLeft /(double)numRisingEdgesForAvg) * 0.0000005);
	*distance += 0.1728;
}
