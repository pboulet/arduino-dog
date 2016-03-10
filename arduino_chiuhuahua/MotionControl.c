#include "include/MotionControl.h"
#include "include/motion.h"

MotionMode motionMode;
int rightWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;
int leftWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;
int moving = 0;
const uint8_t numRisingEdgesForAvg = 8;

void InitMotionControl(uint16_t* servoPosition) {
	*servoPosition = INITIAL_PULSE_WIDTH_TICKS;
	motion_servo_set_pulse_width(MOTION_SERVO_CENTER, *servoPosition);
}

void temperatureSweep(uint16_t* servoPosition) {
	    while(1)
	    {
	    	while(*servoPosition < MAX_PULSE_WIDTH_TICKS){
	    		*servoPosition += 10;
	    		motion_servo_set_pulse_width(MOTION_SERVO_CENTER,*servoPosition);
	    		//vTaskDelayUntil( &xLastWakeTime, ( 10 / portTICK_PERIOD_MS ) );
	    	}

	    	while(servoPosition > MIN_PULSE_WIDTH_TICKS){
	    	    		servoPosition -= 10;
	    	    		motion_servo_set_pulse_width(MOTION_SERVO_CENTER,servoPosition);
	    	    		//vTaskDelayUntil( &xLastWakeTime, ( 10 / portTICK_PERIOD_MS ) );
			}
	    }
}

void setMotionMode(MotionMode _motionMode)
{
	motionMode = _motionMode;

	if (_motionMode == STOP)
	{
		motion_servo_stop(MOTION_WHEEL_LEFT);
		motion_servo_stop(MOTION_WHEEL_RIGHT);
		moving = 0;
		return;
	}

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

void readSpeed(float *speedLeft, float *speedRight, float* distance) {
	uint32_t 	ticCountLeft,
				ticCountRight,
				oneRotLeft,
				oneRotRight,
				observationLeftCtr,
				observationRightCtr;

	ticCountLeft = 0;
	ticCountRight = 0;

	oneRotLeft = 0;
	oneRotRight = 0;

	observationLeftCtr = 0;

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

	*speedLeft = (0.1728F/(float)numRisingEdgesForAvg) / (((float)oneRotLeft / (float)numRisingEdgesForAvg) * 0.0000005F);
	*speedRight = (0.1728F/(float)numRisingEdgesForAvg) / (((float)oneRotLeft /(float)numRisingEdgesForAvg) * 0.0000005F);
	*distance += 0.1728F/numRisingEdgesForAvg;
}
