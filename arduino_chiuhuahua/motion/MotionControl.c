#include "MotionControl.h"

#include "Motion.h"

MotionMode motionMode;
int rightWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;
int leftWheelPulseWidth = INITIAL_PULSE_WIDTH_TICKS;
int moving = 0;
const uint8_t numRisingEdgesForAvg = 32;

uint32_t oneRotLeft = 0;
uint32_t oneRotRight = 0;
uint32_t observationLeftCtr = 0;
uint32_t observationRightCtr = 0;

uint32_t leftRotationTicks[32] = {[0 ... 31] = 45000};
uint32_t rightRotationTicks[32] = {[0 ... 31] = 45000};

uint16_t clockwise = 1;

void initMotionControl(uint16_t* servoPosition) {
	*servoPosition = INITIAL_PULSE_WIDTH_TICKS;
	motion_servo_set_pulse_width(MOTION_SERVO_CENTER, *servoPosition);
	motion_servo_start(MOTION_SERVO_CENTER);
}

/**
 * Function: temperatureSweep
 * Returns: None
 * Desc: recieves the current servo position, and based on its current position sweeps left or right
 */
void temperatureSweep(MotionMode mode, uint16_t* servoPosition) {
		if (mode == STOP){
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER, INITIAL_PULSE_WIDTH_TICKS);
		} else if(clockwise == 1){
            /* increment servo position to create rotation of motor. */
			*servoPosition += 250;
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER,*servoPosition);
			if ( *servoPosition >= MAX_PULSE_WIDTH_TICKS)
				clockwise = 0;
		} else {
		   /* decrement servo position to create rotation of motor in opposite direction */
			*servoPosition -= 250;
			motion_servo_set_pulse_width(MOTION_SERVO_CENTER, *servoPosition);
			if ( *servoPosition <= MIN_PULSE_WIDTH_TICKS)
				clockwise = 1;
		}
}

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
				ticCountRight = 0;


	int leftReadSuccessful = 0;
	while(leftReadSuccessful != 1)
		leftReadSuccessful = motion_enc_read(MOTION_WHEEL_LEFT, &ticCountLeft);

	int rightReadSuccessful = 0;
	while(rightReadSuccessful != 1)
		rightReadSuccessful = motion_enc_read(MOTION_WHEEL_RIGHT, &ticCountRight);


	leftRotationTicks[observationLeftCtr] = ticCountLeft;
	observationLeftCtr = (observationLeftCtr + 1 ) % (numRisingEdgesForAvg - 1);

	rightRotationTicks[observationRightCtr] = ticCountRight;
	observationRightCtr = (observationRightCtr + 1) % (numRisingEdgesForAvg - 1);

	uint32_t rotationTicksCountLeft = 0;
	uint32_t rotationTicksCountRight = 0;
	uint8_t rotationLeftTicksCountForAvg = 0;
	uint8_t rotationRightTicksCountForAvg = 0;

	for(int i = 0; i < numRisingEdgesForAvg; i++){
		if (leftRotationTicks[i] != 0) {
			rotationTicksCountLeft += leftRotationTicks[i];
			rotationLeftTicksCountForAvg++;
		}
	}

	for(int i = 0; i < numRisingEdgesForAvg; i++){
		if (rightRotationTicks[i] != 0) {
			rotationTicksCountRight += rightRotationTicks[i];
			rotationRightTicksCountForAvg++;
		}
	}

	*speedLeft = (0.1728/(double)rotationLeftTicksCountForAvg) / (((double)rotationTicksCountLeft / (double)rotationLeftTicksCountForAvg) * 0.0000005);
	*speedRight = (0.1728/(double)rotationRightTicksCountForAvg) / (((double)rotationTicksCountRight /(double)rotationRightTicksCountForAvg) * 0.0000005);
	*distance += (*speedLeft + *speedRight)/2.0 * 0.100;
}
