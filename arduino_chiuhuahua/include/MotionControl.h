#ifndef INCLUDE_MOTION_CONTROL_H_
#define INCLUDE_MOTION_CONTROL_H_

typedef enum {FORWARD, BACKWARD, SPINLEFT, SPINRIGHT, STOP} MotionMode;

void initMotionControl(uint16_t*);

/**Sets the forward/backward speed of the robot
 * @param[in] An integer between -100 and 100 where -100 is full reverse, 0 is stop, 100 is full forward.
 */
void setMotionMode(MotionMode);

void updateRobotMotion(int, int);

void readSpeed(float*, float*, float*);

void temperatureSweep();

#endif

