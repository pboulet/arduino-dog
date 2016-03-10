#ifndef INCLUDE_MOTION_CONTROL_H_
#define INCLUDE_MOTION_CONTROL_H_

typedef enum {FORWARD, BACKWARDS, SPINLEFT, SPINRIGHT, STOP} MotionMode;

/**Sets the forward/backward speed of the robot
 * @param[in] An integer between -100 and 100 where -100 is full reverse, 0 is stop, 100 is full forward.
 */
void setMotionMode(MotionMode);

/**
 * @param[in]
 */
void updateRobotMotion(int, int);

void readSpeed(float*, float*, float*);

#endif

