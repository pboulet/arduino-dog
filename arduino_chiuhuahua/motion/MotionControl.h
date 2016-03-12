#ifndef INCLUDE_MOTION_CONTROL_H_
#define INCLUDE_MOTION_CONTROL_H_

// we need to expose this dependency here otherwise our
// API cannot compile some of its type definitions
#include <stdint.h>

typedef enum {FORWARD, BACKWARD, SPINLEFT, SPINRIGHT, STOP} MotionMode;

void initMotionControl(uint16_t*);

/**Sets the forward/backward speed of the robot
 * @param[in] An integer between -100 and 100 where -100 is full reverse, 0 is stop, 100 is full forward.
 */
void setMotionMode(MotionMode);

void updateRobotMotion(double, double);

void readSpeed(double*, double*, double*);

/**
 * Function: temperatureSweep
 * Returns: None
 * Desc: recieves the current servo position, and based on its current position sweeps left or right
 */
void temperatureSweep(MotionMode, uint16_t*);

#endif

