/*
 * sonar.h
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Sonar.c
 *
 * \author Patrice Boulet, Ladan Maxamud, Alexander Teske,
 * Adnane Gasmi, Nick Dubus, Justin Langis
 *
 * \date 2016-04-12
 *
 * \brief Module that detects the distance between the robot and
 * objects in front of it by using a sonar.
 *
 */
/******************************************************************************************************************/

#ifndef SONAR_H_
#define SONAR_H_

#include <stdint.h>

/******************************************************************************************************************/


/********************************************* Entry Points  ******************************************************/

/*!\fn InitSonarModule(void)
 * \brief Module initializer.
 *
 *\details Initializes the timer module
 *\details that is used in the sonar module.
 *
 * @returns none
 */
void InitSonarModule(void);

/*!\fn getDistance(float* objectDistance)
 * \brief Gets the distance of the object
 * in front of the robot.  Works for objects
 * ranging from 0.3m to 3m in distance.
 *
 * @param objectDistance distance between the robot
 * and the object in front of it.
 *
 * @returns none
 */
void getDistance(float*);

#endif

/******************************************************************************************************************/
