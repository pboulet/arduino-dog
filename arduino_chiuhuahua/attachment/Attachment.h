/*
 * Attachment.h
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Attachment.h
 *
 * \author  Ladan Maxamud, Adnane Gasmi, Patrice Boulet and Alex Teske
 *
 * \date 2016-04-12
 *
 * \brief Module that handles the attachment
 * of the robot to humans.
 *
 * \details This modules provides functionality to lock a heat source that
 * is close to human temperature, follow it until it attaches to it,
 * or panic if it doesn't find anything to follow for a while.
 */

/******************************************************************************************************************/

#ifndef _ATTACHMENT_H_
#define _ATTACHMENT_H_

// we need to expose this dependency here otherwise our
// API cannot compile some of its type definitions
#include <stdint.h>

/******************************************************************************************************************/

typedef enum {TARGET_HIT, SEARCHING, LOCKED_ON_TARGET, PANIC} AttachmentState;

/******************************************************************************************************************/

/*! \fn FindHuman(uint8_t* temperatures, AttachmentState* state)
 *  \brief Checks if the heat of a human is found.
 *
 *  \details  Checks if the heat of a human is found in the
 *  thermal array sensor's range and updates the "found human"
 *  state and attachment state.
 *
 *  @param temperatures latest temperatures captures by the thermal array sensors
 *  @param state the attachment state of the robot
*/
void FindHuman(uint8_t*, AttachmentState*);

/*! \fn PanicNoHuman(AttachmentState *state)
 *  \brief Panic because there hasn't been any human
 *  found recently.
 *
 *  \details  If no human has been found for quite some
 *  time then it sets the robot to panic mode (spin left
 *  without searching for any human) for a minute
 *  before coming back to search mode.
 *
 *  @param state attachment state of the robot
*/
void FollowHuman(uint8_t*, AttachmentState*);

/*! \fn FollowHuman(uint8_t* temperatures, AttachmentState *state)
 *  \brief Makes the robot follow the heat source of a human.
 *
 *  \details  Moves the robot towards the human target locked in to
 *  be followed if the robot hasn't attached to it.  If the average
 *  temperature of captured by the sensors is higher on the left, turn left.
 *  Similarly, if the average temperature captured by the sensors is higher on the
 *  right, turn right.  Finally, if the average temperature is higher right in
 *  front of the robot, just go forward.
 *
 *  @param temperatures latest temperatures captures by the thermal array sensors
 *  @param state the attachment state of the robot
*/
void PanicNoHuman(AttachmentState*);

#endif
