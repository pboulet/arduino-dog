/*
 * Attachment.h
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Attachment.h
 *
 * \author
 *
 * \date
 *
 * \brief  Module for Attachment Mode
 *
 */

/******************************************************************************************************************/

#ifndef _ATTACHMENT_H_
#define _ATTACHMENT_H_

// we need to expose this dependency here otherwise our
// API cannot compile some of its type definitions
#include <stdint.h>

typedef enum {TARGET_HIT, SEARCHING, LOCKED_ON_TARGET, PANIC} AttachmentState;

void FindHuman(uint8_t*, AttachmentState*);

void FollowHuman(uint8_t*, AttachmentState*);

void PanicNoHuman(AttachmentState*);

#endif
