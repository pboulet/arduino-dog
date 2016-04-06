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

void Sweep(void);

void FindHuman(uint8_t*);

void FollowHuman(uint8_t*);

void PanicNoHuman(void);

#endif
