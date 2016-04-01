/*
 * CommandInterface.h
 * Chico The Robot
 *
 * Authors: Patrice Boulet & Nick Dubus
 */

/******************************************************************************************************************/

/*!	\file MotionControl.h
 *
 * \author Patrice Boulet & Nick Dubus
 *
 * \date April 13th, 2016
 *
 * \brief
 *
 * \details
 *
 *
 */

/******************************************************************************************************************/

/******************************************************************************************************************/

#ifndef INCLUDE_WEB_INTERFACE_H_
#define INCLUDE_WEB_INTERFACE_H

// we need to expose this dependency here otherwise our
// API cannot compile some of its type definitions
#include <stdint.h>

typedef enum {
	FORWARD_CMD,
	BACKWARD_CMD,
	SPINLEFT_CMD,
	SPINRIGHT_CMD,
	STOP_CMD,
	ATTACHMENT_MODE_CMD,
	WEB_CONTROL_MODE_CMD,
	UNKNOWN_CMD
} WebCommand;

/*---------------------------------------  ENTRY POINTS  ---------------------------------------------------------*/

/*!\brief Module initializer.
 *
 *\details
 *
 * @param
 * @returns none
 */
void InitWebInterface(void);


/*!\brief
 *
 *\details
 *
 * @param
 * @returns none
 */
WebCommand GetCommand(void);

#endif

