/*
 * CommandInterface.h
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file CommandInterface.h
 *
 * \author Patric Boulet & Nick Dubus
 *
 * \date April 13th, 2016
 *
 * \brief Sets up a web server that processes
 * client response and parses them to their
 * associated commands in the context of the robot.
 */

/******************************************************************************************************************/

/******************************************************************************************************************/

#ifndef INCLUDE_WEB_INTERFACE_H_
#define INCLUDE_WEB_INTERFACE_H

// we need to expose this dependency here otherwise our
// API cannot compile some of its type definitions
#include <stdint.h>

/*!
 * \enum WebCommand
 * \brief All possible commands to be received
 * from the web interface.
 */
typedef enum {
	FORWARD_CMD,
	BACKWARD_CMD,
	SPINLEFT_CMD,
	SPINRIGHT_CMD,
	STOP_CMD,
	ATTACHMENT_MODE_CMD,
	SCAN_TEMPERATURE_CMD,
	SCAN_DISTANCE_CMD,
	UNKNOWN_CMD
} WebCommand;

/*---------------------------------------  ENTRY POINTS  ---------------------------------------------------------*/

/*!\fn InitWebInterface(void)
 * \brief Module initializer.
 *
 *\details This entry point first enables interrupts in the system,
 *\details which is required for the module to function correctly.
 *\details It then initializes the wi-fi hot spot and web server sequentially.
 *
 * @returns none
 */
void InitWebInterface(void);


/*!\fn GetCommand()
 * \brief Parses client response and returns the associated
 * WebCommand.
 *
 * @returns WebCommand the WebCommand associated with the client response
 * character, if any.
 */
WebCommand GetCommand(void);

/******************************************************************************************************************/

#endif

