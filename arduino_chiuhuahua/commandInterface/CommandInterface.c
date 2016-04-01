/*
 * CommandInterface.c
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file CommandInterface.c
 *
 * \author Patric Boulet & Nick Dubus
 *
 * \date April 13th, 2016
 *
 * \brief
 *
 * \details
 */

/******************************************************************************************************************/

/******************************************************************************************************************/

/* --Includes-- */
#include "FreeRTOS.h"
#include "../include/usart_serial.h"
#include "../motion/Motion.h"
#include "../include/wireless_interface.h"
#include "CommandInterface.h"

/******************************************************************************************************************/

/******************************************************************************************************************/

static void InitHotSpot(void);

static void InitWebServer(void);

/***************************************** ENTRY POINTS  **********************************************************/


/*!\brief Module initializer.
 *
 *\details
 *
 * @param mode
 * @returns none
 */
void InitWebInterface(void) {
	portENABLE_INTERRUPTS();
	InitHotSpot();
	InitWebServer();
}

static void InitHotSpot(void) {
	usart_print_P(PSTR("\r\n\n\nInitializing hot spot..\r\n"));

	gs_initialize_module(USART_2, 9600, USART_0, 115200);
	gs_set_wireless_ssid("PatriceChicoTeam");
	gs_activate_wireless_connection();

	usart_print_P(PSTR("\r\n\n\nDone activating hot spot..\r\n"));
}

/*!\brief
 *
 *\details
 *
 * @param
 * @returns none
 */
static void InitWebServer(void) {
	usart_print_P(PSTR("\r\n\n\nInitializing Web Server...\r\n"));

	configure_web_page("Chico: The Robot", "! Control Interface !", HTML_DROPDOWN_LIST);

	add_element_choice('F', "Move Forward");
	add_element_choice('B', "Move Backward");
	add_element_choice('L', "Rotate Counter-Clockwise");
	add_element_choice('R', "Rotate Clockwise");

	start_web_server();

	usart_print_P(PSTR("\r\n\n\nDone initializing web server...\r\n"));
}


/******************************************************************************************************************/
