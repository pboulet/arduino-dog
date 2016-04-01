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


static const uint8_t NUM_COMMANDS = 8;

const char commands[] = {
	'F',
	'B',
	'L',
	'R',
	'S',
	'A',
	'W',
	'U'
};

const char *commandsTxt[] = {
	"Mode Forward",
	"Mode Backward",
	"Spin Left",
	"Spin Right",
	"Stop",
	"Attachment Mode",
	"Web Interface Mode",
	"Unknown Command"
};

/******************************************************************************************************************/

static void InitHotSpot(void);

static void InitWebServer(void);

static void ConfigureWebPage(void);

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
void InitWebServer(void) {
	usart_print_P(PSTR("\r\n\n\nInitializing Web Server...\r\n"));

	ConfigureWebPage();
	start_web_server();

	usart_print_P(PSTR("\r\n\n\nDone initializing web server...\r\n"));
}

WebCommand GetCommand() {
	process_client_request();

	char clientResponse = get_next_client_response();

	if (clientResponse == commands[FORWARD_CMD]) {
		usart_print_P(PSTR("\r\n\n\nCommand received: Move forward \r\n"));
		return FORWARD_CMD;

	} else if (clientResponse == commands[BACKWARD_CMD]) {
		usart_print_P(PSTR("\r\n\n\nCommand received: Move backward \r\n"));
		return BACKWARD_CMD;

	} else if (clientResponse == commands[SPINLEFT_CMD]) {
		usart_print_P(PSTR("\r\n\n\nCommand received: Spin Left \r\n"));
		return SPINLEFT_CMD;

	} else if (clientResponse == commands[SPINRIGHT_CMD]) {
		usart_print_P(PSTR("\r\n\n\nCommand received: Spin Right \r\n"));
		return SPINRIGHT_CMD;

	} else if (clientResponse == commands[STOP_CMD]) {
		usart_print_P(PSTR("\r\n\n\nCommand received: Stop \r\n"));
		return STOP_CMD;

	} else if (clientResponse == commands[ATTACHMENT_MODE_CMD]) {
		usart_print_P(PSTR("\r\n\n\nCommand received: Attachment mode \r\n"));
		return ATTACHMENT_MODE_CMD;

	} else if (clientResponse == commands[WEB_CONTROL_MODE_CMD]) {
		usart_print_P(PSTR("\r\n\n\nCommand received: Web Control Mode \r\n"));
		return WEB_CONTROL_MODE_CMD;

	} else {
		usart_print_P(PSTR("\r\n\n\nCommand received: Default Command or nothing received\r\n"));
		return UNKNOWN_CMD;
	}
}

static void ConfigureWebPage(void){
	configure_web_page("Chico: The Robot", "! Control Interface !", HTML_DROPDOWN_LIST);

	for(uint8_t cmd = 0; cmd < (NUM_COMMANDS - 1); cmd += 1)
		add_element_choice(commands[cmd], commandsTxt[cmd]);
}

/******************************************************************************************************************/
