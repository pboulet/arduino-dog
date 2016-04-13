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
 * \brief Sets up a web server that processes
 * client response and parses them to their
 * associated commands in the context of the robot.
 */

/******************************************************************************************************************/

/******************************************************************************************************************/

/* --Includes-- */
#include "FreeRTOS.h"
#include "../include/usart_serial.h"
#include "../motion/Motion.h"
#include "wireless_interface.h"
#include "CommandInterface.h"

/******************************************************************************************************************/

/*!
 * \var const uint8_t NUM_COMMANDS
 * \brief total number of valid commands
 */
static const uint8_t NUM_COMMANDS = 9;

/*!
 * Command character client response map.
 * Indexes are mapped to the WebCommand
 * enumeration.
 */
const char commands[] = {
	'F',
	'B',
	'L',
	'R',
	'S',
	'A',
	'T',
	'D',
	'U'
};

/*
 * Command name strings web interface map.
 * Each string represents the name with which the
 * command will be presented to the user in the
 * web command options dropdown.
 * Indexes are mapped to the WebCommand
 * enumeration.
 */
const char *commandsTxt[] = {
	"Mode Forward",
	"Mode Backward",
	"Spin Left",
	"Spin Right",
	"Stop",
	"Attachment Mode",
	"Temperature Scanning",
	"Distance Scanning",
	"Unknown Command"
};

/******************************************************************************************************************/

static void InitHotSpot(void);

static void InitWebServer(void);

static void ConfigureWebPage(void);

/***************************************** ENTRY POINTS  **********************************************************/


/*!\fn InitWebInterface(void)
 * \brief Module initializer.
 *
 *\details This entry point first enables interrupts in the system,
 *\details which is required for the module to function correctly.
 *\details It then initializes the wi-fi hot spot and web server sequentially.
 *
 * @returns none
 */
void InitWebInterface(void) {
	portENABLE_INTERRUPTS();
	InitHotSpot();
	InitWebServer();
}

/*!\fn GetCommand()
 * \brief Parses client response and returns the associated
 * WebCommand.
 *
 * @returns WebCommand the WebCommand associated with the client response
 * character, if any.
 */
WebCommand GetCommand() {
	char clientResponse = process_client_request();

	if (clientResponse == commands[FORWARD_CMD]) {
		return FORWARD_CMD;

	} else if (clientResponse == commands[BACKWARD_CMD]) {
		return BACKWARD_CMD;

	} else if (clientResponse == commands[SPINLEFT_CMD]) {
		return SPINLEFT_CMD;

	} else if (clientResponse == commands[SPINRIGHT_CMD]) {
		return SPINRIGHT_CMD;

	} else if (clientResponse == commands[STOP_CMD]) {
		return STOP_CMD;

	} else if (clientResponse == commands[ATTACHMENT_MODE_CMD]) {
		return ATTACHMENT_MODE_CMD;

	} else if (clientResponse == commands[SCAN_TEMPERATURE_CMD]) {
		return SCAN_TEMPERATURE_CMD;

	} else if ( clientResponse == commands[SCAN_DISTANCE_CMD]) {
		return SCAN_DISTANCE_CMD;

	} else {
		return UNKNOWN_CMD;
	}
}


/*! \fn InitHotSpot(void)
 *  \brief Initializes the gainspan wi-fi hot spot.
 *
 *  \details Initializes the serial communication by
 *  binding appropriate ports and setting the right baud rate
 *  for each one of them.  Then activates the wi-fi hardware
 *  to initialize connections.
*/
static void InitHotSpot(void) {
	usart_print_P(PSTR("\r\n\n\nInitializing hot spot..\r\n"));

	/* Binds the ports on the hardware that will be used for serial communication and
	 * set the baud rate for each one of them.  USART_2 is used for communication with the
	 * Gainspan WiFi module with a baud rate of 9600 and USART_0 is used for communication
	 * with the serial terminal at a baud rate of 115200. */
	gs_initialize_module(USART_2, 9600, USART_0, 115200);

	/* Sets the wireless network’s SSID. */
	gs_set_wireless_ssid("PatriceChicoTeam");

	/* Activates Gaispan WiFi device in Limited AP mode using the configuration
	 * parameters from structure GAINSPAN. */
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

/*!\brief Configures the web page served by the web server.
 *
 *\details Configure web-page with details of web-page title, HTML element type and
 *\details adds all the supported commands to the command dropdown options.
 *
 * @param	none
 * @returns none
 */
static void ConfigureWebPage(void){
	configure_web_page("Chico: The Robot", "! Control Interface !", HTML_DROPDOWN_LIST);

	/* Add a dropdown option for each valid web command on the web page. */
	for(uint8_t cmd = 0; cmd < (NUM_COMMANDS - 1); cmd += 1)
		add_element_choice(commands[cmd], commandsTxt[cmd]);
}

/******************************************************************************************************************/
