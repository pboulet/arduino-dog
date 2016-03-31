/*
 * Chico.c
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Chico.c
 *
 * \author Ladan Maxamud, Patrick Boulet, Nick Dubus,
 * 			Justin Langis, Alexander Teske & Adnane Gasmi.
 *
 * \date March 13th, 2016
 *
 * \brief Main program of Chico the robot.  This program is part
 * of an embedded system (see release documentation for further details).
 * The functions provided by the system are the following:
 * 		- Read and display ambient temperature and temperature in a 180 degrees field in front of the robot.
 * 		- Compute and display the distance traveled.
 * 		- Read and display the current speed.
 * 		- Move forward, backward, spin clockwise and counter clockwise.
 *
 * \details This modules provides a cyclic scheduler to schedule tasks to
 * be executed according to a static schedule.  Moreover, tasks are also
 * defined in this module.  Global variables are used for intertask communication
 * to exchange data, state, etc.
 */

/******************************************************************************************************************/

/******************************************************************************************************************/

/*** Standard and AVR includes ****/
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <avr/io.h>

/* FreeRTOS Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Serial interface include file. */
#include "include/usart_serial.h"

/* Modules includes. */
#include "lcd/Lcd.h"
#include "temperatureReader/TemperatureReader.h"
#include "motion/MotionControl.h"
#include "led/Led.h"
#include "motion/Motion.h"

/* Web server modules */
#include "wireless_interface.h"

/******************************************************************************************************************/

/***************************************  Function Declarations  **************************************************/

static void InitializeHotSpot();

static void InitializeWebServer();

static void ProcessWebServerRequests(void*);

static void CyclicScheduler(void*);

static void ReadTemperatures(void);

static void ReadSpeed(void);

static void Move(void);

static void UpdateLED(void);

static void UpdateInstrumentCluster(void);

static MotionMode GetMotionMode(void);

/******************************************************************************************************************/

int usartfd;
int usartfd2;


/*************************************  Type definitions & Macros  ************************************************/

/*!
 *  \typedef typedef void (*TASK)(void)
 *  \brief Task function pointer definition.
 */
typedef void (*TASK)(void);


/*!
 *  \def typedef #define NUM_MINOR_CYCLES 8
 *  \brief Number of minor cycles in the cyclic scheduler's schedule.
 */
#define NUM_MINOR_CYCLES 8

/*!
 *  \def typedef #define #define MINOR_CYCLE_TIME 50
 *  \brief Duration of minor cycles in the cyclic scheduler, in milliseconds.
 */
#define MINOR_CYCLE_TIME 50


/******************************************************************************************************************/

/******************************************* Global variables *****************************************************/
/*!
 *  \var double distanceTraveled
 *  \brief Total distance traveled by the robot.
 */
double distanceTraveled;

/*!
 *  \var uint8_t *temperatures
 *  \brief Holds the most current temperature reading from the thermal array sensor.
 */
uint8_t *temperatures;

/*!
 *  \var double *speedLeft
 *  \brief Holds the most current speed reading from the encoder of the left wheel.
 */
double *speedLeft;

/*!
 *  \var double *speedRight
 *  \brief Holds the most current speed reading from the encoder of the right wheel.
 */
double *speedRight;						/* Holds the most current speed reading from the encoder of the right wheel. */

/*!
 *  \var double *distance
 *  \brief Holds the most current total distance traveled by the robot.
 */
double *distance;

/*!
 *  \var uint16_t *centerServoPosition
 *  \brief Holds the most current position of the thermal array servo motor.
 */
uint16_t *centerServoPosition;


/*!
 *  \var TASK table[NUM_MINOR_CYCLES]
 *  \brief Cyclic scheduler task table.
 */
TASK table[NUM_MINOR_CYCLES] = {
		Move,
		ReadSpeed,
		UpdateLED,
		ReadSpeed,
		ReadTemperatures,
		ReadSpeed,
		UpdateInstrumentCluster,
		ReadSpeed
};

/******************************************************************************************************************/

/****************************************  ENTRY POINTS  **********************************************************/


/*! \brief Main program entry point.  Initialize serial port communication, all
 * modules required for the functionality of the robot, allocates memory for intertask
 * communication global variables.

 *
 * \details Initialize serial port communication, allocates memory for intertask
 * communication global variables and initializes all modules required for the functionality of
 * the robot before enabling interupts.  A FreeRTOS task consisting of the scheduler is
 * creating and the FreeRTOS scheduler is started, along with interupts being enabled.
 *
 * @return the status of the program when it ends
 */
int main(void)
{
    /* Turn on the serial port for debugging or for other USART reasons. */
	/*  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1) */
	usartfd = usartOpen( USART_2, 9600, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX);
	usartfd2 = usartOpen( USART_0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX);


	motion_init();
	initMotionControl(centerServoPosition);
	initTemperatureReader();
	initLCD();


	usart_print_P(PSTR("\r\n\n\nChico: Initializing...\r\n"));


	portENABLE_INTERRUPTS();
	InitializeHotSpot();
	InitializeWebServer();

	xTaskCreate(ProcessWebServerRequests, (const portCHAR *)"Process Web Server Requests", 1024, NULL, 1, NULL);
	vTaskStartScheduler();


/*	 Allocate memory for intertask communication variables.
	temperatures = malloc(sizeof(uint8_t)*9);
	speedLeft = malloc(sizeof(double));
	speedRight = malloc(sizeof(double));
	distance = malloc(sizeof(double));
	centerServoPosition = malloc(sizeof(uint16_t));

	 No distance was traveled on startup.
	*distance = 0;*/


	usart_print_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...
}

static void InitializeHotSpot() {
	usart_print_P(PSTR("\r\n\n\nInitializing hot spot..\r\n"));

	/* Initialize the hot spot through the GainSpan module. */
	gs_initialize_module(USART_2, 9600, USART_0, 115200);
	gs_set_wireless_ssid("PatriceChicoTeam");
	gs_activate_wireless_connection();
	usart_print_P(PSTR("\r\n\n\nActivating hot spot..\r\n"));


	usart_print_P(PSTR("\r\n\n\nDone activating hot spot..\r\n"));
}

static void InitializeWebServer() {
	usart_print_P(PSTR("\r\n\n\nRunning web server initialization task...\r\n"));
	usart_print_P(PSTR("\r\n\n\nHot spot has been initialized, configuring Web Server...\r\n"));
	configure_web_page("Chico: The Robot", "! Control Interface !", HTML_DROPDOWN_LIST);
	add_element_choice('F', "Forward");
	add_element_choice('R', "Reverse");
	start_web_server();
	usart_print_P(PSTR("\r\n\n\nStarting Web Server...\r\n"));
	usart_print_P(PSTR("\r\n\n\nDone starting web server...\r\n"));
}


static void ProcessWebServerRequests(void* gvParameters) {
	usart_print_P(PSTR("\r\n\n\nRunning web server request processing task...\r\n"));
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	usart_print_P(PSTR("\r\n\n\nWeb server has been initialized, processing requests...\r\n"));

	usart_print_P(PSTR("\r\n\n\nWeb server: processing client requests"));
	while(1) {
		process_client_request();
//		usart_print_P(PSTR("\r\n\n\nWeb server: client requests processed\r\n"));


		char clientResponse = get_next_client_response();
//		usart_print_P(PSTR("\r\n\n\nWeb server: client response was %c\r\n"));
		vTaskDelayUntil( &xLastWakeTime, (5500 / portTICK_PERIOD_MS));
	}
}

/*!\fn CyclicScheduler(void* gvParameters)
 *\brief Cyclic Scheduler task (FreeRTOS task).
 *
 *\details  Runs tasks defined in the static cyclic
 * schedule sequentially undefinitely following the minor cycle duration
 * defined.
 *
 * @param gvParameters task parameters (not used in the context of this system right now)
 * @returns none
 */
static void CyclicScheduler(void* gvParameters) {
    TickType_t xLastWakeTime;							// keeps track of timing

	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	int taskNum = 0;
	/* Cycle through the cyclic scheduling table indefinitely. */
	while(1) {
		if(table[taskNum] != NULL)
			table[taskNum]();
		/* Set minor cycle time. */
		vTaskDelayUntil( &xLastWakeTime, ( MINOR_CYCLE_TIME / portTICK_PERIOD_MS ));
		taskNum = (taskNum+1) % NUM_MINOR_CYCLES;
	}
}

/*! \fn static void ReadSpeed()
 * \brief Reads speed of wheels, the distance traveled,
 * 	and triggers execution of the controller.
 *
 *\details  If the robot is not stopped, reads the speed of both wheels and
 * 	and computes the distance traveled.  Then it triggers the speed controller
 * 	function to adjust the speed of both wheels to match.  If the robot is stopped
 * 	then it only sets the speed of both wheels to zero.
 *
 * @param gvParameters task parameters (not used in the context of this system right now)
 * @returns none
 */
static void ReadSpeed() {
	if ( GetMotionMode() != STOP){
		readSpeed(speedLeft, speedRight, distance);
		updateRobotMotion(*speedLeft, *speedRight);
	} else {
		*speedLeft = 0;
		*speedRight = 0;
	}
}

/*!\brief Sweeps the servo motor of the thermal array sensor from
 * left to right and inversely.  Fetches temperature readings from
 * the thermal array sensor.
 *
 *\details  If the robot is not stopped, reads the speed of both wheels and
 * 	and computes the distance traveled.  Then it triggers the speed controller
 * 	function to adjust the speed of both wheels to match.  If the robot is stopped
 * 	then it only sets the speed of both wheels to zero.
 *
 * @param gvParameters task parameters (not used in the context of this system right now)
 * @returns none
 */
static void ReadTemperatures(void) {
	temperatureSweep(GetMotionMode(), centerServoPosition);
	getTemperatureFromSensor(temperatures);
}

/*!\brief Updates the LED to reflect the current motion mode.
 *
 *\details  If the robot is moving forwards then it sets the LED
 * color to green, if moving backward it sets its color to RED, if
 * its spinning it sets its color to blue and finally if its stopped
 * the color is set to white.
 *
 * @returns none
 */
static void UpdateLED()
{
	switch(GetMotionMode()){
		case FORWARD:
			lightLED(GREEN);
			break;
		case BACKWARD:
			lightLED(RED);
			break;
		case SPINLEFT:
		case SPINRIGHT:
			lightLED(BLUE);
			break;
		case STOP:
			lightLED(WHITE);
			break;
	}
}

/*!\brief Updates data displayed on the instrument cluster
 * of the robot (LCD display).
 *
 *\details	First computes the average speed of the wheels.  Then computes
 * the average temperature readings for the four leftmost pixels on the
 * thermal array sensor and does the same for the four rightmost pixels.
 * Finally, it prints the current speed and distance traveled on the first
 * line of the LCD display as well as the ambient temperature and the
 * average pixels temperature readings on the second row.
 *
 * @returns none
 */
static void UpdateInstrumentCluster(void){
		clearLCD();

		/* Average speed of the two wheels. */
		double speed = (*speedLeft + *speedRight)/ 2.0;

		/* Average of the first four temperature pixels from the left. */
		double tempAvgLeft = (temperatures[1] + temperatures[2] + temperatures[3] + temperatures[4])/4.0;

		/* Average of the first four temperature pixels from the right. */
		double tempAvgRight = (temperatures[5] + temperatures[6] + temperatures[7] + temperatures[8])/4.0;

		char topRow[16] = "";
		char bottomRow[16] = "";

		/*
		 * Prints the average speed of the two wheels as well as the
		 * total distance traveled on the first line of the LCD screen.
		 */
		sprintf(topRow, "%.2f m/s %.2f m", speed, *distance);
		writeLCDRowOne(topRow);

		/*
		 * Prints the ambient temperature, followed by the average temperature for the first four pixel sensors from the left
		 * as well as the average temperature for the first four pixel sensors from the right on the second line of the LCD display.
		 */
		sprintf(bottomRow, "%u %.2f %.2f", temperatures[0], tempAvgLeft, tempAvgRight);
		writeLCDRowTwo(bottomRow);
}

/*!\brief Sets the motion mode of the robot.
 *
 *\details  Delegate task function that calls the
 * motion control module to update the current motion
 * mode status of the system.
 *
 * @returns none
 */
static void Move(void){
	setMotionMode(GetMotionMode());
}

/*!\brief Returns the motion mode of the system
 * based on the distance traveled.
 *
 *\details  It currently follows a static demo choregraphy where it goes 1m forward,
 * 1m backward, does a few rotations counter clockwise, does a few
 * rotations clockwise, and finally stops.
 *
 * @returns current motion mode of the system
 */
static MotionMode GetMotionMode(void){
	if ( *distance < 1.0) {
		return FORWARD;
	} else if ( *distance < 2.0) {
		return BACKWARD;
	} else if ( *distance < 3.0) {
		return SPINLEFT;
	} else if ( *distance < 4.0) {
		return SPINRIGHT;
	} else {						// demo choregraphy is done
		return STOP;
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, portCHAR *pcTaskName )
{
	usart_print_P(PSTR("\r\n\n\n*************STACK OVERFLOW OCCURED************..\r\n"));
	while(1);
}

void vApplicationIdleHook(void){
	usart_print_P(PSTR("\r\n\n\n*************Idle hook hit************..\r\n"));
}

