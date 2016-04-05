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
#include "commandInterface/CommandInterface.h"

/******************************************************************************************************************/

/***************************************  Function Declarations  **************************************************/

static void ReadObjectDistance(void*);

static void ProcessWebServerRequests(void*);

static void Move(void*);

static void ReadTemperatures(void);

static void ReadSpeed(void);

static void UpdateLED(void);

static void UpdateInstrumentCluster(void);

static void UpdateMotionMode(WebCommand);

static void UpdateMode(WebCommand);

static void InitUsart(void);

static void InitIntertaskCommunication(void);

static void InitSubModules(void);

static void CreateTasks(void);

/******************************************************************************************************************/

int usartfd;
int usartfd2;


/*************************************  Type definitions & Macros  ************************************************/

/*!
 *  \typedef typedef void (*TASK)(void)
 *  \brief Task function pointer definition.
 */
typedef void (*TASK)(void);


/******************************************************************************************************************/

/******************************************* Global variables *****************************************************/

typedef enum {
	ATTACHMENT,
	WEB_CONTROL
} Mode;

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
 *  \var MotionMode motionMode
 *  \brief Holds the current motion mode of the robot.
 */
MotionMode motionMode;

/*!
 *	\var Mode mode
 *	\brief Holds the current mode of the robot (either attachment or web control)
 */
Mode mode;

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
	usart_print_P(PSTR("\r\n\n\nChico: Initializing...\r\n"));

	InitUsart();
	InitIntertaskCommunication();
	InitSubModules();
	CreateTasks();

	vTaskStartScheduler();

	usart_print_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...
}

static void InitSubModules(void) {
	InitMotionControl(centerServoPosition);
	InitTemperatureReader();
	InitLCD();
	//InitSonarModule();

	/* Needs to be the last sub-module initialized because it enables interrupts. */
	InitWebInterface();
}

static void InitUsart(void){
    /* Turn on the serial port for debugging or for other USART reasons. */
	/*  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1) */
	usartfd = usartOpen( USART_2, 9600, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX);
	usartfd2 = usartOpen( USART_0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX);
}

static void InitIntertaskCommunication(void) {
	/*	 Allocate memory for intertask communication variables. */
	temperatures = malloc(sizeof(uint8_t)*9);
	speedLeft = malloc(sizeof(double));
	speedRight = malloc(sizeof(double));
	distance = malloc(sizeof(double));
	centerServoPosition = malloc(sizeof(uint16_t));

	/* No distance was traveled on startup. */
	*distance = 0;
	mode = WEB_CONTROL;
	motionMode = STOP;
}

static void CreateTasks(void) {
	//xTaskCreate(ReadObjectDistance, (const portCHAR *)"ReadObjectDistance", 256, NULL, 5, NULL);
	xTaskCreate(Move, (const portCHAR *)"Move", 256, NULL, 1, NULL);
	xTaskCreate(ProcessWebServerRequests, (const portCHAR *)"Process Web Server Requests", 1024, NULL, 2, NULL);
}


static void ProcessWebServerRequests(void* gvParameters) {
	TickType_t xLastWakeTime;

	while(1) {
		xLastWakeTime = xTaskGetTickCount();

		usart_print_P(PSTR("\r\n\n\nRunning Process Web Server Requests Task\r\n"));

		WebCommand cmd = GetCommand();
		UpdateMotionMode(cmd);
		UpdateMode(cmd);

		vTaskDelayUntil( &xLastWakeTime, (5000 / portTICK_PERIOD_MS));
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
	if ( motionMode != STOP){
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
	temperatureSweep(motionMode, centerServoPosition);
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
	switch(motionMode){
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
static void Move(void* gvParameters){
	TickType_t xLastWakeTime;

	while(1){
		xLastWakeTime = xTaskGetTickCount();
		usart_print_P(PSTR("\r\n\n\nRunning Move Task\r\n"));
		if ( motionMode != UNKNOWN ) {
			setMotionMode(motionMode);
		}
		vTaskDelayUntil( &xLastWakeTime, (2000 / portTICK_PERIOD_MS));
	}
}

static void ReadObjectDistance(void* gvParameters) {
	TickType_t xLastWakeTime; // keeps track of timing

	/* The xLastWakeTime variable needs to be initialised with the current tick
	 count.  Note that this is the only time we access this variable.  From this
	 point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	 API function. */
	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		sendPulse();
		receivePulse();
		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_PERIOD_MS));
	}
}

/*!\brief
 *
 *\details
 *
 * @returns
 */
static void UpdateMotionMode(WebCommand cmd){
	switch(cmd){
		case FORWARD_CMD:
			motionMode = FORWARD;
			usart_print_P(PSTR("\r\n\n\nMotion mode is now: forward\r\n"));
			break;
		case BACKWARD_CMD:
			motionMode = BACKWARD;
			usart_print_P(PSTR("\r\n\n\nMotion mode is now: backward\r\n"));
			break;
		case SPINLEFT_CMD:
			motionMode = SPINLEFT;
			usart_print_P(PSTR("\r\n\n\nMotion mode is now: spin left\r\n"));
			break;
		case SPINRIGHT_CMD:
			motionMode = SPINRIGHT;
			usart_print_P(PSTR("\r\n\n\nMotion mode is now: spin right\r\n"));
			break;
		case STOP_CMD:
			motionMode = STOP;
			usart_print_P(PSTR("\r\n\n\nMotion mode is now: stop\r\n"));
			break;
		default:
			// no change in the motion mode
			break;
	}
}

/*!\brief
 *
 *\details
 *
 * @returns
 */
static void UpdateMode(WebCommand cmd) {
	switch(cmd) {
		case ATTACHMENT_MODE_CMD:
			break;
		case WEB_CONTROL_MODE_CMD:
			break;
		default:
			// no change in mode
			break;
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, portCHAR *pcTaskName )
{
	while (1) {
		usart_print_P(PSTR("\r\n\n\n*****STACK OVERFLOW OCCURED****\r\n"));
	}
}
