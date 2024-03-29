
/*
 * Chico.c
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Chico.c
 *
 * \author Ladan Maxamud, Patrice Boulet, Nick Dubus,
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
 * 		- Track and follow a human
 * 		- Read and display the distance of objects in front of the robot.
 *
 * \details This modules provides rate-monotonic scheduler to schedule tasks to
 * be executed according to their priorities.  Moreover, tasks are also
 * defined in this module.  Global variables are used for intertask communication
 * to exchange data and various finite state machine states.
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
#include "sonar/sonar.h"
#include "lcd/Lcd.h"
#include "temperatureReader/TemperatureReader.h"
#include "motion/MotionControl.h"
#include "led/Led.h"
#include "motion/Motion.h"
#include "commandInterface/CommandInterface.h"
#include "attachment/Attachment.h"

/******************************************************************************************************************/

/***************************************  Function Declarations  **************************************************/

static void CommandMode(void* gvParameters);

//static void AttachmentMode(void* gvParameters);

static void ScanTemperatures(void);

static void Attach(void);

static void ProcessWebServerRequests(void*);

static void UpdateInstrumentCluster(void);

static void Move(void);

static void ReadSpeed(void);

static void UpdateLED(void);

static void UpdateModes(WebCommand);

static void InitUsart(void);

static void InitIntertaskCommunication(void);

static void InitSubModules(void);

static void CreateTasks(void);

/******************************************************************************************************************/

int usartfd;
int usartfd2;

/*************************************  Type definitions & Macros  ************************************************/


/******************************************************************************************************************/

/******************************************* Global variables *****************************************************/

/*!
 * \enum Mode
 * \brief Current mode of the robot.
 */
typedef enum {
	ATTACHMENT, MOVEMENT, SCAN_TEMPERATURES, SCAN_DISTANCE
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
uint8_t* temperatures;

/*!
 *  \var double *speedLeft
 *  \brief Holds the most current speed reading from the encoder of the left wheel.
 */
double *speedLeft;

/*!
 *  \var double *speedRight
 *  \brief Holds the most current speed reading from the encoder of the right wheel.
 */
double *speedRight; /* Holds the most current speed reading from the encoder of the right wheel. */

/*!
 *  \var double *distance
 *  \brief Holds the most current total distance traveled by the robot.
 */
double *distance;

/*!
 *  \var double *objectDistance
 *  \brief Holds the distance of the object in front of the robot.
 */
float *objectDistance;

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

/*!
 * \var AttachmentState attachmentState
 * \brief Holds the attachment state.
 */
AttachmentState *attachmentState;


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
int main(void) {
	InitUsart();

	usart_fprintf_P(usartfd2, PSTR("\r\n\n\nChico: Initializing...\r\n"));

	InitIntertaskCommunication();
	InitSubModules();
	CreateTasks();

	//usart_printf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.
	vTaskStartScheduler();

	usart_fprintf_P(usartfd2,
			PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...
}


/*!\fn InitSubModules(void)
 * \brief Initializes all the sub-modules required by the tasks.
 *
 *\details Initializes the motion, temperature reader, LCD, sonar
 *\details and web interface modules that are required for the tasks
 *\details to run.
 *
 * @returns none
 */
static void InitSubModules(void) {
	InitMotionControl(centerServoPosition);
	InitTemperatureReader();
	InitLCD();
	InitSonarModule();

	/* Needs to be the last sub-module initialized because it enables interrupts. */
	InitWebInterface();
}

/*!\fn InitUsart(void)
 * \brief Opens communication ports for usart serial communication.
 *
 * @returns none
 */
static void InitUsart(void) {
	/* Turn on the serial port for debugging or for other USART reasons. */
	/*  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1) */
	usartfd = usartOpen(USART_2, 9600, portSERIAL_BUFFER_TX,
	portSERIAL_BUFFER_RX);
	usartfd2 = usartOpen(USART_0, 115200, portSERIAL_BUFFER_TX,
	portSERIAL_BUFFER_RX);
}

/*!\fn InitIntertaskCommunication(void)
 *
 * \brief Initializes intertask communication variables.
 *
 * \details Allocates memory for intertask communication
 * variables as well as assign initial values to them
 * when applicable.
 *
 * @returns none
 */
static void InitIntertaskCommunication(void) {

	/*	 Allocate memory for intertask communication variables. */
	temperatures = malloc(sizeof(uint8_t) * 9);
	speedLeft = malloc(sizeof(double));
	speedRight = malloc(sizeof(double));
	distance = malloc(sizeof(double));
	centerServoPosition = malloc(sizeof(uint16_t));
	objectDistance = malloc(sizeof(float));
	attachmentState = malloc(sizeof(AttachmentState));

	/* No distance was traveled on startup. */
	*distance = 0;
	*speedLeft = 0;
	*speedRight = 0;
	mode = MOVEMENT;
	motionMode = STOP;
	*attachmentState = SEARCHING;
}

/*!\fn CreateTasks(void)
 *
 * \brief Create FreeRTOS tasks that are run by the scheduler.
 *
 * \details Creates tasks that are run by the scheduler, assigning
 * them a name, allocated stack memory and priority.
 *
 * @returns none
 */
static void CreateTasks(void) {
	xTaskCreate(CommandMode, (const portCHAR*)"Execute CommandMode",
			256, NULL, 4, NULL);
	xTaskCreate(ProcessWebServerRequests, (const portCHAR *)"Process Web Server Requests",
			1024, NULL, 5, NULL);
}

/*!\fn ProcessWebServerRequests(void* gvParameters)
 *
 * \brief Web Server Processing Task. Processes web requests from the
 * web server.  Also parses the client's response.
 *
 * \details Parses the client's web server response
 * and updates the motion mode accordingly.
 *
 * @returns none
 */
static void ProcessWebServerRequests(void* gvParameters) {
	TickType_t xLastWakeTime;

	while (1) {
		xLastWakeTime = xTaskGetTickCount();

		WebCommand cmd = GetCommand();
		UpdateModes(cmd);

		vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
	}
}

/*!\fn CommandMode(void* gvParameters)
 *
 * \brief Comand Mode Task.
 *
 * \details Parses the client's web server response
 * and updates the motion mode accordingly.
 *
 * @returns none
 */
static void CommandMode(void* gvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1) {

		ScanTemperatures();

		if (mode != ATTACHMENT) {

			/* Don't read the object distance unless
			 * you absolutely have to. */
			if (mode == SCAN_DISTANCE)
				getDistance(objectDistance);

			Move();
			UpdateLED();

			/* Don't read sped unless you absolutely
			 * have to. */
			if (mode == MOVEMENT)
				ReadSpeed();

		} else {		// in attachment mode

			Attach();
			FollowHuman(temperatures, attachmentState);
			getDistance(objectDistance);
			PanicNoHuman(attachmentState);
		}

		UpdateInstrumentCluster();

		/* If mode is attachment, the period is shorter to get
		 * a more precise follow functionality. */
		if (mode == ATTACHMENT)
			vTaskDelayUntil(&xLastWakeTime, (400 / portTICK_PERIOD_MS));
		else
			vTaskDelayUntil(&xLastWakeTime, (3000 / portTICK_PERIOD_MS));
	}
}

/*! \fn static void Attach()
 * \brief Tries to lock in on a human
 * target by searching for its heat.
 *
 * @returns none
 */
static void Attach() {
	/* Lowest distance the sensor can measure is 0.3 m
	 * therefore we use an offset measured at the test track. */
	if (*objectDistance < 0.42) {
		setMotionMode(STOP);
		*attachmentState = TARGET_HIT;
	} else {
		if ((*attachmentState) != PANIC)
			*attachmentState = SEARCHING;
		FindHuman(temperatures, attachmentState);
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
if (mode == MOVEMENT) {
	if (motionMode != STOP) {
		readSpeed(speedLeft, speedRight, distance);
		updateRobotMotion(*speedLeft, *speedRight);
	} else {
		*speedLeft = 0;
		*speedRight = 0;
	}
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
//static void ScanTemperatures(void*  gvParameters) {
static void ScanTemperatures(void) {

	if (mode == ATTACHMENT) {
		temperatureSweep(STOP, centerServoPosition);

	} else if (mode == SCAN_TEMPERATURES) {
		/* Override just so it keeps sweeping for now. */
		temperatureSweep(FORWARD, centerServoPosition);

	} else if (mode == SCAN_DISTANCE) {
		temperatureSweep(STOP, centerServoPosition);

	} else {
		temperatureSweep(motionMode, centerServoPosition);

	}
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
static void UpdateLED() {

	switch (motionMode) {

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
 *\brief	Displays different the robot's status and information captured
 *\brief	depending on the current mode.
 *
 *\details	- Displays the temperatures captures by the thermal
 *\details	array sensors if in SCAN_TEMPERATUREs mode
 *\details
 *\details	- Displays the distance of the object in front of the
 *\details	robot if in SCAN_DISTANCE mode.
 *\details
 *\details	- Displays the attachment state of the robot if
 *\details	in ATTACHMENT mode.
 *\details
 *\details	- Displays the current speed and distance traveled
 *\details	if in MOVEMENT mode.
 *
 * @returns none
 */
static void UpdateInstrumentCluster(void) {
	clearLCD();

	char topRow[16] = "";
	char bottomRow[16] = "";

	if (mode == SCAN_TEMPERATURES) {

		/* Print the temperatures captures by the thermal
		 * array sensors. */
		sprintf(topRow, "%02d %02d %02d %02d [%02d]", temperatures[1],
				temperatures[2], temperatures[3], temperatures[4], temperatures[0]);

		sprintf(bottomRow, "%02d %02d %02d %02d", temperatures[5], temperatures[6],
				temperatures[7], temperatures[8]);

	} else if (mode == SCAN_DISTANCE) {

		/* Print the distance of the object in front
		 * of the robot. */
		sprintf(topRow, "Scan Distance");
		sprintf(bottomRow, "Object at:%.2f m", *objectDistance);

	} else if (mode == ATTACHMENT) {

		/* Print the attachment state. */
		AttachmentState state = *attachmentState;
		if (state == PANIC) {
			sprintf(topRow, "PANIC!");
		} else if (state == SEARCHING) {
			sprintf(topRow, "SEARCHING...");
		} else if (state == LOCKED_ON_TARGET) {
			sprintf(topRow, "LOCKED ON");
		} else if (state == TARGET_HIT) {
			sprintf(topRow, "TARGET HIT!");
		}
		sprintf(bottomRow, "Object at:%.2f m", *objectDistance);

	} else if (mode == MOVEMENT) {

		/* Average speed of the two wheels. */
		double speed = (*speedLeft + *speedRight) / 2.0;

		/*
		 * Prints the average speed of the two wheels as well as the
		 * total distance traveled on the first line of the LCD screen.
		 */
		sprintf(topRow, "%.2f m/s %.2f m", speed, *distance);
	}

	writeLCDRowOne(topRow);
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
//static void Move(void* gvParameters){
static void Move(void) {
	if (motionMode != UNKNOWN) {
		setMotionMode(motionMode);
	}
}

/*!\brief Updates modes in function of the
 * web command response of that the user
 * sent via the web interface.
 *
 *\details  Updates the motion mode
 *\details  and the global mode of the robot
 *\details  depending on the type of WebCommand
 *\details  that the user sent via the web interface.
 *
 * @returns
 */
static void UpdateModes(WebCommand cmd) {
	switch (cmd) {
		case FORWARD_CMD:
			motionMode = FORWARD;
			mode = MOVEMENT;
			break;

		case BACKWARD_CMD:
			motionMode = BACKWARD;
			mode = MOVEMENT;
			break;

		case SPINLEFT_CMD:
			motionMode = SPINLEFT;
			mode = MOVEMENT;
			break;

		case SPINRIGHT_CMD:
			motionMode = SPINRIGHT;
			mode = MOVEMENT;
			break;

		case STOP_CMD:
			motionMode = STOP;
			mode = STOP;
			break;

		case SCAN_TEMPERATURE_CMD:
			motionMode = STOP;
			mode = SCAN_TEMPERATURES;
			break;

		case SCAN_DISTANCE_CMD:
			motionMode = STOP;
			mode = SCAN_DISTANCE;
			break;

		case ATTACHMENT_MODE_CMD:
			motionMode = SPINLEFT;
			mode = ATTACHMENT;
			break;

		default:
			// no change in the motion mode
			break;
	}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, portCHAR *pcTaskName) {
	while (1) {
		usart_print_P(PSTR("\r\n\n\n*****STACK OVERFLOW OCCURED****\r\n"));
	}
}
