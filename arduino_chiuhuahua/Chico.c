/*!	\file TemperatureMonitor.c
 * \date February 2nd, 2016
 * \brief Temperature Monitor Module
 * \details Defines and manage task execution to read temperatures
 * \from the TemperatureReader module and to display them with
 * \the LED and LCD modules.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <avr/io.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* serial interface include file. */
#include "usartserial.h"

#include "lcd/Lcd.h"
/* Temperature Reader Module include file. */
#include "temperatureReader/TemperatureReader.h"
#include "motion/MotionControl.h"
#include "led/Led.h"
#include "motion/Motion.h"

/*-----------------------------------------------------------*/

/**
 * A cyclic scheduler implementation.
 */
static void CyclicScheduler(void*);

/**
 * Gets temperature readings from the TPA81 thermal array sensor.
 */
static void ReadTemperatures(void);

/**
 * Reads the current speed of both wheels.
 */
static void ReadSpeed(void);

/**
 * Moves the robot in the currently selected direction.
 */
static void Move(void);

/**
 * Sets the Hydrogen Wifi shield LED based on the temperature readings
 */
static void UpdateLED(void);

/**
 * Updates the values shown on the LCD screen according to the most current
 * readings from the different sensors of the robot.
 */
static void UpdateInstrumentCluster(void);

/**
 * Gets the currently selected motion mode (or direction) of
 * the robot.
 */
static MotionMode GetMotionMode(void);

/*-----------------------------------------------------------*/

int usartfd;

/* Total distance traveled by the robot. */
double distanceTraveled;

/* Holds the most current temperature reading from the thermal array sensor. */
uint8_t *temperatures;

/* Holds the most current speed reading from the encoder of the left wheel. */
double *speedLeft;

/* Holds the most current speed reading from the encoder of the right wheel. */
double *speedRight;

/* Holds the most current total distance traveled by the robot. */
double *distance;

/* Holds the most current position of the thermal array servo motor. */
uint16_t *centerServoPosition;


typedef void (*TASK)(void);

#define NUM_MINOR_CYCLES 16
#define MINOR_CYCLE_TIME 50

/* Cyclic scheduler task table. */
TASK table[NUM_MINOR_CYCLES] = {
		ReadTemperatures,
		Move,
		UpdateLED,
		UpdateInstrumentCluster,
		ReadSpeed
};

/** Main loop.  Creates tasks, set their order and start the FreeRTOS scheduler.
 * @return the status of the program when it ends
 */
int main(void)
{
    // turn on the serial port for debugging or for other USART reasons.
	usartfd = usartOpen( USART0_ID, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	usart_print_P(PSTR("\r\n\n\nChico: Initializing...\r\n")); // Ok, so we're alive...

	/* Initialize intertask communication variables */
	temperatures = malloc(sizeof(uint8_t)*9);
	speedLeft = malloc(sizeof(double));
	speedRight = malloc(sizeof(double));

	distance = malloc(sizeof(double));
	*distance = 0;

	centerServoPosition = malloc(sizeof(uint16_t));

	/* Initialize all modules before enabling interrupts. */
	motion_init();
	initMotionControl(centerServoPosition);
	initTemperatureReader();
	initLCD();

	xTaskCreate(CyclicScheduler,  (const portCHAR *)"Cyclic Scheduler" , 256, NULL, 3,  NULL );
	vTaskStartScheduler();

	usart_print_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...
}

/** Reads the temperatures from the robot's thermal array
 * sensor each 100ms, then displays the average temperature in
 * front of the robot on the LED and all temperatures readings on
 * the LCD.
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

static void ReadSpeed() {
	usart_print_P(PSTR("Reading speed \r\n"));
	if ( GetMotionMode() != STOP){
		readSpeed(speedLeft, speedRight, distance);
		updateRobotMotion(*speedLeft, *speedRight);
	} else {
		*speedLeft = 0;
		*speedRight = 0;
	}
	usart_print_P(PSTR("Done reading speed \r\n"));
}

static void ReadTemperatures(void) {
	usart_print_P(PSTR("Reading temperatures \r\n"));
	temperatureSweep(GetMotionMode(), centerServoPosition);
	getTemperatureFromSensor(temperatures);
	usart_print_P(PSTR("Done reading temperatures \r\n"));
}

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

static void UpdateInstrumentCluster(void){
		clearLCD();
		usart_print_P(PSTR("Updating instrument cluster \r\n"));
		/* Average speed of the two wheels. */
		double speed = (*speedLeft + *speedRight)/ 2.0;

		/* Average of the first four temperature pixels from the left. */
		double tempAvgLeft = (temperatures[0] + temperatures[1] + temperatures[2] + temperatures[3])/4.0;

		/* Average of the first four temperature pixels from the right. */
		double tempAvgRight = (temperatures[4] + temperatures[5] + temperatures[6] + temperatures[7])/4.0;

		char topRow[16] = "";
		char bottomRow[16] = "";

		/*
		 * Prints the average speed of the two wheels as well as the
		 * total distance traveled on the first line of the LCD screen.
		 */
		sprintf(topRow, "%.2f m/s %.2f m", speed, *distance);
		writeLCDRowOne(topRow);

		/*
		 * Prints the average temperature for the first four pixel sensors from the left
		 * as well as the average temperature for the first four pixel sensors from the right.
		 */
		sprintf(bottomRow, "temp %.2f %.2f", tempAvgLeft, tempAvgRight);
		writeLCDRowTwo(bottomRow);
		usart_print_P(PSTR("Done updating the instrument cluster \r\n"));
}

static void Move(void){
	usart_print_P(PSTR("Updating the speed \r\n"));
	setMotionMode(GetMotionMode());
	usart_print_P(PSTR("Done updating the speed \r\n"));
}

/**
 * Gets the current motion mode (or direction)
 * of the robot.
 *
 * It currently follows a static demo
 * choregraphy where it goes 1m forward,
 * 1m backward, does a full 360 degrees rotation
 * counter clockwise, does a full 360 degrees
 * rotation clockwise, and finally stops.
 */
static MotionMode GetMotionMode(void){
	if ( *distance < 0.5) {
		return FORWARD;
	} else if ( *distance < 1.0) {
		return BACKWARD;
	} else if ( *distance < 1.5) {	// TODO: need to figure out the distance for a full 360
		return SPINLEFT;
	} else if ( *distance < 2.0) {	// TODO: need to figure out the distance for a full 360
		return SPINRIGHT;
	} else {						// demo choregraphy is done
		return STOP;
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    portCHAR *pcTaskName )
{
	while(1);
}
