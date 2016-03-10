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
float distanceTraveled;

/* Holds the most current temperature reading from the thermal array sensor. */
uint8_t *temperatures;

/* Holds the most current speed reading from the encoder of the left wheel. */
float *speedLeft;

/* Holds the most current speed reading from the encoder of the right wheel. */
float *speedRight;

/* Holds the most current total distance traveled by the robot. */
float *distance;

/* Holds the most current position of the thermal array servo motor. */
uint16_t *centerServoPosition;


typedef void (*TASK)(void);

#define NUM_MINOR_CYCLES 132
#define MINOR_CYCLE_TIME 50

/* Cyclic scheduler task table. */
TASK table[NUM_MINOR_CYCLES] = {
		Move,
		ReadTemperatures,
		ReadSpeed,
		UpdateInstrumentCluster,
		UpdateLED
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
	speedLeft = malloc(sizeof(float));
	speedRight = malloc(sizeof(float));
	distance = malloc(sizeof(float));
	centerServoPosition = malloc(sizeof(uint16_t));

	/* Initialize all modules before enabling interrupts. */
	initMotionControl(centerServoPosition);
	initTemperatureReader();
	initLCD();
	motion_init();

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

	int taskNum = 0; // to cycle through schedule
	while(1) {
		if(table[taskNum] != NULL)
			table[taskNum]();        					// Sets minor cycle time
		vTaskDelayUntil( &xLastWakeTime, ( MINOR_CYCLE_TIME / portTICK_PERIOD_MS ));
		taskNum = (taskNum+1) % NUM_MINOR_CYCLES;
	}
}

static void ReadSpeed() {
	readSpeed(speedLeft, speedRight, distance);
}

static void ReadTemperatures(void) {
	temperatureSweep(temperatures, centerServoPosition);
	getTemperatureFromSensor(temperatures);
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

		/* Average speed of the two wheels. */
		float speed = (*speedLeft + *speedRight)/ 2.0F;

		/* Average of the first four temperature pixels from the left. */
		float tempAvgLeft = (temperatures[0] + temperatures[1] + temperatures[2] + temperatures[3])/4.0F;

		/* Average of the first four temperature pixels from the right. */
		float tempAvgRight = (temperatures[4] + temperatures[5] + temperatures[6] + temperatures[7])/4.0F;

		char topRow[16];
		char bottomRow[16];

		/*
		 * Prints the average speed of the two wheels as well as the
		 * total distance traveled on the first line of the LCD screen.
		 */
		sprintf(topRow,
			"%.2f %.2f",
			speed,
			*distance);
		writeLCDRowOne(topRow);

		/*
		 * Prints the average temperature for the first four pixel sensors from the left
		 * as well as the average temperature for the first four pixel sensors from the right.
		 */
		sprintf(bottomRow,
			"%.2f %.2f",
			tempAvgLeft,
			tempAvgRight);
		writeLCDRowTwo(bottomRow);
}

static void Move(void){
	setMotionMode(GetMotionMode());
}

static MotionMode GetMotionMode(void){
	if (distanceTraveled < 1) {
		return FORWARD;
	} else if (distanceTraveled < 2) {
		return BACKWARD;
	} else if ( distanceTraveled < 3) {	// need to figure out the distance for a full 360
		return SPINLEFT;
	} else if ( distanceTraveled < 4) {	// need to figure out the distance for a full 360
		return SPINRIGHT;
	} else {							// demo choregraphy is done
		return STOP;
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    portCHAR *pcTaskName )
{
	while(1);
}
