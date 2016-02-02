/*
 * ThermalSensor.c
 *
 */

/*(Doxygen help: use \brief to provide short summary and \details command can be used)*/

/*!	\file template.c
 * \author Ladan Maxamud and Patrice Boulet
 * \date January 27th 2016
 * \brief Thermal Sensor reader
 * \details This module communicates with the thermal sensor TPA81 using the I2C bus protocol.
 *
 *
 *
 */

/* --Includes-- */
#include "include/TemperatureReader.h"

/*---------------------------------------  Function Declarations  -------------------------------------------------*/
void TemperatureSensor(void);
uint8_t *getTemperatureFromSensor(void);

/*! \brief Main entry point for temperature module
 *
 * \details This function
 * \note main() function can be only appear in one module, which is the main entry point of program;
 * hence remove it while coding a supporting module, which itself is not suppose to execute.
 *
 *
 * @return void
 */
void TemperatureSensor(void){
	I2C_Master_Initialise(0xC0);
}

/*---------------------------------------  LOCAL FUNCTIONS  ------------------------------------------------------*/
uint8_t *getTemperatureFromSensor(void){

	/* 1. Send a start sequence
	 * 2. Send 0xD0 ( I2C address of the thermal sensor with the R/W bit low (even address)
	 * 3. Send 0x01 (Internal address of the bearing register - 0x01 == Ambient)
	 * 4. Send a start sequence again (repeated start)
	 * 5. Send 0xD1 ( I2C address of the thermal sensor with the R/W bit high (odd address)
	 * 6. Read data byte from thermal sensor
	 * 7. Send the stop sequence.
	 * */

	uint8_t writeCommand[2] = {0xD0, 0x01};
	uint8_t *readCommand = malloc(sizeof(uint8_t)*10);
	readCommand[0] = 0xD1;

	I2C_Master_Start_Transceiver_With_Data(writeCommand,2);
	I2C_Master_Start_Transceiver_With_Data(readCommand,10);
	I2C_Master_Get_Data_From_Transceiver(readCommand,10);

	return readCommand;
}
