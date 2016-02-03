/*!	\file TemperatureReader.c
 * \author Ladan Maxamud, Patrice Boulet
 * \date January 27th 2016
 * \brief Temperature Reader Module
 * \details This module communicates with the thermal sensor TPA81 using the I2C bus protocol.
 */

/* --Includes-- */
#include "include/TemperatureReader.h"

/*---------------------------------------  Function Declarations  -------------------------------------------------*/
void TemperatureSensor(void);
void getTemperatureFromSensor(uint8_t*);


void TemperatureSensor(void){
	I2C_Master_Initialise(0xC0);
}

/*---------------------------------------  LOCAL FUNCTIONS  ------------------------------------------------------*/
void getTemperatureFromSensor(uint8_t *temperatures){

	/* 1. Send a start sequence
	 * 2. Send 0xD0 ( I2C address of the thermal sensor with the R/W bit low (even address)
	 * 3. Send 0x01 (Internal address of the bearing register - 0x01 == Ambient)
	 * 4. Send a start sequence again (repeated start)
	 * 5. Send 0xD1 ( I2C address of the thermal sensor with the R/W bit high (odd address)
	 * 6. Read data byte from thermal sensor
	 * 7. Send the stop sequence.
	 * */

	uint8_t writeCommand[2] = {0xD0, 0x01};
	temperatures[0] = 0xD1;

	I2C_Master_Start_Transceiver_With_Data(writeCommand,2);
	I2C_Master_Start_Transceiver_With_Data(temperatures,10);
	I2C_Master_Get_Data_From_Transceiver(temperatures,10);
}
