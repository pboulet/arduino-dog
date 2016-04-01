/*
 * TemperatureReader.c
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file TemperatureReader.c
 * \author Ladan Maxamud, Patrice Boulet
 * \date January 27th 2016
 * \brief Temperature Reader Module
 * \details This module communicates with the thermal sensor TPA81 using the I2C bus protocol.
 */

/******************************************************************************************************************/

/******************************************************************************************************************/

/* --Includes-- */
#include "TemperatureReader.h"

/******************************************************************************************************************/

/****************************************  ENTRY POINTS  **********************************************************/

/*!\brief Module initializer.
 *
 *\details  Initialize the module, putting TWI master to its initial standby state.
 *
 * @returns none
 */
void InitTemperatureReader(void){
	I2C_Master_Initialise(MASTER_ADDR);
}

/*!\brief Reads and store temperature readings from the thermal sensor.
 *
 *\details  Reads the temperatures captured by the thermal sensor
 *	following an I2C TWI protocol.  Temperatures are then stored
 *	in the temperatures array parameter given.
 *
 * @param temperatures An array of 9 temperatures. The first one being the ambient
 * 			temperature followed by 8 pixel temperatures
 * 			read from the TPA81 thermal array sensor.
 * @returns none
 */
void getTemperatureFromSensor(uint8_t *temperatures){

	/* 1. Send a start sequence
	 * 2. Send 0xD0 ( I2C address of the thermal sensor with the R/W bit low (even address)
	 * 3. Send 0x01 (Internal address of the bearing register - 0x01 == Ambient)
	 * 4. Send a start sequence again (repeated start)
	 * 5. Send 0xD1 ( I2C address of the thermal sensor with the R/W bit high (odd address)
	 * 6. Read data byte from thermal sensor
	 * 7. Send the stop sequence.
	 */

	uint8_t writeCommand[2];
	writeCommand[0] = I2C_WRITE_ADDR;
	writeCommand[1] = BASE_REGISTER;

	uint8_t *tmp = malloc(10 * sizeof(uint8_t));
	tmp[0] = I2C_READ_ADDR;

	I2C_Master_Start_Transceiver_With_Data(writeCommand,2);
	I2C_Master_Start_Transceiver_With_Data(tmp,10);
	I2C_Master_Get_Data_From_Transceiver(tmp,10);

	/* Don't include the read address into our temperature results. */
	for ( int i = 1; i <= 10; i++){
		temperatures[i -1] = tmp[i];
	}
}

/******************************************************************************************************************/
