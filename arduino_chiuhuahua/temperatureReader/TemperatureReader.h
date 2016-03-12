/*
 * TemperatureReader.h
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file TemperatureReader.h
 * \author Ladan Maxamud, Patrice Boulet
 * \date February 2nd, 2016
 * \brief Temperature Reader Module
 * \details Reads temperatures from a TPA81 thermal array sensor.
 */

#ifndef TEMPERATUREREADER_H_
#define TEMPERATUREREADER_H_

/******************************************************************************************************************/

/*--------- Include --------- */

/* These dependencies have to be exposed here otherwise
 * our declarations doesn't compile.
 */
#include <stdlib.h>
#include "i2cMultiMaster.h"

/******************************************************************************************************************/

/* ------- Constants ------	*/
static const uint8_t MASTER_ADDR = 0xC0; 	// Master i2c address
#define I2C_WRITE_ADDR  0xD0; 				// TPA81 i2c address - master write
#define I2C_READ_ADDR  	0xD1; 				// TPA81 i2c address - master read
#define BASE_REGISTER 	0x01 				// Ambient temperature register

/***************************************** ENTRY POINTS  **********************************************************/

/*!\brief Module initializer.
 *
 *\details  Initialize the module, putting TWI master to its initial standby state.
 *
 * @returns
 */
void initTemperatureReader(void);

/*!\brief Reads and store temperature readings from the thermal sensor.
 *
 *\details  Reads the temperatures captured by the thermal sensor
 *	following an I2C TWI protocol.  Temperatures are then stored
 *	in the temperatures array parameter given.
 *
 * @param temperatures An array of 9 temperatures. The first one being the ambient
 * 			temperature followed by 8 pixel temperatures
 * 			read from the TPA81 thermal array sensor.
 * @returns
 */
void getTemperatureFromSensor(uint8_t*);

/******************************************************************************************************************/

#endif /*TEMPERATUREREADER_H_ */
