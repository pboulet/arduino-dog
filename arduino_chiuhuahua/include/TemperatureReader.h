/*!	\file TemperatureReader.h
 * \author Ladan Maxamud, Patrice Boulet
 * \date February 2nd, 2016
 * \brief Temperature Reader Module
 * \details Reads temperatures from a TPA81 thermal array sensor.
 */

#ifndef TEMPERATUREREADER_H_
#define TEMPERATUREREADER_H_

#include <stdlib.h>
#include "i2cMultiMaster.h"

/*	CONSTANTS	*/
#define MASTER_ADDR  	0xC0; 		// Master i2c address
#define I2C_WRITE_ADDR  0xD0; 		// TPA81 i2c address - master write
#define I2C_READ_ADDR  	0xD1; 		// TPA81 i2c address - master read
#define BASE_REGISER 	0x01 		// Ambient temperature register

/** Initialize the module, putting TWI master to its initial standby state.
 */
void TemperatureSensor(void);

/** Initialize the module, putting TWI master to its initial standby state.
 * @return An array of 9 temperature. The first one being the ambient
 * 			temperature and the following the 8 pixel temperatures
 * 			read from the TPA81 thermal array sensor.
 */
void getTemperatureFromSensor(uint8_t*);

#endif /*TEMPERATUREREADER_H_ */
