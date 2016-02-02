/******************************************************************************************************************/
/* CODING STANDARDS:
 * Program file: Section I. Prologue: description about the file, description author(s), revision control
 * 				information, references, etc.
 */

/*(Doxygen help: use \brief to provide short summary and \details command can be used)*/

/*!	\file template.c
 * \author provide author details
 * \date provide date
 * \brief Provide short summary on function and the contents of file
 * \details Provide detailed description
 *
 *
 */

#ifndef TEMPERATUREREADER_H_
#define TEMPERATUREREADER_H_

#include <stdlib.h>
#include "i2cMultiMaster.h"

/*	CONSTANTS	*/
#define MASTER_ADDR  0xC0; 		// Master i2c address
#define I2C_WRITE_ADDR  0xD0; 	// TPA81 i2c address - master write
#define I2C_READ_ADDR  0xD1; 	// TPA81 i2c address - master read

#define BASE_REGISER 0x01 		// Ambient temperature register

/*	Function Prototypes	*/
void TemperatureSensor(void);
uint8_t *getTemperatureFromSensor(void);


#endif /*TEMPERATUREREADER_H_ */
