/*----------------------------------------------------------------------------
 * File name: lcd.h
 * Author: Justin Langis
 * Description: Module for LCD interaction
 *----------------------------------------------------------------------------*/

#ifndef _LCD_H_
#define _LCD_H_

/*----------------------------------------------------------------------------
 * Description:
 * 		Define system variables needed to establish serial connection to
 *      LCD component
 *
 * Parameters:
 * 		None
 *----------------------------------------------------------------------------*/
void initLCD(void);

/*----------------------------------------------------------------------------
 * Description:
 * 		Control methods used as the foundation for subsequent methods
 *
 * Parameters:
 * 		uint8_t (hexadecimal command)
 *      char* (character array)
 *----------------------------------------------------------------------------*/
void send_ext_cmd(uint8_t);
void send_chars(char*);

/*----------------------------------------------------------------------------
 * Description:
 * 		Writes string to entire LCD, first Row or Second Row respectively
 *
 * Parameters:
 * 		msg: Alphanumeric string to be displayed on LCD
 *----------------------------------------------------------------------------*/
void writeLCD(char*);
void writeLCDRowOne(char*);
void writeLCDRowTwo(char*);

/*----------------------------------------------------------------------------
 * Description:
 * 		Clears all alphanumeric characters currently diplayed on the LCD
 *
 * Parameters:
 * 		None
 *----------------------------------------------------------------------------*/
void clearLCD(void);

#endif
