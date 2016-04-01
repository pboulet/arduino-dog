/*
 * Lcd.h
 * Chico The Robot
 */

/******************************************************************************************************************/

/*!	\file Lcd.h
 *
 * \author Justin Langis
 *
 * \date March 13th, 2016
 *
 * \brief  Module for LCD interaction
 *
 */

/******************************************************************************************************************/

#ifndef _LCD_H_
#define _LCD_H_

/*----------------------------------------------------------------------------
 *! \fn initLCD(void)
 * \brief Define system variables needed to establish serial connection to LCD component
 *----------------------------------------------------------------------------*/
void InitLCD(void);

/*----------------------------------------------------------------------------
 *! \fn writeLCD(char*)
 * \brief Writes string to entire LCD
 *
 * @param[msg]: Alphanumeric string to be displayed on LCD
 *----------------------------------------------------------------------------*/
void writeLCD(char*);

/*----------------------------------------------------------------------------
 *! \fn writeLCDRowOne(char*)
 * \brief Writes a string to the top Row of the LCD
 *
 * @param[msg]: Alphanumeric string to be displayed on LCD's top row
 *----------------------------------------------------------------------------*/
void writeLCDRowOne(char*);

/*----------------------------------------------------------------------------
 *! \fn writeLCDRowTwo(char*)
 * \brief Writes a string to the bottom Row of the LCD
 *
 * @param[msg]: Alphanumeric string to be displayed on LCD's bottom row
 *----------------------------------------------------------------------------*/
void writeLCDRowTwo(char*);

/*----------------------------------------------------------------------------
 * ! \fn clearLCD(void)
 * \brief Clears all alphanumeric characters currently diplayed on the LCD
 *----------------------------------------------------------------------------*/
void clearLCD(void);

#endif
