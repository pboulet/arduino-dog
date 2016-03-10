/*!
 * \file lcd.c
 * \brief Module to handle LCD connection
 * \author Justin Langis
 * \author Nick Dubus
 *
 *  This module provides an easy interface to the LCD hardware
 *  From a higher level, users can simply call wrappers to display
 *  information on the LCD
 */

#include "Lcd.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "usartserial.h"


//! A public variable.
/*!
  /a usartlcd will hold the usart id after initialization.
*/
int usartlcd;

uint8_t const LCD_EXT_CMD = 0xFE;
uint8_t const LCD_CLEAR = 0x01;
uint8_t const LCD_ROW_1 = 0x80;
uint8_t const LCD_ROW_2 = 0xC0;

/*! \fn initLCD()
 *  \brief Initializes the LCD
 *
 *  Method which passes definition variables to underlying libs to establish a connection.
 *  A successful initialization will display a message on the LCD
*/
void initLCD()
{
	// Initialize serial connection to LCD using usart serial lib
	usartlcd = usartOpen(USART1_ID, 9600, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	// Clear the LCD
    clearLCD();

    // Display Initialization Complete Message
    writeLCDRowOne("Initialization");
    writeLCDRowTwo("Complete!");
}

/*! \fn send_ext_cmd(uint8_t cmd)
 *  \brief Used to send extended commands to the LCD hardware
 *  \param cmd an 8 bit hexadecimal command
 *  The method first passes the dedicated extended control character then
 *  passes the /a cmd parameter.
*/
void send_ext_cmd(uint8_t cmd){
    usart_fprintf_P(usartlcd,PSTR("%c"),LCD_EXT_CMD);
    usart_fprintf_P(usartlcd,PSTR("%c"),cmd);
}

/*! \fn send_chars(char* msg)
 *  \brief Used to send a string to the LCD display
 *  \param msg a character array which holds the message to be displayed
 *
 *  The msg array is sent to the LCD display
*/
void send_chars(char* msg)
{
    usart_fprintf_P(usartlcd,PSTR("%s"),msg);
}

/*! \fn void writeLCD(char* msg)
 *  \brief Writes \a msg to the LCD display
 *  \param msg a character array holding the message to display
 *
 *  Function will send \msg to the LCD display.  If the \a msg > 32 characters
 *  it will be truncated to 32 characters.
*/
void writeLCD(char* msg)
{
    //make sure msg is less than 32 characters (e.g. 2 lines x 16 chars each)
    //otherwize truncated it to 32 characters
    if (sizeof(msg) > 32)
        msg[32] = '\0';

    //clear the display
    clearLCD();

    //write message
    send_chars(msg);
}

/*! \fn void writeLCDRowOne(char* msg)
 *  \brief Writes \a msg to the first row of the LCD display
 *  \param msg a character array holding the message to display
 *
 *  Function will send \msg to the first row of the LCD display.  If the \a msg > 16 characters
 *  it will be truncated to 16 characters.
*/
void writeLCDRowOne(char* msg)
{
    //make sure msg is less than 16 characters (e.g. 1 lines x 16 chars)
    //otherwize truncated it to 16 characters
    if (sizeof(msg) > 16)
            msg[16] = '\0';

    //set cursor position to row1
    send_ext_cmd(LCD_ROW_1);
    //avrSerialxPrintf_P(&LCDSerialPort,PSTR("                "));
    send_chars(msg);
}

/*! \fn void writeLCDRowTwo(char* msg)
 *  \brief Writes \a msg to the second row of the LCD display
 *  \param msg a character array holding the message to display
 *
 *  Function will send \msg to the second row of the LCD display.  If the \a msg > 16 characters
 *  it will be truncated to 16 characters.
*/
void writeLCDRowTwo(char* msg)
{
    //make sure msg is less than 16 characters (e.g. 1 lines x 16 chars)
    //otherwize truncated it to 16 characters
    if (sizeof(msg) > 16)
            msg[16] = '\0';
    //clear row2
    //set cursor position to row1
    send_ext_cmd(LCD_ROW_2);
    //avrSerialxPrintf_P(&LCDSerialPort,PSTR("                "));
    send_chars(msg);
}


/*! \fn void clearLCD()
 *  \brief Clears LCD display
 *
 *  Function which clears all characters from the LCD display
*/
void clearLCD()
{
    send_ext_cmd(LCD_CLEAR);
}
