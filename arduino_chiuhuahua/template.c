/*
 * template.c
 * template_project
 *
 *  Created on: Jun 10, 2015
 *      Author: shailendrasingh
 */

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


/******************************************************************************************************************/
/* CODING STANDARDS:
 * Program file: Section II. Include(s): header file includes. System include files and then user include files.
 * 				Ensure to add comments for an inclusion which is not very obvious. Suggested order of inclusion is
 * 								System -> Other Modules -> Same Module -> Specific to this file
 * Note: Avoid nested inclusions.
 */

/* --Includes-- */
#include "include/template.h"

#include <stdio.h>

/*AVR includes*/
#include "avr/io.h"
#include "avr/interrupt.h"


/*include your header files here*/


/******************************************************************************************************************/
/* CODING STANDARDS:
 * Program file: Section III. Defines and typedefs: order of appearance -> constant macros, function macros,
 * 				typedefs and then enums.
 * Naming convention: Use upper case and words joined with an underscore (_). Limit  the  use  of  abbreviations.
 * Constants: define and use constants, rather than using numerical values; it make code more readable, and easier
 * 			  to modify.
 */


/*put your macros, typedefs and enumerations here*/

#define MACRO_SF 999 	/*!<Provide description for documentation purpose*/



/******************************************************************************************************************/
/* CODING STANDARDS:
 * Program file: Section IV. Global   or   external   data   declarations -> externs, non­static globals, and then
 * 				static globals.
 *
 * Naming convention: variables names must be meaningful lower case and words joined with an underscore (_). Limit
 * 					  the  use  of  abbreviations.
 * Guidelines for variable declaration:
 *				1) Do not group unrelated variables declarations even if of same data type.
 * 				2) Do not declare multiple variables in one declaration that spans lines. Start a new declaration
 * 				   on each line, in­stead.
 * 				3) Move the declaration of each local variable into the smallest scope that includes all its uses.
 * 				   This makes the program cleaner.
 */


/*put your global or external data declaration here*/

int global_variable_sf; /*!< Provide description for documentation purpose*/



/******************************************************************************************************************/
/* CODING STANDARDS
 * Program file: Section V. Functions: order on abstraction level or usage; and if independent alphabetical
 * 				or­dering is good choice.
 *
 * 1) Declare all the functions (entry points, external functions, local functions, and ISR-interrupt service
 *    routines) before first function definition in the program file or in header file and include it; and define
 *    functions in the same order as of declaration.
 * 2) Suggested order of declaration and definition of functions is
 * 	  Entry points -> External functions -> Local functions -> ISR-Interrupt Service Routines
 * 3) Declare function names, parameters (names and types) and re­turn type in one line; if not possible fold it at
 *    an appropriate place to make it easily readable.
 * 4) No function definition should be longer than a page or screen long. If it is long, try and split it into two
 *    or more functions.
 * 5) Indentation and Spacing: this can improve the readability of the source code greatly. Tabs should be used to
 *    indent code, rather than spaces; because spaces can often be out by one and lead to confusions.
 * 6) Keep the length of source lines to 79 characters or less, for max­imum readability.
 */

/*---------------------------------------  Function Declarations  -------------------------------------------------*/
/*
 * Declare all your functions, except for entry points, for the module here; ensure to follow the same order while
 * defining them later.
 */
int main(void) __attribute__((OS_main));

float local_function_one(int parameter_one, float parameter_two, char *parameter_three);

float local_function_two(int parameter_one, float parameter_two, char *parameter_three,
		float parameter_four, int parameter_five);

ISR (INT4_vect, ISR_BLOCK );

ISR (USART0_RX_vect, ISR_BLOCK );


/*---------------------------------------  ENTRY POINTS  ---------------------------------------------------------*/
/*define your entry points here*/

/*(Doxygen help: use \brief to provide short summary, \details for detailed description and \param for parameters */


/*! \brief Main entry point for the program. Provide short description
 *
 * \details Provide detailed description on the functionality of program and other related points.
 * \note main() function can be only appear in one module, which is the main entry point of program;
 * hence remove it while coding a supporting module, which itself is not suppose to execute.
 *
 *
 * @return int
 */
int main(void){

}

/*!\brief Provide short description on functionality of function.
 *
 *\details Provide detailed description of function, including calling mechanism and return value.
 * 	- Functions
 *			-# Function ...1
 *			-# Function ...2
 *			-# Function ...3
 *
 * @param parameter_one Provide description
 * @param parameter_two Provide description
 * @param parameter_three Provide description
 * @param parameter_four Provide description
 * @param parameter_five Provide description
 * @return
 */
void entry_point_function_one(int parameter_one, float parameter_two, char *parameter_three,
		float parameter_four, int parameter_five){

	/*put your code here*/

}

/*!\brief Provide short description on functionality of function.
 *
 *\details Provide detailed description of function, including calling mechanism and return value.
 *
 * @param parameter_one Provide description
 * @param parameter_two Provide description
 * @param parameter_three Provide description
 * @return
 */
void entry_point_function_two(int parameter_one, float parameter_two, char *parameter_three){

	/*put your code here*/

}


/*---------------------------------------  LOCAL FUNCTIONS  ------------------------------------------------------*/
/*define your local functions here*/

/*(Doxygen help: use \brief to provide short summary, \details for detailed description and \param for parameters */

/*!\brief Provide short description on functionality of function.
 *
 *\details Provide detailed description of function
 *
 * @param parameter_one Provide description
 * @param parameter_two Provide description
 * @param parameter_three Provide description
 * @return
 */
float local_function_one(int parameter_one, float parameter_two, char *parameter_three){

	/*put your code here*/

	return 0;
}


/*!\brief Provide short description on functionality of function.
 *
 *\details Provide detailed description of function
 *
 * @param parameter_one Provide description
 * @param parameter_two Provide description
 * @param parameter_three Provide description
 * @param parameter_four Provide description
 * @param parameter_five Provide description
 * @return
 */
float local_function_two(int parameter_one, float parameter_two, char *parameter_three,
		float parameter_four, int parameter_five){

	/*put your code here*/

	return 0;

}



/*---------------------------------------  ISR-Interrupt Service Routines  ---------------------------------------*/
/*define your Interrupt Service Routines here*/

/*(Doxygen help: use \brief to provide short summary, \details for detailed description and \param for parameters */

/*!\brief Provide short description on interrupt service routine
 *
 *\details Provide detailed description of interrupt service routine
 */
ISR (INT4_vect, ISR_BLOCK ){

	/*put your code here, which to be executed when interrupt is received or raised*/

}
/*Based on target hardware/micro-controller there can be various interrupts, few of the interrupts from ATmega2560 are
 * INT4_vect -> External Interrupt Request 4
 * USART0_RX_vect -> USART0, Rx Complete
 * TIM1_OVF_vect -> Timer/Counter1 Overflow
 * USART0_TX_vect -> USART0, Tx Complete
 */

/*!\brief Provide short description on interrupt service routine
 *
 *\details Provide detailed description of interrupt service routine
 */
ISR (USART0_RX_vect, ISR_BLOCK ){

	/*put your code here, which to be executed when interrupt is received or raised*/

}
