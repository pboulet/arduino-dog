/*
 * template.h
 * template_project
 *
 *  Created on: Jun 10, 2015
 *      Author: shailendrasingh
 */


/******************************************************************************************************************/
/* CODING STANDARDS
 * Header file: Section I. Prologue: description about the file, description author(s), revision control
 * 				information, references, etc.
 * Note: 1. Header files should be functionally organized.
 *		 2. Declarations   for   separate   subsystems   should   be   in   separate
 */


/*(Doxygen help: use \brief to provide short summary and \details command can be used)*/


/*!	\file template.h
 * \author provide author details
 * \date provide date
 * \brief Provide short summary on function and the contents of file
 * \details Provide detailed description
 *
 *
 */


#ifndef TEMPLATE_H_
#define TEMPLATE_H_
/******************************************************************************************************************/
/* CODING STANDARDS:
 * Header file: Section II. Include(s): header file includes. System include files and then user include files.
 * 				Ensure to add comments for an inclusion which is not very obvious. Suggested order of inclusion is
 * 								System -> Other Modules -> Same Module -> Specific to this file
 * Note: Avoid nested inclusions.
 */


/* --Includes-- */
#include <stdlib.h>


/*include your header files here*/



/******************************************************************************************************************/
/* CODING STANDARDS
 * Header file: Section III. Defines and typedefs: order of appearance -> constant macros, function macros,
 * 				typedefs and then enums.
 *
 * Custom data types and typedef: these definitions are best placed in a header file so that all source code
 * files which rely on that header file have access to the same set of definitions. This also makes it easier
 * to modify.
 * Naming convention: Use upper case and words joined with an underscore (_). Limit  the  use  of  abbreviations.
 * Constants: define and use constants, rather than using numerical values; it make code more readable, and easier
 * 			  to modify.
 * Note: Avoid initialized data definitions.
 */


/*put your macros, typedefs and enumerations here*/

#define MACRO_HF 000 	/*!<Provide description for documentation purpose*/




/******************************************************************************************************************/
/* CODING STANDARDS:
 * Header file: Section IV. Global   or   external   data   declarations -> externs, non­static globals, and then
 * 				static globals.
 *
 * Naming convention: variables names must be meaningful lower case and words joined with an underscore (_). Limit
 * 					  the  use  of  abbreviations.
 */


/*put your macros, typedefs and enumerations here*/

int global_variable_hf; /*!< Provide description for documentation purpose*/




/******************************************************************************************************************/
/* CODING STANDARDS
 * Header file: Section V. Functions: order on abstraction level or usage; and if independent alphabetical
 * 				or­dering is good choice.
 *
 * 1) Declare all the entry point functions.
 * 2) Declare function names, parameters (names and types) and re­turn type in one line; if not possible fold it at
 *    an appropriate place to make it easily readable.
 */


/*---------------------------------------  ENTRY POINTS  ---------------------------------------------------------*/
/*Declare your entry points here*/

void entry_point_function_one(int parameter_one, float parameter_two, char *parameter_three,
		float parameter_four, int parameter_five);

void entry_point_function_two(int parameter_one, float parameter_two, char *parameter_three);


#endif /* TEMPLATE_H_ */
