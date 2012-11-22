/*
 * FreeRTOS+CLI V1.0.0 (C) 2012 Real Time Engineers ltd.
 *
 * FreeRTOS+CLI is an add-on component to FreeRTOS.  It is not, in itself, part 
 * of the FreeRTOS kernel.  FreeRTOS+CLI is licensed separately from FreeRTOS, 
 * and uses a different license to FreeRTOS.  FreeRTOS+CLI uses a dual license 
 * model, information on which is provided below:
 *
 * - Open source licensing -
 * FreeRTOS+CLI is a free download and may be used, modified and distributed
 * without charge provided the user adheres to version two of the GNU General
 * Public license (GPL) and does not remove the copyright notice or this text.
 * The GPL V2 text is available on the gnu.org web site, and on the following
 * URL: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * - Commercial licensing -
 * Businesses and individuals who wish to incorporate FreeRTOS+CLI into
 * proprietary software for redistribution in any form must first obtain a 
 * (very) low cost commercial license - and in-so-doing support the maintenance, 
 * support and further development of the FreeRTOS+CLI product.  Commercial 
 * licenses can be obtained from http://shop.freertos.org and do not require any 
 * source files to be changed.
 *
 * FreeRTOS+CLI is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+CLI unless you agree that you use the software 'as is'.
 * FreeRTOS+CLI is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/FreeRTOS-Plus
 *
 */

/* Standard includes. */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Utils includes. */
#include "FreeRTOS_CLI.h"

//#define DEBUG	1

#ifdef DEBUG
#include "debug_printf.h"
#endif

/*
 * The callback function that is executed when "help" is entered.  This is the
 * only default command that is always present.
 */
static portBASE_TYPE prvHelpCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, int8_t *pcCommandString );

/*
 * Return the number of parameters that follow the command name.
 */
static int8_t prvGetNumberOfParameters( int8_t * pcCommandString );

/* The definition of the "help" command.  This command is always at the front
of the list of registered commands. */
static const xCommandLineInput xHelpCommand = 
{
	(  int8_t * ) "help",
	(  int8_t * ) "help: List the registered command\r\n",
	prvHelpCommand,
	1
};

/* The definition of the Array of commands.  Commands that are registered are
added to this Array. Restricted to 20 commands. */
static const xCommandLineInput *xRegCommands[20];
static int xRegCommand_cnt = 0;

/* A buffer into which command outputs can be written is declared here, rather
than in the command console implementation, to allow multiple command consoles
to share the same buffer.  For example, an application may allow access to the
command interpreter by UART and by Ethernet.  Sharing a buffer is done purely
to save RAM.  Note, however, that the command console itself is not re-entrant,
so only one command interpreter interface can be used at any one time.  For that
reason, no attempt at providing mutual exclusion to the cOutputBuffer array is
attempted. */
static int8_t cOutputBuffer[ configCOMMAND_INT_MAX_OUTPUT_SIZE ];

/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_CLIRegisterCommand( xCommandLineInput * pxCommandToRegister )
{
	portBASE_TYPE xReturn = pdFAIL;

#ifdef DEBUG
	debug_printf("DEBUG CLI: Reg %s\n\r", pxCommandToRegister->pcCommand);
#endif

	taskENTER_CRITICAL();
	{
		
		if (xRegCommand_cnt == 0) {
			xRegCommands[0] = &xHelpCommand;
			xRegCommand_cnt = 1;
		} 

		xRegCommands[xRegCommand_cnt] = pxCommandToRegister;		
		xRegCommand_cnt++;

#ifdef DEBUG
		debug_printf("DEBUG CLI: Reg count %d\n\r", xRegCommand_cnt);
#endif
	}
	taskEXIT_CRITICAL();
		
	xReturn = pdPASS;
	
	return xReturn;
}
/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_CLIProcessCommand( int8_t *pcCommandInput, int8_t * pcWriteBuffer, size_t xWriteBufferLen  )
{
	int i;
	xCommandLineInput *pxCommand = NULL;
	portBASE_TYPE xReturn = pdTRUE;
	int8_t pcRegisteredCommandString[20];

	/* Note:  This function is not re-entrant.  It must not be called from more
	thank one task. */
#ifdef DEBUG
	debug_printf("DEBUG CLI: input %s\n\r", pcCommandInput);
	debug_printf("DEBUG CLI: cmd count %d\n\r", xRegCommand_cnt);
#endif

	for (i=0; i < xRegCommand_cnt; i++) {

#ifdef DEBUG
		debug_printf("DEBUG CLI: cmd %s\n\r", xRegCommands[i]->pcCommand);
#endif		
		memset(pcRegisteredCommandString, 0, 20);
		memcpy(pcRegisteredCommandString, xRegCommands[i]->pcCommand, sizeof(xRegCommands[i]->pcCommand));

		if( strncmp( ( const char * ) pcCommandInput, ( const char * ) pcRegisteredCommandString, strlen( ( const char * ) pcRegisteredCommandString ) ) == 0 ) {

			// The command has been found.  Check it has the expected
			// number of parameters.  If cExpectedNumberOfParameters is -1,
			// then there could be a variable number of parameters and no
			// check is made.

#ifdef DEBUG
			debug_printf("DEBUG CLI: found %s - %d\n\r", pcRegisteredCommandString, xRegCommands[i]->cExpectedNumberOfParameters);
#endif

			if( xRegCommands[i]->cExpectedNumberOfParameters >= 0 ) {

				if( prvGetNumberOfParameters( pcCommandInput ) != xRegCommands[i]->cExpectedNumberOfParameters ) {

					xReturn = pdFALSE;
				} else {
					pxCommand = xRegCommands[i];
				}
			}

			break;
		}//*/
	}

	if( ( pxCommand != NULL ) && ( xReturn == pdFALSE ) ) {
		// The command was found, but the number of parameters with the command
		// was incorrect.
		sprintf( ( char * ) pcWriteBuffer, "\n\rIncorrect parameter(s)\r\n");
		pxCommand = NULL;

	} else if( pxCommand != NULL ) {
		// Call the callback function that is registered to this command.
		xReturn = pxCommand->pxCommandInterpreter( pcWriteBuffer, xWriteBufferLen, pcCommandInput );

		// If xReturn is pdFALSE, then no further strings will be returned
		// after this one, and	pxCommand can be reset to NULL ready to search 
		// for the next entered command.
		if( xReturn == pdFALSE ) {
			pxCommand = NULL;
		}
	} else {
		// pxCommand was NULL, the command was not found.
		sprintf( ( char * ) pcWriteBuffer, ( const char * const ) "\n\rNo such Command.\r\n");
		xReturn = pdFALSE;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

int8_t *FreeRTOS_CLIGetOutputBuffer( void )
{
	return cOutputBuffer;
}
/*-----------------------------------------------------------*/

const int8_t *FreeRTOS_CLIGetParameter(const int8_t *pcCommandString, unsigned portBASE_TYPE uxWantedParameter, portBASE_TYPE *pxParameterStringLength )
{
unsigned portBASE_TYPE uxParametersFound = 0;
const int8_t *pcReturn = NULL;

	*pxParameterStringLength = 0;

	while( uxParametersFound < uxWantedParameter )
	{
		/* Index the character pointer past the current word.  If this is the start
		of the command string then the first word is the command itself. */
		while( ( ( *pcCommandString ) != 0x00 ) && ( ( *pcCommandString ) != ' ' ) )
		{
			pcCommandString++;
		}

		/* Find the start of the next string. */
		while( ( ( *pcCommandString ) != 0x00 ) && ( ( *pcCommandString ) == ' ' ) )
		{
			pcCommandString++;
		}

		/* Was a string found? */
		if( *pcCommandString != 0x00 )
		{
			/* Is this the start of the required parameter? */
			uxParametersFound++;

			if( uxParametersFound == uxWantedParameter )
			{
				/* How long is the parameter? */
				pcReturn = pcCommandString;
				while( ( ( *pcCommandString ) != 0x00 ) && ( ( *pcCommandString ) != ' ' ) )
				{
					( *pxParameterStringLength )++;
					pcCommandString++;
				}

				break;
			}
		}
		else
		{
			break;
		}
	}

	return pcReturn;
}
/*-----------------------------------------------------------*/
/* Help command function */

static portBASE_TYPE prvHelpCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, int8_t *pcCommandString )
{
	int i;	
	char *cCmd_name;
	long lParamLen;
	char pcRegisteredCommandString[10];
	signed portBASE_TYPE xReturn;

	( void ) pcCommandString;

	cCmd_name = FreeRTOS_CLIGetParameter( pcCommandString, 1, &lParamLen);

#ifdef DEBUG
	debug_printf("DEBUG CLI: HELP %s\n\r", cCmd_name);
#endif

	xReturn = pdFALSE;
	for (i=0; i < xRegCommand_cnt; i++) {

		memset(pcRegisteredCommandString, 0, 10);
		memcpy(pcRegisteredCommandString, xRegCommands[i]->pcCommand, sizeof(xRegCommands[i]->pcCommand));

		//Find matching command and return help string.
		if( strncmp( ( const char * ) cCmd_name, ( const char * ) pcRegisteredCommandString, strlen( ( const char * ) pcRegisteredCommandString ) ) == 0 )
		{
			//Copy help string into write buffer.
			strncpy( ( char * ) pcWriteBuffer, ( const char * ) xRegCommands[i]->pcHelpString, xWriteBufferLen );
			xReturn = pdTRUE;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

static int8_t prvGetNumberOfParameters( int8_t * pcCommandString )
{
int8_t cParameters = 0;
portBASE_TYPE xLastCharacterWasSpace = pdFALSE;

	/* Count the number of space delimited words in pcCommandString. */
	while( *pcCommandString != 0x00 )
	{
		if( ( *pcCommandString ) == ' ' )
		{
			if( xLastCharacterWasSpace != pdTRUE )
			{
				cParameters++;
				xLastCharacterWasSpace = pdTRUE;
			}
		}
		else
		{
			xLastCharacterWasSpace = pdFALSE;
		}

		pcCommandString++;
	}

	/* The value returned is one less than the number of space delimited words,
	as the first word should be the command itself. */
	return cParameters;
}

