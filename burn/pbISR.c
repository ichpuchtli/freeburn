/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practial 6 - FreeRTOS Peripheral Interfacing
# Module name : Main 
# Functionality: Pushbutton Interrupt Service Routine.
#
# Author name: M. D'Souza
# Creation date: 220812
# Revision date (name): -
# Changes implemented (date): -
#(Comments): 
------------------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "pio/pio.h"

#include "debug_printf.h"
#include <utility/led.h>

#include "AT91SAM7X256.h"

/* Wrapper for the PB interrupt. */
void vPBISR_Wrapper( void ) __attribute__((naked));

/* Handler called by the PB ISR wrapper.  This must be kept a separate
function to ensure the stack frame is correctly set up. */
void vPBISR_Handler( void ) __attribute__((noinline));


static xSemaphoreHandle xPBSemaphore;		//Pushbutton semaphore

static portTickType xLastTx = 0;

/*-----------------------------------------------------------*/
//Set semaphore to use.
void vPassPBSemaphore( xSemaphoreHandle xSemaphore )
{
	xPBSemaphore = xSemaphore;
}

//Handler called by the ISR wrapper.  This must be kept a separate
//function to ensure the stack frame is correctly set up. 
//This function is used to process the pushbutton interrupt

void vPBISR_Handler( void ) {
	unsigned long status;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	//Clear and read interrupt flags
	status = AT91C_BASE_PIOA->PIO_ISR;
        status &= AT91C_BASE_PIOA->PIO_IMR;
	
        /* The idle hook simply prints the idle tick count */
        if( ( xTaskGetTickCount() - xLastTx ) > ( 1000 / portTICK_RATE_MS ) )
        {
            xLastTx = xTaskGetTickCount();

            //Add your pusbbutton ISR code to trigger a semaphore.
            xSemaphoreGiveFromISR(xPBSemaphore, &xHigherPriorityTaskWoken);
        }

	//Clear AIC flags
  	AT91C_BASE_AIC->AIC_EOICR = 0;
	/* Do a task switch if needed */
  	if (xHigherPriorityTaskWoken) {
      /* This call will ensure that the unblocked task will be executed
         immediately upon completion of the ISR if it has a priority higher
         than the interrupted task. */
         portYIELD_FROM_ISR ();
    }

}


// Wrapper for the PB interrupt.
void vPBISR_Wrapper( void ) {
	/* Save the context of the interrupted task. */
	portSAVE_CONTEXT();
	
	/* Call the handler task to do the actual work.  This must be a separate
	function to ensure the stack frame is correctly set up. */
	vPBISR_Handler();
	/* Restore the context of whichever task is the next to run. */
	portRESTORE_CONTEXT();
        
}
