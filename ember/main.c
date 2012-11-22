/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "semphr.h"
#include "queue.h"
#include "ff.h"

#include "uip.h"
#include "USB-CDC.h"
#include "uIP_Task.h"
#include "BlockQ.h"
#include "blocktim.h"
#include "flash.h"
#include "QPeek.h"
#include "dynamic.h"
#include "debug_printf.h"
#include "FreeRTOS_CLI.h"

#include <aic/aic.h>
#include <pio/pio.h>
#include <pmc/pmc.h>
#include <pwmc/pwmc.h>
#include <utility/led.h>
#include <utility/trace.h>

/* Task Priorities */
#define mainBLOCK_Q_PRI           ( tskIDLE_PRIORITY + 1 )
#define mainFLASH_PRI             ( tskIDLE_PRIORITY + 1 )
#define mainGEN_QUEUE_PRI         ( tskIDLE_PRIORITY ) 
#define mainUSB_PRI               ( tskIDLE_PRIORITY + 1 )
#define mainUIP_PRI               ( tskIDLE_PRIORITY + 1 )
#define SERVO_PRI                 ( tskIDLE_PRIORITY + 1 )
#define PBTASK_PRI                ( tskIDLE_PRIORITY + 1 )
#define CLI_PRI                   ( tskIDLE_PRIORITY + 1 )

/* Stack Sizes */
#define mainUSB_STACK             ( 256 )
#define mainUIP_STACK             ( 256 )
#define SERVO_STACK               ( 400 )
#define PBTASK_STACK              ( 256 )
#define CLI_STACK                 ( 256 )

/* Servo Constants */
#define CHANNEL_PWM_SERVO1                  2
#define CHANNEL_PWM_SERVO2                  3
#define SERVO_PWM_FREQUENCY                 50
#define SERVO_DUTY_CYCLE                    2000
#define MAX_DEGREES                         254  
#define MIN_DEGREES                         52  
#define MAX_DUTY_CYCLE                      100
#define MIN_DUTY_CYCLE                      0


#define xLogLineSize (46)
#define xMainBuffSize (256L)
#define pPinLaser (&PIN_SET[3])
#define RESET_CTRL { 1 << 30U, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1,0}

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Imposes the maximum and minimum duty cycle rated for the servo's */ 
#define vImposeServoLimits(PAN,TILT)                \
    do {                                            \
    if(PAN > MAX_DEGREES) PAN = MAX_DEGREES;        \
    if(PAN < MIN_DEGREES) PAN = MIN_DEGREES;        \
    if(TILT > MAX_DEGREES) TILT = MAX_DEGREES;      \
    if(TILT < MIN_DEGREES) TILT = MIN_DEGREES;      \
    }while(0)

#define vLogPrintf(...) do {                                                   \
        portTickType xSecs, xMins, xHours;                                     \
        xSecs = xTaskGetTickCount()/1000;                                      \
        xMins = xSecs/60;                                                      \
        xHours = xMins/60;                                                     \
        f_printf(&Fil, "%2d:%2d:%2d - ", xHours, xMins % 60, xSecs % 60);      \
        f_printf(&Fil, __VA_ARGS__);                                           \
    } while(0)


void vPBISR_Handler( void ) __attribute__((naked));
void vPBISR_Wrapper( void ) __attribute__((naked));

void prvSetupHardware( void );
void vApplicationIdleHook( void );
void vConfigurePWM(void);

void vLogOpen(void);

/* Task Prototypes */
void vServoTask(void* pvParameters);
void vPBTask(void* pvParameters);
void vCLI_ReceiveTask(void *pvParameters);

static portBASE_TYPE prvLaserCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );
static portBASE_TYPE prvTailCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );
static portBASE_TYPE prvListCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );
static portBASE_TYPE prvUsageCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );

static const xCommandLineInput xLaserCMD =
{
	( char * ) "laser",
	( char * ) "laser on/off: Turn the laser on or off.\r\n",
	prvLaserCMD,
	1
};

static const xCommandLineInput xListCMD =
{
	( char * ) "ls",
	( char * ) "ls: List the directory contents of the uSD card.\r\n",
	prvListCMD,
	0
};

static const xCommandLineInput xTailCMD =
{
	( char * ) "tail",
	( char * ) "tail: Display the last 10 lines of a file on the uSD card \r\n",
	prvTailCMD,
	0
};

static const xCommandLineInput xTaskUsageCMD =
{
	( char * ) "task_usage",
	( char * ) "task_usage: List the current number of tasks running\r\n",
	prvUsageCMD,
	0
};

static const Pin PIN_SET[] = {
    PIN_PUSHBUTTON_1,
    PIN_PWMC_PWM2,
    PIN_PWMC_PWM3,
    PIN_LASER,
    RESET_CTRL,
};

static FATFS Fatfs;    /* File system object */
static FIL Fil;        /* File object */


static portCHAR pcMainBuff[xMainBuffSize]; 

xSemaphoreHandle xPBSemaphore;
xQueueHandle xQueueTuioData;

int main( void )
{

    /* Setup Data Structures */
    xQueueTuioData =  xQueueCreate(1, sizeof(unsigned int) * 10);
    vSemaphoreCreateBinary( xPBSemaphore );
    (void) xSemaphoreTake( xPBSemaphore, 10 );
    vLogOpen();
    
    /* Hardware Setup */
    prvSetupHardware();
    vConfigurePWM();
    PIO_Configure(PIN_SET, PIO_LISTSIZE(PIN_SET));

    /* System Tasks */
    xTaskCreate( vUSBCDCTask, ( signed char * ) "USB", mainUSB_STACK, NULL, mainUSB_PRI, NULL );
    /* Start the standard tasks. */
    vStartBlockingQueueTasks( mainBLOCK_Q_PRI );
    vCreateBlockTimeTasks();
    vStartGenericQueueTasks( mainGEN_QUEUE_PRI );
    vStartQueuePeekTasks();   
    vStartDynamicPriorityTasks();

    /* Register CLI Commands */
    FreeRTOS_CLIRegisterCommand(&xLaserCMD);
    FreeRTOS_CLIRegisterCommand(&xTailCMD);
    FreeRTOS_CLIRegisterCommand(&xListCMD);
    FreeRTOS_CLIRegisterCommand(&xTaskUsageCMD);

    /* User Tasks */
    xTaskCreate(vuIP_Task, (char * ) "uIP_Task", mainUIP_STACK, NULL, mainUIP_PRI, NULL);

    xTaskCreate(vServoTask, (char * ) "Servo_Task", SERVO_STACK, NULL, SERVO_PRI, NULL);

    xTaskCreate(vPBTask, (char * ) "PB_Task", PBTASK_STACK, NULL, PBTASK_PRI, NULL);

    xTaskCreate(vCLI_ReceiveTask, "CLI_Task", CLI_STACK, NULL, CLI_PRI, NULL );

    vTaskStartScheduler();

    return 0;
}

void vServoTask(void* pvParameters){
    
    unsigned portBASE_TYPE puxTuioData[10]; 

    portSHORT sPan, sTilt, sPosX, sPosY;
    portFLOAT fPosX, fPosY;

    sPosX = sPosY = 0;    
    fPosX = fPosY = 0.0f;    
    
    sPan = 125;
    sTilt = 220;

    // Establish TCP Connection
    u16_t ripaddr[2];
    uip_ipaddr(&ripaddr,192,168,0,4);
    uip_connect(&ripaddr, htons(3000));

    for( ; ; ) {
 
        PWMC_SetDutyCycle(CHANNEL_PWM_SERVO1, sPan);
        PWMC_SetDutyCycle(CHANNEL_PWM_SERVO2, sTilt);

        vTaskDelay(100);

        if( xQueueReceive( xQueueTuioData, (void*) puxTuioData, 10) ){

            LED_Toggle(0);

            fPosX = *(float *)&puxTuioData[2]; 	//range 0...1
            fPosY = *(float *)&puxTuioData[3]; 	//range 0...1

            sPosX = (portSHORT) ( fPosX * 12.0f ); //range 0...12
            sPosY = (portSHORT) ( fPosY * 10.0f ); //range 0...10

            if( sPosX < 0 || sPosX > 12 ) continue;
            if( sPosY < 0 || sPosY > 10 ) continue;

            sPan  -= ( sPosX - 5 );
            sTilt += ( sPosY - 5 );

            vImposeServoLimits( sPan, sTilt );

            vLogPrintf("[vServoTask] %2dx, %2dy, %3dp, %3dt\r\n", sPosX, sPosY, sPan, sTilt);
        
            debug_printf("[vServoTask] %2dx, %2dy, %3dp, %3dt\r\n", sPosX, sPosY, sPan, sTilt);

            f_sync(&Fil);
        }

    }

}

/* Pushbutton Input Task */
void vPBTask(void *pvParameters) {

    portBASE_TYPE xCount;

    portENTER_CRITICAL();
    //Call vPassPBSemaphore (pbISR.c)
    vPassPBSemaphore(xPBSemaphore);
    //Setup the PIO interrupt and set ISR to point to PB ISR Wrapper.
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
    AT91C_BASE_PIOA->PIO_ISR; 
    AT91C_BASE_PIOA->PIO_IDR = 0xFFFFFFFF;
    AIC_ConfigureIT(AT91C_ID_PIOA, AT91C_AIC_PRIOR_LOWEST, vPBISR_Wrapper);
    AT91C_BASE_PIOA->PIO_IER = PIN_SET[0].mask;
    AIC_EnableIT(AT91C_ID_PIOA);
    portEXIT_CRITICAL();

    for ( ; ; ) {

        vTaskDelay(100);

        if(xSemaphoreTake(xPBSemaphore, 10)){

            debug_printf("[vPBTask] xPBSemaphore Taken\r\n");

            for(xCount = 0; xCount < 20; xCount++){

                xCount % 2 ? PIO_Clear(pPinLaser) : PIO_Set(pPinLaser);

                vTaskDelay(50);
            }
        }

        vTaskDelay(100);
    }
}
static portBASE_TYPE prvUsageCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    portCHAR* pcBuf;

    vTaskList( pcMainBuff );

    pcBuf = pcMainBuff;

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    while( *pcBuf != (portLONG) NULL )
        vUSBSendByte(*pcBuf++);
    
    return pdFALSE;
}

static portBASE_TYPE prvTailCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    FRESULT rc;				/* Result code */
    FILINFO fno;			/* File information object */
    portBASE_TYPE xOffset, xCount, xBytesToRead, xBytesRead;

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    xOffset = f_tell(&Fil) - ( 10 * xLogLineSize);

    if( xOffset < 0 )
        xOffset = 0;

    xBytesToRead = f_tell(&Fil) - xOffset;

    (void) f_lseek(&Fil, xOffset );

    while ( xBytesToRead > 0 ) {

        /* Read a chunk of file */
        (void) f_read(&Fil, pcMainBuff, MIN(xBytesToRead,xMainBuffSize), (UINT*) &xBytesRead);

        for ( xCount = 0; xCount < xBytesRead; xCount++)
            vUSBSendByte(pcMainBuff[xCount]);

        xBytesToRead -= xBytesRead;
    }

    return pdFALSE;
}

static portBASE_TYPE prvListCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    FRESULT rc;				/* Result code */
    FILINFO fno;			/* File information object */
    DIR dir;				/* Directory object */
    portBASE_TYPE xTotalSize = 0;

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    rc = f_opendir(&dir, "");

    debug_printf("     Size  Name\r\n");

    for (;;) {

        rc = f_readdir(&dir, &fno);		/* Read a directory item */

        if (rc || !fno.fname[0]) break;	/* Error or end of dir */

        if (fno.fattrib & AM_DIR){

            debug_printf("    <dir>  %s\r\n", fno.fname);

        }else{

            debug_printf("%8luB  %s\r\n", fno.fsize, fno.fname);

            xTotalSize += fno.fsize;

        }
    }
    
    debug_printf("total ~%dKB\r\n", xTotalSize/1024);

    return pdFALSE;
}

 
static portBASE_TYPE prvLaserCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    unsigned portBASE_TYPE uxParamLen; 
    const char* pcParamString;

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    //Get parameters from command string
    pcParamString = FreeRTOS_CLIGetParameter(pcCMDStr, 1, &uxParamLen);

    if ( strcmp(pcParamString, "on") == 0 ){
        // Turn Laser on
        PIO_Set(pPinLaser);
    }else{
        // Turn Laser off
        PIO_Clear(pPinLaser);
    }

    return pdFALSE;
}

void vCLI_ReceiveTask(void *pvParameters) {

    char cRxedChar;
    char cInputString[20];
    char cInputIndex = 0;
    int8_t *pcOutputString;
    portBASE_TYPE xReturned;

    //Initialise pointer to CLI output buffer.
    pcOutputString = FreeRTOS_CLIGetOutputBuffer();

    for (;;) {

        //Receive character from USB receive
        cRxedChar = ucUSBReadByte();

        if ( (cRxedChar != 0) && (cRxedChar != 5)) {

            //Process only if return is received.
            if (cRxedChar == '\r') {

                //Put null character in command input string.
                cInputString[cInputIndex] = '\0';

                //Process command input string.
                (void) FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, 20 );

                cInputIndex = 0;
        
            } else if( cRxedChar == '\b' ) { 

                // Backspace was pressed.  Erase the last character in the
                //string - if any.
                if( cInputIndex > 0 ) {
                        cInputIndex--;
                        cInputString[ cInputIndex ] = '\0';
                }

            } else {
                // A character was entered.  Add it to the string
                // entered so far.  When a \n is entered the complete
                // string will be passed to the command interpreter.
                if( cInputIndex < 20 ) {
                        cInputString[ cInputIndex ] = cRxedChar;
                        cInputIndex++;
                }

                //reflect byte
                vUSBSendByte(cRxedChar);

            }
        } 

        vTaskDelay(50);
    }
}

void vLogOpen( void ) {

    portCHAR pcLogName[14];
    portBASE_TYPE xVersion = 1;

    f_mount(0, &Fatfs);		/* Register volume work area (never fails) */

    for ( ; ; ) {

        sprintf(pcLogName, "6342_V%d.TXT", xVersion++); 

        if ( f_open(&Fil, pcLogName, FA_READ | FA_WRITE | FA_CREATE_NEW ) )
            continue;
        
        break;
    }

    f_sync(&Fil);
}

DWORD get_fattime (void)
{
    return  ((DWORD)(2012 - 1980) << 25) /* Year = 2012 */
            | ((DWORD)1 << 21)	         /* Month = 1 */
            | ((DWORD)1 << 16)	         /* Day_m = 1*/
            | ((DWORD)0 << 11)	         /* Hour = 0 */
            | ((DWORD)0 << 5)	         /* Min = 0 */
            | ((DWORD)0 >> 1);	         /* Sec = 0 */
}

/*-----------------------------------------------------------*/
/* Hardware Initialisation */
void prvSetupHardware( void )
{
    portDISABLE_INTERRUPTS();

    /* When using the JTAG debugger the hardware is not always initialised to
    the correct default state.  This line just ensures that this does not
    cause all interrupts to be masked at the start. */
    AT91C_BASE_AIC->AIC_EOICR = 0;

    /* Most setup is performed by the low level init function called from the
    startup asm file. */

    /* Enable the peripheral clock. */
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOB;
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_EMAC;

    /* Initialise the LED outputs for use by the demo application tasks. */
    LED_Configure(0);
}

/* Idle Application Task */
void vApplicationIdleHook( void )
{
    static portTickType xLastTx = 0;

    /* The idle hook simply prints the idle tick count */
    if( ( xTaskGetTickCount() - xLastTx ) > ( 1000 / portTICK_RATE_MS ) )
    {
        xLastTx = xTaskGetTickCount();
        LED_Toggle(0);
    }
}

void vConfigurePWM(void)
{
    // Enable PWMC peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;

    // Set clock A for servo's at 50hz * Duty cycle (samples).
    PWMC_ConfigureClocks(2 * SERVO_PWM_FREQUENCY * SERVO_DUTY_CYCLE, 0, BOARD_MCK);

    // Configure PWMC channel for SERVO (center-aligned, inverted polarity)
    PWMC_ConfigureChannel(CHANNEL_PWM_SERVO1, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_PWM_SERVO1, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_PWM_SERVO1, MIN_DEGREES);
    
    PWMC_ConfigureChannel(CHANNEL_PWM_SERVO2, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_PWM_SERVO2, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_PWM_SERVO2, MIN_DEGREES);
    
    // Enable channel #1 and #2
    PWMC_EnableChannel(CHANNEL_PWM_SERVO1);
    PWMC_EnableChannel(CHANNEL_PWM_SERVO2);

}
