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
#include "uIP_Task.h"

#include "USB-CDC.h"
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

#include "nrf24l01plus.h"

/* Task Priorities */
#define mainBLOCK_Q_PRI           ( tskIDLE_PRIORITY + 1 )
#define mainFLASH_PRI             ( tskIDLE_PRIORITY + 1 )
#define mainGEN_QUEUE_PRI         ( tskIDLE_PRIORITY ) 
#define mainUSB_PRI               ( tskIDLE_PRIORITY + 2 )
#define mainUIP_PRI               ( tskIDLE_PRIORITY + 1 )
#define SERVER_PRI                ( tskIDLE_PRIORITY + 1 )
#define PBTASK_PRI                ( tskIDLE_PRIORITY + 1 )
#define CLI_PRI                   ( tskIDLE_PRIORITY + 1 )
#define RADIO_PRI                 ( tskIDLE_PRIORITY + 1 )
#define TRACK_PRI                 ( tskIDLE_PRIORITY + 1 )
#define BCTRL_PRI                 ( tskIDLE_PRIORITY + 1 )

/* Stack Sizes */
#define DFL_STACK_SIZE             256

#define mainUSB_STACK             ( DFL_STACK_SIZE )
#define SERVER_STACK              ( DFL_STACK_SIZE )
#define mainUIP_STACK             ( 256 )
#define TRACK_STACK               ( DFL_STACK_SIZE )
#define RADIO_STACK               ( DFL_STACK_SIZE * 2 )
#define PBTASK_STACK              ( DFL_STACK_SIZE )
#define CLI_STACK                 ( DFL_STACK_SIZE )
#define BCTRL_STACK               ( DFL_STACK_SIZE )

/* Servo Constants */
#define CHANNEL_PWM_SERVO1        2
#define CHANNEL_PWM_SERVO2        3
#define SERVO_PWM_FREQUENCY       50
#define SERVO_DUTY_CYCLE          2000
#define MAX_DEGREES               254  
#define MIN_DEGREES               52  
#define MAX_DUTY_CYCLE            100
#define MIN_DUTY_CYCLE            0

#define xLogLineSize (30)
#define xMainBuffSize (2024L)

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


#define vSendYawCTRL(YAW) do {                                   \
    TxPkt->dir = yaw;                                      \
    xQueueSendToBack(xRadioTX_Queue, (uint8_t*) TxPkt, 0); \
}while(0)

#define vSendAltitudeCTRL(ALT) do {                              \
    TxPkt->alt = alt;                                      \
    xQueueSendToBack(xRadioTX_Queue, (uint8_t*) TxPkt, 0); \
}while(0)

#define vStablizeTxPkt(PACKET) do {    \
    PACKET->bid = 8;                   \
    PACKET->motors = 0;                \
    PACKET->fwd = 0x7F;                \
    PACKET->dir = 0x7F;                \
    PACKET->alt = 0x7F;                \
    PACKET->cmd = 0;                   \
    PACKET->data = 0;                  \
}while(0)

#define SPEC_CMD_PASSKEY            0xC2 /* 16-bit unsigned requires student ID */
#define SPEC_CMD_MAGNETO            0xC3 /* Magnetometer x-axis 16-bit unsigned */
#define SPEC_CMD_IRRANGE            0xC4 /* IR Rangefinder ADC 0-255 16-bit unsigned */
#define SPEC_CMD_RIGH180            0xC7 /* None */
#define SPEC_CMD_LEFT180            0xC8 /* None */

void vPBISR_Handler( void ) __attribute__((naked));
void vPBISR_Wrapper( void ) __attribute__((naked));

void prvSetupHardware( void );
void vApplicationIdleHook( void );
void vConfigurePWM(void);
void vConfigureRadio(void);
void vLogOpen(void);
void vTrackingTask(void *pvParameters);
void vServerTask(void* pvParameters);
void vRadioTask(void* pvParameters);
void vCLI_ReceiveTask(void *pvParameters);
void vPBTask(void *pvParameters);
void vBlimpController(void *pvParameters);

portBASE_TYPE prvTopCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );
portBASE_TYPE prvStopUpdatesCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );
portBASE_TYPE prvGetPassKeyCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );
portBASE_TYPE prvGetMagRawXCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );
portBASE_TYPE prvEnableMotorsCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );
portBASE_TYPE prvSelectBlimpCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr );

struct UpLinkPkt {

    volatile uint8_t  bid; /* Blip ID */
    volatile uint8_t  motors; /* Enable Motors 0 = OFF, 1 = On */
    volatile uint8_t  fwd; /* Forward Motor Control 0 = Reverse, 0xFF = Forward */
    volatile uint8_t  dir; /* 0 = Left, 0xFF = Right */
    volatile uint8_t  alt; /* 127 = Neutral, +-1 unit == +-15mm in altitude */ 
    volatile uint8_t  cmd; /* Special Command */
    volatile uint16_t data; /* Special Data */
    uint8_t  padding[24];
};

struct DownLinkPkt {

    volatile uint8_t  bid; /* Blip ID */
    volatile uint8_t  motorsACK; /* Enable Motors 0 = OFF, 1 = On */
    volatile uint8_t  fwdACK; /* Forward Motor Control 0 = Reverse, 0xFF = Forward */
    volatile uint8_t  dirACK; /* 0 = Left, 0xFF = Right */
    volatile uint8_t  altACK; /* 127 = Neutral, +-1 unit == +-15mm in altitude */ 
    volatile uint8_t  cmdACK; /* Special Command */
    volatile uint16_t data; /* Special Data */
    volatile uint16_t IRRange; /* Calibrated IR Distance values (0-3000mm) */
    volatile uint16_t mag; /* Calibrated Magnetometer heading (0-360degrees) */
    uint8_t padding[20];
};

static const xCommandLineInput xSelectBlimpCMD =
{
	( uint8_t * ) "bid",
	( uint8_t * ) "bid: change the blimp id\r\n",
	prvSelectBlimpCMD,
	1
};

static const xCommandLineInput xTopCMD =
{
	( uint8_t * ) "top",
	( uint8_t * ) "top: List the current number of tasks running\r\n",
	prvTopCMD,
	0
};

static const xCommandLineInput xStopUpdatesCMD =
{
	( uint8_t * ) "mesg",
	( uint8_t * ) "mesg: Stop displaying periodic updates\r\n",
	prvStopUpdatesCMD,
	0
};

static const xCommandLineInput xEnableMotorsCMD =
{
	( uint8_t * ) "motors",
	( uint8_t * ) "motors: Toggle motors\r\n",
	prvEnableMotorsCMD,
	0
};

static const xCommandLineInput xGetPassKeyCMD =
{
	( uint8_t * ) "pass",
	( uint8_t * ) "pass: Retreive your unique passkey\r\n",
	prvGetPassKeyCMD,
	0
};

static const xCommandLineInput xGetMagRawXCMD =
{
	( uint8_t * ) "mag",
	( uint8_t * ) "mag: Retreive the blimp's magnetometer readings\r\n",
	prvGetMagRawXCMD,
	0
};

static const Pin PIN_SET[] = {
    PIN_PUSHBUTTON_1,
    PIN_PWMC_PWM2,
    PIN_PWMC_PWM3,
    PIN_LASER,
    RESET_CTRL,
    nrf24l01plus_MISO,
    nrf24l01plus_MOSI,
    nrf24l01plus_SCK,
    nrf24l01plus_CE,
    nrf24l01plus_CSN,
};


//TODO

char json_string[256] = {0};

static FATFS Fatfs;    /* File system object */
static FIL Fil;        /* File object */

static int8_t pcMainBuff[xMainBuffSize]; 
static portBASE_TYPE xUpdates = 1;

static uint8_t pcTxPktBuffer[32];
static uint8_t pcRxPktBuffer[32];
static uint8_t pcRequestPktBuffer[32];

static struct UpLinkPkt*   ActiveTxPkt = (struct UpLinkPkt*) pcTxPktBuffer;
static struct DownLinkPkt* ActiveRxPkt = (struct DownLinkPkt*) pcRxPktBuffer;
static struct UpLinkPkt*   RequestPkt = (struct UpLinkPkt*) pcRequestPktBuffer;
static portBASE_TYPE RequestPending = 0;

struct uip_conn tuio_conn;

xSemaphoreHandle xPBSemaphore;

xQueueHandle xQueueTuioData;
xQueueHandle xQueueGET;

int main( void )
{
    /* Hardware Setup */
    prvSetupHardware();
    PIO_Configure(PIN_SET, PIO_LISTSIZE(PIN_SET));

    /* System Tasks */
    xTaskCreate( vUSBCDCTask, ( int8_t * ) "USB", mainUSB_STACK, NULL, mainUSB_PRI, NULL );
    /* Start the standard tasks. */
    vStartBlockingQueueTasks( mainBLOCK_Q_PRI );
    vCreateBlockTimeTasks();
    vStartGenericQueueTasks( mainGEN_QUEUE_PRI );
    vStartQueuePeekTasks();   
    vStartDynamicPriorityTasks();

    /* Setup Data Structures */
    xRadioRX_Queue = xQueueCreate(5, sizeof(uint8_t)*32);
    xRadioTX_Queue = xQueueCreate(5, sizeof(uint8_t)*32);

    xQueueTuioData =  xQueueCreate(1, sizeof(uint32_t) * 10);
    xQueueGET =  xQueueCreate(5, sizeof(uint8_t) * 20);

    /* Register CLI Commands */
    FreeRTOS_CLIRegisterCommand(&xTopCMD);
    FreeRTOS_CLIRegisterCommand(&xGetPassKeyCMD);
    FreeRTOS_CLIRegisterCommand(&xGetMagRawXCMD);
    FreeRTOS_CLIRegisterCommand(&xStopUpdatesCMD);
    FreeRTOS_CLIRegisterCommand(&xEnableMotorsCMD);
    FreeRTOS_CLIRegisterCommand(&xSelectBlimpCMD);

    /* User Tasks */
    xTaskCreate(vuIP_Task, (char * ) "AuIP", mainUIP_STACK, NULL, mainUIP_PRI, NULL);
    xTaskCreate(vTrackingTask, (uint8_t * ) "Tracker", TRACK_STACK, NULL, TRACK_PRI, NULL);
    xTaskCreate(vPBTask, (uint8_t * ) "APB", PBTASK_STACK, NULL, PBTASK_PRI, NULL);
    xTaskCreate(vCLI_ReceiveTask, "ACLI", CLI_STACK, NULL, CLI_PRI, NULL );
    xTaskCreate(vRadioTask, "ARadio", RADIO_STACK, NULL, RADIO_PRI, NULL );
    xTaskCreate(vBlimpController, "ABCTRL", BCTRL_STACK, NULL, BCTRL_PRI, NULL );

    vTaskStartScheduler();

    return 0;
}

void vSelectBlimpId(int id){


    ActiveTxPkt->bid = id;

    RF_CHANNEL = 2 * id;

    nrf24l01plus_init();
}

void vRadioTask(void* pvParameters){

    uint8_t tx_buf[TX_PLOAD_WIDTH]; // TX Buffer
    uint8_t rx_buf[TX_PLOAD_WIDTH]; // RX Buffer	

    struct DownLinkPkt* rxPkt = (struct DownLinkPkt*) rx_buf;
    struct UpLinkPkt* txPkt = (struct UpLinkPkt*) tx_buf;

    nrf24l01plus_init();

    vLogOpen();

    vStablizeTxPkt(ActiveTxPkt);

    for (;;) {

        nrf24l01plus_receive_packet();

        vTaskDelay(100);

        if( xQueueReceive(xRadioTX_Queue, tx_buf, 10) ){

            debug_printf("[vRadioTask](Tx) %db %dm %dd %df %da %Xc %Xd\r\n", txPkt->bid,
                    txPkt->motors, txPkt->fwd, txPkt->dir, txPkt->alt, txPkt->cmd,
                    txPkt->data);

            nrf24l01plus_mode_tx_send(tx_buf);

            while(!nrf24l01plus_Send_Packet(tx_buf));

            nrf24l01plus_mode_rx();
        }

        if (xQueueReceive(xRadioRX_Queue, rx_buf, 10) ) {

            if( xUpdates ) {
            debug_printf("[vRadioTask](Rx) %db %dm %df %dd %da %dIR %dmag %Xc %Xdata\r\n",
                    rxPkt->bid, rxPkt->motorsACK, rxPkt->fwdACK, rxPkt->dirACK,
                    rxPkt->altACK, rxPkt->IRRange, rxPkt->mag, rxPkt->cmdACK, rxPkt->data);
            }

            vLogPrintf("%d, %d, %d, %d, %d, %d\r\n", rxPkt->bid, rxPkt->IRRange,
                    rxPkt->mag, rxPkt->fwdACK, rxPkt->dirACK, rxPkt->altACK);

            f_sync(&Fil);

            if( RequestPending != 1 ) continue;
                
            // Request Pending
            
            // Request Not fulfilled resend request
            if( RequestPkt->cmd != rxPkt->cmdACK ){
                xQueueSendToBack(xRadioTX_Queue, (int8_t*) RequestPkt, 0);
                continue;
            }

            // Request fulfilled
            debug_printf("Special Request fulfilled!\r\n");

            switch( rxPkt->cmdACK ){
            
                case SPEC_CMD_PASSKEY:
                    debug_printf("\r\nPassKey = %d\r\n\r\n", rxPkt->data);
                    break;
                case SPEC_CMD_MAGNETO:
                    debug_printf("\r\nMagnetometer = %d\r\n\r\n", rxPkt->data);
                    break;

                default: break;

            }

            RequestPending = 0;
        }
    }
}

void vBlimpController(void *pvParameters){
    
    char pcGETRequest[20];
    char* pcButtonName; 

    for ( ; ; ) {

        vTaskDelay(100);

        if( !xQueueReceive( xQueueGET, (void*) pcGETRequest, 100) ) continue;
        
        pcButtonName = strchr(pcGETRequest, '?');

        debug_printf("[vBlimpController] New Command: %c\r\n", pcButtonName[1]);

        switch( pcButtonName[1] ){

            case 'w': ActiveTxPkt->fwd += 10; break;
            case 'a': ActiveTxPkt->dir += 10; break;
            case 's': ActiveTxPkt->fwd -= 10; break;
            case 'd': ActiveTxPkt->dir -= 10; break;

            case '+': ActiveTxPkt->alt += 10; break;
            case '-': ActiveTxPkt->alt -= 10; break;

            default:
                /* Motors off, dir = 0x7F etc.. */
                vStablizeTxPkt(ActiveTxPkt);
                break;

        }

        xQueueSendToBack(xRadioTX_Queue, (int8_t*) ActiveTxPkt, 0);		
    }

}

void vTuioConnect(void){

    struct uip_conn* conn;

    // Establish TCP Connection
    u16_t ripaddr[2];
    uip_ipaddr(&ripaddr,192,168,0,4);
    conn = uip_connect(&ripaddr, htons(3000));

    memcpy( (void*) &tuio_conn, (void*) conn, sizeof(struct uip_conn) );
}

void vTuioConnectionHold(void){

    if( tuio_conn.tcpstateflags == UIP_TIME_WAIT){
        tuio_conn.tcpstateflags = UIP_ESTABLISHED;
    }

    if( tuio_conn.tcpstateflags == UIP_CLOSED){
        vTuioConnect();
    }
}

void vTrackingTask(void* pvParameters){


    unsigned portBASE_TYPE puxTuioData[10]; 
    static portTickType xLastTx = 0;

    portSHORT sPan, sTilt, sPosX, sPosY, currID, prevID;
    portFLOAT fPosX, fPosY, depth, currX, prevX;

    sPosX = sPosY = 0;    
    fPosX = fPosY = 0.0f;    
    
    sPan = 125;
    sTilt = 90;

    vTuioConnect();

    vConfigurePWM();

    PWMC_SetDutyCycle(CHANNEL_PWM_SERVO1, sPan);
    PWMC_SetDutyCycle(CHANNEL_PWM_SERVO2, sTilt);

    for( ; ; ) {

        vTuioConnectionHold();

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
            sTilt -= ( sPosY - 5 );

            vImposeServoLimits( sPan, sTilt );

            PWMC_SetDutyCycle(CHANNEL_PWM_SERVO1, sPan);
            PWMC_SetDutyCycle(CHANNEL_PWM_SERVO2, sTilt);
        
            debug_printf("[vTrackingTask] %2dx, %2dy, %3dp, %3dt\r\n", sPosX, sPosY, sPan, sTilt );


        }
    }
}

/* Pushbutton Input Task */
void vPBTask(void *pvParameters) {

    vSemaphoreCreateBinary( xPBSemaphore );
    (void) xSemaphoreTake( xPBSemaphore, 10 );

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

        vTaskDelay(200);

        if(xSemaphoreTake(xPBSemaphore, 10)){

            debug_printf("[vPBTask] xPBSemaphore Taken\r\n");

            for(xCount = 0; xCount < 20; xCount++){

                xCount % 2 ? PIO_Clear(pPinLaser) : PIO_Set(pPinLaser);

                vTaskDelay(50);
            }
 
        }
    }
}

portBASE_TYPE prvTopCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    int8_t* pcBuf;

    vTaskList( pcMainBuff );

    pcBuf = pcMainBuff;

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    while( *pcBuf != (portLONG) NULL )
        vUSBSendByte(*pcBuf++);
    
    return pdFALSE;
}


portBASE_TYPE prvEnableMotorsCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    if( ActiveTxPkt->motors == 0 ){
        ActiveTxPkt->motors = 1;
        debug_printf("Motors Enabled!\r\n");
    }else{
        ActiveTxPkt->motors = 0;
        debug_printf("Motors Disabled!\r\n");
    }

    xQueueSendToBack(xRadioTX_Queue, (int8_t*) ActiveTxPkt, 0);		

    return pdFALSE;
}

portBASE_TYPE prvStopUpdatesCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    if ( xUpdates = !xUpdates ) {
        debug_printf("Updates Enabled!\r\n");
    }else{
        debug_printf("Updates Disabled!\r\n");
    }

    return pdFALSE;
}

portBASE_TYPE prvGetPassKeyCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    ActiveTxPkt->cmd = SPEC_CMD_PASSKEY;
    ActiveTxPkt->data = 0x6342;

    memcpy((void*) RequestPkt, (void*) ActiveTxPkt, sizeof(struct UpLinkPkt) );

    xQueueSendToBack(xRadioTX_Queue, (int8_t*) ActiveTxPkt, 0);

    debug_printf("PassKey Requested!\r\n");

    RequestPending = 1;

    return pdFALSE;
}


portBASE_TYPE prvSelectBlimpCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    long lParam_len; 
    char *cCmd_string;

    //Get parameters from command string
    cCmd_string = FreeRTOS_CLIGetParameter(pcCMDStr, 1, &lParam_len);

    cCmd_string[lParam_len] = '\0';

    int i = atoi(cCmd_string);

    vSelectBlimpId(i);

    return pdFALSE;
}

portBASE_TYPE prvGetMagRawXCMD(int8_t *pcBuff, size_t xBuffLen, const int8_t *pcCMDStr ){

    vUSBSendByte('\r');
    vUSBSendByte('\n');

    ActiveTxPkt->cmd = SPEC_CMD_MAGNETO;
    ActiveTxPkt->data = 0x0000;

    memcpy((void*) RequestPkt, (void*) ActiveTxPkt, sizeof(struct UpLinkPkt) );

    xQueueSendToBack(xRadioTX_Queue, (int8_t*) ActiveTxPkt, 0);

    debug_printf("Magnetometer Reading Requested!\r\n");

    RequestPending = 1;

    return pdFALSE;
}

void vCLI_ReceiveTask(void *pvParameters) {

uint8_t cRxeduint8_t;
uint8_t cInputString[20];
uint8_t cInputIndex = 0;
int8_t *pcOutputString;
portBASE_TYPE xReturned;

//Initialise pointer to CLI output buffer.
pcOutputString = FreeRTOS_CLIGetOutputBuffer();

for (;;) {

        //Receive uint8_tacter from USB receive
        cRxeduint8_t = ucUSBReadByte();

        if ( (cRxeduint8_t != 0) && (cRxeduint8_t != 5)) {

            //Process only if return is received.
            if (cRxeduint8_t == '\r') {

                //Put null uint8_tacter in command input string.
                cInputString[cInputIndex] = '\0';

                //Process command input string.
                (void) FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, 20 );

                cInputIndex = 0;
        
            } else if( cRxeduint8_t == '\b' ) { 

                // Backspace was pressed.  Erase the last uint8_tacter in the
                //string - if any.
                if( cInputIndex > 0 ) {
                        cInputIndex--;
                        cInputString[ cInputIndex ] = '\0';
                }

            } else {
                // A uint8_tacter was entered.  Add it to the string
                // entered so far.  When a \n is entered the complete
                // string will be passed to the command interpreter.
                if( cInputIndex < 20 ) {
                        cInputString[ cInputIndex ] = cRxeduint8_t;
                        cInputIndex++;
                }

                //reflect byte
                vUSBSendByte(cRxeduint8_t);
            }
        } 

        vTaskDelay(50);
    }
}

void vLogOpen( void ) {

    int8_t pcLogName[14];
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
