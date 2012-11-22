 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practical 3 - I2C Bus, the MMA8452Q and PWM Servo Control
# Module name : Main 
# Functionality: Interfacing to the MMA8452Q using I2C, PWM servo control, 
#        ADC of joystick input. The joystick input and accelerometer are used 
#        as inputs to control the servo. 
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Chris Rieger
# Creation date: 270612
# Revision date (name):
# Changes implemented (date): 
#(Comments): 
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <board.h>
#include <usart/usart.h>
#include <adc/adc.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <dbgu/dbgu.h>
#include <pmc/pmc.h>
#include <pwmc/pwmc.h>
#include <tc/tc.h>
#include <utility/math.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <dbgu/dbgu.h>

#include "mma8452.h"
#include "nrf24l01plus.h"
#include "crc16.h"


//------------------------------------------------------------------------------
//         Definitions and variables
//------------------------------------------------------------------------------

// PWM
#define CHANNEL_SERVO1              2
#define CHANNEL_SERVO2              3

#define SERVO_PWM_FREQUENCY         50
#define SERVO_DUTY_CYCLE            2000
#define MAX_DEGREES                 254  // Servo limits
#define MIN_DEGREES                 52   // 

// ADC
#define BOARD_ADC_FREQ              5000000
#define ADC_VREF                    3300  // 3.3 volts

// RF Payload Width
#define PAYLOAD_WIDTH 21

// Size in bytes of the buffer used for reading data from the USB & USART
#define DATABUFFERSIZE \
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

#define JOYSTICK_PUSHBUTTON { 1 << 3, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEGLITCH} 
#define RESET_CTRL { 1 << 30U, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1,0}

#define ROTATE_ENUM(INT, SIZE) INT++; INT %= SIZE 

// Possible servo control inputs
enum InputDevices { JOY_STICK_DEV, ACCEL_DEV, TERMINAL_DEV};
enum RelayDR { RELAY_USART, RELAY_NONE };

// Indicates that the conversion is finished for the ADC.
static volatile unsigned char conversionDone;

// Servo control global defaulted to use the joy stick 
volatile int SERVO_CTRL_MODE = JOY_STICK_DEV;
volatile int DATA_RELAY = RELAY_USART;

// Holds the last char sent through usb 
volatile int TERMINAL_CHAR = ' ';

// Buffer for storing incoming USB data.
static char usbBuffer[DATABUFFERSIZE];
static char usbSendBuffer[DATABUFFERSIZE];

char expectedPayload[DATABUFFERSIZE];

unsigned char USART_INCOMING_DATA[DATABUFFERSIZE];
volatile int USART_RX_COUNTER = 0;

// Buffer for storing incoming USART data
unsigned char USART_TX_BUFF[DATABUFFERSIZE];
unsigned char USART_RX_BUFF[DATABUFFERSIZE];

// Indicates the TX Payload buffer is full or complete with return key 
volatile char RF_PAYLOAD_READY = 0;

// Pio pins to configure.
static const Pin pins[] = {
    PIN_PUSHBUTTON_1,
    JOYSTICK_PUSHBUTTON,
    PIN_LED_0,
    PINS_DBGU,
    PIN_PWMC_PWM2,
    PIN_PWMC_PWM3,
    PIN_LASER,
    PINS_ADC,
    PINS_TWI,
    RESET_CTRL,
    BOARD_RF_CE,
    PIN_SPI0_NPCS0,
    BOARD_RF_SPI_PINS,
    PIN_USART0_RXD,   
    PIN_USART0_TXD
};

// Pio pins to enable/disable mux on the netduino.
static const Pin pinsMux[] = {PIN_MUX1, PIN_MUX2};
char samID[] = {0x42, 0x34, 0x63, 0x42}; // CHANGE THIS TO YOUR OWN!
char vanessaID[] = { 0x42, 0x36, 0x09, 0x55 }; // CHANGE THIS TO YOUR OWN!

typedef struct {
    char type;
    char dest[4];
    char src[4];
    char payload[PAYLOAD_WIDTH];
    short crc;
}__attribute__((packed)) Packet; // 32 bytes long. Data structure for packets. 

Packet *TX_PACKET = (Packet*) tx_buf;
Packet *RX_PACKET = (Packet*) rx_buf;

// MMA8452
short mma8452Data[3];  // Stores the 12-bit signed value.

//------------------------------------------------------------------------------
//         Local functions
//------------------------------------------------------------------------------
void USBDCallbacks_Resumed(void)
{
}

void USBDCallbacks_Suspended(void)
{
}

void UTIL_Loop(volatile unsigned int loop)
{
    while(loop--);    
}

void UTIL_WaitTimeInUs(unsigned int mck, unsigned int time_us)
{
    volatile unsigned int i = 0;
    i = (mck / 1000000) * time_us;
    i = i / 3;
    UTIL_Loop(i);
}

// Converts Hex output from the ADC to a millivolt equivalent.
static unsigned int ConvHex2mV( unsigned int valueToConvert )
{
    return((ADC_VREF * valueToConvert)/0x3FF);
}

// Toggles the case of all alphabetical letters
void ToggleCase(char* buffer){
    
    int i; 

    for(i = 0; i < strlen(buffer); i++){
        
        if(isalpha(buffer[i])) {
            
            if(islower(buffer[i]) ){
                buffer[i] += 'A' - 'a';
            }else{
                buffer[i] -= 'A' - 'a';
            }

        }

    }

}

// Sends a RF packet
void TransmitRFPayload(void){

    TX_PACKET->crc = crc16(0, tx_buf, 30);

    nrf24l01plus_mode_tx_send();

    while(!nrf24l01plus_Send_Packet());

    nrf24l01plus_mode_rx();
 
    memset(TX_PACKET->payload, 0, PAYLOAD_WIDTH);
    
    RF_PAYLOAD_READY = 0;

}

// Fills the USART payload buffer
void PopulateUSARTPayload(unsigned char* buffer, int bytes){

    static unsigned int size = 0;
    unsigned int i;

    for(i = 0; i < bytes; i++){

        // Return Key or TX Packet FULL
        if( buffer[i] == '\r' || buffer[i] == '\n' || size == ( DATABUFFERSIZE - 1 ) ){

            USART_TX_BUFF[size] = '\n';
            USART_TX_BUFF[size+1] = '\0';

            size = 0;

            /* TODO
            if(DATA_RELAY == RELAY_USART){
                memcpy(expectedPayload,USART_TX_BUFF,strlen(USART_TX_BUFF) );
                ToggleCase(expectedPayload);
            }
            */

            USART_WriteBuffer(AT91C_BASE_US0, USART_TX_BUFF, strlen(USART_TX_BUFF) );

            break;
        }

        USART_TX_BUFF[size++] = buffer[i];
    }

}

// Fills the RF payload buffer 
void PopulateRFPayload(unsigned char* buffer, int bytes){

    static int rf_buff_size = 0;
    int i;

    for(i = 0; i < bytes; i++){

        // Return Key or TX Packet FULL
        if( buffer[i] == '\0' || buffer[i] == '\r' || buffer[i] == '\n' || rf_buff_size == (PAYLOAD_WIDTH - 1) ){
            rf_buff_size = 0;
            RF_PAYLOAD_READY = 1;
            break;
        }

        TX_PACKET->payload[rf_buff_size++] = buffer[i];
    }
   
}


// Callback invoked when data has been received on the USB.
static void UsbDataReceived(unsigned int unused,
                            unsigned char status,
                            unsigned int received,
                            unsigned int remaining)
{
    // Check that data has been received successfully
    if (status == USBD_STATUS_SUCCESS) {
        // Send data back through USB
        //CDCDSerialDriver_Write(usbBuffer, received, 0, 0);

            PopulateRFPayload(usbBuffer, received);

        // Grab first byte from the buffer, used to control pan & tilt
        TERMINAL_CHAR = *usbBuffer;

        // Check if bytes have been discarded
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {
            TRACE_WARNING("UsbDataReceived: %u bytes discarded\n\r",
                  remaining);
        }
    }
    else {
        TRACE_WARNING( "UsbDataReceived: Transfer error\n\r");
    }
}

// Interrupt handler for USART0.
void ISR_USART0(void)   
{   
    unsigned int status = AT91C_BASE_US0->US_CSR;   

    PIO_Set(&pins[2]);
    
    if ( (status & AT91C_US_RXBUFF ) == AT91C_US_RXBUFF) {   
    
        USART_ReadBuffer(AT91C_BASE_US0, USART_RX_BUFF, 1);
        
        USART_INCOMING_DATA[USART_RX_COUNTER++] = USART_RX_BUFF[0];

        if(USART_RX_BUFF[0] == '\n'|| USART_RX_BUFF[0] == '\0' || USART_RX_BUFF[0] == '\r'){

            sprintf(usbSendBuffer, "\r\nRECEIVED FROM LASER: %s\r\n", USART_INCOMING_DATA );
            CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);

            if(DATA_RELAY == RELAY_USART){
                //TODO ToggleCase(USART_INCOMING_DATA);
                PopulateRFPayload(USART_INCOMING_DATA,strlen(USART_INCOMING_DATA)); 
            }

            memset(USART_INCOMING_DATA, '\0', DATABUFFERSIZE);
       
            USART_RX_COUNTER = 0;

        }
    }

    PIO_Clear(&pins[2]);
}

// Interrupt handler for the
// Interrupt handler for the ADC. Signals that the conversion is finished by
// setting a flag variable.
static void ISR_ADC(void)
{
    unsigned int status;
    unsigned int id_channel;

    status = ADC_GetStatus(AT91C_BASE_ADC);
    TRACE_DEBUG("status =0x%X\n\r", status);

    for(id_channel=ADC_CHANNEL_2;id_channel<=ADC_CHANNEL_3;id_channel++) {
        if (ADC_IsChannelInterruptStatusSet(status, id_channel)) {
            TRACE_DEBUG("channel %d\n\r", id_channel);
            ADC_DisableIt(AT91C_BASE_ADC, id_channel);
            conversionDone |= 1<<id_channel;
        }
    }
}

/// Interrupt handler for pushbutton 1.
void ISR_PushButton(void)
{
    // Check if the button has been pressed
    if (!PIO_Get(&pins[0])) {
        ROTATE_ENUM(SERVO_CTRL_MODE, 3);
    }
}

/// Interrupt handler for pushbutton 1.
void ISR_JoyStickButton(void)
{
    // Check if the button has been pressed
    if (!PIO_Get(&pins[1])) {

        // Do stuff here when button is pushed.
        ROTATE_ENUM(DATA_RELAY, 2);

        sprintf(usbSendBuffer, "MODE SWITCH: %d\r\n", DATA_RELAY);
        CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
    }
}
// Configures USART0 in hardware handshaking mode, asynchronous, 8 bits, 1 stop   
// bit, no parity, 2400 bauds and enables its transmitter and receiver.   
void ConfigureUsart(void)
{   
    unsigned int mode = AT91C_US_USMODE_NORMAL   
                        | AT91C_US_CLKS_CLOCK   
                        | AT91C_US_CHRL_8_BITS   
                        | AT91C_US_PAR_NONE   
                        | AT91C_US_NBSTOP_1_BIT   
                        | AT91C_US_CHMODE_NORMAL;  
  

    // Enable the peripheral clock in the PMC   
    PMC_EnablePeripheral(AT91C_ID_US0);
   
    // Configure the USART in the desired mode @2400 bauds
    USART_Configure(AT91C_BASE_US0, mode, 9600, BOARD_MCK);
	
    // Configure the RXBUFF interrupt
    AIC_ConfigureIT(AT91C_ID_US0, 0, ISR_USART0);
    AIC_EnableIT(AT91C_ID_US0);

    USART_SetTransmitterEnabled(AT91C_BASE_US0, 1);   
    USART_SetReceiverEnabled(AT91C_BASE_US0, 1);   

    AT91C_BASE_US0->US_IER = AT91C_US_RXBUFF;


}
// Initialise TWI
void Init_TWI(void)
{
    // Enable MUX -- Turns Analog Pins 4&5 into TWI.
    PIO_Configure(pinsMux, PIO_LISTSIZE(pinsMux));
    PIO_Clear(&pinsMux[0]);
    PIO_Clear(&pinsMux[1]);
    
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
    TWI_ConfigureMaster(AT91C_BASE_TWI, TWCK, BOARD_MCK);
    TWID_Initialize(&twid, AT91C_BASE_TWI);
    AIC_ConfigureIT(AT91C_ID_TWI, 0, ISR_Twi);
    AIC_EnableIT(AT91C_ID_TWI);
}

// nitialise PWM
void Init_PWM(void)
{
    // Enable PWMC peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;

    // Set clock A for servo's at 50hz * Duty cycle (samples).
    PWMC_ConfigureClocks(2 * SERVO_PWM_FREQUENCY * SERVO_DUTY_CYCLE, 0, BOARD_MCK);

    // Configure PWMC channel for SERVO (center-aligned, inverted polarity)
    PWMC_ConfigureChannel(CHANNEL_SERVO1, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_SERVO1, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_SERVO1, MIN_DEGREES);
    
    PWMC_ConfigureChannel(CHANNEL_SERVO2, AT91C_PWMC_CPRE_MCKA, AT91C_PWMC_CALG, AT91C_PWMC_CPOL);
    PWMC_SetPeriod(CHANNEL_SERVO2, SERVO_DUTY_CYCLE);
    PWMC_SetDutyCycle(CHANNEL_SERVO2, MIN_DEGREES);
    
    // Enable channel #1 and #2
    PWMC_EnableChannel(CHANNEL_SERVO1);
    PWMC_EnableChannel(CHANNEL_SERVO2);
}

// Initialise ADC
void Init_ADC(void)
{
    ADC_Initialize(AT91C_BASE_ADC,
                AT91C_ID_ADC,
                AT91C_ADC_TRGEN_DIS,
                0,
                AT91C_ADC_SLEEP_NORMAL_MODE,
                AT91C_ADC_LOWRES_10_BIT,
                BOARD_MCK,
                BOARD_ADC_FREQ,
                10,
                1200);
                
    ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_2);
    ADC_EnableChannel(AT91C_BASE_ADC, ADC_CHANNEL_3);

    // Configure interrupts..
    AIC_ConfigureIT(AT91C_ID_ADC, 0, ISR_ADC);
    AIC_EnableIT(AT91C_ID_ADC);
}

// Configures the pushbutton to generate interrupts when pressed.
void Init_Button(void)
{

    // Initialize interrupts
    PIO_InitializeInterrupts(AT91C_AIC_PRIOR_LOWEST);

    PIO_ConfigureIt(&pins[0], (void (*)(const Pin *)) ISR_PushButton);
    PIO_EnableIt(&pins[0]);

    PIO_ConfigureIt(&pins[1], (void (*)(const Pin *)) ISR_JoyStickButton);
    PIO_EnableIt(&pins[1]);
}

// Update Servo Components
void UpdateServoController(void){
        
    static unsigned int pan = 154;
    static unsigned int tilt = 154;
    static int XJoyInt, XAccelInt, YJoyInt, YAccelInt, i;

    // Servo /////////////
    // ADC
    conversionDone = 0;

    // Re-enable interupts for new measurement.
    ADC_EnableIt(AT91C_BASE_ADC, ADC_CHANNEL_2);
    ADC_EnableIt(AT91C_BASE_ADC, ADC_CHANNEL_3);

    // Start measurement
    ADC_StartConversion(AT91C_BASE_ADC);        

    // Wait for Conversion to finish
    while(conversionDone != ((1<<ADC_CHANNEL_2)|(1<<ADC_CHANNEL_3)));
     
    XJoyInt = ConvHex2mV(ADC_GetConvertedData(AT91C_BASE_ADC, ADC_CHANNEL_2));
    YJoyInt = ConvHex2mV(ADC_GetConvertedData(AT91C_BASE_ADC, ADC_CHANNEL_3));
    
    // Read Accelerometer Data
    mma8452_readData(mma8452Data);
    XAccelInt = mma8452Data[0];
    YAccelInt = mma8452Data[1];

    switch(SERVO_CTRL_MODE){

        case JOY_STICK_DEV:
            tilt -= (1650 - XJoyInt) / 512;
            pan -= (1650 - YJoyInt) / 512; 
            break;

        case ACCEL_DEV: 
            pan = 152 + XAccelInt / 16;
            tilt = 152 + YAccelInt / 16;
            break;

        case TERMINAL_DEV:

            switch(TERMINAL_CHAR){
                case 'w': tilt -= 2; break;
                case 's': tilt += 2; break;
                case 'a': pan += 2; break;
                case 'd': pan -= 2; break;
                default:
                    TERMINAL_CHAR = ' '; // Ignore anything other than wasd
            }

    } 

    ImposeServoLimits(&pan, &tilt);
    PWMC_SetDutyCycle(CHANNEL_SERVO1, pan);
    PWMC_SetDutyCycle(CHANNEL_SERVO2, tilt);

    if(i++ % 3 == 0){
        sprintf(usbSendBuffer, "Pan: %3d Tilt: %3d\r\n", pan, tilt);
        CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
    }

    // Servo /////////////

}

// Initialize RF Components
void Init_RF_Comm(void){

    memset(tx_buf,'\0', sizeof(Packet) );
    memset(rx_buf,'\0', sizeof(Packet) );

    TX_PACKET->type = 0xA1;
    RX_PACKET->type = 0xA1;

    memcpy(TX_PACKET->src,samID,4);
}


//------------------------------------------------------------------------------
/// Main function
//------------------------------------------------------------------------------
int main(void)
{

    // Configure io pins
    PIO_Configure(pins, PIO_LISTSIZE(pins));

    // USB // BOT driver initialization
    CDCDSerialDriver_Initialize(); 

    while(USBD_GetState() >= USBD_STATE_CONFIGURED);
     
    // Configure TWI, ADC, PWM, Push Button.
    Init_TWI();
    Init_ADC();
    Init_PWM();
    Init_Button();
    mma8452_init();
    nrf24l01plus_init();
    ConfigureUsart();
    Init_RF_Comm();

    // Infinite loop
    while(1){

        // Check for user input on debug line.
        CDCDSerialDriver_Read(usbBuffer, DATABUFFERSIZE, (TransferCallback) UsbDataReceived,0);

        // RF /////////////
        if(nrf24l01plus_receive_packet()){ // Check inbound RF traffic
            
            sprintf(usbSendBuffer, "\r\nRECEIVED FROM RADIO: %s\r\n", RX_PACKET->payload);
            CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);

            /*
             
            if(DATA_RELAY == RELAY_USART){

                // Check RF Packet Case
                if(strcmp(RX_PACKET->payload, expectedPayload) != 0) {

                    memset(expectedPayload, '\0', DATABUFFERSIZE);

                    sprintf(usbSendBuffer, "\r\nOther system is not responding correctly\r\n");
                    CDCDSerialDriver_Write(usbSendBuffer, strlen(usbSendBuffer), 0, 0);
                }
                                
            TODO 
            */
            
        }

        if(RF_PAYLOAD_READY){
            TransmitRFPayload();
        }
        // RF /////////////
        
        UpdateServoController();
        
        UTIL_WaitTimeInUs(BOARD_MCK, 10000); // delay 10ms
    }
}

