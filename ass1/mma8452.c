 /* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practical 3 - I2C Bus, the MMA8452Q and PWM Servo Control
# Module name : mma8452
# Functionality: A collection of low level calls to the MMA8452 for initialising
#        and interfacing with the device.
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Chris Rieger
# Creation date: 270612
# Revision date (name):
# Changes implemented (date): 
#(Comments): 
------------------------------------------------------------------------------*/

#include "mma8452.h"

// TWI driver instance.
Twid twid;

// Set the scale below either 2, 4 or 8
const unsigned char SCALE = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
 
// Set the output data rate below. Value should be between 0 and 7
const unsigned char dataRate = 0;  // 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56

// TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
void ISR_Twi(void)
{
    TWID_Handler(&twid);
}

// Read the WHO_AM_I register, this is a good test of communication
int mma8452_isconnect(void)
{
    unsigned char c;
	TWID_Read(&twid, MMA8452_ADDRESS, 0x0D, 1, &c, 1, 0);
	return c == 0x2A;
}

// Read accellerometer data from the mma8452 and put them in destination.
// Destination must be a an array of at least 3 ints.
void mma8452_readData(short * destination)
{
    int i;
    unsigned char rawData[6];  // x/y/z accel register data stored here

    TWID_Read(&twid, MMA8452_ADDRESS, 0x01, 1, &rawData[0], 6, 0);
  
    /* loop to calculate 12-bit ADC and g value for each axis */
    for (i=0; i<6; i+=2)
    {
	    // We first construct the 12 bit two's complement number in the most
		// significant bytes of the short int (destination). We then scale it back
		// dividing by 2^4, keeping the sign.
        destination[i/2] = ((rawData[i] << 8) | rawData[i+1]);
		destination[i/2] = destination[i/2] / 16;
    }
}

// Convert the accleration values into actual g's.
void mma8452_raw2g(short * data, float * accelG)
{
    int i;
    for (i=0; i<3; i++) {
        accelG[i] = (float) data[i]/((1<<10)/(2*SCALE));  // get actual g value, this depends on scale being set
    }
 }
 
// Sets the MMA8452 to standby mode.
// It must be in standby to change most register settings.
void mma8452_standby(void)
{
    unsigned char c;
    TWID_Read(&twid, MMA8452_ADDRESS, 0x2A, 1, &c, 1, 0);
    c = c & ~(0x01);
    TWID_Write(&twid, MMA8452_ADDRESS, 0x2A, 1, &c, 1, 0);
}

// Sets the MMA8452 to active mode.
// Needs to be in this mode to output data.
void mma8452_active(void)
{
    unsigned char c;
    TWID_Read(&twid, MMA8452_ADDRESS, 0x2A, 1, &c, 1, 0);
    c = c | 0x01;
    TWID_Write(&twid, MMA8452_ADDRESS, 0x2A, 1, &c, 1, 0);
}

// Initialise the MMA8452
void mma8452_init(void)
{
    unsigned char temp;
    mma8452_standby();  // Must be in standby to change registers
  
    /* Set up the full scale range to 2, 4, or 8g. */
    if ((SCALE==2)||(SCALE==4)||(SCALE==8)) {
        temp = SCALE >> 2;
        TWID_Write(&twid, MMA8452_ADDRESS, 0x0E, 1, &temp, 1, 0);
    } else {
        temp = 0;
        TWID_Write(&twid, MMA8452_ADDRESS, 0x0E, 1, &temp, 1, 0);
    }
    
    /* Setup the 3 data rate bits, from 0 to 7 */
    TWID_Read(&twid, MMA8452_ADDRESS, 0x2A, 1, &temp, 1, 0);
    temp = temp & ~(0x38);
    TWID_Write(&twid, MMA8452_ADDRESS, 0x2A, 1, &temp, 1, 0);
    if (dataRate <= 7) {
        TWID_Read(&twid, MMA8452_ADDRESS, 0x2A, 1, &temp, 1, 0);
        temp = temp | (dataRate << 3);
        TWID_Write(&twid, MMA8452_ADDRESS, 0x2A, 1, &temp, 1, 0);
    }

    mma8452_active();  // Set to active to start reading
}
