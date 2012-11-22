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

#include <twi/twi.h>
#include <drivers/twi/twid.h>

// TWI clock frequency in Hz.
#define TWCK            400000

// Slave address.
#define MMA8452_ADDRESS 0x1D // Sparkfun board defaults SAO to 1. If jumper on base is set, address is 0x1C

extern const unsigned char SCALE;

extern const unsigned char dataRate;

extern Twid twid;

extern void ISR_Twi(void);

extern int mma8452_isconnect(void);

extern void mma8452_readData(short * destination);

extern void mma8452_raw2g(short * data, float * accelG);

extern void mma8452_standby(void);

extern void mma8452_active(void);

extern void mma8452_init();
