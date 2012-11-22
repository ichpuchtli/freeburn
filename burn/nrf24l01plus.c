/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Proj3
# Module name : nrf24l01plus
# Functionality: Provides low level radio calls used to initialise and interface
#        with the nrf24l01+.
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Chris Rieger
# Creation date: 010712
# Revision date (name): 071012
# Changes implemented (date): Updated for Blimp control which using the arduino
# rf24 library.
#(Comments): 
------------------------------------------------------------------------------*/
 
#include "FreeRTOS.h"
#include "queue.h"

#include "nrf24l01plus.h"

// The ACMA has allowed frequencies 2300.000 MHz - 2450.000 MHz.
// The nrf24l01+ can operate on 125 channels, however, please
// limit yourself to the lower 50 channels. More information
// can be found here: http://www.acma.gov.au
//
int RF_CHANNEL = 16;

const Pin CE = nrf24l01plus_CE;
const Pin CSN = nrf24l01plus_CSN;
const Pin MISO = nrf24l01plus_MISO;
const Pin MOSI = nrf24l01plus_MOSI;
const Pin SCK = nrf24l01plus_SCK;

const char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0xD2,0xF0,0xF0,0xF0,0xF0
}; // Define a static TX address.

const char RX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0xE1,0xF0,0xF0,0xF0,0xF0
}; // Define a static RX address.


// Initialises the SPI controller on the at91.
void Init_SPI(void)
{
    // Disable these from PIO controller.
    AT91C_BASE_PIOA->PIO_PDR = 
        AT91C_PA16_MISO0 | 
        AT91C_PA17_MOSI0 | 
        AT91C_PA18_SPCK0; 
    
    SPI_Configure(AT91C_BASE_SPI0,
        AT91C_ID_SPI0,
        AT91C_SPI_MSTR |
        AT91C_SPI_PS_FIXED |
        AT91C_SPI_MODFDIS |
        SPI_PCS(1) // Need to configure a CS, however, we are using a software CS.  
    ); 

    //Configure SPI Chip Select Register.
    SPI_ConfigureNPCS(AT91C_BASE_SPI0,
        1,                  // NPCS#
        AT91C_SPI_NCPHA |   // Mode 0 (CPOL=0,CPHA=0)
        (128 << 8) |        // SCBR: SPCK=MCK/SCBR //8 or 64
        (0 << 16) |         // DLYBS: Delay from NPCS to SPCK in MCK cycles
        (0 << 24) |         // DLYBCT: Delay between consec xfers
        AT91C_SPI_BITS_8    // 8-data bits
    ); 
    
    SPI_Enable(AT91C_BASE_SPI0);
}

// Writes value to given register over SPI.
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
    unsigned char status;

    PIO_Clear(&CSN);                           // CSN low, init SPI transaction
    SPI_Write(AT91C_BASE_SPI0, 0, reg);        // select register
    while(!SPI_IsFinished(AT91C_BASE_SPI0));   // Wait for write to complete
    status = SPI_Read(AT91C_BASE_SPI0);        // read status of the 24l01
    SPI_Write(AT91C_BASE_SPI0, 0, value);      // ..and write value to it..
    while(!SPI_IsFinished(AT91C_BASE_SPI0));   // Wait for write to complete
    PIO_Set(&CSN);                             // CSN high again

    return(status);                // return nRF24L01 status unsigned char
}

// Returns value from given register.
unsigned char SPI_Rd(unsigned char reg)
{
    unsigned char reg_val;

    PIO_Clear(&CSN);                           // CSN low, initialize SPI communication...
    SPI_Write(AT91C_BASE_SPI0, 0, reg);        // Select register to read from..
    while(!SPI_IsFinished(AT91C_BASE_SPI0));   // Wait for write to complete
    SPI_Write(AT91C_BASE_SPI0, 0, 0);          // Send Dummy Byte 
    while(!SPI_IsFinished(AT91C_BASE_SPI0));   // Wait for write to complete
    reg_val = SPI_Read(AT91C_BASE_SPI0);       // ..then read register value
    PIO_Set(&CSN);                             // CSN high, terminate SPI communication
  
    return(reg_val);               // return register value
}

// Reads response of a multi-byte SPI transaction.
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
    unsigned char status,i;

    PIO_Clear(&CSN);                           // Set CSN low, init SPI tranaction
    SPI_Write(AT91C_BASE_SPI0, 0, reg);        // Select register to write to and read status unsigned char
    while(!SPI_IsFinished(AT91C_BASE_SPI0));   // Wait for write to complete
    status = SPI_Read(AT91C_BASE_SPI0);        // read status of the 24l01

    for(i=0;i<bytes;i++)
    {
        SPI_Write(AT91C_BASE_SPI0, 0, 0);             // write dummy byte
            while(!SPI_IsFinished(AT91C_BASE_SPI0));  // Wait for write to complete
        pBuf[i] = SPI_Read(AT91C_BASE_SPI0);          // Read unsigned char from nRF24L01
    }

    PIO_Set(&CSN);                                    // Set CSN high again

    return(status);                  // return nRF24L01 status unsigned char
}

// Writes multple bytes to a given register over SPI.
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
    unsigned char status,i;

    PIO_Clear(&CSN);                                 // Set CSN low, init SPI tranaction
    SPI_Write(AT91C_BASE_SPI0, 0, reg);              // Select register to write to and read status unsigned char
    while(!SPI_IsFinished(AT91C_BASE_SPI0));         // Wait for write to complete
    status = SPI_Read(AT91C_BASE_SPI0);              // read status of the 24l01
    for(i=0;i<bytes; i++)                            // then write all unsigned char in buffer(*pBuf)
    {
        SPI_Write(AT91C_BASE_SPI0, 0, *pBuf++);
        while(!SPI_IsFinished(AT91C_BASE_SPI0));     // Wait for write to complete
    }
    PIO_Set(&CSN);                                   // Set CSN high again
  
    return(status);                // return nRF24L01 status unsigned char
}

// Initialise radio (in rx mode)
void nrf24l01plus_init(void)
{    
    Init_SPI(); // Enable SPI
    
    PIO_Clear(&CE);
    PIO_Set(&CSN);    
        
    PIO_Clear(&CE);  
    
    SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x4F);
    SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);
    SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P1, RX_ADDRESS, TX_ADR_WIDTH); // Reading pipe
    SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x03);          // Enable Pipe0 + Pipe1
    SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
    SPI_RW_Reg(WRITE_REG + RF_CH, RF_CHANNEL);        // Select RF channel
    SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
    
    PIO_Set(&CE);
}

// Switch to TX mode
void nrf24l01plus_mode_tx_send(unsigned char *tx_buf)
{
    PIO_Clear(&CE);    
    SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX.
    SPI_RW_Reg(FLUSH_TX,0);                                  
    SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);   // write playload to TX_FIFO
    PIO_Set(&CE);                             // Set CE pin high to enable RX device   
}

// Switch to RX mode
void nrf24l01plus_mode_rx(void)
{
    PIO_Clear(&CE);    
    SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. 
    PIO_Set(&CE);                             // Set CE pin high to enable RX device
}

// Is fifo empty?
int nrf24l01plus_rxFifoEmpty(void)
{
    unsigned char fifoStatus;
    fifoStatus = SPI_Rd(FIFO_STATUS);
    return (fifoStatus & FIFO_RX_EMPTY);
}

// Attempts to receive a packet and puts it into rx_buf. Returns 1 on successful receive. 
int nrf24l01plus_receive_packet(void)
{
    int rec = 0;
    unsigned char rx_buf[TX_PLOAD_WIDTH];                     // temp buffer
    unsigned char status = SPI_Rd(STATUS);                  // read register STATUS's value
    if((status & RX_DR) && !nrf24l01plus_rxFifoEmpty())     // if receive data ready interrupt and FIFO full.
    {
        SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);  // read playload to rx_buf
        SPI_RW_Reg(FLUSH_RX,0);                             // clear RX_FIFO
        xQueueSendToBack(xRadioRX_Queue, rx_buf, 0);        // Add packet to queue
        rec = 1;
    }
    SPI_RW_Reg(WRITE_REG+STATUS,status);                    // clear RX_DR or TX_DS or MAX_RT interrupt flag
    
    return rec;
}

// Sends packet currenting in the tx_buf.
int nrf24l01plus_Send_Packet(unsigned char *tx_buf)
{
    int rec = 0;
    unsigned char status = SPI_Rd(STATUS);                // read register STATUS's value
    if(status&TX_DS || status&MAX_RT)                     // if receive data ready (TX_DS) interrupt
    {
        PIO_Clear(&CE);    
        SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);
        SPI_RW_Reg(FLUSH_TX,0);                                  
        SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);   // write playload to TX_FIFO
        rec = 1;
        PIO_Set(&CE);
    }
    SPI_RW_Reg(WRITE_REG+STATUS,status);                 // clear RX_DR or TX_DS or MAX_RT interrupt flag
    return rec;
}
