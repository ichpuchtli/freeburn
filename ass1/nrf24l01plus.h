/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practical 4 - SPI Bus and the nRF24l01+
# Module name : nrf24l01plus
# Functionality: Provides low level radio calls used to initialise and interface
#        with the nrf24l01+.
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Chris Rieger
# Creation date: 010712
# Revision date (name):
# Changes implemented (date): 
#(Comments): 
------------------------------------------------------------------------------*/

#include <pio/pio.h>
#include <spi/spi.h>

#ifndef _nrf24l01plus_H
#define _nrf24l01plus_H

/// RF pins.
#define nrf24l01plus_MISO {1 << 16, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT}
#define nrf24l01plus_MOSI {1 << 17, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define nrf24l01plus_SCK {1 << 18, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}

#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  32  // 32 unsigned chars TX payload

extern unsigned char rx_buf[TX_PLOAD_WIDTH];
extern unsigned char tx_buf[TX_PLOAD_WIDTH];

extern const Pin CE;
extern const Pin CSN;
extern const Pin MISO;
extern const Pin MOSI;
extern const Pin SCK;

void Init_SPI(void);

extern unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value);

extern unsigned char SPI_Rd(unsigned char reg);

extern unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes);

extern unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes);

void nrf24l01plus_init(void);

void nrf24l01plus_mode_tx_send(void);

void nrf24l01plus_mode_rx(void);

int nrf24l01plus_rxFifoEmpty(void);

int nrf24l01plus_receive_packet(void);

int nrf24l01plus_Send_Packet(void);

// SPI(nRF24L01) commands
#define READ_REG        0x00	// Define read command to register
#define WRITE_REG       0x20	// Define write command to register
#define RD_RX_PLOAD     0x61	// Define RX payload register address
#define WR_TX_PLOAD     0xA0	// Define TX payload register address
#define FLUSH_TX        0xE1	// Define flush TX register command
#define FLUSH_RX        0xE2	// Define flush RX register command
#define ACTIVATE        0x50	// ACTIVATE additional features
#define REUSE_TX_PL     0xE3	// Define reuse TX payload register command
#define R_RX_PL_WID     0x60	// Define Read RX-payload width command
#define W_ACK_PAYLOAD   0xA8	// Write payload to be used in ACK packet on pipe PPP
#define W_TX_PAYLOAD_NOACK 0xB0	// Used in TX mode, Disable AUTOACK on this specific packet
#define OP_NOP          0xFF	// Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00	// 'Config' register address
#define EN_AA           0x01	// 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02	// 'Enabled RX addresses' register address
#define SETUP_AW        0x03	// 'Setup address width' register address
#define SETUP_RETR      0x04	// 'Setup Auto. Retrans' register address
#define RF_CH           0x05	// 'RF channel' register address
#define RF_SETUP        0x06	// 'RF setup' register address
#define STATUS          0x07	// 'Status' register address
#define OBSERVE_TX      0x08	// 'Observe TX' register address
#define RPD             0x09	// 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A	// 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B	// 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C	// 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D	// 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E	// 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F	// 'RX address pipe5' register address
#define TX_ADDR         0x10	// 'TX address' register address
#define RX_PW_P0        0x11	// 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12	// 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13	// 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14	// 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15	// 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16	// 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17	// 'FIFO Status Register' register address
#define DYNPD           0x1C	// 'Enable dynamic payload length' register address
#define FEATURE         0x1D	// Additional features register, needed to enable the additional commands

// SPI(nRF24L01) registers(bitmasks)
#define ERX_P0		0x01	// Enable Pipe 0 (register EN_RXADDR)
#define ERX_P1		0x02	// Enable Pipe 1 (register EN_RXADDR)
#define ERX_P2		0x04	// Enable Pipe 2 (register EN_RXADDR)
#define ERX_P3		0x08	// Enable Pipe 3 (register EN_RXADDR)
#define ERX_P4		0x10	// Enable Pipe 4 (register EN_RXADDR)
#define ERX_P5		0x20	// Enable Pipe 5 (register EN_RXADDR)

#define FIFO_RX_EMPTY		0x01
#define FIFO_RX_FULL		0x02
#define FIFO_TX_EMPTY		0x10
#define FIFO_TX_FULL		0x20
#define FIFO_TX_REUSE		0x40

#define RX_DR    0x40
#define TX_DS    0x20
#define MAX_RT   0x10

#endif
