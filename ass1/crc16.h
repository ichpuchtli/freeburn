/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practical 4 - SPI Bus and the nRF24l01+
# Module name : nrf24l01plus
# Functionality: Implements the standard CRC-16:
#                Width 16
#                Poly  0x8005 (x^16 + x^15 + x^2 + 1)
#                Init  0
# Hardware platform: Netduino (AT91SAM7)
#
# Author name: Ben Gardner
# Creation date: 010105
# Revision date (name): 020712 Chris Rieger,
# Changes implemented (date): 
#(Comments): This source code is licensed under the GNU General Public License,
# Version 2. Copyright (c) 2005 Ben Gardner <bgardner@wabtec.com>
------------------------------------------------------------------------------*/

#ifndef __CRC16_H
#define __CRC16_H

#define u8  unsigned char
#define u16 unsigned short
typedef  unsigned int  size_t;

extern u16 const crc16_table[256];

extern u16 crc16(u16 crc, const u8 *buffer, size_t len);

static inline u16 crc16_byte(u16 crc, const u8 data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

#endif /* __CRC16_H */

