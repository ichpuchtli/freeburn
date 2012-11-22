#/*
#    FreeRTOS V7.1.1 - Copyright (C) 2012 Real Time Engineers Ltd.
#	
#
#    ***************************************************************************
#     *                                                                       *
#     *    FreeRTOS tutorial books are available in pdf and paperback.        *
#     *    Complete, revised, and edited pdf reference manuals are also       *
#     *    available.                                                         *
#     *                                                                       *
#     *    Purchasing FreeRTOS documentation will not only help you, by       *
#     *    ensuring you get running as quickly as possible and with an        *
#     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
#     *    the FreeRTOS project to continue with its mission of providing     *
#     *    professional grade, cross platform, de facto standard solutions    *
#     *    for microcontrollers - completely free of charge!                  *
#     *                                                                       *
#     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
#     *                                                                       *
#     *    Thank you for using FreeRTOS, and thank you for your support!      *
#     *                                                                       *
#    ***************************************************************************
#
#
#    This file is part of the FreeRTOS distribution.
#
#    FreeRTOS is free software; you can redistribute it and/or modify it under
#    the terms of the GNU General Public License (version 2) as published by the
#    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
#    >>>NOTE<<< The modification to the GPL is included to allow you to
#    distribute a combined work that includes FreeRTOS without being obliged to
#    provide the source code for proprietary components outside of the FreeRTOS
#    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
#    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
#    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#    more details. You should have received a copy of the GNU General Public
#    License and the FreeRTOS license exception along with FreeRTOS; if not it
#    can be viewed here: http://www.freertos.org/a00114.html and also obtained
#    by writing to Richard Barry, contact details for whom are available on the
#    FreeRTOS WEB site.
#
#    1 tab == 4 spaces!
#
#    http://www.FreeRTOS.org - Documentation, latest information, license and
#    contact details.
#
#    http://www.SafeRTOS.com - A version that is certified for use in safety
#    critical systems.
#
#    http://www.OpenRTOS.com - Commercial support, development, porting,
#    licensing and training services.
#*/


RTOS_SOURCE_DIR=../source/freeRTOS/Source
DEMO_COMMON_DIR=../source/freeRTOS/Common/Minimal
DEMO_INCLUDE_DIR=../source/freeRTOS/Common/include
UIP_COMMON_DIR=../source/netduinoplus/uip/uip
AT91LIB = ../source/at91lib
NETDUINO_DIR = ../source/netduinoplus
CLI_SOURCE_DIR=../source/freeRTOS/freeRTOS_CLI
UTILITY = $(AT91LIB)/utility
PERIPH = $(AT91LIB)/peripherals
PROGNAME = main

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size
LDSCRIPT=$(NETDUINO_DIR)/atmel-rom.ld

LINKER_FLAGS=-mthumb -nostartfiles -Xlinker -o$(PROGNAME).elf -Xlinker -M -Xlinker -Map=$(PROGNAME).map -static

DEBUG=-g
OPTIM=-O3


CFLAGS= $(DEBUG) \
		$(OPTIM) \
		-T$(LDSCRIPT) \
		-I . \
		-I $(NETDUINO_DIR) \
		-I $(RTOS_SOURCE_DIR)/include \
		-I $(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S \
		-I $(DEMO_INCLUDE_DIR) \
		-I $(NETDUINO_DIR)/uip \
		-I $(UIP_COMMON_DIR)/uip \
		-I $(UIP_COMMON_DIR)/apps/webserver \
		-I $(NETDUINO_DIR)/SrcAtmel \
		-I $(NETDUINO_DIR)/USB \
		-I $(NETDUINO_DIR)/SDCard \
		-I$(AT91LIB)/peripherals \
		-I$(AT91LIB) \
		-I $(CLI_SOURCE_DIR) \
		-D SAM7_GCC \
		-D THUMB_INTERWORK \
		-mcpu=arm7tdmi \
		-D PACK_STRUCT_END=__attribute\(\(packed\)\) \
		-D ALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\) \
		-fomit-frame-pointer \
		-mthumb-interwork \
		-fno-strict-aliasing \
		-fno-dwarf2-cfi-asm
					
THUMB_SOURCE= \
		$(PROGNAME).c \
		$(DEMO_COMMON_DIR)/BlockQ.c \
		$(DEMO_COMMON_DIR)/blocktim.c \
		$(DEMO_COMMON_DIR)/integer.c \
		$(DEMO_COMMON_DIR)/GenQTest.c \
		$(DEMO_COMMON_DIR)/QPeek.c \
		$(DEMO_COMMON_DIR)/dynamic.c \
		$(RTOS_SOURCE_DIR)/list.c \
		$(RTOS_SOURCE_DIR)/queue.c \
		$(RTOS_SOURCE_DIR)/tasks.c \
		$(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S/port.c \
		$(RTOS_SOURCE_DIR)/portable/MemMang/heap_1.c \
		$(NETDUINO_DIR)/USB/USB-CDC.c \
		$(NETDUINO_DIR)/debug_printf.c \
		$(PERIPH)/aic/aic.c \
		$(PERIPH)/pio/pio.c \
		$(PERIPH)/pmc/pmc.c \
		$(PERIPH)/spi/spi.c \
		$(PERIPH)/pwmc/pwmc.c \
		$(PERIPH)/pit/pit.c \
		$(UTILITY)/led.c \
		$(NETDUINO_DIR)/ParTest/ParTest.c \
		$(NETDUINO_DIR)/syscalls.c \
		$(UIP_COMMON_DIR)/uip/uip_arp.c \
		$(UIP_COMMON_DIR)/uip/psock.c \
		$(UIP_COMMON_DIR)/uip/timer.c \
		$(UIP_COMMON_DIR)/uip/uip.c \
		$(NETDUINO_DIR)/uip/uIP_Task.c \
		$(NETDUINO_DIR)/uip/SAM7_EMAC.c \
		$(NETDUINO_DIR)/SDCard/ff.c \
		$(NETDUINO_DIR)/SDCard/mmc_spi.c \
		$(CLI_SOURCE_DIR)/FreeRTOS_CLI.c \
		./webserver/httpd.c \
		./webserver/httpd-cgi.c \
		./webserver/httpd-fs.c \
		./webserver/http-strings.c \
		./nrf24l01plus.c		\
		./tuioclient.c

ARM_SOURCE= \
		$(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S/portISR.c \
		$(RTOS_SOURCE_DIR)/portable/GCC/ARM7_AT91SAM7S/lib_AT91SAM7X256.c \
		$(NETDUINO_DIR)/SrcAtmel/Cstartup_SAM7.c \
		$(NETDUINO_DIR)/uip/EMAC_ISR.c \
		$(NETDUINO_DIR)/USB/USBIsr.c  \
		./pbISR.c

THUMB_OBJS = $(THUMB_SOURCE:.c=.o)
ARM_OBJS = $(ARM_SOURCE:.c=.o)


all: $(PROGNAME).bin

$(PROGNAME).bin : $(PROGNAME).hex
	$(OBJCOPY) $(PROGNAME).elf -O binary $(PROGNAME).bin
	arm-none-eabi-size -A -t $(PROGNAME).elf
	 
$(PROGNAME).hex : $(PROGNAME).elf
	$(OBJCOPY) $(PROGNAME).elf -O ihex $(PROGNAME).hex

$(PROGNAME).elf : $(THUMB_OBJS) $(ARM_OBJS) $(NETDUINO_DIR)/boot.s Makefile
	$(CC) $(CFLAGS) $(ARM_OBJS) $(THUMB_OBJS) $(LIBS) $(NETDUINO_DIR)/boot.s $(LINKER_FLAGS) 

$(THUMB_OBJS) : %.o : %.c Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) -mthumb $< -o $@

$(ARM_OBJS) : %.o : %.c Makefile FreeRTOSConfig.h
	$(CC) -c $(CFLAGS) $< -o $@

clean :
	rm -f *.elf
	rm -f *.hex
	rm -f *.bin
	rm -f *.map
	rm -f $(THUMB_OBJS)
	rm -f $(ARM_OBJS)
	
	

