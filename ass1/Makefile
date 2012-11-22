# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support 
# ----------------------------------------------------------------------------
# Copyright (c) 2008, Atmel Corporation
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
#
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#		User-modifiable options
#-------------------------------------------------------------------------------
CHIP  = at91sam7x512

# Trace level used for compilation
# (can be overriden by adding TRACE_LEVEL=#number to the command-line)
# TRACE_LEVEL_DEBUG      5
# TRACE_LEVEL_INFO       4
# TRACE_LEVEL_WARNING    3
# TRACE_LEVEL_ERROR      2
# TRACE_LEVEL_FATAL      1
# TRACE_LEVEL_NO_TRACE   0
TRACE_LEVEL = 0

# Optimization level, put in comment for debugging
OPTIMIZATION = -Os

# AT91 library directory
AT91LIB = ../source/at91lib

# Output file basename
OUTPUT = ass-1-$(CHIP)

#Which memories to output, flash or sram
MEMORIES = flash 

# Output directories
BIN = bin
OBJ = obj

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Compilation tools
CC = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy

# Flags
INCLUDES = -I$(AT91LIB)/boards/ -I$(AT91LIB)/peripherals 
INCLUDES += -I$(AT91LIB)/components -I$(AT91LIB)/memories -I$(AT91LIB) -I$(AT91LIB)/boards/$(CHIP)

CFLAGS = -Wall -mlong-calls -ffunction-sections -lm
CFLAGS += -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -DTRACE_LEVEL=$(TRACE_LEVEL)
ASFLAGS = -g $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -D__ASSEMBLY__
LDFLAGS = -g $(OPTIMIZATION) -nostartfiles -Wl,--gc-sections

#-------------------------------------------------------------------------------
#		Files
#-------------------------------------------------------------------------------

# Directories where source files can be found
USB = $(AT91LIB)/usb
PERIPH = $(AT91LIB)/peripherals
BOARDS = $(AT91LIB)/boards
UTILITY = $(AT91LIB)/utility
DRV = $(AT91LIB)/drivers

VPATH += $(USB)/device/cdc-serial $(USB)/device/core $(USB)/common/core
VPATH += $(USB)/common/cdc $(DRV)/twi
VPATH += $(PERIPH)/dbgu $(PERIPH)/pio $(PERIPH)/pwmc $(PERIPH)/aic $(PERIPH)/twi $(PERIPH)/pmc
VPATH += $(PERIPH)/cp15 $(PERIPH)/spi $(PERIPH)/adc $(PERIPH)/pit $(PERIPH)/tc $(PERIPH)/usart

VPATH += $(UTILITY)
VPATH += $(BOARDS) $(BOARDS)/$(CHIP)

# Objects built from C source files
C_OBJECTS = main.o mma8452.o nrf24l01plus.o crc16.o


C_OBJECTS += CDCDSerialDriver.o CDCDSerialDriverDescriptors.o
C_OBJECTS += CDCSetControlLineStateRequest.o CDCLineCoding.o
C_OBJECTS += USBD_OTGHS.o USBD_UDP.o USBD_UDPHS.o USBDDriver.o
C_OBJECTS += USBDCallbacks_Initialized.o
C_OBJECTS += USBDCallbacks_Reset.o
#C_OBJECTS += USBDCallbacks_Resumed.o
#C_OBJECTS += USBDCallbacks_Suspended.o
C_OBJECTS += USBDDriverCb_CfgChanged.o
C_OBJECTS += USBDDriverCb_IfSettingChanged.o
C_OBJECTS += USBSetAddressRequest.o USBGenericDescriptor.o USBInterfaceRequest.o
C_OBJECTS += USBGenericRequest.o USBGetDescriptorRequest.o 
C_OBJECTS += USBSetConfigurationRequest.o USBFeatureRequest.o
C_OBJECTS += USBEndpointDescriptor.o USBConfigurationDescriptor.o

C_OBJECTS += string.o twid.o stdio.o led.o math.o 
C_OBJECTS += dbgu.o pio.o pio_it.o pwmc.o twi.o aic.o pmc.o cp15.o adc.o spi.o usart.o
C_OBJECTS += board_memories.o board_lowlevel.o

# Objects built from Assembly source files
ASM_OBJECTS = board_cstartup.o
ASM_OBJECTS += cp15_asm.o

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT)

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------

all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ):
	mkdir $@

define RULES
C_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(C_OBJECTS))
ASM_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(ASM_OBJECTS))

$(1): $$(ASM_OBJECTS_$(1)) $$(C_OBJECTS_$(1))
	$(CC) $(LDFLAGS) -T"$(AT91LIB)/boards/$(CHIP)/$$@.lds" -o $(OUTPUT)-$$@.elf $$^
	$(OBJCOPY) -O binary $(OUTPUT)-$$@.elf $(OUTPUT)-$$@.bin
	$(SIZE) $$^ $(OUTPUT)-$$@.elf

$$(C_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.c Makefile $(OBJ) $(BIN)
	$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<

$$(ASM_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.S Makefile $(OBJ) $(BIN)
	$(CC) $(ASFLAGS) -D$(1) -c -o $$@ $$<

endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-rm -f $(OBJ)/*.o $(BIN)/*.bin $(BIN)/*.elf

