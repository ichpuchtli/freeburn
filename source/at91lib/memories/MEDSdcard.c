/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "MEDSdcard.h"
#include <board.h>
#include <aic/aic.h>
#include <pio/pio.h>
#include <dbgu/dbgu.h>
#include <mci/mci.h>
#include <utility/trace.h>
#include <utility/assert.h>
#include <utility/math.h>
#include <memories/sdmmc/sdmmc_mci.h>

#include <string.h>


#define TINY_MODE_FOR_FAT 1

//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------

/// MCI driver instance.
static Mci mciDrv;

/// SDCard driver instance.
static SdCard sdDrv;

/// buffer for Read/Write operation
static unsigned char buffer[SD_BLOCK_SIZE];

//------------------------------------------------------------------------------
//      Internal Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// MCI0 interrupt handler. Forwards the event to the MCI driver handler.
//------------------------------------------------------------------------------
void ISR_Mci(void)
{
    MCI_Handler(&mciDrv);
}

//------------------------------------------------------------------------------
/// Configure the PIO for SD
//------------------------------------------------------------------------------
void ConfigurePIO(unsigned char mciID)
{
    #ifdef BOARD_SD_PINS 
    const Pin pinSd0[] = {BOARD_SD_PINS};
    #endif
    
    #ifdef BOARD_SD_MCI1_PINS 
    const Pin pinSd1[] = {BOARD_SD_MCI1_PINS};
    #endif
  
    if(mciID == 0) {
        #ifdef BOARD_SD_PINS 
        PIO_Configure(pinSd0, PIO_LISTSIZE(pinSd0));
        #endif
    } else {
        #ifdef BOARD_SD_MCI1_PINS
        PIO_Configure(pinSd1, PIO_LISTSIZE(pinSd1));
        #endif
    }
}

//------------------------------------------------------------------------------
/// Waits for a SD card to be connected.
//------------------------------------------------------------------------------
void WaitSdConn(unsigned char mciID)
{
    Pin* pPin = 0;
  
    if(mciID == 0) {
        #ifdef BOARD_SD_PIN_CD      
        Pin pinMciCardDetect = BOARD_SD_PIN_CD;
        pPin = &pinMciCardDetect;
        #endif
    } else {
        #ifdef BOARD_SD_MCI1_PIN_CD      
        Pin pinMciCardDetect = BOARD_SD_MCI1_PIN_CD;
        pPin = &pinMciCardDetect;        
        #endif      
    }
    
    if(pPin != 0) {
      
        PIO_Configure(pPin, 1);
        TRACE_INFO("Please connect a SD card ...\n\r");
        while (PIO_Get(pPin) != 0);
        TRACE_INFO("SD card connection detected\n\r");
    } else {
      
        TRACE_INFO("SD card detection not available, assuming card is present\n\r");      
    }
}

//------------------------------------------------------------------------------
/// Checks if the device is write protected.
//------------------------------------------------------------------------------
void CheckProtection(unsigned char mciID)
{
    Pin* pPin = 0;  
  
    if(mciID == 0) {
        #ifdef BOARD_SD_PIN_WP      
        Pin pinMciWriteProtect = BOARD_SD_PIN_WP;
        pPin = &pinMciWriteProtect;
        #endif
    } else {
        #ifdef BOARD_SD_MCI1_PIN_WP      
        Pin pinMciWriteProtect = BOARD_SD_MCI1_PIN_WP;
        pPin = &pinMciWriteProtect;        
        #endif      
    }
    
    if(pPin != 0) {
        PIO_Configure(pPin, 1);
        if (PIO_Get(pPin) != 0) {
            TRACE_INFO("SD card is write-protected\n\r");
        }
        else {
            TRACE_INFO("SD card is NOT write-protected.\n\r");
        }  
    } else {
        TRACE_INFO("Cannot check if SD card is write-protected\n\r");
    }
}

//------------------------------------------------------------------------------
//! \brief  Reads a specified amount of data from a SDCARD memory
//! \param  media    Pointer to a Media instance
//! \param  address  Address of the data to read
//! \param  data     Pointer to the buffer in which to store the retrieved
//!                   data
//! \param  length   Length of the buffer
//! \param  callback Optional pointer to a callback function to invoke when
//!                   the operation is finished
//! \param  argument Optional pointer to an argument for the callback
//! \return Operation result code
//------------------------------------------------------------------------------
#if TINY_MODE_FOR_FAT == 0
static unsigned char MEDSdcard_Read(Media         *media,
                                   unsigned int  address,
                                   void          *data,
                                   unsigned int  length,
                                   MediaCallback callback,
                                   void          *argument)
{
    unsigned int block;
    unsigned int offset;    
    unsigned char error;
    unsigned int size;    
    unsigned int remainingLength;
    unsigned char* pDataTmp;
    
    // Check that the media is ready
    if (media->state != MED_STATE_READY) {

        TRACE_INFO("Media busy\n\r");
        return MED_STATUS_BUSY;
    }

    // Check that the data to read is not too big
    if ((length + address) > (media->baseAddress + media->size)) {

        TRACE_WARNING("MEDSdcard_Read: Data too big: %d, %d\n\r", length, address);
        return MED_STATUS_ERROR;
    }

    // Enter Busy state
    media->state = MED_STATE_BUSY;

    // Read data
    block = address / SD_BLOCK_SIZE;
    offset = address % SD_BLOCK_SIZE;
    remainingLength = length;
    pDataTmp = data;
    
    if(offset != 0) {
      
        // the adress is not block aligned
        memset((unsigned int *)buffer, 0xFF, SD_BLOCK_SIZE);         
        
        error = SD_ReadBlock(&sdDrv, block, 1, buffer);
        ASSERT(!error, "\n\r-F- Failed to read block (%d) #%d\n\r", error, block);        
        
        size = min( (SD_BLOCK_SIZE - offset), length);
        memcpy(pDataTmp, &buffer[offset], size);  
        remainingLength -= size;
        pDataTmp += size;
        block++;
    }
      
    while(remainingLength >= SD_BLOCK_SIZE) {

        // Read entire block and copy directly in final data buffer
        error = SD_ReadBlock(&sdDrv, block, 1, pDataTmp);
        ASSERT(!error, "\n\r-F- Failed to read block (%d) #%d\n\r", error, block);
        remainingLength -= SD_BLOCK_SIZE;
        pDataTmp += SD_BLOCK_SIZE;
        block++;
    }
    
    if(remainingLength != 0) {
      
        // copy remaining bytes if needed
        error = SD_ReadBlock(&sdDrv, block, 1, buffer);
        ASSERT(!error, "\n\r-F- Failed to read block (%d) #%d\n\r", error, block);      
        memcpy(pDataTmp, buffer, remainingLength);  
    }
    
    // Leave the Busy state
    media->state = MED_STATE_READY;

    // Invoke callback
    if (callback != 0) {

        callback(argument, MED_STATUS_SUCCESS, 0, 0);
    }

    return MED_STATUS_SUCCESS;
}
#elif TINY_MODE_FOR_FAT == 1
static unsigned char MEDSdcard_Read(Media         *media,
                                   unsigned int  address,
                                   void          *data,
                                   unsigned int  length,
                                   MediaCallback callback,
                                   void          *argument)
{
    unsigned int block;
    unsigned int offset;    
    unsigned char error;  
    unsigned int remainingLength;
    unsigned char* pDataTmp;
    
    // Check that the media is ready
    if (media->state != MED_STATE_READY) {

        TRACE_INFO("Media busy\n\r");
        return MED_STATUS_BUSY;
    }

    // Check that the data to read is not too big
    if ((length + address) > (media->baseAddress + media->size)) {

        TRACE_WARNING("MEDSdcard_Read: Data too big: %d, %d\n\r", length, address);
        return MED_STATUS_ERROR;
    }

    // Enter Busy state
    media->state = MED_STATE_BUSY;

    // Read data
    block = address / SD_BLOCK_SIZE;
    offset = address % SD_BLOCK_SIZE;
    remainingLength = length;
    pDataTmp = data;
     
    while(remainingLength > 0) {

        // Read entire block and copy directly in final data buffer
        error = SD_ReadBlock(&sdDrv, block, 1, pDataTmp);
        ASSERT(!error, "\n\r-F- Failed to read block (%d) #%d\n\r", error, block);
        remainingLength -= SD_BLOCK_SIZE;
        pDataTmp += SD_BLOCK_SIZE;
        block++;
    }
    

    // Leave the Busy state
    media->state = MED_STATE_READY;

    // Invoke callback
    if (callback != 0) {

        callback(argument, MED_STATUS_SUCCESS, 0, 0);
    }

    return MED_STATUS_SUCCESS;
}
#endif
//------------------------------------------------------------------------------
//! \brief  Writes data on a SDRAM media
//! \param  media    Pointer to a Media instance
//! \param  address  Address at which to write
//! \param  data     Pointer to the data to write
//! \param  length   Size of the data buffer
//! \param  callback Optional pointer to a callback function to invoke when
//!                   the write operation terminates
//! \param  argument Optional argument for the callback function
//! \return Operation result code
//! \see    Media
//! \see    MediaCallback
//------------------------------------------------------------------------------
#if TINY_MODE_FOR_FAT == 0
static unsigned char MEDSdcard_Write(Media         *media,
                                    unsigned int  address,
                                    void          *data,
                                    unsigned int  length,
                                    MediaCallback callback,
                                    void          *argument)
{
    unsigned int block;
    unsigned int offset;    
    unsigned char error;
    unsigned int size;    
    unsigned int remainingLength = length;
    unsigned char* pDataTmp;

    // Check that the media if ready
    if (media->state != MED_STATE_READY) {

        TRACE_WARNING("MEDSdcard_Write: Media is busy\n\r");
        return MED_STATUS_BUSY;
    }

    // Check that address is dword-aligned
    if (address%4 != 0) {

        TRACE_WARNING("MEDSdcard_Write: Address must be dword-aligned\n\r");
        return MED_STATUS_ERROR;
    }

    // Check that the data to write is not too big
    if ((length + address) > (media->baseAddress + media->size)) {

        TRACE_WARNING("MEDSdcard_Write: Data too big\n\r");
        return MED_STATUS_ERROR;
    }

    // Put the media in Busy state
    media->state = MED_STATE_BUSY;

    // Write data
    block = address / SD_BLOCK_SIZE;
    offset = address % SD_BLOCK_SIZE;
    remainingLength = length;
    pDataTmp = data;  
    
    if(offset != 0) {
      
        // the adress is not block aligned
        memset((unsigned int *)buffer, 0xFF, SD_BLOCK_SIZE);         
        
        error = SD_ReadBlock(&sdDrv, block, 1, buffer);
        ASSERT(!error, "\n\r-F- Failed to read block (%d) #%d\n\r", error, block);        
        
        size = min( (SD_BLOCK_SIZE - offset), length);
        memcpy(&buffer[offset], pDataTmp, size);  
        
        error = SD_WriteBlock(&sdDrv, block, 1, buffer);
        ASSERT(!error, "\n\r-F- Failed to write block (%d) #%d\n\r", error, block);        
        
        remainingLength -= size;
        pDataTmp += size;
        block++;
    }
      
    while(remainingLength >= SD_BLOCK_SIZE) {

        // Read entire block and copy directly in final data buffer
        error = SD_WriteBlock(&sdDrv, block, 1, pDataTmp);
        ASSERT(!error, "\n\r-F- Failed to read block (%d) #%d\n\r", error, block);
        remainingLength -= SD_BLOCK_SIZE;
        pDataTmp += SD_BLOCK_SIZE;
        block++;
    }
    
    if(remainingLength != 0) {
      
        // write remaining bytes if needed
        memset((unsigned int *)buffer, 0xFF, SD_BLOCK_SIZE);         
        
        error = SD_ReadBlock(&sdDrv, block, 1, buffer);
        ASSERT(!error, "\n\r-F- Failed to read block (%d) #%d\n\r", error, block);        
        
        memcpy(buffer, pDataTmp, remainingLength);  
        
        error = SD_WriteBlock(&sdDrv, block, 1, buffer);
        ASSERT(!error, "\n\r-F- Failed to write block (%d) #%d\n\r", error, block);
    }    
    
    // Leave the Busy state
    media->state = MED_STATE_READY;

    // Invoke the callback if it exists
    if (callback != 0) {

        callback(argument, MED_STATUS_SUCCESS, 0, 0);
    }

    return MED_STATUS_SUCCESS;
}
#elif TINY_MODE_FOR_FAT == 1
static unsigned char MEDSdcard_Write(Media         *media,
                                    unsigned int  address,
                                    void          *data,
                                    unsigned int  length,
                                    MediaCallback callback,
                                    void          *argument)
{
    unsigned int block;  
    unsigned char error; 
    unsigned int remainingLength = length;
    unsigned char* pDataTmp;

    // Check that the media if ready
    if (media->state != MED_STATE_READY) {

        TRACE_WARNING("MEDSdcard_Write: Media is busy\n\r");
        return MED_STATUS_BUSY;
    }

    // Check that the data to write is not too big
    if ((length + address) > (media->baseAddress + media->size)) {

        TRACE_WARNING("MEDSdcard_Write: Data too big\n\r");
        return MED_STATUS_ERROR;
    }

    // Put the media in Busy state
    media->state = MED_STATE_BUSY;

    // Write data
    block = address / SD_BLOCK_SIZE;
    remainingLength = length;
    pDataTmp = data;  
    
    while(remainingLength > 0) {

        // Read entire block and copy directly in final data buffer
        error = SD_WriteBlock(&sdDrv, block, 1, pDataTmp);
        ASSERT(!error, "\n\r-F- Failed to read block (%d) #%d\n\r", error, block);
        remainingLength -= SD_BLOCK_SIZE;
        pDataTmp += SD_BLOCK_SIZE;
        block++;
    }
    
    // Leave the Busy state
    media->state = MED_STATE_READY;

    // Invoke the callback if it exists
    if (callback != 0) {

        callback(argument, MED_STATUS_SUCCESS, 0, 0);
    }

    return MED_STATUS_SUCCESS;
}
#endif
//------------------------------------------------------------------------------
//      Exported Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initializes a Media instance and the associated physical interface
/// \param  media Pointer to the Media instance to initialize
//------------------------------------------------------------------------------
void MEDSdcard_Initialize(Media *media, unsigned char mciID)
{    
    TRACE_INFO("MEDSdcard init\n\r");

    // Initialize SDcard
    //--------------------------------------------------------------------------
    
    // Configure SDcard pins
    ConfigurePIO(mciID);
    
    // Wait for SD card connection (if supported)
    WaitSdConn(mciID);

    // Check if card is write-protected (if supported)
    CheckProtection(mciID);

    // Initialize the MCI driver
    if(mciID == 0) {
        AIC_ConfigureIT(BOARD_SD_MCI_ID,  AT91C_AIC_PRIOR_LOWEST, ISR_Mci);
        MCI_Init(&mciDrv, BOARD_SD_MCI_BASE, BOARD_SD_MCI_ID, BOARD_SD_SLOT);
        AIC_EnableIT(BOARD_SD_MCI_ID);
    } else {
        #if defined(BOARD_SD_MCI1_ID)
        AIC_ConfigureIT(BOARD_SD_MCI1_ID,  AT91C_AIC_PRIOR_LOWEST, ISR_Mci);
        MCI_Init(&mciDrv, BOARD_SD_MCI1_BASE, BOARD_SD_MCI1_ID, BOARD_SD_SLOT);
        AIC_EnableIT(BOARD_SD_MCI1_ID);      
        #else
        TRACE_ERROR("SD/MMC card initialization failed (MCI1 not supported)\n\r");
        #endif
    }

    // Initialize the SD card driver
    if (SD_Init(&sdDrv, (SdDriver *)&mciDrv)) {
    
        TRACE_ERROR("SD/MMC card initialization failed\n\r");
    }
    else {
    
        TRACE_INFO("SD/MMC card initialization successful\n\r");
        TRACE_INFO("Card size: %d MB\n\r", SD_TOTAL_SIZE(&sdDrv)/(1024*1024));
        TRACE_INFO("Block size: %d Bytes\n\r", (SD_TOTAL_SIZE(&sdDrv) / SD_TOTAL_BLOCK(&sdDrv)) );
    }
    MCI_SetSpeed(&mciDrv, 10000000);      
  
    // Initialize media fields
    //--------------------------------------------------------------------------         
    media->write = MEDSdcard_Write;
    media->read = MEDSdcard_Read;
    media->handler = 0;
    media->flush = 0;
    media->baseAddress = 0;
    media->size = SD_TOTAL_SIZE(&sdDrv);
    media->state = MED_STATE_READY;

    media->transfer.data = 0;
    media->transfer.address = 0;
    media->transfer.length = 0;
    media->transfer.callback = 0;
    media->transfer.argument = 0;
    
    numMedias++;    
}

//------------------------------------------------------------------------------
/// erase all the Sdcard
/// \param  media Pointer to the Media instance to initialize
//------------------------------------------------------------------------------
void MEDSdcard_EraseAll(Media *media)
{    
    unsigned int block;  
    unsigned int multiBlock = 1; // change buffer size for multiblocks
    unsigned char error;
    
    TRACE_INFO("MEDSdcard Erase All ...\n\r");

    // Clear the block buffer
    memset(buffer, 0, SD_BLOCK_SIZE * multiBlock);
    
    for (block=0; block < (SD_TOTAL_BLOCK(&sdDrv)-multiBlock); block += multiBlock) {

        error = SD_WriteBlock(&sdDrv, block, multiBlock, buffer);
        ASSERT(!error, "\n\r-F- Failed to write block (%d) #%u\n\r", error, block);  
    }
}

//------------------------------------------------------------------------------
/// erase block
/// \param  media Pointer to the Media instance to initialize
/// \param  block to erase
//------------------------------------------------------------------------------
void MEDSdcard_EraseBlock(Media *media, unsigned int block)
{    
    unsigned char error;
    
    // Clear the block buffer
    memset(buffer, 0, SD_BLOCK_SIZE);
    
    error = SD_WriteBlock(&sdDrv, block, 1, buffer);
    ASSERT(!error, "\n\r-F- Failed to write block (%d) #%u\n\r", error, block);  
}
