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

#include "ManagedNandFlash.h"
#include "NandSpareScheme.h"
#include "NandFlashModel.h"
#include "RawNandFlash.h"
#include <utility/trace.h>
#include <utility/assert.h>

#include <string.h>

//------------------------------------------------------------------------------
//         Internal definitions
//------------------------------------------------------------------------------

// Casts
#define ECC(managed)    ((struct EccNandFlash *) managed)
#define RAW(managed)    ((struct RawNandFlash *) managed)
#define MODEL(managed)  ((struct NandFlashModel *) managed)

// Values returned by the CheckBlock() function
#define BADBLOCK        255
#define GOODBLOCK       254

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Returns 1 if a nandflash device is virgin (i.e. has never been used as a
/// managed nandflash); otherwise return 0.
/// \param managed  Pointer to a ManagedNandFlash instance.
//------------------------------------------------------------------------------
static unsigned char IsDeviceVirgin(const struct ManagedNandFlash *managed)
{
    struct NandBlockStatus blockStatus;
    const struct NandSpareScheme *scheme = NandFlashModel_GetScheme(MODEL(managed));
    unsigned char spare[NandCommon_MAXPAGESPARESIZE];
    unsigned char badBlockMarker;
    
    unsigned char error;

    // Read spare area of page #0
    error = RawNandFlash_ReadPage(RAW(managed), 0, 0, 0, spare);
    ASSERT(!error, "ManagedNandFlash_IsDeviceVirgin: Failed to read page #0\n\r");

    // Retrieve bad block marker and block status from spare area
    NandSpareScheme_ReadBadBlockMarker(scheme, spare, &badBlockMarker);
    NandSpareScheme_ReadExtra(scheme, spare, &blockStatus, 4, 0);

    // Check if block is marked as bad
    if (badBlockMarker != 0xFF) {

        // Device is not virgin, since page #0 is guaranteed to be good
        return 0;
    }
    // If device is not virgin, then block status will be set to either
    // FREE, DIRTY or LIVE
    else if (blockStatus.status != NandBlockStatus_DEFAULT) {

        // Device is not virgin
        return 0;
    }

    return 1;
}

//------------------------------------------------------------------------------
/// Returns BADBLOCK if the given block of a nandflash device is bad; returns
/// GOODBLOCK if the block is good; or returns a NandCommon_ERROR code.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param block  Number of block to check.
//------------------------------------------------------------------------------
static unsigned char CheckBlock(
    const struct ManagedNandFlash *managed,
    unsigned short block)
{
    unsigned char spare[NandCommon_MAXPAGESPARESIZE];
    unsigned char error;
    unsigned int i;
    unsigned char pageSpareSize = NandFlashModel_GetPageSpareSize(MODEL(managed));

    // Read spare area of first page of block
    error = RawNandFlash_ReadPage(RAW(managed), block, 0, 0, spare);
    if (error) {

        TRACE_ERROR("CheckBlock: Cannot read page #0 of block #%d\n\r", block);
        return error;
    }

    // Make sure it is all 0xFF
    for (i=0; i < pageSpareSize; i++) {

        if (spare[i] != 0xFF) {

            return BADBLOCK;
        }
    }

    // Read spare area of second page of block
    error = RawNandFlash_ReadPage(RAW(managed), block, 1, 0, spare);
    if (error) {

        TRACE_ERROR("CheckBlock: Cannot read page #1 of block #%d\n\r", block);
        return error;
    }

    // Make sure it is all 0xFF
    for (i=0; i < pageSpareSize; i++) {

        if (spare[i] != 0xFF) {

            return BADBLOCK;
        }
    }

    return GOODBLOCK;
}

//------------------------------------------------------------------------------
/// Physically writes the status of a block inside its first page spare area.
/// Returns 0 if successful; otherwise returns a NandCommon_ERROR_xx code.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param block  Block number.
//------------------------------------------------------------------------------
static unsigned char WriteBlockStatus(
    const struct ManagedNandFlash *managed,
    unsigned short block)
{
    unsigned char spare[NandCommon_MAXPAGESPARESIZE];

    memset(spare, 0xFF, NandCommon_MAXPAGESPARESIZE);
    NandSpareScheme_WriteExtra(NandFlashModel_GetScheme(MODEL(managed)),
                               spare,
                               &(managed->blockStatuses[block]),
                               4,
                               0);
    return RawNandFlash_WritePage(RAW(managed), block, 0, 0, spare);
}

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initializes a ManagedNandFlash instance. Scans the device to retrieve or
/// create block status information.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param model  Pointer to the underlying nand chip model. Can be 0.
/// \param commandAddress  Address at which commands are sent.
/// \param addressAddress  Address at which addresses are sent.
/// \param dataAddress  Address at which data is sent.
/// \param pinChipEnable  Pin controlling the CE signal of the NandFlash.
/// \param pinReadyBusy  Pin used to monitor the ready/busy signal of the Nand.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_Initialize(
    struct ManagedNandFlash *managed,
    const struct NandFlashModel *model,
    unsigned int commandAddress,
    unsigned int addressAddress,
    unsigned int dataAddress,
    const Pin pinChipEnable,
    const Pin pinReadyBusy)
{
    unsigned char error;
    unsigned char spare[NandCommon_MAXPAGESPARESIZE];
    unsigned int numBlocks;
    unsigned int pageSpareSize;
    const struct NandSpareScheme *scheme;
    unsigned int block;
    struct NandBlockStatus blockStatus;
    unsigned char badBlockMarker;
    unsigned int eraseCount, minEraseCount, maxEraseCount;

    TRACE_DEBUG("ManagedNandFlash_Initialize()\n\r");

    // Initialize EccNandFlash
    error = EccNandFlash_Initialize(ECC(managed),
                                    model,
                                    commandAddress,
                                    addressAddress,
                                    dataAddress,
                                    pinChipEnable,
                                    pinReadyBusy);
    if (error) {

        return error;
    }

    // Retrieve model information
    numBlocks = NandFlashModel_GetDeviceSizeInBlocks(MODEL(managed));
    pageSpareSize = NandFlashModel_GetPageSpareSize(MODEL(managed));
    scheme = NandFlashModel_GetScheme(MODEL(managed));

    // Initialize block statuses
    // First, check if device is virgin
    if (IsDeviceVirgin(managed)) {

        TRACE_FATAL("Device is virgin, doing initial block scanning ...\n\r");

        // Perform initial scan of the device
        for (block=0; block < numBlocks; block++) {

            // Check if block is bad
            error = CheckBlock(managed, block);
            if (error == BADBLOCK) {

                // Mark block as bad
                TRACE_DEBUG("Block #%d is bad\n\r", block);
                managed->blockStatuses[block].status = NandBlockStatus_BAD;
            }
            else if (error == GOODBLOCK) {

                // Mark block as free with erase count 0
                TRACE_DEBUG("Block #%d is free\n\r", block);
                managed->blockStatuses[block].status = NandBlockStatus_FREE;
                managed->blockStatuses[block].eraseCount = 0;

                // Write status in spare of block first page
                error = WriteBlockStatus(managed, block);
                if (error) {

                    TRACE_ERROR("ManagedNandFlash_Initialize: Failed to write spare area\n\r");
                    return error;
                }
            }
            else {

                TRACE_ERROR("ManagedNandFlash_Initialize: Cannot scan device\n\r");
                return error;
            }
        }
    }
    else {

        TRACE_DEBUG("Device is already managed, retrieving information ...\n\r");

        // Retrieve block statuses from their first page spare area
        // (find maximum and minimum wear at the same time)
        minEraseCount = 0xFFFFFFFF;
        maxEraseCount = 0;
        for (block=0; block < numBlocks; block++) {

            // Read spare of first page
            error = RawNandFlash_ReadPage(RAW(managed), block, 0, 0, spare);
            if (error) {

                TRACE_ERROR("ManagedNandFlash_Initialize: Cannot retrieve info from block #%d\n\r",
                          block);
            }

            // Retrieve bad block marker and block status
            NandSpareScheme_ReadBadBlockMarker(scheme, spare, &badBlockMarker);
            NandSpareScheme_ReadExtra(scheme, spare, &blockStatus, 4, 0);

            // If they do not match, block must be bad
            if ((badBlockMarker != 0xFF) && (blockStatus.status != NandBlockStatus_BAD)) {

                TRACE_DEBUG("Block #%d is bad\n\r", block);
                managed->blockStatuses[block].status = NandBlockStatus_BAD;
            }
            // Check that block status is not default (meaning block is not managed)
            else if (blockStatus.status == NandBlockStatus_DEFAULT) {

                ASSERT(0, "Block #%d is not managed\n\r", block);
            }
            // Otherwise block status is accurate
            else {

                TRACE_DEBUG("Block #%03d : status = %2d | eraseCount = %d\n\r",
                          block, blockStatus.status, blockStatus.eraseCount);
                managed->blockStatuses[block] = blockStatus;

                // Check for min/max erase counts
                if (blockStatus.eraseCount < minEraseCount) {

                    minEraseCount = blockStatus.eraseCount;
                }
                if (blockStatus.eraseCount > maxEraseCount) {

                    maxEraseCount = blockStatus.eraseCount;
                }

                //// Clean block
                //// Release LIVE blocks
                //if (managed->blockStatuses[block].status == NandBlockStatus_LIVE) {
                //
                //    ManagedNandFlash_ReleaseBlock(managed, block);
                //}
                //// Erase DIRTY blocks
                //if (managed->blockStatuses[block].status == NandBlockStatus_DIRTY) {
                //
                //    ManagedNandFlash_EraseBlock(managed, block);
                //}
            }
        }

        // Display erase count information
        TRACE_FATAL("|--------|------------|--------|--------|--------|\n\r");
        TRACE_FATAL("|  Wear  |   Count    |  Free  |  Live  | Dirty  |\n\r");
        TRACE_FATAL("|--------|------------|--------|--------|--------|\n\r");

        for (eraseCount=minEraseCount; eraseCount <= maxEraseCount; eraseCount++) {

            unsigned int count = 0, live = 0, dirty = 0, free = 0;
            for (block=0; block < numBlocks; block++) {

                if ((managed->blockStatuses[block].eraseCount == eraseCount)
                    && (managed->blockStatuses[block].status != NandBlockStatus_BAD)) {

                    count++;
                
                    switch (managed->blockStatuses[block].status) {
                        case NandBlockStatus_LIVE: live++; break;
                        case NandBlockStatus_DIRTY: dirty++; break;
                        case NandBlockStatus_FREE: free++; break;
                    }
                }
            }

            if (count > 0) {
            
                TRACE_FATAL("|  %4d  |  %8d  |  %4d  |  %4d  |  %4d  |\n\r",
                          eraseCount, count, free, live, dirty);
            }
        }
        TRACE_FATAL("|--------|------------|--------|--------|--------|\n\r");
    }

    return 0;
}

//------------------------------------------------------------------------------
/// Allocates a FREE block of a managed nandflash and marks it as LIVE.
/// Returns 0 if successful; otherwise returns NandCommon_ERROR_WRONGSTATUS if
/// the block is not FREE.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param block  Number of block to allocate.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_AllocateBlock(
    struct ManagedNandFlash *managed,
    unsigned short block)
{
    TRACE_INFO("ManagedNandFlash_AllocateBlock(%d)\n\r", block);

    // Check that block is FREE
    if (managed->blockStatuses[block].status != NandBlockStatus_FREE) {

        TRACE_ERROR("ManagedNandFlash_AllocateBlock: Block must be FREE\n\r");
        return NandCommon_ERROR_WRONGSTATUS;
    }

    // Change block status to LIVE
    managed->blockStatuses[block].status = NandBlockStatus_LIVE;
    return WriteBlockStatus(managed, block);
}

//------------------------------------------------------------------------------
/// Releases a LIVE block of a nandflash and marks it as DIRTY.
/// Returns 0 if successful; otherwise returns NandCommon_ERROR_WRONGSTATUS if
/// the block is not LIVE, or a RawNandFlash_WritePage error.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param block  Number of block to release.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_ReleaseBlock(
    struct ManagedNandFlash *managed,
    unsigned short block)
{
    unsigned char spare[NandCommon_MAXPAGESPARESIZE];

    TRACE_INFO("ManagedNandFlash_ReleaseBlock(%d)\n\r", block);

    // Check that block is LIVE
    if (managed->blockStatuses[block].status != NandBlockStatus_LIVE) {

        TRACE_ERROR("ManagedNandFlash_ReleaseBlock: Block must be LIVE\n\r");
        return NandCommon_ERROR_WRONGSTATUS;
    }

    // Change block status to DIRTY
    managed->blockStatuses[block].status = NandBlockStatus_DIRTY;
    return WriteBlockStatus(managed, block);
}

//------------------------------------------------------------------------------
/// Erases a DIRTY block of a managed NandFlash.
/// Returns the RawNandFlash_EraseBlock code or NandCommon_ERROR_WRONGSTATUS.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param block  Number of block to erase.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_EraseBlock(
    struct ManagedNandFlash *managed,
    unsigned short block)
{
    unsigned char error;
    unsigned char spare[NandCommon_MAXPAGESPARESIZE];

    TRACE_INFO("ManagedNandFlash_EraseBlock(%d)\n\r", block);

    // Check block status
    if (managed->blockStatuses[block].status != NandBlockStatus_DIRTY) {

        TRACE_ERROR("ManagedNandFlash_EraseBlock: Block must be DIRTY\n\r");
        return NandCommon_ERROR_WRONGSTATUS;
    }

    // Erase block
    error = RawNandFlash_EraseBlock(RAW(managed), block);
    if (error) {

        return error;
    }

    // Update block status
    managed->blockStatuses[block].status = NandBlockStatus_FREE;
    managed->blockStatuses[block].eraseCount++;
    return WriteBlockStatus(managed, block);
}

//------------------------------------------------------------------------------
/// Reads the data and/or the spare area of a page on a managed nandflash. If
/// the data pointer is not 0, then the block MUST be LIVE.
/// Returns NandCommon_ERROR_WRONGSTATUS if the block is not LIVE and the data
/// pointer is not null; Otherwise, returns EccNandFlash_ReadPage().
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param block  Number of block to read page from.
/// \param page  Number of page to read inside the given block.
/// \param data  Data area buffer, can be 0.
/// \param spare  Spare area buffer, can be 0.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_ReadPage(
    const struct ManagedNandFlash *managed,
    unsigned short block,
    unsigned short page,
    void *data,
    void *spare)
{
    // Check that the block is LIVE if data is requested
    if ((managed->blockStatuses[block].status != NandBlockStatus_LIVE)
        && (managed->blockStatuses[block].status != NandBlockStatus_DIRTY)) {

        TRACE_ERROR("ManagedNandFlash_ReadPage: Block must be LIVE or DIRTY.\n\r");
        return NandCommon_ERROR_WRONGSTATUS;
    }

    // Read data with ECC verification
    return EccNandFlash_ReadPage(ECC(managed), block, page, data, spare);
}

//------------------------------------------------------------------------------
/// Writes the data and/or spare area of a LIVE page on a managed NandFlash.
/// Returns NandCommon_ERROR_WRONGSTATUS if the page is not LIVE; otherwise,
/// returns EccNandFlash_WritePage().
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param block  Number of the block to write.
/// \param page  Number of the page to write inside the given block.
/// \param data  Data area buffer.
/// \param spare  Spare area buffer.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_WritePage(
    const struct ManagedNandFlash *managed,
    unsigned short block,
    unsigned short page,
    void *data,
    void *spare)
{
    // Check that the block is LIVE
    if (managed->blockStatuses[block].status != NandBlockStatus_LIVE) {

        TRACE_ERROR("ManagedNandFlash_WritePage: Block must be LIVE.\n\r");
        return NandCommon_ERROR_WRONGSTATUS;
    }

    // Write data with ECC calculation
    return EccNandFlash_WritePage(ECC(managed), block, page, data, spare);
}

//------------------------------------------------------------------------------
/// Copy the data & spare area of one page to another page. The source block
/// can be either LIVE or DIRTY, and the destination block must be LIVE; they
/// must both have the same parity.
/// Returns 0 if successful; NandCommon_ERROR_WRONGSTATUS if one or more page
/// is not live; otherwise returns an NandCommon_ERROR_xxx code.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param sourceBlock  Source block number.
/// \param sourcePage  Number of source page inside the source block.
/// \param destBlock  Destination block number.
/// \param destPage  Number of destination page inside the dest block.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_CopyPage(
    const struct ManagedNandFlash *managed,
    unsigned short sourceBlock,
    unsigned short sourcePage,
    unsigned short destBlock,
    unsigned short destPage)
{
    unsigned char error;

    ASSERT((sourcePage & 1) == (destPage & 1),
           "ManagedNandFlash_CopyPage: source & dest pages must have the same parity\n\r");

    TRACE_INFO("ManagedNandFlash_CopyPage(B#%d:P#%d -> B#%d:P#%d)\n\r",
              sourceBlock, sourcePage, destBlock, destPage);

    // Check block statuses
    if ((managed->blockStatuses[sourceBlock].status != NandBlockStatus_LIVE)
         && (managed->blockStatuses[sourceBlock].status != NandBlockStatus_DIRTY)) {

        TRACE_ERROR("ManagedNandFlash_CopyPage: Source block must be LIVE or DIRTY.\n\r");
        return NandCommon_ERROR_WRONGSTATUS;
    }
    if (managed->blockStatuses[destBlock].status != NandBlockStatus_LIVE) {

        TRACE_ERROR("ManagedNandFlash_CopyPage: Destination block must be LIVE.\n\r");
        return NandCommon_ERROR_WRONGSTATUS;
    }

    // If destination page is page #0, block status information must not be
    // overwritten
    if (destPage == 0) {

        unsigned char data[NandCommon_MAXPAGEDATASIZE];
        unsigned char spare[NandCommon_MAXPAGESPARESIZE];

        // Read data & spare to copy
        error = EccNandFlash_ReadPage(ECC(managed), sourceBlock, sourcePage, data, spare);
        if (error) {

            return error;
        }

        // Write destination block status information in spare
        NandSpareScheme_WriteExtra(NandFlashModel_GetScheme(MODEL(managed)),
                                   spare,
                                   &(managed->blockStatuses[destBlock]),
                                   4,
                                   0);

        // Write page
        error = RawNandFlash_WritePage(RAW(managed), destBlock, destPage, data, spare);
        if (error) {

            return error;
        }
    }
    // Otherwise, a normal copy can be done
    else {

        return RawNandFlash_CopyPage(RAW(managed),
                                     sourceBlock,
                                     sourcePage,
                                     destBlock,
                                     destPage);
    }

    return 0;
}

//------------------------------------------------------------------------------
/// Copies the data from a whole block to another block on a nandflash. Both
/// blocks must be LIVE.
/// Returns 0 if successful; otherwise returns NandCommon_ERROR_WRONGSTATUS if
/// at least one of the blocks is not free, or a NandCommon_ERROR_xxx code.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param sourceBlock  Source block number.
/// \param destBlock  Destination block number.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_CopyBlock(
    const struct ManagedNandFlash *managed,
    unsigned short sourceBlock,
    unsigned short destBlock)
{
    unsigned short numPages = NandFlashModel_GetBlockSizeInPages(MODEL(managed));
    unsigned char error;
    unsigned short page;

    ASSERT(sourceBlock != destBlock,
           "ManagedNandFlash_CopyBlock: Source block must be different from dest. block\n\r");

    TRACE_INFO("ManagedNandFlash_CopyBlock(B#%d->B#%d)\n\r",
              sourceBlock, destBlock);

    // Copy all pages
    for (page=0; page < numPages; page++) {

        error = ManagedNandFlash_CopyPage(managed,
                                          sourceBlock,
                                          page,
                                          destBlock,
                                          page);
        if (error) {

            TRACE_ERROR("ManagedNandFlash_CopyPage: Failed to copy page %d\n\r", page);
            return error;
        }
    }

    return 0;
}

//------------------------------------------------------------------------------
/// Erases all the blocks which are currently marked as DIRTY.
/// Returns 0 if successful; otherwise, returns a NandCommon_ERROR code.
/// \param managed  Pointer to a ManagedNandFlash instance.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_EraseDirtyBlocks(
    struct ManagedNandFlash *managed)
{
    unsigned int i;
    unsigned char error;

    // Erase all dirty blocks
    for (i=0; i < NandFlashModel_GetDeviceSizeInBlocks(MODEL(managed)); i++) {

        if (managed->blockStatuses[i].status == NandBlockStatus_DIRTY) {

            error = ManagedNandFlash_EraseBlock(managed, i);
            if (error) {

                return error;
            }
        }
    }

    return 0;
}

//------------------------------------------------------------------------------
/// Looks for the youngest block having the desired status among the blocks
/// of a managed nandflash. If a block is found, its index is stored inside
/// the provided variable (if pointer is not 0).
/// Returns 0 if a block has been found; otherwise returns either
/// NandCommon_ERROR_NOBLOCKFOUND if there are no blocks having the desired
/// status.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param block  Pointer to the block number variable.
//------------------------------------------------------------------------------
unsigned char ManagedNandFlash_FindYoungestBlock(
    const struct ManagedNandFlash *managed,
    unsigned char status,
    unsigned short *block)
{
    unsigned char found = 0;
    unsigned short bestBlock = 0;
    unsigned int i;

    // Go through the block array
    for (i=0; i < NandFlashModel_GetDeviceSizeInBlocks(MODEL(managed)); i++) {

        // Check status
        if (managed->blockStatuses[i].status == status) {

            // If no block was found, i becomes the best block
            if (!found) {

                found = 1;
                bestBlock = i;
            }
            // Compare the erase counts otherwise
            else if (managed->blockStatuses[i].eraseCount
                     < managed->blockStatuses[bestBlock].eraseCount) {

                bestBlock = i;
            }
        }
    }

    if (found) {

        if (block) {

            *block = bestBlock;
        }
        return 0;
    }
    else {

        return NandCommon_ERROR_NOBLOCKFOUND;
    }
}

//------------------------------------------------------------------------------
/// Counts and returns the number of blocks having the given status in a
/// managed nandflash.
/// \param managed  Pointer to a ManagedNandFlash instance.
/// \param status  Desired block status.
//------------------------------------------------------------------------------
unsigned short ManagedNandFlash_CountBlocks(
    const struct ManagedNandFlash *managed,
    unsigned char status)
{
    unsigned int i;
    unsigned short count = 0;

    // Examine each block
    for (i=0; i < NandFlashModel_GetDeviceSizeInBlocks(MODEL(managed)); i++) {

        if (managed->blockStatuses[i].status == status) {

            count++;
        }
    }

    return count;
}

