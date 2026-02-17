/****************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file dc_crc64.c
 *
 * @defgroup Dc_DataCompression DC Data Compression
 *
 * @ingroup Dc_DataCompression
 *
 * @description
 *      Implementation of the CRC-64 operations.
 *
 *****************************************************************************/

#include "dc_crc64.h"

/**
 * @description
 *     Calculates CRC-64 checksum for given Buffer List
 *
 *     Function loop through all of the flat buffers in the buffer list.
 *     CRC is calculated for each flat buffer, but output CRC from
 *     buffer[0] is used as input seed for buffer[1] CRC calculation
 *     (and so on until looped through all flat buffers).
 *     Resulting CRC is final CRC for all buffers in the buffer list struct
 *
 * @param[in]  bufferList      Pointer to data byte array to calculate CRC on
 * @param[in]  consumedBytes   Total number of bytes to calculate CRC on
 *                             (for all buffer in buffer list)
 * @param[in]  seedChecksums   Input checksum from where the calculation will
 *                             start from.
 *
 * @retval Cpa64U              64bit long CRC checksum for given buffer list
 */
Cpa64U dcCalculateCrc64(const CpaBufferList *pBufferList,
                        Cpa32U consumedBytes,
                        Cpa64U seedChecksum)
{
    Cpa32U i = 0;
    Cpa64U computeLength = 0;
    Cpa32U flatBufferLength = 0;
    Cpa64U currentCrc = seedChecksum;
    CpaFlatBuffer *pBuffer = &pBufferList->pBuffers[0];

    /* The data used as loop boundry might be tainted. Here is check
     * if numBuffers is reasonable. There is no way to return error
     * if it is but returning seedChecksum will give hint that something
     * is wrong. */
    const Cpa32U num =
        pBufferList->numBuffers < MAX_SGL_NUM ? pBufferList->numBuffers : 0;
    for (i = 0; i < num; i++)
    {
        flatBufferLength = pBuffer->dataLenInBytes;

        /* Get number of bytes based on remaining data (consumedBytes) and
         * max buffer length, then calculate CRC on them */
        if (consumedBytes > flatBufferLength)
        {
            computeLength = flatBufferLength;
            consumedBytes -= flatBufferLength;
        }
        else
        {
            computeLength = consumedBytes;
            consumedBytes = 0;
        }
        currentCrc =
            crc64_ecma_norm_by8(currentCrc, pBuffer->pData, computeLength);
        pBuffer++;
    }

    return currentCrc;
}
