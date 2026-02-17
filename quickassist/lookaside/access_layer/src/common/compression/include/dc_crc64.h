/****************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/

#ifndef DC_CRC64_H_
#define DC_CRC64_H_

#include <cpa_types.h>
#include <cpa_dc.h>
#include "lac_common.h"
#include "dc_session.h"

/**
 * @description
 *     Calculates CRC-64 ECMA-182 checksum for given buffer
 *
 * @param[in]  initial_crc    Initial CRC-64 value (used for multi-segment calc)
 * @param[in]  buffer         Pointer to data byte array to calculate CRC on
 * @param[in]  buffer_length  Length of data array
 *
 * @retval Cpa64U             64bit long CRC checksum for given buffer
 */
extern Cpa64U crc64_ecma_norm_by8(Cpa64U initial_crc,
                                  const Cpa8U *buffer,
                                  Cpa64U buffer_length);

/**
 * @description
 *     Helper function to calculate CRC64 checksum on a buffer list.
 *
 *     For a given SGL, the CRC64 checksum is calculated based on the
 *     size of the buffer list.
 *
 * @param[out] checksum       New CRC64 checksum value
 * @param[in]  pBufferList    virtual address of src SGL to calculate CRC on.
 * @param[in]  consumedBytes  total number of bytes inside of pUserSrcBuff
 *                            to calculate CRC checksum for.
 * @param[in]  seedChecksum   Input checksum from where the calculation
 *                            will start from.
 * @note
 *     currently only CRC-64 ECMA-182 (0x42f0E1EBA9EA3693) algorithm
 *     is supported for calculating CRCs on input and output buffers
 *     and CRC value is expected to be 64bit long.
 */
Cpa64U dcCalculateCrc64(const CpaBufferList *pBufferList,
                        Cpa32U consumedBytes,
                        Cpa64U seedChecksum);

#endif /* end of DC_CRC64_H_ */
