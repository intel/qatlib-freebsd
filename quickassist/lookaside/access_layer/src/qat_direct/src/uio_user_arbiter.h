/*****************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2023 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2023 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *
 *****************************************************************************/

/*****************************************************************************
 * @file adf_user_uio_ring.h
 *
 * @description
 *      This file contains the arbiter related interfaces
 *
 *****************************************************************************/

#ifndef ADF_UIO_USER_ARBITER_H
#define ADF_UIO_USER_ARBITER_H

#include <adf_platform_common.h>
#include <icp_platform.h>

/* Macros for reading/writing to the arbitration registers */
/* Arbiter offset within the ETR bar */
#define ICP_ARB_OFFSET_START 0x30000
/* Arbiter BAR */
#define ICP_ARB_BAR csr_base_addr + ICP_ARB_OFFSET_START
/* Arbiter slot size */
#define ICP_ARB_REG_SLOT 0x1000
/* Size of a regular register */
#define ICP_ARB_REG_SIZE 0x4

/* Offset starts on per register basis */
#define ICP_ARB_SARB_CFG_OFFSET_START 0x000
#define ICP_ARB_WTR_OFFSET_START 0x010
#define ICP_ARB_RO_EN_OFFSET_START 0x090
#define ICP_ARB_WQCFG_OFFSET_START 0x100
#define ICP_ARB_WQSTAT_OFFSET_START 0x140
#define ICP_ARB_WRKTHD2SARBMAP_OFFSET_START 0x180
#define ICP_ARB_RINGSRVARBEN_OFFSET_START 0x19C

#define READ_CSR_ARB_RINGSRVARBEN(csr_base_addr, index)                        \
    ICP_ADF_CSR_RD(csr_base_addr,                                              \
                   ICP_ARB_RINGSRVARBEN_OFFSET_START +                         \
                       ICP_ARB_REG_SLOT * index)

#define WRITE_CSR_ARB_RINGSRVARBEN(csr_base_addr, index, value)                \
    ICP_ADF_CSR_WR(csr_base_addr,                                              \
                   ICP_ARB_RINGSRVARBEN_OFFSET_START +                         \
                       ICP_ARB_REG_SLOT * index,                               \
                   value)

#define READ_CSR_ARB_WRKTHD2SRVARBMAP(csr_base_addr, index)                    \
    ICP_ADF_CSR_RD(ICP_ARB_BAR,                                                \
                   ICP_ARB_WRKTHD2SARBMAP_OFFSET_START +                       \
                       ICP_ARB_REG_SIZE * index);

#define WRITE_CSR_ARB_WRKTHD2SRVARBMAP(csr_base_addr, index, value)            \
    ICP_ADF_CSR_WR(ICP_ARB_BAR,                                                \
                   ICP_ARB_WRKTHD2SARBMAP_OFFSET_START +                       \
                       ICP_ARB_REG_SIZE * index,                               \
                   value);

static __inline__ void adf_update_ring_arb_enable(adf_dev_ring_handle_t *ring)
{
    unsigned int arbenable;
#ifndef ICP_WITHOUT_THREAD
    int32_t status;
    pthread_mutex_t *mutex =
        *(pthread_mutex_t **)(ring->bank_data->user_bank_lock);

    /*Lock the register to enable/disable arbiter*/
    status = pthread_mutex_lock(mutex);
    if (status)
    {
        ADF_ERROR("Failed to lock bank with error %d\n", status);
        return;
    }
#endif
    arbenable = READ_CSR_ARB_RINGSRVARBEN(ring->csr_addr, 0);
    arbenable |= ring->bank_data->ring_mask & 0xFF;
    WRITE_CSR_ARB_RINGSRVARBEN(ring->csr_addr, 0, arbenable);
    arbenable = READ_CSR_ARB_RINGSRVARBEN(ring->csr_addr, 0);
#ifndef ICP_WITHOUT_THREAD
    pthread_mutex_unlock(mutex);
#endif
}

static __inline__ void adf_update_ring_arb_disable(adf_dev_ring_handle_t *ring)
{
    unsigned int arbenable;
#ifndef ICP_WITHOUT_THREAD
    int32_t status;
    pthread_mutex_t *mutex =
        *(pthread_mutex_t **)(ring->bank_data->user_bank_lock);

    /*Lock the register to enable/disable arbiter*/
    status = pthread_mutex_lock(mutex);
    if (status)
    {
        ADF_ERROR("Failed to lock bank with error %d\n", status);
        return;
    }
#endif
    arbenable = READ_CSR_ARB_RINGSRVARBEN(ring->csr_addr, 0);
    arbenable &= ~ring->bank_data->ring_mask & 0xFF;
    WRITE_CSR_ARB_RINGSRVARBEN(ring->csr_addr, 0, arbenable);
    arbenable = READ_CSR_ARB_RINGSRVARBEN(ring->csr_addr, 0);
#ifndef ICP_WITHOUT_THREAD
    pthread_mutex_unlock(mutex);
#endif
}

#endif /*ADF_UIO_USER_ARBITER_H*/
