/*****************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
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
