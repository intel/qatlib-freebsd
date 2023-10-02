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
 * @file adf_user_transport_ctrl.c
 *
 * @description
 *      Transport Controller for user space
 *
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#ifndef ICP_WITHOUT_THREAD
#include <pthread.h>
#endif
#include <time.h>
#include <sys/time.h>
#include <sched.h>

#include "cpa.h"
#include "icp_platform.h"
#include "icp_accel_devices.h"
#include "icp_adf_accel_mgr.h"
#include "adf_kernel_types.h"
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_platform.h"
#include "adf_dev_ring_ctl.h"
#include "adf_user_transport.h"
#include "icp_adf_init.h"
#include "adf_devmgr.h"
#include "uio_user_bundles.h"
#include "uio_user_ring.h"
#include "uio_user_arbiter.h"

#include "uio_user_cfg.h"
#include "uio_user_bundles.h"

/*
 * Time to wait for a thread to exit in ms
 */
#define TIME_TO_WAIT 50
/*
 * Value indicating not to bind a thread to a core.
 */
#define DO_NOT_BIND 0xFFFF

#define BREAK_ON_NULL(val, msg, stat)                                          \
    if (NULL == val)                                                           \
    {                                                                          \
        ADF_ERROR(msg);                                                        \
        stat = CPA_STATUS_FAIL;                                                \
        break;                                                                 \
    }

STATIC Cpa32U *ringInflights[ADF_MAX_DEVICES] = { NULL };

extern void *adf_get_bank_base_addr(int accelId,
                                    int bankid,
                                    uint32_t *offset,
                                    uint32_t *size);
adf_dev_bank_handle_t *get_banks(icp_accel_dev_t *dev)
{
    return dev->banks;
}

int init_bank_from_accel(icp_accel_dev_t *accel_dev,
                         adf_dev_bank_handle_t *bank)
{
    uint32_t size = 0;
    struct adf_uio_user_bundle *bundle = NULL;
    bundle = uio_get_bundle_from_accelid(accel_dev->accelId, bank->bank_number);
    if (NULL == bundle)
    {
        ADF_ERROR("%s return -1\n", __func__);
        return -1;
    }
    bank->csr_addr = (uint32_t *)bundle->ptr;
    bank->bundle = bundle;

    /*  allocate ring handles for this bank  */
    size = sizeof(adf_dev_ring_handle_t *) * (accel_dev->maxNumRingsPerBank);
    bank->rings = ICP_MALLOC_GEN(size);

    if (NULL == bank->rings)
    {
        uio_free_bundle(bundle);
        ADF_ERROR("%s return -1\n", __func__);
        return -1;
    }

    ICP_MEMSET(bank->rings, 0, size);

    bank->max_num_rings = accel_dev->maxNumRingsPerBank;

    return 0;
}

#ifdef CONFIG_USE_UIO_DYNAMIC
static void adf_proxy_set_bank_default_info(icp_accel_dev_t *dev)
{
    adf_dev_bank_handle_t *banks = dev->banks;
    int32_t i;

    for (i = 0; i < dev->maxNumBanks; i++, banks++)
    {
        banks->bank_number = i;
        banks->bank_offset = 0;
        banks->timed_coalesc_enabled = 0;
        banks->number_msg_coalesc_enabled = 0;
        banks->flag_interrupts_enabled = 0;
        banks->tx_rings_mask = 0xff;
        banks->ring_mask = 0;
    }
}
#endif

STATIC CpaStatus adf_proxy_populate_bank_ring_info(icp_accel_dev_t *accel_dev)
{
    adf_dev_bank_handle_t *bankHandler;
    Cpa32U *inflight;
    Cpa32U numOfBanksPerDevice = 0;
    Cpa32U size = 0;
    Cpa32U device_id = 0;

    device_id = accel_dev->accelId;
    numOfBanksPerDevice = accel_dev->maxNumBanks;

    /* allocate bank handler array */
    size = sizeof(adf_dev_bank_handle_t) * numOfBanksPerDevice;
    bankHandler = ICP_MALLOC_GEN(size);
    if (NULL == bankHandler)
    {
        ADF_ERROR("Failed to allocate memory - bankHandler\n");
        return CPA_STATUS_FAIL;
    }
    ICP_MEMSET(bankHandler, 0, size);
    accel_dev->banks = bankHandler;
    adf_proxy_set_bank_default_info(accel_dev);

    /* allocate ring in-flight array ring put/get optimization */
    size = sizeof(*inflight) * (accel_dev->maxNumRingsPerBank >> 1) *
           numOfBanksPerDevice;
    inflight = ICP_MALLOC_GEN(size);
    if (NULL == inflight)
    {
        ADF_ERROR("Failed to allocate memory - ringInflights\n");
        ICP_FREE(accel_dev->banks);
        return CPA_STATUS_FAIL;
    }

    ringInflights[device_id] = inflight;

    return CPA_STATUS_SUCCESS;
}

STATIC CpaStatus adf_proxy_populate_device_info(icp_accel_dev_t *accel_dev)
{
    if (adf_proxy_populate_bank_ring_info(accel_dev) != CPA_STATUS_SUCCESS)
        return CPA_STATUS_FAIL;

    return CPA_STATUS_SUCCESS;
}

void adf_proxy_depopulate_bank_ring_info(icp_accel_dev_t *accel_dev)
{
    Cpa32U device_id = 0;

    device_id = accel_dev->accelId;
    ICP_FREE(accel_dev->banks);
    ICP_FREE(ringInflights[device_id]);

    return;
}

void adf_proxy_depopulate_device_info(icp_accel_dev_t *accel_dev)
{
    adf_proxy_depopulate_bank_ring_info(accel_dev);
    return;
}

STATIC inline int adf_dev_bank_handle_get(adf_dev_bank_handle_t *bank)
{
    return __sync_fetch_and_add(&bank->refs, 1);
}

STATIC inline int adf_dev_bank_handle_put(adf_dev_bank_handle_t *bank)
{
    return __sync_fetch_and_sub(&bank->refs, 1);
}

STATIC inline int adf_dev_bank_handle_check(adf_dev_bank_handle_t *bank)
{
    return __sync_val_compare_and_swap(&bank->refs, 0, 0);
}

/*
 * Check and free the ring and bundle
 * This function is used to free the bundle and ring in the bank
 * handle when the reference count is set as ZERO.
 */
STATIC void adf_free_bundle(adf_dev_bank_handle_t *bank)
{
    if (0 == adf_dev_bank_handle_check(bank))
    {
        ICP_FREE(bank->rings);
        uio_free_bundle(bank->bundle);
        bank->bundle = NULL;
        bank->rings = NULL;
    }
    return;
}

/*
 * Create a transport handle
 * The function sends ioctl request to adf user proxy to create
 * a ring and then mmaps it to userspace memory. If it is a response
 * ring and there is no reading thread running for the device
 * the function creates one.
 */
CpaStatus icp_adf_transCreateHandle(icp_accel_dev_t *accel_dev,
                                    icp_transport_type trans_type,
                                    const char *section,
                                    const Cpa32U accel_nr,
                                    const Cpa32U bank_nr,
                                    const char *service_name,
                                    const icp_adf_ringInfoService_t info,
                                    icp_trans_callback callback,
                                    icp_resp_deliv_method resp,
                                    const Cpa32U num_msgs,
                                    const Cpa32U msg_size,
                                    icp_comms_trans_handle *trans_handle)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    adf_dev_ring_handle_t *pRingHandle = NULL;
    Cpa32U ring_number = 0;
    Cpa32U in_flight_index = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(trans_handle);

    adf_dev_bank_handle_t *banks = accel_dev->banks;
#ifdef CONFIG_USE_UIO_DYNAMIC
    adf_dev_bank_handle_t *bank = &banks[bank_nr];
    int ring_rnum, nodeid = accel_dev->pkg_id;
    char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES];

    /* here init the bank: get a free bundle from UIO and mmap it
     * It's not suitable to put it into device init stage,
     * for at that time, we don't know which device the user will be use */
    if (NULL == bank->bundle)
    {
        if (CPA_STATUS_SUCCESS != ICP_MUTEX_LOCK(bank->user_bank_lock))
        {
            ADF_ERROR("%s Failed to lock user bank\n", __func__);
            return CPA_STATUS_RESOURCE;
        }
        if (0 > init_bank_from_accel(accel_dev, bank))
        {
            if (CPA_STATUS_SUCCESS != ICP_MUTEX_UNLOCK(bank->user_bank_lock))
            {
                ADF_ERROR("%s Failed to unlock user bank\n", __func__);
            }
            ADF_ERROR("%s return -1\n", __func__);
            return CPA_STATUS_FAIL;
        }

        if (CPA_STATUS_SUCCESS != ICP_MUTEX_UNLOCK(bank->user_bank_lock))
        {
            ADF_ERROR("%s Failed to unlock user bank\n", __func__);
        }

        /* turn the interrupt off */
        ICP_ADF_CSR_WR(
            bank->csr_addr, ICP_DH895xCC_RING_CSR_FLAG_AND_COL_EN, 0);
    }
    if (CPA_STATUS_SUCCESS ==
        icp_adf_cfgGetParamValue(accel_dev, section, service_name, val))
    {
        ring_rnum = strtoul(val, NULL, 10);
        if ((ring_rnum < 0) || (ring_rnum >= accel_dev->maxNumRingsPerBank))
        {
            ADF_ERROR("Invalid ring num\n");
            adf_free_bundle(bank);
            return CPA_STATUS_FAIL;
        }
    }
    else
    {
        adf_free_bundle(bank);
        return CPA_STATUS_FAIL;
    }

    /* Reserve the ring in the kernel driver */
    if (CPA_STATUS_SUCCESS !=
        icp_adf_reserve_ring(accel_dev->accelId, bank_nr, ring_rnum))
    {
        adf_free_bundle(bank);
        return CPA_STATUS_FAIL;
    }
#endif

    /* allocate and setup ring handle structure */
    pRingHandle = ICP_MALLOC_GEN(sizeof(adf_dev_ring_handle_t));
    if (NULL == pRingHandle)
    {
        ADF_ERROR("unable to allocate pRingHandle buffer\n");
        if (CPA_STATUS_SUCCESS !=
            icp_adf_release_ring(accel_dev->accelId, bank_nr, ring_rnum))
        {
            ADF_ERROR("Failed to release the ring \n");
        }
        adf_free_bundle(bank);
        return CPA_STATUS_FAIL;
    }

    ICP_MEMSET(pRingHandle, 0, sizeof(adf_dev_ring_handle_t));
    pRingHandle->accel_dev = accel_dev;
    pRingHandle->trans_type = trans_type;
    pRingHandle->service_name =
        ICP_MALLOC_GEN(ICP_STRNLEN(service_name, ICP_MAX_STR_LEN) + 1);
    if (NULL == pRingHandle->service_name)
    {
        ADF_ERROR("unable to allocate service buffer\n");
        ICP_FREE(pRingHandle);
        if (CPA_STATUS_SUCCESS !=
            icp_adf_release_ring(accel_dev->accelId, bank_nr, ring_rnum))
        {
            ADF_ERROR("Failed to release the ring \n");
        }
        adf_free_bundle(bank);
        return CPA_STATUS_FAIL;
    }

    memcpy(pRingHandle->service_name,
           service_name,
           ICP_STRNLEN(service_name, ICP_MAX_STR_LEN) + 1);
    pRingHandle->service_name_len =
        ICP_STRNLEN(service_name, ICP_MAX_STR_LEN) + 1;
    pRingHandle->callback = callback;
    pRingHandle->accel_num = accel_nr;
    pRingHandle->bank_num = bank_nr;
    pRingHandle->section_name =
        ICP_MALLOC_GEN(ICP_STRNLEN(section, ICP_MAX_STR_LEN) + 1);
    if (NULL == pRingHandle->section_name)
    {
        ADF_ERROR("unable to allocate section name buffer\n");
        ICP_FREE(pRingHandle->service_name);
        ICP_FREE(pRingHandle);
        if (CPA_STATUS_SUCCESS !=
            icp_adf_release_ring(accel_dev->accelId, bank_nr, ring_rnum))
        {
            ADF_ERROR("Failed to release the ring \n");
        }
        adf_free_bundle(bank);
        return CPA_STATUS_FAIL;
    }

    memcpy(pRingHandle->section_name,
           section,
           ICP_STRNLEN(section, ICP_MAX_STR_LEN) + 1);
    pRingHandle->section_name_len = ICP_STRNLEN(section, ICP_MAX_STR_LEN) + 1;
    pRingHandle->resp = resp;
    pRingHandle->info = info;
    /* Zero the masks to get rid of garbage. */
#ifndef CONFIG_USE_UIO_DYNAMIC
    pRingHandle->interrupt_user_mask = 0;
    pRingHandle->interrupt_kernel_mask = 0;
    pRingHandle->pollingMask = 0;
#else
    if (ICP_RESP_TYPE_POLL == resp)
    {
        /* Set the polling mask for this ring handle. */
        pRingHandle->pollingMask = 1 << ring_rnum;
    }
    else if (ICP_RESP_TYPE_NONE != resp)
    {
        ADF_ERROR("Not implemented yet\n");
        ICP_FREE(pRingHandle->service_name);
        pRingHandle->service_name = NULL;
        ICP_FREE(pRingHandle->section_name);
        pRingHandle->section_name = NULL;
        ICP_FREE(pRingHandle);
        if (CPA_STATUS_SUCCESS !=
            icp_adf_release_ring(accel_dev->accelId, bank_nr, ring_rnum))
        {
            ADF_ERROR("Failed to release the ring \n");
        }
        adf_free_bundle(bank);
        return CPA_STATUS_FAIL;
    }

#endif
    /* To save size we are going to reuse the ring space var
     * to pass down the ring num_msgs. space is in bytes
     * and is only used later in the msg put and get logic.
     * num_msgs is used only to allocate appropriate
     * memory for the ring and is not subsequently stored
     */
    pRingHandle->space = num_msgs;
    pRingHandle->message_size = msg_size;
#ifndef CONFIG_USE_UIO_DYNAMIC
    /* send the create handle request to kernel space  */
    ret = ICP_IOCTL(
        accel_dev->devCtlFileHdl, ADF_DEV_RING_IOC_CREATEHANDLE, pRingHandle);

    if (ret != 0)
    {
        ADF_ERROR("ioctl failed, ret=%d\n", ret);
        ICP_FREE(pRingHandle->service_name);
        pRingHandle->service_name = NULL;
        ICP_FREE(pRingHandle->section_name);
        pRingHandle->section_name = NULL;
        ICP_FREE(pRingHandle);
        icp_adf_release_ring(accel_dev->accelId, bank_nr, ring_rnum);
        return CPA_STATUS_FAIL;
    }
    pRingHandle->accel_dev = accel_dev;
    *trans_handle = (icp_comms_trans_handle *)pRingHandle;
    status = icp_adf_transGetRingNum(*trans_handle, &ring_number);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("icp_adf_transGetRingNum failed\n");
        icp_adf_transReleaseHandle(*trans_handle);
        *trans_handle = NULL;
        return CPA_STATUS_FAIL;
    }

    /* mmap the ring memory into userspace so it can be accessed directly
     * in userspace as we will write and read to/from it  */
    pRingHandle->ring_virt_addr =
        ICP_MMAP(0,
                 pRingHandle->ring_size,
                 PROT_READ | PROT_WRITE,
                 MAP_FILE | MAP_SHARED | MAP_LOCKED,
                 accel_dev->devCtlFileHdl ring_number * getpagesize());
    if (MAP_FAILED == pRingHandle->ring_virt_addr)
    {
        ADF_ERROR("mmap failed\n");
        icp_adf_transReleaseHandle(*trans_handle);
        *trans_handle = NULL;
        return CPA_STATUS_FAIL;
    }
#else
    status = adf_init_ring(pRingHandle,
                           bank,
                           ring_rnum,
                           bank->csr_addr,
                           num_msgs,
                           msg_size,
                           nodeid);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("adf_init_ring failed\n");
        ICP_FREE(pRingHandle->section_name);
        ICP_FREE(pRingHandle->service_name);
        ICP_FREE(pRingHandle);
        icp_adf_release_ring(accel_dev->accelId, bank_nr, ring_rnum);
        /* Do not clean bank pointers as they are already used by other ring */
        if (-EBUSY != status)
        {
            adf_free_bundle(bank);
        }
        return CPA_STATUS_FAIL;
    }
    adf_dev_bank_handle_get(bank);
    pRingHandle->accel_dev = accel_dev;
    *trans_handle = (icp_comms_trans_handle *)pRingHandle;
    status = icp_adf_transGetRingNum(*trans_handle, &ring_number);
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("icp_adf_transGetRingNum failed\n");
        icp_adf_transReleaseHandle(*trans_handle);
        *trans_handle = NULL;
        return status;
    }
#endif

    if ((NULL != callback) && (ICP_RESP_TYPE_IRQ == resp))
    {
        ADF_ERROR("Can't create IRQ driven rings in IRQless mode\n");
        icp_adf_transReleaseHandle(*trans_handle);
        ICP_FREE(bank->rings);
        uio_free_bundle(bank->bundle);
        bank->bundle = NULL;
        return CPA_STATUS_FAIL;
    }
    /* callback has been overwritten in kernelspace
     * so have to set it to the userspace callback again */
    pRingHandle->callback = callback;
    (bank->rings)[ring_rnum] = pRingHandle;
    banks[pRingHandle->bank_num].interrupt_mask |=
        pRingHandle->interrupt_user_mask;
#ifndef CONFIG_USE_UIO_DYNAMIC
    pRingHandle->bank_data = &banks[pRingHandle->bank_num];

    banks[pRingHandle->bank_num].bank_number = pRingHandle->bank_num;
    banks[pRingHandle->bank_num].bank_offset = pRingHandle->bank_offset;
    banks[pRingHandle->bank_num].timed_coalesc_enabled =
        pRingHandle->timed_coalesc_enabled;
    banks[pRingHandle->bank_num].number_msg_coalesc_enabled =
        pRingHandle->number_msg_coalesc_enabled;
    banks[pRingHandle->bank_num].flag_interrupts_enabled =
        pRingHandle->flag_interrupts_enabled;
#endif
    banks[pRingHandle->bank_num].pollingMask |= pRingHandle->pollingMask;

    /* request and response ring will share the same index */
    if (pRingHandle->ring_num < accel_dev->maxNumRingsPerBank / 2)
    {
        in_flight_index =
            ((accel_dev->maxNumRingsPerBank * pRingHandle->bank_num) >> 1) +
            pRingHandle->ring_num;
    }
    else
    {
        in_flight_index =
            ((accel_dev->maxNumRingsPerBank * pRingHandle->bank_num) >> 1) +
            (pRingHandle->ring_num - accel_dev->maxNumRingsPerBank / 2);
    }
    /* Initialise the pRingHandle in-flight */
    /* Take it as in-flight as only thread is used for initalization*/
    pRingHandle->in_flight =
        (Cpa32U *)(ringInflights[accel_dev->accelId] + in_flight_index);
    *pRingHandle->in_flight = 0;

    /* Initialise the pRingHandle atomic flag. */
    osalAtomicSet(1, (OsalAtomic *)&(pRingHandle->pollingInProgress));
    pRingHandle->user_lock = ICP_MALLOC_GEN(sizeof(ICP_MUTEX));
    if (!pRingHandle->user_lock)
    {
        ADF_ERROR("Could not alloc memory for ring lock\n");
        icp_adf_transReleaseHandle(*trans_handle);
        *trans_handle = NULL;
        return CPA_STATUS_FAIL;
    }
    if (CPA_STATUS_SUCCESS != ICP_MUTEX_INIT(pRingHandle->user_lock))
    {
        ADF_ERROR("Could not initialize user lock mutex\n");
        icp_adf_transReleaseHandle(*trans_handle);
        *trans_handle = NULL;
        ICP_FREE(bank->rings);
        uio_free_bundle(bank->bundle);
        bank->bundle = NULL;
        return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_SUCCESS;
}

/*
 * Release a transport handle
 * The function sends ioctl request to adf user proxy to release a ring
 */
CpaStatus icp_adf_transReleaseHandle(icp_comms_trans_handle trans_handle)
{
    int ret = 0;
    icp_accel_dev_t *accel_dev = NULL;
    adf_dev_ring_handle_t *pRingHandle = NULL;
    adf_dev_bank_handle_t *pbanks = NULL;
    Cpa32U ring_number;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    pRingHandle = (adf_dev_ring_handle_t *)trans_handle;
    ICP_CHECK_FOR_NULL_PARAM(pRingHandle->accel_dev);
    accel_dev = pRingHandle->accel_dev;
    ret = icp_adf_transGetRingNum(pRingHandle, &ring_number);
    if (CPA_STATUS_SUCCESS != ret)
    {
        ADF_ERROR("icp_adf_transGetRingNum failed\n");
        return ret;
    }

    pbanks = accel_dev->banks;

    /* update user process IRQ mask
     * Every time userspace ring gets a message it reads it from the ring
     * and as the last step needs to enable the IRQ for the ring so
     * the driver could get notifications that there is data on the ring.
     * So this is important to keep the IRQ mask up to date */
    pbanks[pRingHandle->bank_num].interrupt_mask &=
        (~(1 << pRingHandle->ring_num));
    pbanks[pRingHandle->bank_num].pollingMask &=
        (~(1 << pRingHandle->ring_num));
    /* send the request down to the kernel.
     * NOTE: Don't send release_handle if the kernel proxy is not running
     * The proxy down there will cleanup the rings anyway.
     * */
#ifndef CONFIG_USE_UIO_DYNAMIC
    if (accel_dev->adfSubsystemStatus)
    {
        ret = ICP_IOCTL(accel_dev->devCtlFileHdl,
                        ADF_DEV_RING_IOC_RELEASEHANDLE,
                        pRingHandle);
        if (ret != 0)
        {
            ADF_ERROR("ioctl failed, ret=%d\n", ret);
        }
    }
    ICP_MUNMAP(pRingHandle->ring_virt_addr, pRingHandle->ring_size);
    pRingHandle->ring_virt_addr = NULL;
#else

    /* Clean ringbuffer only in case if bundle fd is valid.
     * NOTE: Reset destroys /dev/qatX device and grow it up again -
     * what causes fd invalidation.
     */
    if (CPA_STATUS_SUCCESS ==
        uio_bundle_valid(
            (struct adf_uio_user_bundle *)pRingHandle->bank_data->bundle))
    {
        adf_cleanup_ring(pRingHandle);
    }

#endif
    if (NULL != pRingHandle->service_name)
    {
        ICP_FREE(pRingHandle->service_name);
        pRingHandle->service_name = NULL;
        ICP_FREE(pRingHandle->section_name);
        pRingHandle->section_name = NULL;
    }

    if (NULL != pRingHandle->user_lock)
    {
        if (CPA_STATUS_SUCCESS != ICP_MUTEX_UNINIT(pRingHandle->user_lock))
        {
            ADF_ERROR("Failed to uninialze user lock mutex\n");
        }
        ICP_FREE(pRingHandle->user_lock);
        pRingHandle->user_lock = NULL;
    }

    ret = icp_adf_release_ring(
        accel_dev->accelId, pRingHandle->bank_num, pRingHandle->ring_num);
    if (CPA_STATUS_SUCCESS != ret)
    {
        ADF_ERROR("Failed to release the ring \n");
    }

    pbanks = pbanks + pRingHandle->bank_num;
    if (NULL != pbanks->rings)
    {
        pbanks->rings[pRingHandle->ring_num] = NULL;
        adf_dev_bank_handle_put(pbanks);
        adf_free_bundle(pbanks);
    }
    ICP_FREE(pRingHandle);
    pRingHandle = NULL;
    return CPA_STATUS_SUCCESS;
}

/*
 * Returns ring number for the trans handle
 */
CpaStatus icp_adf_transGetRingNum(icp_comms_trans_handle trans_handle,
                                  Cpa32U *ringNum)
{
    adf_dev_ring_handle_t *pRingHandle = NULL;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    pRingHandle = (adf_dev_ring_handle_t *)trans_handle;
    ICP_CHECK_FOR_NULL_PARAM(pRingHandle->accel_dev);
    *ringNum =
        (pRingHandle->bank_num * pRingHandle->accel_dev->maxNumRingsPerBank) +
        pRingHandle->ring_num;
    return CPA_STATUS_SUCCESS;
}


/*
 * Put a message on the transport handle
 */
CpaStatus icp_adf_transPutMsg(icp_comms_trans_handle trans_handle,
                              Cpa32U *inBuf,
                              Cpa32U bufLen)
{
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_handle;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_PARAM_RANGE(bufLen * ICP_ADF_BYTES_PER_WORD,
                          pRingHandle->message_size,
                          pRingHandle->message_size + 1);
    return adf_user_put_msg(pRingHandle, inBuf);
}

/*
 * Retrieve handle in-flight requests
 */
CpaStatus icp_adf_getInflightRequests(icp_comms_trans_handle trans_handle,
                                      Cpa32U *maxInflightRequests,
                                      Cpa32U *numInflightRequests)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    Cpa32U numInflightReq = 0;

    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_handle;

    ICP_CHECK_FOR_NULL_PARAM(trans_handle);
    ICP_CHECK_FOR_NULL_PARAM(maxInflightRequests);
    ICP_CHECK_FOR_NULL_PARAM(numInflightRequests);

    status = adf_user_get_inflight_requests(
        pRingHandle, maxInflightRequests, &numInflightReq);
    if (CPA_STATUS_SUCCESS == status)
    {
        *numInflightRequests = numInflightReq;
    }
    else
    {
        ADF_ERROR("adf_user_get_inflight_requests failed with %d status\n",
                  status);
    }
    return status;
}

/*
 * adf_user_unmap_rings
 * Device is going down - unmap all rings allocated for this device
 */
CpaStatus adf_user_unmap_rings(icp_accel_dev_t *accel_dev)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    adf_dev_ring_handle_t *pRingHandle = NULL;
    adf_dev_bank_handle_t *bank = NULL;
    int i = 0, l = 0;

    bank = accel_dev->banks;
    for (i = 0; i < accel_dev->maxNumBanks; i++)
    {
        if (NULL == bank->rings)
            continue;

        for (l = 0; l < accel_dev->maxNumRingsPerBank; l++)
        {
            pRingHandle = (bank->rings)[i];
#ifdef CONFIG_USE_UIO_BUNDLE_ADDR
            if (pRingHandle)
                adf_ring_freebuf(pRingHandle);

            if (pRingHandle && pRingHandle->ring_virt_addr)
            {
#else
            ret = munmap(pRingHandle->ring_virt_addr, pRingHandle->ring_size);
            if (ret)
            {
                ADF_ERROR("Failed unmapping ring memory\n");
                stat = CPA_STATUS_FAIL;
            }
            pRingHandle->ring_virt_addr = NULL;
#endif
            }
        }
    }
    return stat;
}

/*
 * Internal functions which performs all the
 * tasks necessary to poll a response ring.
 */
CpaStatus adf_pollRing(icp_accel_dev_t *accel_dev,
                       adf_dev_ring_handle_t *pRingHandle,
                       Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_RETRY;

    /* Check to see if this ring is already being polled by
     * another core or thread. DecAndTest returns TRUE
     * only if pRingHandle->pollingInProgress was previously
     * equal to one and then sets the var to zero. While
     * pRingHandle->pollingInProgress is still zero no other
     * thread will be able to poll. pollingInProgress is
     * reset to one once the notify function is done.
     */
    if (osalAtomicDecAndTest((OsalAtomic *)&(pRingHandle->pollingInProgress)))
    {
        /*Set the ring response quota.*/
        pRingHandle->ringResponseQuota = response_quota;
        status = adf_user_notify_msgs_poll(pRingHandle);
        osalAtomicSet(1, (OsalAtomic *)&(pRingHandle->pollingInProgress));
    }
    return status;
}
/*
 * This function allows the user to poll the response rings of a given
 * bank to determine if any of the rings have messages that need to be
 * read. This method is used as an alternative to reading messages
 * via the ISR method.
 * N.B. response quota is per ring.
 */
CpaStatus icp_sal_pollBank(Cpa32U accelId,
                           Cpa32U bank_number,
                           Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_RETRY;
    icp_accel_dev_t *accel_dev = NULL;
    adf_dev_bank_handle_t *bank = NULL;
    adf_dev_bank_handle_t *banks = NULL;
    adf_dev_ring_handle_t *pRingHandle = NULL;
    Cpa8U *csr_base_addr = NULL;
    Cpa32U csrVal = 0;
    Cpa32U ringnum_in_bank = 0;
    Cpa32U stat_total = 0;

    /* Find the accel device associated with the accelId
     * passed in.
     */
    accel_dev = adf_devmgrGetAccelDevByAccelId(accelId);
    if (!accel_dev)
    {
        ADF_ERROR("There is no accel device associated"
                  " with this accel id.\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    ICP_CHECK_PARAM_LT_MAX(bank_number, accel_dev->maxNumBanks);
    ICP_CHECK_PARAM_GT_MIN(bank_number, 0);
    banks = accel_dev->banks;
    bank = &banks[bank_number];
    if (CPA_STATUS_SUCCESS != ICP_MUTEX_LOCK(bank->user_bank_lock))
    {
        ADF_ERROR("%s Failed to lock user bank\n", __func__);
        return CPA_STATUS_RESOURCE;
    }

    csr_base_addr = (Cpa8U *)bank->csr_addr;

    /* Read the ring status CSR to determine which rings are empty. */
    csrVal = READ_CSR_E_STAT(bank->bank_offset);
    /* Complement to find which rings have data to be processed. */
    csrVal = ~csrVal;
    /* Return RETRY if the bank polling rings
     * are all empty. */
    if (!(csrVal & bank->pollingMask))
    {
        if (CPA_STATUS_SUCCESS != ICP_MUTEX_UNLOCK(bank->user_bank_lock))
        {
            ADF_ERROR("%s Failed to unlock user bank\n", __func__);
        }
        return CPA_STATUS_RETRY;
    }

    /*
     * Loop over all rings within this bank.
     * The ringHandles structure is global to all
     * rings hence while we loop over all rings in the
     * bank we use ring_number to get the global
     * RingHandle.
     */
    for (ringnum_in_bank = 0; ringnum_in_bank < accel_dev->maxNumRingsPerBank;
         ringnum_in_bank++)
    {
        pRingHandle = (bank->rings)[ringnum_in_bank];
        /* If this ring has not being created move to next ring. */
        if (NULL == pRingHandle)
        {
            continue;
        }
        /* And with polling ring mask.
         * If the there is no data on this ring
         * move to the next one.*/
        if (!(csrVal & pRingHandle->pollingMask))
        {
            continue;
        }
        /*Poll the ring. */
        status = adf_pollRing(accel_dev, pRingHandle, response_quota);
        if (CPA_STATUS_SUCCESS == status)
        {
            stat_total++;
        }
    }
    /* Return SUCCESS if adf_pollRing returned SUCCESS
     * at any stage. */
    if (CPA_STATUS_SUCCESS != ICP_MUTEX_UNLOCK(bank->user_bank_lock))
    {
        ADF_ERROR("%s Failed to unlock user bank\n", __func__);
    }
    if (stat_total)
    {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_RETRY;
}

/*
 * This function allows the user to poll all the response rings
 * belonging to a process per device.
 * This method is used as an alternative to the reading messages
 * via the ISR method.
 * N.B. response_quota is per ring.
 */
CpaStatus icp_sal_pollAllBanks(Cpa32U accelId, Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_RETRY;
    icp_accel_dev_t *accel_dev = NULL;
    adf_dev_bank_handle_t *bank = NULL;
    adf_dev_bank_handle_t *banks = NULL;
    Cpa32U bank_num = 0;
    Cpa32U stat_total = 0;

    /* Find the accel device associated with the accelId
     * passed in.
     */
    accel_dev = adf_devmgrGetAccelDevByAccelId(accelId);
    if (!accel_dev)
    {
        ADF_ERROR("There is no accel device associated"
                  " with this accel id.\n");
        return CPA_STATUS_INVALID_PARAM;
    }

    /* Loop over banks and call icp_sal_pollBank. */
    banks = accel_dev->banks;
    for (bank_num = 0; bank_num < accel_dev->maxNumBanks; bank_num++)
    {
        bank = &(banks)[bank_num];
        /* if there are no polling rings on this bank
         * continue to the next bank number. */
        if (bank->pollingMask == 0)
        {
            continue;
        }
        status = icp_sal_pollBank(accelId, bank_num, response_quota);
        if (CPA_STATUS_SUCCESS == status)
        {
            stat_total++;
        }
    }
    /* Return SUCCESS if icp_sal_pollBank returned SUCCESS
     * at any stage. icp_sal_pollBank cannot
     * return fail in the above case. */
    if (stat_total)
    {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_RETRY;
}

/*
 * This will set the fd of the UIO device this instance
 * handler is using. If more than one transaction handler
 * are ever present, this will need to be refactored to
 * return the appropriate fd of the appropriate bank
 */
CpaStatus icp_adf_transGetFdForHandle(icp_comms_trans_handle trans_hnd, int *fd)
{
    int local_fd = -1;
    adf_dev_ring_handle_t *ring_handle = (adf_dev_ring_handle_t *)trans_hnd;
    struct adf_uio_user_bundle *bundle =
        (struct adf_uio_user_bundle *)ring_handle->bank_data->bundle;
    local_fd = bundle->fd;

    if (local_fd >= 0)
    {
        *fd = local_fd;
        return CPA_STATUS_SUCCESS;
    }
    else
    {
        return CPA_STATUS_FAIL;
    }
}

/*
 * This function allows the user to poll the response ring. The
 * ring number to be polled is supplied by the user via the
 * trans handle for that ring. The trans_hnd is a pointer
 * to an array of trans handles. This ring is
 * only polled if it contains data.
 * This method is used as an alternative to the reading messages
 * via the ISR method.
 * This function will return RETRY if the ring is empty.
 */
CpaStatus icp_adf_pollInstance(icp_comms_trans_handle *trans_hnd,
                               Cpa32U num_transHandles,
                               Cpa32U response_quota)
{
    CpaStatus status = CPA_STATUS_RETRY;
    adf_dev_ring_handle_t *ring_hnd = NULL;
    adf_dev_ring_handle_t *ring_hnd_first = NULL;
    Cpa32U i = 0;
    Cpa32U stat_total = 0;

    ICP_CHECK_FOR_NULL_PARAM(trans_hnd);
    ring_hnd_first = (adf_dev_ring_handle_t *)trans_hnd[0];
    if (!ring_hnd_first)
    {
        return CPA_STATUS_FAIL;
    }
    if (CPA_STATUS_SUCCESS != ICP_MUTEX_LOCK(ring_hnd_first->user_lock))
    {
        ADF_ERROR("%s Failed to lock user lock\n", __func__);
        return CPA_STATUS_RESOURCE;
    }

    /* No READ_CSR_E_STAT access here, could detect the empty status by
     * monitoring the ring */
    for (i = 0; i < num_transHandles; i++)
    {
        ring_hnd = (adf_dev_ring_handle_t *)trans_hnd[i];
        if (!ring_hnd)
        {
            if (CPA_STATUS_SUCCESS !=
                ICP_MUTEX_UNLOCK(ring_hnd_first->user_lock))
            {
                ADF_ERROR("%s Failed to unlock user lock\n", __func__);
            }
            return CPA_STATUS_FAIL;
        }
        /* And with polling ring mask. If the
         * polling ring has no data move to the
         * next ring handle.*/
        if (!ring_hnd->pollingMask)
        {
            continue;
        }
        /* Poll the ring. */
        status = adf_pollRing(ring_hnd->accel_dev, ring_hnd, response_quota);
        if (CPA_STATUS_SUCCESS == status)
        {
            stat_total++;
        }
    }
    if (CPA_STATUS_SUCCESS != ICP_MUTEX_UNLOCK(ring_hnd_first->user_lock))
    {
        ADF_ERROR("%s Failed to unlock user lock\n", __func__);
    }
    /* If any of the rings in the instance had data and was polled
     * return SUCCESS. */
    if (stat_total)
    {
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_RETRY;
}

/*
 * Function initializes internal transport data
 */
CpaStatus adf_user_transport_init(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0;
    Cpa32S x = 0;
    adf_dev_bank_handle_t *bank = NULL;
    adf_dev_bank_handle_t *banks = NULL;
    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    status = adf_proxy_populate_device_info(accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    banks = accel_dev->banks;
    for (i = 0; i < accel_dev->maxNumBanks; i++)
    {
        bank = &banks[i];
        bank->user_bank_lock = ICP_MALLOC_GEN(sizeof(ICP_MUTEX));
        if (!bank->user_bank_lock)
        {
            ADF_ERROR("Could not alloc memory for bank mutex\n");
            for (x = i - 1; x >= 0; x--)
            {
                bank = &banks[x];
                if (CPA_STATUS_SUCCESS !=
                    ICP_MUTEX_UNINIT(bank->user_bank_lock))
                {
                    ADF_ERROR("Failed to uninitialize user bank lock mutex\n");
                }
                ICP_FREE(bank->user_bank_lock);
            }
            adf_proxy_depopulate_device_info(accel_dev);
            return CPA_STATUS_FAIL;
        }
        if (CPA_STATUS_SUCCESS != ICP_MUTEX_INIT(bank->user_bank_lock))
        {
            ADF_ERROR("Failed to initialize user bank lock mutex\n");
        }
    }

    return status;
}

/*
 * Function deinitializes internal transport data
 */
CpaStatus adf_user_transport_exit(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    Cpa32U i = 0;
    adf_dev_bank_handle_t *bank = NULL;
    adf_dev_bank_handle_t *banks = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    banks = accel_dev->banks;
    for (i = 0; i < accel_dev->maxNumBanks; i++)
    {
        bank = &(banks)[i];
        if (NULL != bank->bundle)
        {
            uio_free_bundle(bank->bundle);
            bank->bundle = NULL;
        }

        if (bank->user_bank_lock)
        {
            if (CPA_STATUS_SUCCESS != ICP_MUTEX_UNINIT(bank->user_bank_lock))
            {
                ADF_ERROR("Failed to uninitialize user back lock");
            }
            ICP_FREE(bank->user_bank_lock);
            bank->user_bank_lock = NULL;
        }

        if (bank->rings)
        {
            ICP_FREE(bank->rings);
            bank->rings = NULL;
        }
    }
    adf_proxy_depopulate_device_info(accel_dev);
    return status;
}

CpaStatus icp_adf_flush_requests(icp_comms_trans_handle trans_handle)
{
    CpaStatus status = CPA_STATUS_FAIL;
    int32_t adf_status = 0;
    adf_dev_ring_handle_t *pRingHandle = (adf_dev_ring_handle_t *)trans_handle;

    ICP_CHECK_FOR_NULL_PARAM(pRingHandle);

    adf_status = adf_user_flush_requests(pRingHandle);
    if (0 == adf_status)
    {
        status = CPA_STATUS_SUCCESS;
    }

    return status;
}


