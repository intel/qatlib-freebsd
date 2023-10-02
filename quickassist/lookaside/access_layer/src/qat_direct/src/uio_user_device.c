/***************************************************************************
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
 ***************************************************************************/
#include <errno.h>
#include <adf_kernel_types.h>
#include <adf_user_init.h>
#include <icp_platform.h>
#include <adf_user.h>
#include <adf_user_transport.h>
#include <icp_adf_user_proxy.h>

#include "uio_user.h"

#define ADF_MAX_PENDING_EVENT 10

/*
 * User space copy of acceleration devices
 */
STATIC icp_accel_dev_t *accel_tbl[ADF_MAX_DEVICES] = { 0 };

/*
 * Need to keep track of what device is currently in reset state
 */
STATIC char accel_dev_reset_stat[ADF_MAX_DEVICES] = { 0 };

/*
 * Need to preserv sal handle during restart
 */
STATIC void *accel_dev_sal_hdl_ptr[ADF_MAX_DEVICES] = { 0 };

/*
 * Mutex guarding access to accel_tbl on exit
 */
#ifndef ICP_WITHOUT_THREAD
STATIC pthread_mutex_t accel_tbl_mutex;
#endif
/*
 * Number of acceleration devices
 */
STATIC Cpa16U num_of_instances = 0;

/*
 * icp_adf_get_numDevices
 * This function is used to determine the number of devices
 */
CpaStatus icp_adf_get_numDevices(Cpa32U *num_devices)
{
    int fd = -1;
    int res = 0;
    Cpa32U num_dev = 0;
    CpaStatus status = CPA_STATUS_FAIL;

    ICP_CHECK_FOR_NULL_PARAM(num_devices);

    fd = ICP_OPEN(ADF_CTL_DEVICE_NAME, O_RDWR);
    if (fd < 0)
    {
        return CPA_STATUS_UNSUPPORTED;
    }

    /* send the request down to get the device
     * information from kernel space. */
    res = ICP_IOCTL(fd, IOCTL_GET_NUM_DEVICES, &num_dev);
    if (!res)
    {
        *num_devices = num_dev;
        status = CPA_STATUS_SUCCESS;
    }
    ICP_CLOSE(fd);

    return status;
}

CpaBoolean icp_adf_isDeviceAvailable(void)
{
    unsigned int num_dev = 0;
    CpaBoolean result = CPA_FALSE;

    if (CPA_STATUS_SUCCESS == icp_adf_get_numDevices(&num_dev))
    {
        if (num_dev > 0)
            result = CPA_TRUE;
    }

    return result;
}

int32_t adf_cleanup_device(int32_t dev_id)
{
    int32_t stat = CPA_STATUS_SUCCESS;
    icp_accel_dev_t *dev;
#ifndef ICP_WITHOUT_THREAD
    if (pthread_mutex_lock(&accel_tbl_mutex))
    {
        ADF_ERROR("Failed to lock mutex \n");
        return CPA_STATUS_FAIL;
    }
#endif
    if (accel_tbl[dev_id] == NULL)
    {
#ifndef ICP_WITHOUT_THREAD
        pthread_mutex_unlock(&accel_tbl_mutex);
#endif
        return 0;
    }

    dev = accel_tbl[dev_id];

    stat = adf_user_transport_exit(dev);
    uio_destroy_accel_dev(accel_tbl[dev_id]);
    accel_tbl[dev_id] = NULL;

    num_of_instances--;
#ifndef ICP_WITHOUT_THREAD
    pthread_mutex_unlock(&accel_tbl_mutex);
#endif
    return stat;
}

int32_t adf_cleanup_devices(void)
{
    int32_t i;

    for (i = 0; i < ADF_MAX_DEVICES; i++)
    {
        if (adf_cleanup_device(i) != 0)
        {
#ifndef ICP_WITHOUT_THREAD
            pthread_mutex_destroy(&accel_tbl_mutex);
#endif
            ADF_ERROR("Failed to cleanup device %d\n", i);
            return CPA_STATUS_FAIL;
        }
    }
#ifndef ICP_WITHOUT_THREAD
    pthread_mutex_destroy(&accel_tbl_mutex);
#endif

    return 0;
}

int32_t adf_init_devices(void)
{
    int32_t i = 0;
#ifndef ICP_WITHOUT_THREAD
    pthread_mutex_init(&accel_tbl_mutex, NULL);
#endif
    for (i = 0; i < ADF_MAX_DEVICES; i++)
    {
        accel_tbl[i] = NULL;
    }

    return 0;
}

/*
 * adf_stop_system
 * Sets the user proxy running state to stopped
 */
STATIC inline void adf_stop_system(icp_accel_dev_t *accel_dev)
{
    accel_dev->adfSubsystemStatus = 0;
}

/*
 * adf_start_system
 * Sets the user proxy running state to started
 */
STATIC inline void adf_start_system(icp_accel_dev_t *accel_dev)
{
    accel_dev->adfSubsystemStatus = 1;
}

/*
 * subsystem_notify
 * Forwards the event to each registered subsystem
 */
CpaStatus subsystem_notify(icp_accel_dev_t *accel_dev, Cpa32U event)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    CpaStatus stat_proxy = CPA_STATUS_SUCCESS;
    CpaStatus stat_restart = CPA_STATUS_SUCCESS;

    if (!accel_dev && event != ADF_EVENT_RESTARTED)
        return CPA_STATUS_INVALID_PARAM;

    switch (event)
    {
        case ADF_EVENT_INIT:
            if (accel_dev_sal_hdl_ptr[accel_dev->accelId])
            {
                accel_dev->pSalHandle =
                    accel_dev_sal_hdl_ptr[accel_dev->accelId];
                accel_dev_sal_hdl_ptr[accel_dev->accelId] = NULL;
            }
            stat = adf_user_subsystemInit(accel_dev);
            break;
        case ADF_EVENT_START:
            stat = adf_user_subsystemStart(accel_dev);
            adf_start_system(accel_dev);
            if (accel_dev_reset_stat[accel_dev->accelId])
            {
                accel_dev_reset_stat[accel_dev->accelId] = 0;
                stat_restart = adf_subsystemRestarted(accel_dev);
            }
            break;
        case ADF_EVENT_STOP:
            adf_stop_system(accel_dev);
            stat = adf_user_subsystemStop(accel_dev);
            break;
        case ADF_EVENT_SHUTDOWN:
            stat = adf_user_subsystemShutdown(accel_dev);
            /* Close user proxy for given device */
            if (stat == CPA_STATUS_SUCCESS)
            {
                stat_proxy = adf_cleanup_device(accel_dev->accelId);
            }
            break;
        case ADF_EVENT_RESTARTING:
            accel_dev_reset_stat[accel_dev->accelId] = 1;
            stat = adf_subsystemRestarting(accel_dev);
            accel_dev_sal_hdl_ptr[accel_dev->accelId] = accel_dev->pSalHandle;
            break;
        case ADF_EVENT_RESTARTED:
            stat = icp_adf_find_new_devices();
            break;
        case ADF_EVENT_ERROR:
            stat = adf_subsystemError(accel_dev);
            break;
        default:
            stat = CPA_STATUS_INVALID_PARAM;
    }

    if (CPA_STATUS_SUCCESS != stat_proxy)
    {
        ADF_ERROR("Failed to close process proxy\n");
    }
    if (CPA_STATUS_SUCCESS != stat_restart)
    {
        ADF_ERROR("Failed to restart device\n");
        stat = stat_restart;
    }
    return stat;
}

static int adf_proxy_get_dev_events(int dev_id);

static int32_t adf_proxy_get_device(int dev_id)
{
    int32_t err;

    if (NULL != accel_tbl[dev_id])
        return 0; /* Already created. */

    if (!uio_acces_dev_exist(dev_id))
        return 0;

    err = uio_create_accel_dev(&accel_tbl[dev_id], dev_id);
    if (NULL == accel_tbl[dev_id])
    {
        err = ENOMEM;
        goto adf_proxy_get_device_exit;
    }

    err = adf_user_transport_init(accel_tbl[dev_id]);
    if (0 != err)
    {
        goto adf_proxy_get_device_init_failed;
    }
    adf_proxy_get_dev_events(dev_id);
    num_of_instances++;
    return 0;

adf_proxy_get_device_init_failed:
    free(accel_tbl[dev_id]);
    accel_tbl[dev_id] = NULL;
adf_proxy_get_device_exit:
    return err;
}

static int adf_proxy_get_dev_events(int dev_id)
{
    enum adf_event event[] = { ADF_EVENT_INIT, ADF_EVENT_START };
    size_t i = 0;

    if (accel_tbl[dev_id] != NULL)
    {
        for (i = 0; i < sizeof(event) / sizeof(event[0]); i++)
        {
            subsystem_notify(accel_tbl[dev_id], event[i]);
        }
    }

    return 0;
}

CpaStatus adf_proxy_get_devices(void)
{
    int32_t ctr = 0;
    Cpa32U num_dev = 0;

    if (icp_adf_get_numDevices(&num_dev))
        return CPA_STATUS_FAIL;

    for (ctr = 0; ctr < num_dev; ctr++)
    {
        if (adf_proxy_get_device(ctr))
        {
            ADF_ERROR("adf_proxy_get_device error ctr\n");
            return CPA_STATUS_FAIL;
        }
    }
    return CPA_STATUS_SUCCESS;
}

CpaStatus icp_adf_find_new_devices(void)
{
    return adf_proxy_get_devices();
}

CpaStatus icp_adf_poll_device_events(void)
{
    Cpa32U accelId;
    Cpa32U event;
    while (adf_proxy_poll_event(&accelId, &event))
    {
        if (accelId < ADF_MAX_DEVICES)
            subsystem_notify(accel_tbl[accelId], event);
        else
            ADF_ERROR("Invalid accelId (%d) from event poll\n", accelId);
    }

    return CPA_STATUS_SUCCESS;
}

/*
 *  * icp_qa_dev_get
 *   * Function increments the device usage counter.
 *    */
void icp_qa_dev_get(icp_accel_dev_t *pAccelDev)
{
    __sync_fetch_and_add(&pAccelDev->usageCounter, 1);
    return;
}

/*
 *  * icp_qa_dev_get
 *   * Function decrements the device usage counter.
 *    */
void icp_qa_dev_put(icp_accel_dev_t *pAccelDev)
{
    __sync_fetch_and_sub(&pAccelDev->usageCounter, 1);
    return;
}

/*
 * adf_devmgrGetAccelDevByAccelId
 * Check the accel table for a structure that contains the correct
 * accel ID. If the accelId is found return the pointer to the accelerator
 * structure.
 * Returns a pointer to the accelerator structure or NULL if not found.
 */
icp_accel_dev_t *adf_devmgrGetAccelDevByAccelId(Cpa32U accelId)
{
    icp_accel_dev_t **ptr = accel_tbl;
    Cpa16U i = 0;

    for (i = 0; i < ADF_MAX_DEVICES; i++, ptr++)
    {
        if (NULL != *ptr)
        {
            if ((*ptr)->accelId == accelId)
            {
                return *ptr;
            }
        }
    }
    return NULL;
}

/*
 * icp_adf_getAccelDevByAccelId
 * Same as adf_devmgrGetAccelDevByAccelId() but for external use
 * structure.
 * Returns a pointer to the accelerator structure or NULL if not found.
 */
icp_accel_dev_t *icp_adf_getAccelDevByAccelId(Cpa32U accelId)
{
    return adf_devmgrGetAccelDevByAccelId(accelId);
}

/*
 * icp_adf_is_dev_in_reset
 * Check if device is in reset state.
 */
CpaBoolean icp_adf_is_dev_in_reset(icp_accel_dev_t *accel_dev)
{
    return (CpaBoolean)accel_dev_reset_stat[accel_dev->accelId];
}

/*
 * icp_adf_check_device_by_fd
 * Function checks the status of the firmware/hardware for a given device
 * provided the fd has already been opened.
 */
static CpaStatus icp_adf_check_device_by_fd(int fd, Cpa32U accelId)
{
    struct adf_dev_heartbeat_status_ctl hb_status = {};

    if (fd < 0)
        return CPA_STATUS_FAIL;

    hb_status.device_id = accelId;
    if (ICP_IOCTL(fd, IOCTL_HEARTBEAT_ACCEL_DEV, &hb_status))
        return CPA_STATUS_FAIL;

    switch (hb_status.status)
    {
        case DEV_HB_ALIVE:
            return CPA_STATUS_SUCCESS;
        case DEV_HB_UNSUPPORTED:
            return CPA_STATUS_UNSUPPORTED;
        case DEV_HB_UNRESPONSIVE:
        default:
            return CPA_STATUS_FAIL;
    }

    return CPA_STATUS_FAIL;
}

/*
 * icp_adf_check_device
 * Function checks the status of the firmware/hardware for a given device.
 */
CpaStatus icp_adf_check_device(Cpa32U accelId)
{
    CpaStatus ret = CPA_STATUS_FAIL;
    int fd = ICP_OPEN(ADF_CTL_DEVICE_NAME, O_RDWR);
    if (fd < 0)
    {
        return CPA_STATUS_UNSUPPORTED;
    }

    ret = icp_adf_check_device_by_fd(fd, accelId);
    ICP_CLOSE(fd);
    return ret;
}

/*
 * icp_adf_check_all_devices
 * Function checks the status of the firmware/hardware for all devices.
 */
CpaStatus icp_adf_check_all_devices(void)
{
    Cpa32U i;
    CpaStatus res = CPA_STATUS_FAIL;
    for (i = 0; i < ADF_MAX_DEVICES; i++)
    {
        if (NULL != accel_tbl[i])
        {
            res = icp_adf_check_device(i);
            if (CPA_STATUS_SUCCESS != res)
            {
                ADF_ERROR("Device Check failed for device %d\n", i);
                return res;
            }
        }
    }
    return CPA_STATUS_SUCCESS;
}

#ifdef ICP_HB_FAIL_SIM
/*
 * icp_adf_heartbeat_simulate_failure
 * Function simulates a heartbeat failure for a given device.
 */
CpaStatus icp_adf_heartbeat_simulate_failure(Cpa32U accelId)
{
    CpaStatus ret = CPA_STATUS_SUCCESS;
    int fd = ICP_OPEN(ADF_CTL_DEVICE_NAME, O_RDWR);
    struct adf_user_cfg_ctl_data dev_data = { { 0 } };
    dev_data.device_id = accelId;

    if (fd < 0)
    {
        return CPA_STATUS_UNSUPPORTED;
    }

    ret = ICP_IOCTL(fd, IOCTL_HEARTBEAT_SIM_FAIL, &dev_data);

    if (ret)
    {
        ADF_ERROR("HB fail simulation ioctl call failed. RC:%d Errno:%d\n",
                  ret,
                  errno);
        ret = CPA_STATUS_FAIL;
    }

    ICP_CLOSE(fd);
    return ret;
}

#endif /* QAT_HB_FAIL_SIM */

/*
 * icp_amgr_getNumInstances
 * Return the number of acceleration devices it the system.
 */
CpaStatus icp_amgr_getNumInstances(Cpa16U *pNumInstances)
{
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);
    *pNumInstances = num_of_instances;
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_amgr_getInstances
 * Return table of acceleration instances it the system.
 */
CpaStatus icp_amgr_getInstances(Cpa16U numInstances,
                                icp_accel_dev_t **pCyInstances)
{
    Cpa16U i = 0, x = 0;
    ICP_CHECK_FOR_NULL_PARAM(pCyInstances);

    if (numInstances > num_of_instances)
    {
        /* Too many acceleration devices requested */
        ADF_ERROR("Too many instances of accel device requested\n");
        return CPA_STATUS_FAIL;
    }

    for (i = 0; i < ADF_MAX_DEVICES; i++)
    {
        if (NULL != accel_tbl[i])
        {
            pCyInstances[x++] = (icp_accel_dev_t *)accel_tbl[i];
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_adf_GetInstances
 * Return the acceleration instance by name.
 */
CpaStatus icp_amgr_getAccelDevByName(unsigned char *instanceName,
                                     icp_accel_dev_t **pAccel_dev)
{
    Cpa16U i = 0;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_dev);
    ICP_CHECK_FOR_NULL_PARAM(instanceName);

    for (i = 0; i < ADF_MAX_DEVICES; i++)
    {
        if (NULL != accel_tbl[i])
        {
            if (strncmp((const char *)instanceName,
                        (const char *)accel_tbl[i]->pAccelName,
                        MAX_ACCEL_NAME_LEN) == 0)
            {
                *pAccel_dev = (icp_accel_dev_t *)accel_tbl[i];
                return CPA_STATUS_SUCCESS;
            }
        }
    }
    return CPA_STATUS_FAIL;
}

/*
 * icp_amgr_getAccelDevByCapabilities
 * Returns a started accel device that implements
 * the capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAccelDevByCapabilities(Cpa32U capabilitiesMask,
                                             icp_accel_dev_t **pAccel_devs,
                                             Cpa16U *pNumInstances)
{
    icp_accel_dev_t **ptr = accel_tbl;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_devs);
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);
    Cpa16U i = 0;

    *pNumInstances = 0;
    for (i = 0; i < ADF_MAX_DEVICES; i++, ptr++)
    {
        if (NULL != *ptr)
        {
            if ((*ptr)->accelCapabilitiesMask & capabilitiesMask)
            {
                if ((*ptr)->adfSubsystemStatus)
                {
                    *pAccel_devs = (icp_accel_dev_t *)*ptr;
                    *pNumInstances = 1;
                    return CPA_STATUS_SUCCESS;
                }
            }
        }
    }
    return CPA_STATUS_FAIL;
}

/*
 * icp_amgr_getAllAccelDevByCapabilities
 * Returns table of accel devices that are started and that implement
 * at least one of the capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAllAccelDevByCapabilities(Cpa32U capabilitiesMask,
                                                icp_accel_dev_t **pAccel_devs,
                                                Cpa16U *pNumInstances)
{
    icp_accel_dev_t **ptr = accel_tbl;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_devs);
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);
    Cpa16U i = 0;

    *pNumInstances = 0;
    for (i = 0; i < ADF_MAX_DEVICES; i++, ptr++)
    {
        if (NULL != *ptr)
        {
            if ((*ptr)->accelCapabilitiesMask & capabilitiesMask)
            {
                if ((*ptr)->adfSubsystemStatus)
                {
                    pAccel_devs[(*pNumInstances)++] = (icp_accel_dev_t *)*ptr;
                }
            }
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_amgr_getAllAccelDevByEachCapabilities
 * Returns table of accel devices that are started and implement
 * each of the capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAllAccelDevByEachCapability(Cpa32U capabilitiesMask,
                                                  icp_accel_dev_t **pAccel_devs,
                                                  Cpa16U *pNumInstances)
{
    icp_accel_dev_t **ptr = accel_tbl;
    ICP_CHECK_FOR_NULL_PARAM(pAccel_devs);
    ICP_CHECK_FOR_NULL_PARAM(pNumInstances);
    Cpa16U i = 0;

    *pNumInstances = 0;
    for (i = 0; i < ADF_MAX_DEVICES; i++, ptr++)
    {
        if (NULL != *ptr)
        {
            Cpa32U enabled_caps =
                (*ptr)->accelCapabilitiesMask & capabilitiesMask;
            if (enabled_caps == capabilitiesMask)
            {
                if ((*ptr)->adfSubsystemStatus)
                {
                    pAccel_devs[(*pNumInstances)++] = (icp_accel_dev_t *)*ptr;
                }
            }
        }
    }
    return CPA_STATUS_SUCCESS;
}

/*
 * icp_amgr_getAccelDevCapabilities
 * Returns accel devices capabilities specified in capabilitiesMask.
 */
CpaStatus icp_amgr_getAccelDevCapabilities(icp_accel_dev_t *accel_dev,
                                           Cpa32U *pCapabilitiesMask)
{
    icp_accel_dev_t *pAccelDev = NULL;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pCapabilitiesMask);

    pAccelDev = accel_dev;
    *pCapabilitiesMask = pAccelDev->accelCapabilitiesMask;
    return CPA_STATUS_SUCCESS;
}

/*
 * adf_devmgrGetAccelHead
 * Sets the AccelDev to the head of the accelerator table.
 * Note: This function returns pointer to acceleration table
 * unlike the same function in kernelspace where is returns
 * pointer to list head.
 */
CpaStatus adf_devmgrGetAccelHead(icp_accel_dev_t **pAccelDev)
{
    ICP_CHECK_FOR_NULL_PARAM(pAccelDev);
    *pAccelDev = (icp_accel_dev_t *)accel_tbl;
    return CPA_STATUS_SUCCESS;
}
