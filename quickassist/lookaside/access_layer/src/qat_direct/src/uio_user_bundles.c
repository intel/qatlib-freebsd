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
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#ifndef ICP_WITHOUT_THREAD
#include <pthread.h>
#endif
#include "cpa.h"
#include "icp_platform.h"

#include "uio_user.h"
#include "adf_kernel_types.h"
#include "adf_user.h"
#include "uio_user_cfg.h"
#include "qae_mem.h"

#define UIO_CONTROL_MUTEX_NAME "uio_ctl_lock"
#define MAP_INDEX 0
#define MAX_PATHLEN 256

#define QAT_UIO_IOC_MAGIC 'b'
#define IOCTL_GET_BUNDLE_SIZE _IOR(QAT_UIO_IOC_MAGIC, 0, int32_t)
#define IOCTL_ALLOC_BUNDLE _IOW(QAT_UIO_IOC_MAGIC, 1, int)
#define IOCTL_GET_ACCEL_TYPE _IOR(QAT_UIO_IOC_MAGIC, 2, uint32_t)
#define IOCTL_ADD_MEM_FD _IOW(QAT_UIO_IOC_MAGIC, 3, int)

#ifndef ICP_WITHOUT_THREAD
static pthread_mutex_t *uio_control_mutex;
#endif

int uio_control_mutex_init(void)
{
#ifndef ICP_WITHOUT_THREAD
    return uio_shared_mutex_create(UIO_CONTROL_MUTEX_NAME, &uio_control_mutex);
#else
    return 0;
#endif
}

int uio_control_mutex_destroy(void)
{
#ifndef ICP_WITHOUT_THREAD
    return uio_shared_mutex_destroy(uio_control_mutex);
#else
    return 0;
#endif
}

static int open_qat(int dev_id)
{
    char path[MAX_PATHLEN];
    int fd;

    snprintf(path, sizeof(path), "/dev/qat%d", dev_id);
    fd = ICP_OPEN(path, O_RDWR);
    if (fd == -1)
        ADF_ERROR("failed to open\n");

    return (fd);
}

/**
 * This function is using 'udev_device_get_sysattr_value()' function.
 *
 * From man pages: The retrieved value is cached in the device.
 * Repeated calls will return the same value and not open the attribute again.
 *
 * More calls of 'get_bundle_from_dev_cached()' without enumerating
 * of the device via 'uio_udev_get_device_from_devid()' gives incorrect values
 * and is major issue for 'used' flag! (e.g. 'uio_ctrl/bundle_0/used').
 */
STATIC INLINE struct adf_uio_user_bundle *get_bundle_from_dev_cached(
    int accel_id,
    int bundle_nr)
{
    struct adf_uio_user_bundle *bundle = NULL;
    uint32_t val;
    int mem_fd = 0;
    int mem_init_status = 0;
    bundle = ICP_MALLOC_GEN(sizeof(*bundle));
    uint32_t bundle_size = 0;

    if (!bundle)
    {
        ADF_ERROR("failed to allocate bundle structure\n");
        return NULL;
    }
    bundle->fd = open_qat(accel_id);

    if (bundle->fd < 0)
    {
        ADF_ERROR("failed to open the device\n");
        ICP_FREE(bundle);
        return NULL;
    }

    if (ICP_IOCTL(bundle->fd, IOCTL_GET_ACCEL_TYPE, &val) == -1)
    {
        ICP_CLOSE(bundle->fd);
        ICP_FREE(bundle);
        ADF_ERROR("ioctl(GET_ACCEL_TYPE) failed\n");
        return NULL;
    }

    if (ICP_IOCTL(bundle->fd, IOCTL_GET_BUNDLE_SIZE, &bundle_size) == -1)
    {
        ICP_CLOSE(bundle->fd);
        ICP_FREE(bundle);
        ADF_ERROR("ioctl(GET_BUNDLE_SIZE) failed\n");
        return NULL;
    }
    bundle->size = bundle_size;

    if (ICP_IOCTL(bundle->fd, IOCTL_ALLOC_BUNDLE, &bundle_nr) == -1)
    {
        ICP_CLOSE(bundle->fd);
        ICP_FREE(bundle);
        ADF_ERROR("ioctl(ALLOC_BUNDLE) failed\n");
        return NULL;
    }

    bundle->ptr = ICP_MMAP(
        NULL, bundle_size, PROT_READ | PROT_WRITE, MAP_SHARED, bundle->fd, 0);
    if (NULL == bundle->ptr)
    {
        ICP_CLOSE(bundle->fd);
        ICP_FREE(bundle);
        ADF_ERROR("mmap failed\n");
        return NULL;
    }

    mem_init_status = qaeMemInitAndReturnFd(&mem_fd);
    if (mem_init_status != 0)
    {
        uio_free_bundle(bundle);
        ADF_ERROR("qaeMemReturnFd failed\n");
        return NULL;
    }
    if (ICP_IOCTL(bundle->fd, IOCTL_ADD_MEM_FD, &mem_fd) == -1)
    {
        uio_free_bundle(bundle);
        ADF_ERROR("ioctl(IOCTL_ADD_MEM_FD) failed\n");
        return NULL;
    }

    return bundle;
}

struct adf_uio_user_bundle *uio_get_bundle_from_accelid(int accelid,
                                                        int bundle_nr)
{
    struct adf_uio_user_bundle *bundle;
    bundle = get_bundle_from_dev_cached(accelid, bundle_nr);
    return bundle;
}

static void uio_free_bundle_ptr(void *bundle_ptr, unsigned long size)
{
    ICP_MUNMAP(bundle_ptr, size);
}

void uio_free_bundle(struct adf_uio_user_bundle *bundle)
{
    uio_free_bundle_ptr(bundle->ptr, bundle->size);
    ICP_CLOSE(bundle->fd);
    ICP_FREE(bundle);
}

static CpaStatus uio_populate_dev_name(icp_accel_dev_t *dev)
{
    switch (dev->deviceType)
    {
        case DEVICE_DH895XCC:
            strncpy(dev->deviceName,
                    ADF_DH895XCC_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_DH895XCCVF:
            strncpy(dev->deviceName,
                    ADF_DH895XCCVF_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_C62X:
            strncpy(dev->deviceName,
                    ADF_C62X_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_C62XVF:
            strncpy(dev->deviceName,
                    ADF_C62XVF_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_C3XXX:
            strncpy(dev->deviceName,
                    ADF_C3XXX_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_C3XXXVF:
            strncpy(dev->deviceName,
                    ADF_C3XXXVF_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_200XX:
            strncpy(dev->deviceName,
                    ADF_200XX_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_200XXVF:
            strncpy(dev->deviceName,
                    ADF_200XXVF_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_C4XXX:
            strncpy(dev->deviceName,
                    ADF_C4XXX_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_C4XXXVF:
            strncpy(dev->deviceName,
                    ADF_C4XXXVF_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        case DEVICE_4XXX:
            strncpy(dev->deviceName,
                    ADF_4XXX_DEVICE_NAME,
                    ADF_DEVICE_TYPE_LENGTH - 1);
            break;
        default:
            strncpy(dev->deviceName, "Unknown", ADF_DEVICE_TYPE_LENGTH - 1);
            break;
    }
    dev->deviceName[ADF_DEVICE_TYPE_LENGTH] = '\0';

    return CPA_STATUS_SUCCESS;
}

static int uio_populate_accel_dev(icp_accel_dev_t *accel_dev, int accel_id)
{
    char config_value[ADF_CFG_MAX_VAL_LEN_IN_BYTES];

    if (NULL == accel_dev)
    {
        ADF_ERROR("failed to populate accel dev\n");
        return EINVAL;
    }
    ICP_MEMSET(accel_dev, '\0', sizeof(*accel_dev));
    accel_dev->maxNumRingsPerBank = 16;
    accel_dev->accelId = accel_id;

    accel_dev->devCtlFileHdl = ICP_OPEN(ADF_CTL_DEVICE_NAME, O_RDWR);
    if (accel_dev->devCtlFileHdl < 0)
    {
        ADF_ERROR("failed to open %s\n", ADF_CTL_DEVICE_NAME);
        return EINVAL;
    }

    /* read Maximal Number of Banks */
    if (CPA_STATUS_SUCCESS !=
        icp_adf_cfgGetParamValue(
            accel_dev, ADF_GENERAL_SEC, ADF_DEV_MAX_BANKS, config_value))
    {
        return EINVAL;
    }
    accel_dev->maxNumBanks =
        (Cpa32U)strtoul(config_value, NULL, ADF_CFG_BASE_DEC);

    /* Get capabilities mask */
    if (CPA_STATUS_SUCCESS !=
        icp_adf_cfgGetParamValue(accel_dev,
                                 ADF_GENERAL_SEC,
                                 ADF_DEV_CAPABILITIES_MASK,
                                 config_value))
    {
        return EINVAL;
    }
    accel_dev->accelCapabilitiesMask =
        (Cpa32U)strtoul(config_value, NULL, ADF_CFG_BASE_HEX);

    /* Get max num rings per bank */
    icp_adf_cfgGetParamValue(
        accel_dev, ADF_GENERAL_SEC, ADF_DEV_MAX_RINGS_PER_BANK, config_value);
    accel_dev->maxNumRingsPerBank =
        (Cpa32U)strtoul(config_value, NULL, ADF_CFG_BASE_DEC);

    /* Get dc extended capabilities */
    if (CPA_STATUS_SUCCESS !=
        icp_adf_cfgGetParamValue(
            accel_dev, ADF_GENERAL_SEC, ADF_DC_EXTENDED_FEATURES, config_value))
    {
        return EINVAL;
    }
    accel_dev->dcExtendedFeatures =
        (Cpa32U)strtoul(config_value, NULL, ADF_CFG_BASE_HEX);

    accel_dev->pkg_id = 0;
    /* Get Device info */
    if (CPA_STATUS_SUCCESS != icp_adf_getDevInfo(accel_dev))
    {
        return EINVAL;
    }
    /* Populate Device Name */
    if (CPA_STATUS_SUCCESS != uio_populate_dev_name(accel_dev))
    {
        return EINVAL;
    }

    return 0;
}

int uio_acces_dev_exist(int dev_id)
{
    char path[MAX_PATHLEN];
    int exist;

    snprintf(path, sizeof(path), "/dev/qat%d", dev_id);
    exist = ICP_ACCESS(path, F_OK);
    if (exist == 0)
        return 1;
    else
        return 0;
}

int uio_bundle_valid(struct adf_uio_user_bundle *bundle)
{

    ICP_CHECK_FOR_NULL_PARAM(bundle);

    uint32_t val;
    if (ICP_IOCTL(bundle->fd, IOCTL_GET_ACCEL_TYPE, &val) < 0)
    {
        return CPA_STATUS_FAIL;
    }
    return CPA_STATUS_SUCCESS;
}

int uio_create_accel_dev(icp_accel_dev_t **accel_dev, int dev_id)
{
    CpaStatus status;

    *accel_dev = ICP_MALLOC_GEN(sizeof(**accel_dev));
    if (!*accel_dev)
        return ENOMEM;

    if (!uio_acces_dev_exist(dev_id))
    {
        status = EINVAL;
        goto accel_fail;
    }

    return uio_populate_accel_dev(*accel_dev, dev_id);

accel_fail:
    ICP_FREE(*accel_dev);
    *accel_dev = NULL;
    return status;
}

void uio_destroy_accel_dev(icp_accel_dev_t *accel_dev)
{
    ICP_CLOSE(accel_dev->devCtlFileHdl);
    ICP_FREE(accel_dev);
}
