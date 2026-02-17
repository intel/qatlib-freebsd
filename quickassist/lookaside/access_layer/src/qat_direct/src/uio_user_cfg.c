/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#include "cpa.h"
#include "adf_kernel_types.h"
#include "adf_user.h"
#include "icp_platform.h"
#include "icp_platform_user.h"
#include "icp_accel_devices.h"
#include "icp_adf_init.h"
#include "icp_adf_accel_mgr.h"
#include "uio_user.h"
#include "uio_user_cfg.h"

#define ADF_MAX_SERVICE_TYPE 0x7

/*
 * Open kernel driver interface
 */
static int open_dev(icp_accel_dev_t *accel_dev)
{
    int file_desc = -1;

    if (NULL == accel_dev)
    {
        file_desc = ICP_OPEN(ADF_CTL_DEVICE_NAME, O_RDWR);
        if (file_desc < 0)
        {
            ADF_ERROR("Error: Failed to open device %s\n", ADF_CTL_DEVICE_NAME);
        }
    }
    else
    {
        file_desc = accel_dev->devCtlFileHdl;
    }

    return file_desc;
}

/*
 * Close kernel driver interface
 */
static void close_dev(icp_accel_dev_t *accel_dev, int fd)
{
    if (NULL == accel_dev)
        ICP_CLOSE(fd);
}

/*
 * icp_adf_cfgGetParamValue
 * This function is used to determine the value configured for the
 * given parameter name.
 */
CpaStatus icp_adf_cfgGetParamValue(icp_accel_dev_t *accel_dev,
                                   const char *pSection,
                                   const char *pParamName,
                                   char *pParamValue)
{
    CpaStatus status = CPA_STATUS_FAIL;
    struct adf_user_cfg_ctl_data config = { { 0 } };
    struct adf_user_cfg_key_val kval = { { 0 } };
    struct adf_user_cfg_section section = { { 0 } };

    int fd = -1;
    int res = 0;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);
    ICP_CHECK_FOR_NULL_PARAM(pSection);
    ICP_CHECK_FOR_NULL_PARAM(pParamName);
    ICP_CHECK_FOR_NULL_PARAM(pParamValue);

    /* do ioctl to get the data */
    fd = open_dev(accel_dev);
    if (fd < 0)
    {
        return CPA_STATUS_FAIL;
    }

    config.device_id = accel_dev->accelId;
    config.config_section = &section;
    strncpy(section.name, pSection, ADF_CFG_MAX_SECTION_LEN_IN_BYTES - 1);
    section.name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES - 1] = '\0';
    section.params = &kval;
    strncpy(kval.key, pParamName, ADF_CFG_MAX_KEY_LEN_IN_BYTES - 1);
    kval.key[ADF_CFG_MAX_KEY_LEN_IN_BYTES - 1] = '\0';

    /* send the request down to get the configuration
     * information from kernel space. */
    res = ICP_IOCTL(fd, IOCTL_GET_CFG_VAL, &config);
    if (!res)
    {
        strncpy(pParamValue, kval.val, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
        status = CPA_STATUS_SUCCESS;
    }

    close_dev(accel_dev, fd);

    return status;
}

Cpa16U icp_adf_cfgGetBusAddress(Cpa16U packageId)
{
    icp_accel_dev_t *accel_dev = NULL;
    struct adf_dev_status_info dev_info = { 0 };
    int fd = -1;
    Cpa16U bdf = 0xFFFF;

    accel_dev = icp_adf_getAccelDevByAccelId(packageId);

    fd = open_dev(accel_dev);
    if (fd < 0)
    {
        return bdf;
    }

    dev_info.accel_id = packageId;
    if (!ICP_IOCTL(fd, IOCTL_STATUS_ACCEL_DEV, &dev_info))
    {
        /* Device bus address (B.D.F)
         * Bit 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
         *    |-BUS-------------------|-DEVICE-------|-FUNCT--|
         */
        bdf = dev_info.fun & 0x07;
        bdf |= (dev_info.dev & 0x1F) << 3;
        bdf |= (dev_info.bus & 0xFF) << 8;
    }

    close_dev(accel_dev, fd);

    return bdf;
}

enum ring_ioctl_ops
{
    RING_OP_RESERVE,
    RING_OP_RELEASE,
    RING_OP_ENABLE,
    RING_OP_DISABLE
};

static CpaStatus ring_ioctl(Cpa16U accel_id,
                            Cpa16U bank_nr,
                            Cpa16U ring_nr,
                            enum ring_ioctl_ops op)
{
    icp_accel_dev_t *accel_dev = NULL;
    CpaStatus status = CPA_STATUS_SUCCESS;
    struct adf_user_reserve_ring reserve = { 0 };
    int fd = -1;
    int res;

    accel_dev = icp_adf_getAccelDevByAccelId(accel_id);

    fd = open_dev(accel_dev);
    if (fd < 0)
    {
        return CPA_STATUS_FAIL;
    }

    reserve.accel_id = accel_id;
    reserve.bank_nr = bank_nr;
    reserve.ring_mask = 1 << ring_nr;

    switch (op)
    {
        case RING_OP_RESERVE:
            res = ICP_IOCTL(fd, IOCTL_RESERVE_RING, &reserve);
            break;
        case RING_OP_RELEASE:
            res = ICP_IOCTL(fd, IOCTL_RELEASE_RING, &reserve);
            break;
        case RING_OP_ENABLE:
            res = ICP_IOCTL(fd, IOCTL_ENABLE_RING, &reserve);
            break;
        case RING_OP_DISABLE:
            res = ICP_IOCTL(fd, IOCTL_DISABLE_RING, &reserve);
            break;
        default:
            ADF_ERROR("Error: invalid ring operation %d\n", op);
            status = CPA_STATUS_FAIL;
    }

    if (res == -1)
    {
        status = CPA_STATUS_FAIL;
    }

    close_dev(accel_dev, fd);

    return status;
}

CpaStatus icp_adf_reserve_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr)
{
    return ring_ioctl(accel_id, bank_nr, ring_nr, RING_OP_RESERVE);
}

CpaStatus icp_adf_release_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr)
{
    return ring_ioctl(accel_id, bank_nr, ring_nr, RING_OP_RELEASE);
}

CpaStatus icp_adf_enable_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr)
{
    return ring_ioctl(accel_id, bank_nr, ring_nr, RING_OP_ENABLE);
}

CpaStatus icp_adf_disable_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr)
{
    return ring_ioctl(accel_id, bank_nr, ring_nr, RING_OP_DISABLE);
}

CpaStatus icp_adf_getDevInfo(icp_accel_dev_t *accel_dev)
{
    CpaStatus status = CPA_STATUS_FAIL;
    struct adf_dev_status_info dev_info = { 0 };
    int fd = -1;
    int res = -1;

    ICP_CHECK_FOR_NULL_PARAM(accel_dev);

    fd = open_dev(accel_dev);
    if (fd < 0)
    {
        return CPA_STATUS_FAIL;
    }

    dev_info.accel_id = accel_dev->accelId;

    res = ICP_IOCTL(fd, IOCTL_STATUS_ACCEL_DEV, &dev_info);
    if (!res)
    {
        accel_dev->deviceType = (device_type_t)dev_info.type;
        accel_dev->pkg_id = dev_info.node_id;
        accel_dev->deviceMemAvail = dev_info.device_mem_available;
        accel_dev->sku = dev_info.sku;
        accel_dev->pciDevId = dev_info.pci_device_id;

        strncpy(
            accel_dev->deviceName, dev_info.name, ADF_DEVICE_TYPE_LENGTH - 1);

        status = CPA_STATUS_SUCCESS;
    }
    else
    {
        ADF_ERROR("Failed to get the type of the device \n");
    }

    close_dev(accel_dev, fd);

    return status;
}

static CpaStatus is_functionality_supported(int fd, int accelId)
{
    struct adf_dev_status_info dev_info = { 0 };

    dev_info.type = DEV_UNKNOWN;
    dev_info.accel_id = accelId;

    if (0 != ICP_IOCTL(fd, IOCTL_STATUS_ACCEL_DEV, &dev_info))
    {
        ADF_ERROR("Failed to get the type of the device\n");
        return CPA_STATUS_FAIL;
    }

    if (DEV_DH895XCCVF == dev_info.type || DEV_C62XVF == dev_info.type ||
        DEV_C3XXXVF == dev_info.type || DEV_C4XXXVF == dev_info.type ||
        DEV_200XXVF == dev_info.type)
    {
        return CPA_STATUS_UNSUPPORTED;
    }
    else
    {
        return CPA_STATUS_SUCCESS;
    }
}

/*
 * icp_adf_reset_device
 *
 * reset device - calls the IOCTL in
 * the driver which resets the device based on accelId
 *
 * In addition, if SAL is started, function calls subsystem_notify routine.
 * Subsystem notify routine is responsible for notifying subsystems across
 * the process.
 * NOTE: Call is skipped if SAL is not started.
 *
 */
CpaStatus icp_adf_reset_device(Cpa32U accelId)
{
    int fd = -1;
    CpaStatus status = CPA_STATUS_SUCCESS;
    struct adf_user_cfg_ctl_data ctl_data = { { 0 } };

    fd = ICP_OPEN(ADF_CTL_DEVICE_NAME, O_RDWR);
    if (fd < 0)
    {
        ADF_ERROR("Error: Could not open %s\n", ADF_CTL_DEVICE_NAME);
        return CPA_STATUS_FAIL;
    }

    status = is_functionality_supported(fd, accelId);

    if (CPA_STATUS_SUCCESS == status)
    {

        ctl_data.device_id = accelId;
        if (0 != ICP_IOCTL(fd, IOCTL_RESET_ACCEL_DEV, &ctl_data))
        {
            if (EBUSY == errno)
                ADF_ERROR("device busy\n");
            else
                ADF_ERROR("Failed to reset device\n");
            status = CPA_STATUS_FAIL;
        }
    }
    ICP_CLOSE(fd);
    return status;
}
