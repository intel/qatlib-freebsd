/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#include <icp_accel_devices.h>
#include <icp_platform.h>

CpaStatus icp_adf_getDynInstance(icp_accel_dev_t *accel_dev,
                                 adf_service_type_t stype,
                                 Cpa32U *pinstance_id)
{
    return CPA_STATUS_FAIL;
}

CpaStatus icp_adf_putDynInstance(icp_accel_dev_t *accel_dev,
                                 adf_service_type_t stype,
                                 Cpa32U instance_id)
{
    return CPA_STATUS_FAIL;
}

CpaStatus icp_adf_getNumAvailDynInstance(icp_accel_dev_t *accel_dev,
                                         adf_service_type_t stype,
                                         Cpa32U *num)
{
    *num = 0;
    return CPA_STATUS_FAIL;
}
