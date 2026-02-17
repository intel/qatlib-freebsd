/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#ifndef UIO_USER_CFG_H

#define UIO_USER_CFG_H

#include "icp_accel_devices.h"

CpaStatus icp_adf_cfgGetParamValue(icp_accel_dev_t *accel_dev,
                                   const char *section,
                                   const char *param,
                                   char *value);
Cpa16U icp_adf_cfgGetBusAddress(Cpa16U packageId);
CpaStatus icp_adf_getDevInfo(icp_accel_dev_t *accel_dev);
CpaStatus icp_adf_reserve_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr);
CpaStatus icp_adf_release_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr);
CpaStatus icp_adf_enable_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr);
CpaStatus icp_adf_disable_ring(Cpa16U accel_id, Cpa16U bank_nr, Cpa16U ring_nr);

#endif /* end of include guard: UIO_USER_CFG_H */
