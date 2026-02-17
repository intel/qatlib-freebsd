/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#ifndef UIO_USER_H

#define UIO_USER_H

#include "uio_user_shared_mutex.h"
#include "uio_user_bundles.h"

int32_t adf_init_devices(void);
CpaStatus adf_proxy_get_devices(void);
int32_t adf_cleanup_devices(void);
CpaStatus subsystem_notify(icp_accel_dev_t *accel_dev, Cpa32U event);
int adf_proxy_poll_event(Cpa32U *dev_id, enum adf_event *event);
#endif /* end of include guard: UIO_USER_H */
