/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#ifndef UIO_USER_BUNDLES_H

#define UIO_USER_BUNDLES_H

#include "icp_accel_devices.h"
int uio_control_mutex_init(void);
int uio_control_mutex_destroy(void);
struct adf_uio_user_bundle
{
    unsigned int device_minor;
    int fd;
    void *ptr;
    uint32_t size;
};

struct adf_uio_user_bundle *uio_get_bundle_from_accelid(int accelid,
                                                        int bundle_nr);
int uio_acces_dev_exist(int dev_id);
int uio_bundle_valid(struct adf_uio_user_bundle *bundle);
int uio_create_accel_dev(icp_accel_dev_t **accel_dev, int dev_id);

void uio_destroy_accel_dev(icp_accel_dev_t *accel_dev);
void uio_free_bundle(struct adf_uio_user_bundle *bundle);

#endif /* end of include guard: UIO_USER_BUNDLES_H */
