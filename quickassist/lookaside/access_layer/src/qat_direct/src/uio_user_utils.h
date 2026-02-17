/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#ifndef UIO_USER_UTILS_H

#define UIO_USER_UTILS_H

#define UIO_IS_ACCEL_STARTED(bit, status) (status & (0x1UL << bit))

#define UIO_SYS_NAME "name"
#define UIO_SYS_VERSION "version"
#define UIO_SYS_MAP_NAME "maps/map0/name"
#define UIO_SYS_MAP_SIZE "maps/map0/size"
#define UIO_SYS_MAP_OFFSET "maps/map0/offset"
#define UIO_SYS_MAP_ADDR "maps/map0/addr"
#define UIO_MAX_DIR_NAME_LENGTH 64

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#define UDEV_DEVICE_UIOCTL_BASE "uio_ctrl/"
#define UDEV_DEVICE_UIO_BASE "uio/"
#define UDEV_ATTRIBUTE_ACCELID UDEV_DEVICE_UIOCTL_BASE "accelid"
#define UDEV_ATTRIBUTE_FIRST_MINOR UDEV_DEVICE_UIOCTL_BASE "first_minor"
#define UDEV_ATTRIBUTE_LAST_MINOR UDEV_DEVICE_UIOCTL_BASE "last_minor"
#define UDEV_ATTRIBUTE_ACCEL_TYPE UDEV_DEVICE_UIOCTL_BASE "type"
#define UDEV_ATTRIBUTE_ACCEL_REVID UDEV_DEVICE_UIOCTL_BASE "revid"
#define UDEV_ATTRIBUTE_MMP_ADDR UDEV_DEVICE_UIOCTL_BASE "mmp_addr"
#define UDEV_ATTRIBUTE_NUM_BUNDLES UDEV_DEVICE_UIOCTL_BASE "num_bundles"
#define UDEV_ATTRIBUTE_USED_BUNDLES UDEV_DEVICE_UIOCTL_BASE "used_bundles"
#define UDEV_ATTRIBUTE_DEV_MINOR UDEV_DEVICE_UIOCTL_BASE "bundle_%d/dev_minor"
#define UDEV_ATTRIBUTE_DEV_NAME UDEV_DEVICE_UIOCTL_BASE "bundle_%d/dev_name"
#define UDEV_ATTRIBUTE_HARDWARE_BUNDLE                                         \
    UDEV_DEVICE_UIOCTL_BASE "bundle_%d/hardware_bundle"
#define UDEV_ATTRIBUTE_USED UDEV_DEVICE_UIOCTL_BASE "bundle_%d/used"
#define UDEV_ATTRIBUTE_USER_PID UDEV_DEVICE_UIOCTL_BASE "bundle_%d/user_pid"
#define UDEV_ATTRIBUTE_MAP0_ADDR UDEV_DEVICE_UIO_BASE "uio%d/maps/map0/addr"
#define UDEV_ATTRIBUTE_MAP0_NAME UDEV_DEVICE_UIO_BASE "uio%d/maps/map0/name"
#define UDEV_ATTRIBUTE_MAP0_OFFSET UDEV_DEVICE_UIO_BASE "uio%d/maps/map0/offset"
#define UDEV_ATTRIBUTE_MAP0_SIZE UDEV_DEVICE_UIO_BASE "uio%d/maps/map0/size"
#define UDEV_ATTRIBUTE_NODEID "numa_node"

struct udev_device;

CpaStatus uio_udev_get_device_from_devid(int devid, struct udev_device **dev);
CpaStatus uio_udev_ref(struct udev_device *dev);
CpaStatus uio_udev_free_device(struct udev_device *dev);
CpaStatus uio_udev_read_str(struct udev_device *dev,
                            char *value,
                            unsigned size,
                            const char *attribute,
                            ...);
CpaStatus uio_udev_read_long(struct udev_device *dev,
                             unsigned long *value,
                             const char *attribute,
                             ...);
CpaStatus uio_udev_read_uint(struct udev_device *dev,
                             unsigned int *value,
                             const char *attribute,
                             ...);

#endif /* end of include guard: UIO_USER_UTILS_H */
