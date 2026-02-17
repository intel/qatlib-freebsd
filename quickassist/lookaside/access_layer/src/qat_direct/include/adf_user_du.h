/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#ifndef ADF_USER_DU_H

#define ADF_USER_DU_H

#include <sys/types.h>
#include <sys/ioctl.h>
#include "adf_user.h"

enum adf_svc_type
{
    ADF_SVC_ASYM = 0,
    ADF_SVC_SYM = 1,
    ADF_SVC_DC = 2,
    ADF_SVC_NONE = 3
};


/*
 * struct adf_user_du - to be used with:
 * IOCTL_DU_QUERY to get utilization for device.
 *
 * @pf_addr:            Bus Device Function of the PF
 * @vf_addr:            Bus Device Function of the VF
 * @svc_type:           service type to measure device utilization
 * @slau_supported:     for PF, it is total device capacity
 *                      for VF, it is user configured SLA
 * @slau_utilized:      returned value of utilization of given service
 * @slau_util_percent:  slau percentage computed against device capacity
 */
struct adf_user_du
{
    struct adf_pci_address pf_addr;
    struct adf_pci_address vf_addr;
    enum adf_svc_type svc_type;
    u16 slau_supported;
    u16 slau_utilized;
    u16 slau_util_percent;
};

#define IOCTL_DU_START _IOWR(ADF_CTL_IOC_MAGIC, 19, struct adf_pci_address)
#define IOCTL_DU_STOP _IOWR(ADF_CTL_IOC_MAGIC, 20, struct adf_pci_address)
#define IOCTL_DU_QUERY _IOWR(ADF_CTL_IOC_MAGIC, 21, struct adf_user_du)
#define IOCTL_DU_QUERY_VF _IOWR(ADF_CTL_IOC_MAGIC, 22, struct adf_user_du)

#define ADF_MAX_SLA 64

struct adf_user_sla
{
    struct adf_pci_address pci_addr;
    u16 sla_id;
    enum adf_svc_type svc_type;
    u16 rate_in_slau;
} __packed;

struct adf_user_service
{
    enum adf_svc_type svc_type;
    u16 max_svc_rate_in_slau;
    u16 avail_svc_rate_in_slau;
} __packed;

struct adf_user_sla_caps
{
    struct adf_pci_address pf_addr;
    u16 max_slas;
    u16 avail_slas;
    u16 used_slas;
    struct adf_user_service services[ADF_MAX_SERVICES];
} __packed;

struct adf_user_slas
{
    struct adf_pci_address pf_addr;
    struct adf_user_sla slas[ADF_MAX_SLA];
    u16 used_slas;
};

#define IOCTL_SLA_GET_CAPS                                                     \
    _IOWR(ADF_CTL_IOC_MAGIC, 14, struct adf_user_sla_caps)
#define IOCTL_SLA_CREATE _IOWR(ADF_CTL_IOC_MAGIC, 15, struct adf_user_sla)
#define IOCTL_SLA_UPDATE _IOWR(ADF_CTL_IOC_MAGIC, 16, struct adf_user_sla)
#define IOCTL_SLA_DELETE _IOWR(ADF_CTL_IOC_MAGIC, 17, struct adf_user_sla)
#define IOCTL_SLA_GET_LIST _IOWR(ADF_CTL_IOC_MAGIC, 18, struct adf_user_slas)

#endif /* end of include guard: ADF_USER_DU_H */
