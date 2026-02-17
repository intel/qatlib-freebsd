/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#ifndef ADF_USER_H

#define ADF_USER_H

#include <sys/types.h>
#include <sys/ioctl.h>
#include "icp_accel_devices.h"

#define MAX_DEVICE_NAME_SIZE 32
#define ADF_MAX_SERVICES 3
#define ADF_CFG_MAX_PROCESS_LEN 32

#define ADF_DH895XCC_DEVICE_NAME "dh895xcc"
#define ADF_DH895XCCVF_DEVICE_NAME "dh895xccvf"
#define ADF_C62X_DEVICE_NAME "c6xx"
#define ADF_C62XVF_DEVICE_NAME "c6xxvf"
#define ADF_C3XXX_DEVICE_NAME "c3xxx"
#define ADF_C3XXXVF_DEVICE_NAME "c3xxxvf"
#define ADF_200XX_DEVICE_NAME "200xx"
#define ADF_200XXVF_DEVICE_NAME "200xxvf"
#define ADF_C4XXX_DEVICE_NAME "c4xxx"
#define ADF_C4XXXVF_DEVICE_NAME "c4xxxvf"
#define ADF_4XXX_DEVICE_NAME "4xxx"
#define ADF_4XXXVF_DEVICE_NAME "4xxxvf"

#define ADF_DC_EXTENDED_FEATURES "Device_DcExtendedFeatures"
#define ADF_DEV_MAX_RINGS_PER_BANK "Device_Max_Rings_Per_Bank"
#define ADF_DEV_CAPABILITIES_MASK "Device_Capabilities_Mask"
#define ADF_GENERAL_SEC "GENERAL"
#define ADF_DEV_MAX_BANKS "Device_Max_Banks"

enum adf_device_type
{
    DEV_UNKNOWN = 0,
    DEV_DH895XCC,
    DEV_DH895XCCVF,
    DEV_C62X,
    DEV_C62XVF,
    DEV_C3XXX,
    DEV_C3XXXVF,
    DEV_200XX,
    DEV_200XXVF,
    DEV_C4XXX,
    DEV_C4XXXVF,
    DEV_D15XX,
    DEV_D15XXVF,
    DEV_4XXX,
    DEV_4XXXVF
};

enum adf_cfg_val_type
{
    ADF_DEC,
    ADF_HEX,
    ADF_STR
};

enum adf_device_heartbeat_status
{
    DEV_HB_UNRESPONSIVE = 0,
    DEV_HB_ALIVE,
    DEV_HB_UNSUPPORTED
};

struct adf_dev_heartbeat_status_ctl
{
    uint32_t device_id;
    enum adf_device_heartbeat_status status;
};

struct adf_dev_miscellaneous_stats
{
    uint64_t misc_counter;
};

struct adf_pci_address
{
    unsigned char bus;
    unsigned char dev;
    unsigned char func;
} __packed;

struct adf_dev_status_info
{
    enum adf_device_type type;
    uint16_t accel_id;
    uint16_t instance_id;
    uint8_t num_ae;
    uint8_t num_accel;
    uint8_t num_logical_accel;
    uint8_t banks_per_accel;
    uint8_t state;
    uint8_t bus;
    uint8_t dev;
    uint8_t fun;
    int domain;
    char name[MAX_DEVICE_NAME_SIZE];
    uint8_t sku;
    uint32_t node_id;
    uint32_t device_mem_available;
    uint32_t pci_device_id;
};

#define ADF_CTL_IOC_MAGIC 'a'
#define IOCTL_GET_NUM_DEVICES _IOR(ADF_CTL_IOC_MAGIC, 4, int32_t)
#define IOCTL_HEARTBEAT_ACCEL_DEV                                              \
    _IOWR(ADF_CTL_IOC_MAGIC, 9, struct adf_dev_heartbeat_status_ctl)
#ifdef QAT_HB_FAIL_SIM
#define IOCTL_HEARTBEAT_SIM_FAIL _IOW(ADF_CTL_IOC_MAGIC, 99, uint32_t)
#endif

#define IOCTL_STATUS_ACCEL_DEV                                                 \
    _IOWR(ADF_CTL_IOC_MAGIC, 3, struct adf_dev_status_info)
#define IOCTL_GET_CFG_VAL                                                      \
    _IOW(ADF_CTL_IOC_MAGIC, 5, struct adf_user_cfg_ctl_data)
#define IOCTL_RESET_ACCEL_DEV                                                  \
    _IOW(ADF_CTL_IOC_MAGIC, 10, struct adf_user_cfg_ctl_data)

struct adf_user_cfg_key_val
{
    char key[ADF_CFG_MAX_KEY_LEN_IN_BYTES];
    char val[ADF_CFG_MAX_VAL_LEN_IN_BYTES];
    union {
        struct adf_user_cfg_key_val *next;
        uint64_t padding3;
    };
    enum adf_cfg_val_type type;
};

struct adf_user_cfg_section
{
    char name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES];
    union {
        struct adf_user_cfg_key_val *params;
        uint64_t padding1;
    };
    union {
        struct adf_user_cfg_section *next;
        uint64_t padding3;
    };
};

struct adf_user_cfg_ctl_data
{
    union {
        struct adf_user_cfg_section *config_section;
        uint64_t padding;
    };
    uint32_t device_id;
};

struct adf_user_reserve_ring
{
    uint32_t accel_id;
    uint32_t bank_nr;
    uint32_t ring_mask;
};

#define IOCTL_RESERVE_RING                                                     \
    _IOWR(ADF_CTL_IOC_MAGIC, 10, struct adf_user_reserve_ring)
#define IOCTL_RELEASE_RING                                                     \
    _IOWR(ADF_CTL_IOC_MAGIC, 11, struct adf_user_reserve_ring)
#define IOCTL_ENABLE_RING                                                      \
    _IOWR(ADF_CTL_IOC_MAGIC, 12, struct adf_user_reserve_ring)
#define IOCTL_DISABLE_RING                                                     \
    _IOWR(ADF_CTL_IOC_MAGIC, 13, struct adf_user_reserve_ring)

struct adf_user_section_data
{
    char name[ADF_CFG_MAX_PROCESS_LEN];
    uint32_t device_id;
    uint8_t is_section_present;
};

int32_t adf_init_devices(void);
CpaStatus adf_proxy_get_devices(void);
int32_t adf_cleanup_devices(void);

#ifdef QAT_DBG

#define QATD_CMD_BSF_TO_ID (7)
#define QATD_CMD_ID_TO_BSF (8)

struct qatd_ioctl_bsf2id_req
{
    int domain;
    unsigned short bus;
    unsigned short dev;
    unsigned char func;
    int request_result;
    unsigned int device_id;
};

#define IOCTL_QATD_BSF_TO_ID                                                   \
    _IOWR(QATD_MAGIC, QATD_CMD_BSF_TO_ID, struct qatd_ioctl_bsf2id_req)
#define IOCTL_QATD_ID_TO_BSF                                                   \
    _IOWR(QATD_MAGIC, QATD_CMD_ID_TO_BSF, struct qatd_ioctl_bsf2id_req)

#endif

#endif /* end of include guard: ADF_USER_H */
