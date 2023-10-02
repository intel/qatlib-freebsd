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
    u32 device_id;
    u8 is_section_present;
};

int32_t adf_init_devices(void);
CpaStatus adf_proxy_get_devices(void);
int32_t adf_cleanup_devices(void);

#endif /* end of include guard: ADF_USER_H */
