/******************************************************************************
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
 *****************************************************************************/

/******************************************************************************
 * @file icp_adf_dbg_log.h
 *
 * @description
 *      This file contains the QAT logger utilities
 *
 *****************************************************************************/

#ifndef ICP_ADF_DBG_LOG_H_
#define ICP_ADF_DBG_LOG_H_

#include "cpa.h"

#include <pthread.h>

/******************************************************************************
 *                                  Defines                                   *
 *****************************************************************************/
#define QATD_DEVICE_FILENAME "/dev/qat_debug"
#define QATD_PROC_MMAP_FILE_NAME "proc.mmaps.dev"
#define QATD_MSG_PREAMBLE 0xacc00cca

#define QATD_DAEMON_KEY 0xACC0107
#define QATD_IPC_MSGTYPE 1
#define QATD_IPC_PERM 0666

#define QATD_INSTANCE_ENABLED 1
#define QATD_INSTANCE_DISABLED 0
#define QATD_PUT_RING_FULL -2
#define QATD_PUT_RING_NULL -3

/******************************************************************************
 *                               Public structures                            *
 *****************************************************************************/
typedef struct icp_adf_dbg_sync_msg_s
{
    long msg_type;
    int buffer_id;
} icp_adf_dbg_sync_msg_t;

typedef struct icp_adf_dbg_dev_handle_s
{
    /* QAT Device ID */
    unsigned int accel_id;
    /* File descriptor */
    int fd;
} icp_adf_dbg_dev_handle_t;

typedef struct icp_adf_dbg_handle_s
{
    icp_adf_dbg_dev_handle_t *dev_handle;
    int qat_dbg_enabled;
    unsigned int qat_dbg_level;
    unsigned int qat_dbg_buffer_size;
    void *qat_dbg_buffer_desc;
    int qat_dbg_daemon_msgq_id;
#ifndef ICP_WITHOUT_THREAD
    pthread_mutex_t *resp_api_lock;
#else
    void *resp_api_lock;
#endif
    /* Virt to phys callback */
    void *(*qat_dbg_phys2virt_client)(uintptr_t);
    int (*qat_dbg_sec_cleanup_cb)(uint8_t *);
} icp_adf_dbg_handle_t;

typedef struct icp_adf_dbg_hw_config_s
{
    union {
        struct
        {
            unsigned short cipherAlg;
            unsigned short cipherMode;
            unsigned short hashAlg;
            unsigned short hashMode;
        } s;
        struct
        {
            unsigned short compType;
            unsigned short huffType;
            unsigned short hash_algo;
        } s1;
    } u;
} icp_adf_dbg_hw_config_t;

typedef struct icp_adf_dbg_entry_header_s
{
    unsigned int preamble;
    /* Timestamp */
    unsigned long long ts;
    /* Ring number */
    unsigned short ring;
    /* Bank number */
    unsigned short bank;
    /* PID */
    unsigned int pid;
    /* Message type: FW REQ, FW RESP, APICALL */
    unsigned short msg_type;
    /* API Call */
    unsigned short api_type;
    /* Session data */
    icp_adf_dbg_hw_config_t content_desc;
    /* Message length */
    unsigned short msg_len;
    /* Source buffer SGL length */
    unsigned short src_sgl_len;
    /* Destination buffer SGL length */
    unsigned short dst_sgl_len;
    /* Miscellaneous data length */
    unsigned short misc_len;
} icp_adf_dbg_entry_header_t;

typedef struct icp_adf_dbg_content_desc_s
{
    /* Message type: FW REQ, FW RESP, APICALL */
    unsigned short msg_type;
    /* API Call */
    unsigned short api_type;
    /* Msg */
    void *msg;
    /* Message size */
    unsigned short msg_size;
    /* Source SGL pointer */
    void *src_sgl;
    /* Source buffer SGL size */
    unsigned short src_sgl_size;
    /* Source buffer SGL physical address */
    uint64_t src_phy_addr;
    /* Destination SGL pointer */
    void *dst_sgl;
    /* Destination buffer SGL size */
    unsigned short dst_sgl_size;
    /* Destination buffer SGL physical address */
    uint64_t dst_phy_addr;
    /* Ring number */
    unsigned short ring;
    /* Bank number */
    unsigned short bank;
    /* Misc */
    void *misc;
    /* Misc size */
    unsigned short misc_size;
    /* Session data */
    icp_adf_dbg_hw_config_t content_desc;
} icp_adf_dbg_content_desc_t;

/* Verbosity level */
enum icp_adf_dbg_level
{
    /* Collect no data */
    QATD_LEVEL_NO_COLLECT = 0,
    /* Collect only API calls */
    QATD_LEVEL_API_CALLS,
    /* Collect FW requests & responses only */
    QATD_LEVEL_FW_CALLS,
    /* Collect all above */
    QATD_LEVEL_ALL
};

/* Type of logged message */
enum icp_adf_dbg_message_type
{
    QATD_MSG_REQUEST = 0,
    QATD_MSG_REQUEST_DPDK,
    QATD_MSG_RESPONSE,
    QATD_MSG_APICALL
};

/********************************************************************/
/*                    QAT API call specific                         */
/********************************************************************/
enum icp_adf_dbg_api_type
{
    QATD_CPACYDHPHASE1KEYGENOPDATA = 0,
    QATD_CPACYDHPHASE2SECRETKEYGENOPDATA,
    QATD_CPACYDRBGGENOPDATA,
    QATD_CPACYDRBGRESEEDOPDATA,
    QATD_CPACYDSAGPARAMGENOPDATA,
    QATD_CPACYDSAPPARAMGENOPDATA,
    QATD_CPACYDSARSIGNOPDATA,
    QATD_CPACYDSARSSIGNOPDATA,
    QATD_CPACYDSASSIGNOPDATA,
    QATD_CPACYDSAVERIFYOPDATA,
    QATD_CPACYDSAYPARAMGENOPDATA,
    QATD_CPACYECDHPOINTMULTIPLYOPDATA,
    QATD_CPACYECDSASIGNROPDATA,
    QATD_CPACYECDSASIGNRSOPDATA,
    QATD_CPACYECDSASIGNSOPDATA,
    QATD_CPACYECDSAVERIFYOPDATA,
    QATD_CPACYECPOINTMULTIPLYOPDATA,
    QATD_CPACYECPOINTVERIFYOPDATA,
    QATD_CPACYECMONTEDWDSPOINTMULTIPLYOPDATA,
    QATD_CPACYKEYGENMGFOPDATA,
    QATD_CPACYKEYGENMGFOPDATAEXT,
    QATD_CPACYKEYGENSSLOPDATA,
    QATD_CPACYKEYGENTLSOPDATA,
    QATD_CPACYKEYGENHKDFOPDATA,
    QATD_CPACYKPTECDSASIGNRSOPDATA,
    QATD_CPACYLNMODEXPOPDATA,
    QATD_CPACYLNMODINVOPDATA,
    QATD_CPACYNRBGOPDATA,
    QATD_CPACYPRIMETESTOPDATA,
    QATD_CPACYRANDGENOPDATA,
    QATD_CPACYRANDSEEDOPDATA,
    QATD_CPACYRSADECRYPTOPDATA,
    QATD_CPACYRSAENCRYPTOPDATA,
    QATD_CPACYRSAKEYGENOPDATA,
    QATD_CPACYSYMDPOPDATA,
    QATD_CPACYSYMOPDATA,
    QATD_CPADCBATCHOPDATA,
    QATD_CPADCCHAINOPDATA,
    QATD_CPADCDPOPDATA,
    QATD_CPADCOPDATA,
    QATD_DPDK_SYM,
    QATD_DPDK_ASYM,
    QATD_DPDK_COMP
};

/******************************************************************************
 *                              Public functions                              *
 *****************************************************************************/

/*
 * icp_adf_log_req
 *
 * Description:
 * Log FW request
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_req(icp_adf_dbg_handle_t *handle,
                    icp_adf_dbg_content_desc_t *db_content_desc);

/*
 * icp_adf_log_resp
 *
 * Description:
 * Log FW response
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_resp(icp_adf_dbg_handle_t *handle,
                     unsigned short ring,
                     unsigned short bank,
                     void *msg,
                     unsigned short msg_size);

/*
 * icp_adf_log_apicall
 *
 * Description:
 * Log API call
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_apicall(icp_adf_dbg_handle_t *handle,
                        icp_adf_dbg_content_desc_t *db_content_desc);

/*
 * icp_adf_log_set_phys_to_virt_cb
 *
 * Description:
 * Set callback which is going to be used to translate
 * physical address to virtual
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_set_phys_to_virt_cb(icp_adf_dbg_handle_t *handle,
                                    void *(*qat_dbg_user_phys2virt)(uintptr_t));

/*
 * icp_adf_log_set_sec_cleanup_cb
 *
 * Description:
 * Set callback which is going to be used to clean up sensitive
 * content from FW request snapshot stored in crash dump or
 * continuous sync log files.
 */

void icp_adf_log_set_sec_cleanup_cb(icp_adf_dbg_handle_t *handle,
                                    int (*qat_dbg_sec_cleanup_cb)(uint8_t *));

/*
 * icp_adf_log_init_device
 *
 * Description:
 * Initialize QAT Debuggablity for device id
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_init_device(icp_adf_dbg_dev_handle_t *dev_handle,
                            unsigned int accel_id);

/*
 * icp_adf_log_deinit_device
 *
 * Description:
 * De-initialize QAT Debuggablity
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_deinit_device(icp_adf_dbg_dev_handle_t *dev_handle);

/*
 * icp_adf_log_init_handle
 *
 * Description:
 * Initialize QAT Debuggablity handle
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_init_handle(icp_adf_dbg_dev_handle_t *dev_handle,
                            icp_adf_dbg_handle_t *buffer_handle);

/*
 * icp_adf_log_deinit_handle
 *
 * Description:
 * De-initialize QAT Debuggablity handle
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_deinit_handle(icp_adf_dbg_handle_t *handle);

/*
 * icp_adf_log_is_enabled
 *
 * Description:
 * Check is QAT Debuggablity is enabled
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_is_enabled(icp_adf_dbg_handle_t *handle,
                           enum icp_adf_dbg_level level);

/*
 * icp_adf_log_slice_hang_notif
 *
 * Description:
 * Notifies QAT Debuggability about slice hang
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
int icp_adf_log_slice_hang_notify(icp_adf_dbg_handle_t *handle);
#endif /* ICP_ADF_DBG_LOG_H_ */
