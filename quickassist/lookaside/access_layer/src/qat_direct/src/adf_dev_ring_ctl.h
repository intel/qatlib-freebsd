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
#ifndef ADF_DEV_RING_CTL_H
#define ADF_DEV_RING_CTL_H

#define OSAL_DEV_DRV_COMMON_H

#include <icp_accel_devices.h>
#include <icp_adf_init.h>
#include <icp_adf_transport.h>

#define EMPTY_RING_SIG_BYTE 0x7f
#define EMPTY_RING_SIG_WORD 0x7f7f7f7f

#define BYTESPERWORD 4

typedef struct adf_dev_bank_handle_s
{
    uint32_t accel_num;
    uint32_t bank_number;
    /*This is the offset the bank from the base csr address
     *(bank_size * bank_number)*/
    unsigned int bank_offset;
    uint32_t interrupt_mask;
    uint32_t pollingMask;
    void *user_bank_lock;
    uint32_t timed_coalesc_enabled : 1;
    uint32_t number_msg_coalesc_enabled : 1;

    /*!timed_coalesc_enabled || number_msg_coalesc_enabled*/
    unsigned int flag_interrupts_enabled : 1;
    unsigned int is_intflagandcolen_csr_available : 1;

    uint16_t tx_rings_mask;
    uint16_t ring_mask; /*enabled rings*/
    uint32_t *csr_addr;
    void *bundle;
    /* this the ring handle for this banks  */
    struct adf_dev_ring_handle_s **rings;
    /* The maximum number of rings available per bank */
    uint32_t max_num_rings;
    int refs;
} adf_dev_bank_handle_t;

typedef struct adf_dev_ring_handle_s
{
    icp_accel_dev_t *accel_dev;
    icp_transport_type trans_type;
    char *service_name;
    uint32_t service_name_len;
    char *section_name;
    uint32_t section_name_len;
    uint32_t accel_num;
    uint32_t bank_num;
    /* This is the offset the bank from the base csr address
     * (bank_size * bank_number) */
    uint32_t bank_offset;
    uint32_t ring_num;
    uint32_t ring_size;
    uint32_t message_size;

    icp_adf_ringInfoService_t info;
    icp_trans_callback callback;
    icp_resp_deliv_method resp;

    /* Result Parameters */
    void *ring_virt_addr;
    uint64_t ring_phys_base_addr;
    uint32_t interrupt_user_mask;
    uint32_t pollingMask;
    uint32_t timed_coalesc_enabled : 1;
    uint32_t number_msg_coalesc_enabled : 1;
    /* !timed_coalesc_enabled || number_msg_coalesc_enabled */
    unsigned int flag_interrupts_enabled : 1;
    unsigned int is_intflagandcolen_csr_available : 1;
    uint32_t is_wireless : 1;
    uint32_t is_dyn : 1;
    adf_dev_bank_handle_t *bank_data;

    /* userspace shadow values */
    void *user_lock;
    uint32_t head;
    uint32_t tail;
    uint32_t space;
    uint32_t modulo;
    uint32_t ringResponseQuota;
    int64_t pollingInProgress;
    Cpa32U *in_flight;
    uint32_t max_requests_inflight;
    uint32_t coal_resps_write_count;
    uint32_t min_resps_per_head_write;
    uint32_t coal_rqsts_write_count;
    uint32_t min_rqsts_per_tail_write;
    /* the offset  of the actual csr tail */
    uint32_t csrTailOffset;

    uint32_t *csr_addr;
} adf_dev_ring_handle_t;


#endif /*ADF_DEV_RING_CTL_H*/
