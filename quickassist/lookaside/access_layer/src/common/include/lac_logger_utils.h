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
 * @file lac_logger_utils.h
 *
 * @defgroup LacLogger
 *
 * @ingroup LacCommon
 *
 * Helper functions for QAT logging
 *
 *****************************************************************************/

#ifndef LAC_LOGGER_UTILS_H_
#define LAC_LOGGER_UTILS_H_

/******************************************************************************
 *                   Include public/global header files                       *
 *****************************************************************************/
#include "cpa.h"
#include "lac_common.h"
#include "icp_accel_devices.h"
#include "icp_adf_transport.h"
#include "icp_adf_dbg_log.h"

typedef void *lac_dbg_op_data_comms;
typedef void *lac_dbg_session_handle;
typedef void *lac_dbg_dc_session_handle;

/* Traditional API */
CpaStatus LacLogger_prepFWReq_trad(icp_comms_trans_handle trans_handle,
                                   icp_adf_dbg_content_desc_t *dbg_desc,
                                   void *in_buf);

/* Data-plane API */
CpaStatus LacLogger_logDbgReq_cydp(icp_comms_trans_handle trans_handle,
                                   lac_dbg_session_handle session_handle,
                                   void *in_buf);

CpaStatus LacLogger_logDbgReq_dcdp(icp_comms_trans_handle trans_handle,
                                   lac_dbg_dc_session_handle session_handle,
                                   void *in_buf);

void LacLogger_WriteRingRollBack_dp(icp_comms_trans_handle trans_handle,
                                    Cpa32U numRequestsPlaced,
                                    Cpa32U numInFlightsAdded);

/* API calls */
CpaStatus LacLogger_logApiCall(CpaInstanceHandle instanceHandle,
                               enum icp_adf_dbg_api_type api_type,
                               lac_dbg_op_data_comms op_data,
                               Cpa16U op_data_size);

/* Misc */
CpaBoolean LacLogger_isEnabled(icp_comms_trans_handle trans_handle,
                               enum icp_adf_dbg_level log_level);

CpaStatus LacLogger_setPhys2VirtCallback(
    CpaInstanceHandle instanceHandle,
    void *(*qat_dbg_user_phys2virt)(uintptr_t));

CpaStatus LacLogger_sliceHangNotify(icp_comms_trans_handle trans_handle);

/* Default security cleanup callback */
int LacLogger_secCleanupImpl(uint8_t *qat_msg);

#endif /* LAC_LOGGER_UTILS_H_ */
