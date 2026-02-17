/******************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
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
#include "icp_accel_devices.h"
#include "icp_adf_dbg_log.h"

typedef void *lac_comms_trans_handle;
/**< @ingroup LacLogger
 *      Transport (ring) handle type definiton as pointer. Defined to include
 *      only required header files.
 */
typedef void *lac_dbg_op_data_comms;
typedef void *lac_dbg_session_handle;
typedef void *lac_dbg_dc_session_handle;

/* Traditional API */
CpaStatus LacLogger_prepFWReq_trad(lac_comms_trans_handle trans_handle,
                                   icp_adf_dbg_content_desc_t *dbg_desc,
                                   void *in_buf);

/* Data-plane API */
CpaStatus LacLogger_logDbgReq_cydp(lac_comms_trans_handle trans_handle,
                                   lac_dbg_session_handle session_handle,
                                   void *in_buf);

CpaStatus LacLogger_logDbgReq_dcdp(lac_comms_trans_handle trans_handle,
                                   lac_dbg_dc_session_handle session_handle,
                                   void *in_buf);

void LacLogger_WriteRingRollBack_dp(lac_comms_trans_handle trans_handle,
                                    Cpa32U numRequestsPlaced,
                                    Cpa32U numInFlightsAdded);

/* API calls */
CpaStatus LacLogger_logApiCall(CpaInstanceHandle instanceHandle,
                               enum icp_adf_dbg_api_type api_type,
                               lac_dbg_op_data_comms op_data,
                               Cpa16U op_data_size);

/* Misc */
CpaBoolean LacLogger_isEnabled(lac_comms_trans_handle trans_handle,
                               enum icp_adf_dbg_level log_level);

CpaStatus LacLogger_sliceHangNotify(lac_comms_trans_handle trans_handle);

/* Default security cleanup callback */
int LacLogger_secCleanupImpl(uint8_t *qat_msg);

#endif /* LAC_LOGGER_UTILS_H_ */
