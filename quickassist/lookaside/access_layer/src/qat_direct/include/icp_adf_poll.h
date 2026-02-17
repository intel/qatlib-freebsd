/*****************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 *****************************************************************************/

/*****************************************************************************
 * @file icp_adf_poll.h
 *
 * @description
 *      File contains Public API Definitions for the polling method.
 *
 *****************************************************************************/
#ifndef ICP_ADF_POLL_H
#define ICP_ADF_POLL_H

#include "cpa.h"
/*
 * icp_adf_pollInstance
 *
 * Description:
 * Poll an instance. In order to poll an instance
 * sal will pass in a table of trans handles from which
 * the ring to be polled can be obtained and subsequently
 * polled.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on polling a ring with data
 *   CPA_STATUS_FAIL      on failure
 *   CPA_STATUS_RETRY     if ring has no data on it
 *                        or ring is already being polled.
 */
CpaStatus icp_adf_pollInstance(icp_comms_trans_handle *trans_hnd,
                               Cpa32U num_transHandles,
                               Cpa32U response_quota);

#endif /* ICP_ADF_POLL_H */
