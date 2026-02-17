/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/

/**
 ***************************************************************************
 * @file icp_sal_dc_error_simulation.h
 *
 * @defgroup icpSalDcErrorSimulation DC Error Simulation API
 *
 * @ingroup icpSal
 *
 * @description
 *      Sal functions for DC Error Simulation.
 *      It contains the APIs under the macro ICP_DC_ERROR_SIMULATION.
 *
 ***************************************************************************/

#ifndef ICP_SAL_DC_ERROR_SIMULATION_H
#define ICP_SAL_DC_ERROR_SIMULATION_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ICP_DC_ERROR_SIMULATION

#include "cpa_dc.h"

/*
 * icp_sal_dc_simulate_error
 *
 * @description:
 *  This function injects a simulated compression error for a defined
 *  number of compression requests
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @param[in] numErrors              Num DC Errors
 *                                   0 - No Error injection
 *                                   1-0xFE - Num Errors to Inject
 *                                   0xFF - Always inject Error
 * @param[in] dcError                DC Error Type
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 */
CpaStatus icp_sal_dc_simulate_error(Cpa8U numErrors, Cpa8S dcError);

/*
 * icp_sal_cnv_simulate_error
 *
 * @description:
 *  This function enables the CnVError injection for the
 *  session passed in. All Compression requests sent within
 *  the session are injected with CnV errors. This error injection
 *  is for the duration of the session. Resetting the session
 *  results in setting being cleared.
 *  CnV error injection does not apply to Data Plane API.
 *
 * @note Only applies when compressAndVerify is on and
 *  compressAndVerifyAndRecover is off.
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      The session has been initialized via cpaDcInitSession function
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @param[in] dcInstance             Instance Handle
 * @param[in] pSessionHandle         Session Handle
 *
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported feature
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 *
 */
CpaStatus icp_sal_cnv_simulate_error(CpaInstanceHandle dcInstance,
                                     CpaDcSessionHandle pSessionHandle);

/*
 * icp_sal_ns_cnv_simulate_error
 *
 * @description:
 *  This function enables the CnVError injection for the
 *  No-Session case. All Compression requests sent
 *  to the dcInstance that is  passed in as a parameter,
 *  are injected with CnV errors. This CnV error injection
 *  does not apply to Data Plane API.
 *  This function is for GEN4 devices.
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @param[in] dcInstance             Instance Handle
 *
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported feature
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 *
 */
CpaStatus icp_sal_ns_cnv_simulate_error(CpaInstanceHandle dcInstance);

/*
 * icp_sal_ns_cnv_reset_error
 *
 * @description:
 *  This function resets the CnVError injection for the
 *  specific dcInstance that is passed in as a parameter
 *  for the No-Session operations.
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @param[in] dcInstance             Instance Handle
 *
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported feature
 * @retval CPA_STATUS_INVALID_PARAM  Invalid parameter passed in
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 *
 */
CpaStatus icp_sal_ns_cnv_reset_error(CpaInstanceHandle dcInstance);

#endif /* ICP_DC_ERROR_SIMULATION */

#ifdef __cplusplus
} /* close the extern "C" { */
#endif

#endif
