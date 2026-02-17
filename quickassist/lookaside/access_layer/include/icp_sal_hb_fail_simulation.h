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
 * @file icp_sal_hb_fail_simulation.h
 *
 * @defgroup icpSalHbFailSimulation Heartbeat Failure Simulation API
 *
 * @ingroup icpSal
 *
 * @description
 *      Sal functions for heartbeat failure simulation.
 *      It contains the APIs under the macro ICP_HB_FAIL_SIM.
 *
 ***************************************************************************/

#ifndef ICP_SAL_HB_FAIL_SIMULATION_H
#define ICP_SAL_HB_FAIL_SIMULATION_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ICP_HB_FAIL_SIM

#include "cpa.h"

/*
 * icp_sal_heartbeat_simulate_failure
 *
 * @description:
 *  This function simulates a heartbeat failure
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      This along with a icp_sal_check call will notify the heartbeat
 *      error to user space
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[in] packageId              The package Id can be found by calling
 *                                   cpaCyInstanceGetInfo2() or
 *                                   cpaDcInstanceGetInfo2().
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 */
CpaStatus icp_sal_heartbeat_simulate_failure(Cpa32U packageId);

#endif /* ICP_HB_FAIL_SIM */

#ifdef __cplusplus
} /* close the extern "C" { */
#endif

#endif
