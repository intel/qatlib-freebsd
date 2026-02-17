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
 * @file icp_sal_ring_exception_simulation.h
 *
 * @defgroup icpSalRingExceptionSimulation Ring Exception Simulation API
 *
 * @ingroup icpSal
 *
 * @description
 *      Sal functions for ring exception simulation.
 *      It contains the APIs under the macro ICP_RING_EXCEPTION_SIMULATION.
 *
 ***************************************************************************/

#ifndef ICP_SAL_RING_EXCEPTION_SIMULATION_H
#define ICP_SAL_RING_EXCEPTION_SIMULATION_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ICP_RING_EXCEPTION_SIM

#include "cpa.h"

/*
 * icp_sal_ring_exception_simulate
 *
 * @description:
 *  This function simulates a ring exception
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      This causes ring halt that could be cleared
 *      only by a RP reset
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[in] ring                   Ring handle
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 *
 */
CpaStatus icp_sal_ring_exception_simulate(void *ring);

/*
 * icp_sal_ring_exception_stop_simulate
 *
 * @description:
 *  This function stops a ring exception simulation
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[in] accelId                Ring handle
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 *
 */
CpaStatus icp_sal_ring_exception_stop_simulate(void *ring);

/*
 * icp_sal_rp_exception_is_set
 *
 * @description:
 *  This function checks if ring exception bit is set
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[in] accelId                Ring handle
 * @retval CPA_STATUS_SUCCESS        ring exception bit is set
 * @retval CPA_STATUS_FAIL           ring exception bit is not set
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 *
 */
CpaStatus icp_sal_rp_exception_is_set(void *ring);

/*
 * icp_sal_check_ring_error
 *
 * @description:
 * This function checks if ring exception or ring halt
 * errors occur.
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[in] accelId    Ring handle
 * @retval 0             no ring errors or unsupported function
 * @retval -EINTR        ring exception occurred
 * @retval -EL2HLT       ring halt occurred
 * @retval -EFAULT       RINGSTAT rp_exception/rp_halt is not supported
 */
Cpa32S icp_sal_check_ring_error(void *ring);

#endif /* ICP_RING_EXCEPTION_SIM */

#ifdef __cplusplus
} /* close the extern "C" { */
#endif

#endif
