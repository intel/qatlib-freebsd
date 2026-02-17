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
 * @file icp_sal.h
 *
 * @defgroup icpSal User Space and Kernel Space API
 *
 * @ingroup icpSal
 *
 * @description
 *      Functions for both user space and kernel space.
 *
 *      The two APIs in this file icp_sal_dc_simulate_error &
 *      icp_sal_get_dc_error have been moved to
 *      icp_sal_dc_error_simulation.h and icp_sal_user.h.
 *      To avoid build breakage, included icp_sal_dc_error_simulation.h &
 *      icp_sal_user.h in this header file.
 *      This header file will be deleted in future.
 ***************************************************************************/

#ifndef ICP_SAL_H
#define ICP_SAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "icp_sal_user.h"
#ifdef ICP_DC_ERROR_SIMULATION
#include "icp_sal_dc_error_simulation.h"
#endif

#ifdef __cplusplus
} /* close the extern "C" { */
#endif

#endif
