/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ****************************************************************************/
#include "adf_kernel_types.h"
#include "adf_user.h"
#include "icp_sal_tl.h"

/* This function stops the telemetry feature */
CpaStatus icp_sal_dev_telemetry_stop(struct adf_pci_address *pPciAddr)
{
    return CPA_STATUS_UNSUPPORTED;
}

/* This function starts the telemetry feature */
CpaStatus icp_sal_dev_telemetry_start(struct adf_pci_address *pPciAddr)
{
    return CPA_STATUS_UNSUPPORTED;
}
