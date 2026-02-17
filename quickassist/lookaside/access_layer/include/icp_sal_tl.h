/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ****************************************************************************/
/****************************************************************************
 * @file icp_sal_tl.h
 *
 * @description
 *        This is list of Telemetry APIs. It contains function prototypes
 *        for managing QAT Telemetry on Intel(R) QuickAssist Technology.
 *
 ****************************************************************************/
#ifndef ICP_SAL_TL_H
#define ICP_SAL_TL_H

#include "cpa.h"
#include "adf_kernel_types.h"
#include "adf_user.h"

/*
 ******************************************************************
 * @ingroup SalUserTl
 *        Stop Telemetry for QAT device
 *
 * @description
 *        This function is used to stop the Telemetry feature for
 *        a specified QAT device.
 *
 * @param[in]  pPciAddr       Pointer to PCI address of QAT device
 *
 * @retval CPA_STATUS_SUCCESS         Operation successful
 * @retval CPA_STATUS_FAIL            Operation failed
 * @retval CPA_STATUS_UNSUPPORTED     Operation unsupported
 *
 ******************************************************************
 */
CpaStatus icp_sal_dev_telemetry_stop(struct adf_pci_address *pPciAddr);
/*
 ******************************************************************
 * @ingroup SalUserTl
 *        Start Telemetry for QAT device
 *
 * @description
 *        This function is used to start the Telemetry feature for
 *        a specified QAT device.
 *
 * @param[in]  pPciAddr       Pointer to PCI address of QAT device
 *
 * @retval CPA_STATUS_SUCCESS         Operation successful
 * @retval CPA_STATUS_FAIL            Operation failed
 * @retval CPA_STATUS_UNSUPPORTED     Operation unsupported
 *
 ******************************************************************
 */
CpaStatus icp_sal_dev_telemetry_start(struct adf_pci_address *pPciAddr);
#endif
