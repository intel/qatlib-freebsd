/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ****************************************************************************/
/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "cpa.h"
#include "adf_kernel_types.h"
#include "adf_user_du.h"
#include "icp_sal_du.h"

CpaStatus icp_sal_userDuStart(struct adf_pci_address *pPf)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus icp_sal_userDuStop(struct adf_pci_address *pPf)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus icp_sal_userDuQuery(struct adf_pci_address *pPf,
                              enum adf_svc_type svcType,
                              Cpa32U *pCapacityInSlaUnit,
                              Cpa32U *pUtilInSlaUnit,
                              Cpa16U *pUtilInPct)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus icp_sal_userDuQueryVf(struct adf_pci_address *pPf,
                                struct adf_pci_address *pVf,
                                enum adf_svc_type svcType,
                                Cpa32U *pSlaInSlaUnit,
                                Cpa32U *pUtilInSlaUnit,
                                Cpa16U *pUtilInPct)
{
    return CPA_STATUS_UNSUPPORTED;
}
