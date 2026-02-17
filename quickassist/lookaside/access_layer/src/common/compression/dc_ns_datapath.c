/****************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file dc_ns_datapath.c
 *
 * @defgroup Dc_DataCompression DC Data Compression
 *
 * @ingroup Dc_DataCompression
 *
 * @description
 *      Implementation of the Data Compression datapath operations.
 *
 *****************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/
#include <stdlib.h>
#include "cpa.h"
#include "cpa_dc.h"
#include "cpa_dc_dp.h"

/*
*******************************************************************************
* Include private header files
*******************************************************************************
*/
#include "dc_session.h"
#include "dc_datapath.h"
#include "sal_statistics.h"
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_log.h"
#include "sal_types_compression.h"
#include "dc_stats.h"
#include "lac_buffer_desc.h"
#include "lac_sal.h"
#include "lac_sync.h"
#include "sal_service_state.h"
#include "sal_qat_cmn_msg.h"
#ifdef ICP_DC_ERROR_SIMULATION
#include "dc_err_sim.h"
#endif
#include "dc_error_counter.h"
#include "dc_crc32.h"

CpaStatus cpaDcNsDecompressData(CpaInstanceHandle dcInstance,
                                CpaDcNsSetupData *pSetupData,
                                CpaBufferList *pSrcBuff,
                                CpaBufferList *pDestBuff,
                                CpaDcOpData *pOpData,
                                CpaDcRqResults *pResults,
                                CpaDcCallbackFn callbackFn,
                                void *callbackTag)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus cpaDcNsCompressData(CpaInstanceHandle dcInstance,
                              CpaDcNsSetupData *pSetupData,
                              CpaBufferList *pSrcBuff,
                              CpaBufferList *pDestBuff,
                              CpaDcOpData *pOpData,
                              CpaDcRqResults *pResults,
                              CpaDcCallbackFn callbackFn,
                              void *callbackTag)
{
    return CPA_STATUS_UNSUPPORTED;
}
