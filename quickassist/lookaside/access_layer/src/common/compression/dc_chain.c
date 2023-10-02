/****************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2023 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2023 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *
 ***************************************************************************/

/**
 *****************************************************************************
 * @file dc_chain.c
 *
 * @ingroup Dc_Chaining
 *
 * @description
 *      Implementation of the chaining session operations.
 *
 *****************************************************************************/

/*
 *******************************************************************************
 * Include public/global header files
 *******************************************************************************
 */
#ifndef ICP_DC_ONLY
#include "cpa.h"

#include "icp_qat_fw.h"
#include "icp_qat_fw_comp.h"
#include "icp_qat_hw.h"

/*
 *******************************************************************************
 * Include private header files
 *******************************************************************************
 */
#include "sal_types_compression.h"
#include "cpa_dc_chain.h"
#include "lac_session.h"
#include "dc_session.h"
#include "dc_datapath.h"
#include "dc_stats.h"
#include "lac_mem_pools.h"
#include "lac_log.h"
#include "sal_types_compression.h"
#include "lac_buffer_desc.h"
#include "sal_service_state.h"
#include "sal_qat_cmn_msg.h"
#include "lac_sym_qat_hash_defs_lookup.h"
#include "sal_string_parse.h"
#include "lac_sym.h"
#include "lac_session.h"
#include "lac_sym_qat.h"
#include "lac_sym_hash.h"
#include "lac_sym_alg_chain.h"
#include "lac_sym_auth_enc.h"

CpaStatus cpaDcChainGetSessionSize(CpaInstanceHandle dcInstance,
                                   CpaDcChainOperations operation,
                                   Cpa8U numSessions,
                                   CpaDcChainSessionSetupData *pSessionData,
                                   Cpa32U *pSessionSize)

{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus cpaDcChainInitSession(CpaInstanceHandle dcInstance,
                                CpaDcSessionHandle pSessionHandle,
                                CpaDcChainOperations operation,
                                Cpa8U numSessions,
                                CpaDcChainSessionSetupData *pSessionData,
                                CpaDcCallbackFn callbackFn)

{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus cpaDcChainSetCrcControlData(CpaInstanceHandle dcInstance,
                                      CpaDcSessionHandle pSessionHandle,
                                      CpaCrcControlData *pCrcControlData)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus cpaDcChainRemoveSession(const CpaInstanceHandle dcInstance,
                                  CpaDcSessionHandle pSessionHandle)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus cpaDcChainResetSession(const CpaInstanceHandle dcInstance,
                                 CpaDcSessionHandle pSessionHandle)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus cpaDcChainPerformOp(CpaInstanceHandle dcInstance,
                              CpaDcSessionHandle pSessionHandle,
                              CpaBufferList *pSrcBuff,
                              CpaBufferList *pDestBuff,
                              CpaDcChainOperations operation,
                              Cpa8U numOpDatas,
                              CpaDcChainOpData *pChainOpData,
                              CpaDcChainRqResults *pResults,
                              void *callbackTag)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus cpaDcChainPerformOp2(CpaInstanceHandle dcInstance,
                               CpaDcSessionHandle pSessionHandle,
                               CpaBufferList *pSrcBuff,
                               CpaBufferList *pDestBuff,
                               CpaBufferList *pInterBuff,
                               CpaDcChainOpData2 opData,
                               CpaDcChainRqVResults *pResults,
                               void *callbackTag)
{
    return CPA_STATUS_UNSUPPORTED;
}
#endif
