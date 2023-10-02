/***************************************************************************
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
 ***************************************************************************
 *
 * @file lac_ecsm2.c
 *
 * @ingroup Lac_Ecsm2
 *
 * SM2 functions
 * SM2 algorithm is using a fixed EC curve.
 * The length of the params is fixed to LAC_EC_SM2_SIZE_BYTES(32 bytes).
 * More details in http://tools.ietf.org/html/draft-shen-sm2-ecdsa-02
 *
 * @lld_start
 *
 * @lld_overview
 * This file implements SM2 api functions.
 * @lld_dependencies
 * - \ref LacAsymCommonQatComms "PKE QAT Comms" : For creating and sending
 * messages to the QAT
 * - \ref LacMem "Mem" : For memory allocation and freeing, and translating
 * between scalar and pointer types
 * - OSAL : For atomics and logging
 *
 * @lld_initialisation
 * On initialization this component clears the stats.
 *
 * @lld_module_algorithms
 *
 * @lld_process_context
 *
 * @lld_end
 *
 ***************************************************************************/

/*
 * ****************************************************************************
 * * Include public/global header files
 * ****************************************************************************
 * */
/* API Includes */
#include "cpa.h"
#include "cpa_cy_im.h"
#include "cpa_cy_ecsm2.h"

/* OSAL Includes */
#include "Osal.h"

/* ADF Includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/* QAT includes */
#include "icp_qat_fw_la.h"
#include "icp_qat_fw_mmp.h"
#include "icp_qat_fw_mmp_ids.h"
#include "icp_qat_fw_pke.h"

/* Look Aside Includes */
#include "lac_log.h"
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_pke_utils.h"
#include "lac_pke_qat_comms.h"
#include "lac_sync.h"
#include "lac_ec.h"
#include "lac_list.h"
#include "sal_service_state.h"
#include "lac_sal_types_crypto.h"
#include "sal_statistics.h"

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 point multiplication operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2PointMultiply(
    const CpaInstanceHandle instanceHandle_in,
    const CpaCyEcPointMultiplyCbFunc pEcsm2PointMulCb,
    void *pCallbackTag,
    const CpaCyEcsm2PointMultiplyOpData *pEcsm2PointMulOpData,
    CpaBoolean *pMultiplyStatus,
    CpaFlatBuffer *pXk,
    CpaFlatBuffer *pYk)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 point generator multiplication operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2GeneratorMultiply(
    const CpaInstanceHandle instanceHandle_in,
    const CpaCyEcPointMultiplyCbFunc pEcsm2GenMulCb,
    void *pCallbackTag,
    const CpaCyEcsm2GeneratorMultiplyOpData *pEcsm2GenMulOpData,
    CpaBoolean *pMultiplyStatus,
    CpaFlatBuffer *pXk,
    CpaFlatBuffer *pYk)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 point verify operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2PointVerify(
    const CpaInstanceHandle instanceHandle_in,
    const CpaCyEcPointVerifyCbFunc pEcsm2PointVeirfyCb,
    void *pCallbackTag,
    const CpaCyEcsm2PointVerifyOpData *pEcsm2PointVerifyOpData,
    CpaBoolean *pPointVerifyStatus)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 signature operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2Sign(const CpaInstanceHandle instanceHandle_in,
                         const CpaCyEcsm2SignCbFunc pEcsm2SignCb,
                         void *pCallbackTag,
                         const CpaCyEcsm2SignOpData *pEcsm2SignOpData,
                         CpaBoolean *pSignStatus,
                         CpaFlatBuffer *pR,
                         CpaFlatBuffer *pS)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 signature verify operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2Verify(const CpaInstanceHandle instanceHandle_in,
                           const CpaCyEcsm2VerifyCbFunc pEcsm2VerifyCb,
                           void *pCallbackTag,
                           const CpaCyEcsm2VerifyOpData *pEcsm2VerifyOpData,
                           CpaBoolean *pVerifyStatus)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 encryption operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2Encrypt(const CpaInstanceHandle instanceHandle_in,
                            const CpaCyGenFlatBufCbFunc pEcsm2EncCb,
                            void *pCallbackTag,
                            const CpaCyEcsm2EncryptOpData *pEcsm2EncOpData,
                            CpaCyEcsm2EncryptOutputData *pEcsm2EncOutputData)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 point decryption operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2Decrypt(const CpaInstanceHandle instanceHandle_in,
                            const CpaCyGenFlatBufCbFunc pEcsm2DecCb,
                            void *pCallbackTag,
                            const CpaCyEcsm2DecryptOpData *pEcsm2DecOpData,
                            CpaCyEcsm2DecryptOutputData *pEcsm2DecOutputData)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 key exchange phase 1 operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2KeyExPhase1(
    const CpaInstanceHandle instanceHandle_in,
    const CpaCyGenFlatBufCbFunc pEcsm2KeyExPhase1Cb,
    void *pCallbackTag,
    const CpaCyEcsm2KeyExPhase1OpData *pEcsm2KeyExPhase1OpData,
    CpaCyEcsm2KeyExOutputData *pEcsm2KeyExPhase1OutputData)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ecsm2
 *
 * @description
 *     SM2 key exchange phase 2 operation
 *
 ***************************************************************************/
CpaStatus cpaCyEcsm2KeyExPhase2(
    const CpaInstanceHandle instanceHandle_in,
    const CpaCyGenFlatBufCbFunc pEcsm2KeyExPhase2Cb,
    void *pCallbackTag,
    const CpaCyEcsm2KeyExPhase2OpData *pEcsm2KeyExPhase2OpData,
    CpaCyEcsm2KeyExOutputData *pEcsm2KeyExPhase2OutputData)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus cpaCyEcsm2QueryStats64(const CpaInstanceHandle instanceHandle_in,
                                 CpaCyEcsm2Stats64 *pEcsm2Stats)
{
    return CPA_STATUS_UNSUPPORTED;
}
