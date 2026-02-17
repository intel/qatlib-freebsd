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
