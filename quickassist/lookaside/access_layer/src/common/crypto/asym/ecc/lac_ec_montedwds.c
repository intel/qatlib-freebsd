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
 * @file lac_ec_montedwds.c
 *
 * @ingroup Lac_Ec_MontEdwds
 *
 * Elliptic Curve Montgomery and Edwards functions
 *
 * @lld_start
 *
 * @lld_overview
 * This file implements Elliptic Curve Montgomery and Edwards api functions.
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
****************************************************************************
* Include public/global header files
****************************************************************************
*/

/* API Includes */
#include "cpa.h"
#include "cpa_cy_ec.h"
#include "cpa_cy_im.h"

/* OSAL Includes */
#include "Osal.h"

/* ADF Includes */
#include "icp_adf_init.h"
#include "icp_adf_transport.h"
#include "icp_accel_devices.h"
#include "icp_adf_debug.h"

/* QAT FW includes */
#include "icp_qat_fw_la.h"
#include "icp_qat_fw_mmp.h"
#include "icp_qat_fw_mmp_ids.h"

/* Look Aside Includes */
#include "lac_log.h"
#include "lac_common.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_pke_utils.h"
#include "lac_pke_qat_comms.h"
#include "lac_sync.h"
#include "lac_ec.h"
#include "lac_sal_types.h"
#include "lac_sal.h"
#include "lac_list.h"
#include "lac_sym_qat.h"
#include "lac_sal_types_crypto.h"
#include "lac_sal_ctrl.h"
#include "sal_service_state.h"
#include "sal_statistics.h"

/*
****************************************************************************
* Define static function definitions
****************************************************************************
*/

#ifdef ICP_PARAM_CHECK
/**
 ***************************************************************************
 * @ingroup Lac_Ec_MontEdwds
 *      EC_MONTEDWDS Point Multiply function to perform basic checks on the
 *      IN parameters (e.g. checks data buffers for NULL and 0 dataLen)
 ***************************************************************************/
STATIC CpaStatus LacEcMontEdwds_PointMultiplyBasicParamCheck(
    const CpaInstanceHandle instanceHandle,
    const CpaCyEcMontEdwdsPointMultiplyOpData *pOpData,
    const CpaBoolean *pMultiplyStatus,
    const CpaFlatBuffer *pXk,
    const CpaFlatBuffer *pYk)
{
    /* Check for null parameters */
    LAC_CHECK_NULL_PARAM(pOpData);
    LAC_CHECK_NULL_PARAM(pMultiplyStatus);

    switch (pOpData->curveType)
    {
        case CPA_CY_EC_MONTEDWDS_CURVE25519_TYPE:
            if (pOpData->generator)
            {
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->k, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(pXk, CHECK_EQUALS, 32);
            }
            else
            {
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->k, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->x, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(pXk, CHECK_EQUALS, 32);
            }
            break;
        case CPA_CY_EC_MONTEDWDS_ED25519_TYPE:
            if (pOpData->generator)
            {
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->k, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(pXk, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(pYk, CHECK_EQUALS, 32);
            }
            else
            {
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->k, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->x, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->y, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(pXk, CHECK_EQUALS, 32);
                LAC_CHECK_FLAT_BUFFER_PARAM(pYk, CHECK_EQUALS, 32);
            }
            break;
        case CPA_CY_EC_MONTEDWDS_CURVE448_TYPE:
            if (pOpData->generator)
            {
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->k, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(pXk, CHECK_EQUALS, 64);
            }
            else
            {
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->k, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->x, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(pXk, CHECK_EQUALS, 64);
            }
            break;
        case CPA_CY_EC_MONTEDWDS_ED448_TYPE:
            if (pOpData->generator)
            {
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->k, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(pXk, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(pYk, CHECK_EQUALS, 64);
            }
            else
            {
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->k, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->x, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(&pOpData->y, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(pXk, CHECK_EQUALS, 64);
                LAC_CHECK_FLAT_BUFFER_PARAM(pYk, CHECK_EQUALS, 64);
            }
            break;
        default:
            return CPA_STATUS_INVALID_PARAM;
    }

    return CPA_STATUS_SUCCESS;
}
#endif

/**
 ***************************************************************************
 * @ingroup Lac_Ec_MontEdwds
 *      EC_MONTEDWDS Point Multiply synchronous function
 ***************************************************************************/
STATIC CpaStatus LacEcMontEdwds_PointMultiplySyn(
    const CpaInstanceHandle instanceHandle,
    const CpaCyEcMontEdwdsPointMultiplyOpData *pOpData,
    CpaBoolean *pMultiplyStatus,
    CpaFlatBuffer *pXk,
    CpaFlatBuffer *pYk)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    lac_sync_op_data_t *pSyncCallbackData = NULL;
    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;

    status = LacSync_CreateSyncCookie(&pSyncCallbackData);

    /* Call the asynchronous version of the function
     * with the generic synchronous callback function as a parameter.
     */
    if (CPA_STATUS_SUCCESS == status)
    {
        status = cpaCyEcMontEdwdsPointMultiply(instanceHandle,
                                               LacSync_GenDualFlatBufVerifyCb,
                                               pSyncCallbackData,
                                               pOpData,
                                               pMultiplyStatus,
                                               pXk,
                                               pYk);
    }
    else
    {
        LAC_EC_STAT_INC(numEcPointMultiplyRequestErrors, pCryptoService);
        return status;
    }

    if (CPA_STATUS_SUCCESS == status)
    {
        CpaStatus wCbStatus = CPA_STATUS_FAIL;
        wCbStatus = LacSync_WaitForCallback(pSyncCallbackData,
                                            LAC_PKE_SYNC_CALLBACK_TIMEOUT,
                                            &status,
                                            pMultiplyStatus);

        if (CPA_STATUS_SUCCESS != wCbStatus)
        {
            LAC_EC_STAT_INC(numEcPointMultiplyCompletedError, pCryptoService);
            status = wCbStatus;
        }
    }
    else
    {
        /* As the Request was not sent the Callback will never
         * be called, so need to indicate that we're finished
         * with cookie so it can be destroyed.
         */
        LacSync_SetSyncCookieComplete(pSyncCallbackData);
    }

    LacSync_DestroySyncCookie(&pSyncCallbackData);
    return status;
}

/**
 ***************************************************************************
 * @ingroup Lac_Ec_MontEdwds
 *      EC MONTEDWDS Point Multiply internal callback
 ***************************************************************************/
STATIC void LacEcMontEdwds_PointMultiplyCallback(
    CpaStatus status,
    CpaBoolean multiplyStatus,
    CpaInstanceHandle instanceHandle,
    lac_pke_op_cb_data_t *pCbData)
{
    CpaCyEcPointMultiplyCbFunc pCb = NULL;
    void *pCallbackTag = NULL;
    CpaCyEcPointMultiplyOpData *pOpData = NULL;
    CpaFlatBuffer *pXk = NULL;
    CpaFlatBuffer *pYk = NULL;
    sal_crypto_service_t *pCryptoService =
        (sal_crypto_service_t *)instanceHandle;

    /* extract info from callback data structure */
    LAC_ASSERT_NOT_NULL(pCbData);
    pCb = (CpaCyEcPointMultiplyCbFunc)pCbData->pClientCb;
    pCallbackTag = pCbData->pCallbackTag;
    pOpData = (CpaCyEcPointMultiplyOpData *)pCbData->pClientOpData;
    pXk = pCbData->pOutputData1;
    pYk = pCbData->pOutputData2;

    LAC_ASSERT_NOT_NULL(pCb);
    LAC_ASSERT_NOT_NULL(pOpData);
    LAC_ASSERT_NOT_NULL(pXk);
    LAC_ASSERT_NOT_NULL(pYk);

    /* increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_EC_STAT_INC(numEcPointMultiplyCompleted, pCryptoService);
    }
    else
    {
        LAC_EC_STAT_INC(numEcPointMultiplyCompletedError, pCryptoService);
    }

    /* For Montgomery & Edwards curves multiplyStatus is always true,
     * so no effect on statistics.
     */

    /* invoke the user callback */
    pCb(pCallbackTag, status, pOpData, multiplyStatus, pXk, pYk);
}

CpaStatus cpaCyEcMontEdwdsPointMultiply(
    const CpaInstanceHandle instanceHandle_in,
    const CpaCyEcPointMultiplyCbFunc pCb,
    void *pCallbackTag,
    const CpaCyEcMontEdwdsPointMultiplyOpData *pOpData,
    CpaBoolean *pMultiplyStatus,
    CpaFlatBuffer *pXk,
    CpaFlatBuffer *pYk)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaInstanceHandle instanceHandle = NULL;
    sal_crypto_service_t *pCryptoService = NULL;

    if (CPA_INSTANCE_HANDLE_SINGLE == instanceHandle_in)
    {
        instanceHandle = Lac_GetFirstHandle(SAL_SERVICE_TYPE_CRYPTO_ASYM);
    }
    else
    {
        instanceHandle = instanceHandle_in;
    }

#ifdef ICP_PARAM_CHECK
    /* Check for valid acceleration handle */
    LAC_CHECK_INSTANCE_HANDLE(instanceHandle);
    SAL_CHECK_ADDR_TRANS_SETUP(instanceHandle);
#endif

    /* Ensure LAC is running - return error if not */
    SAL_RUNNING_CHECK(instanceHandle);

#ifdef ICP_PARAM_CHECK
    /* Ensure this is a crypto instance with pke enabled */
    SAL_CHECK_INSTANCE_TYPE(
        instanceHandle,
        (SAL_SERVICE_TYPE_CRYPTO | SAL_SERVICE_TYPE_CRYPTO_ASYM));
#endif

    /* Check if the API has been called in synchronous mode */
    if (NULL == pCb)
    {
#ifdef ICP_TRACE
#ifdef ICP_PARAM_CHECK
        /* Check for valid pointers */
        LAC_CHECK_NULL_PARAM(pMultiplyStatus);
#endif
        status = LacEcMontEdwds_PointMultiplySyn(
            instanceHandle, pOpData, pMultiplyStatus, pXk, pYk);

        LAC_LOG7("Called with params (0x%lx, 0x%lx, 0x%lx, 0x%lx, "
                 "%d, 0x%lx, 0x%lx)\n",
                 (LAC_ARCH_UINT)instanceHandle,
                 (LAC_ARCH_UINT)pCb,
                 (LAC_ARCH_UINT)pCallbackTag,
                 (LAC_ARCH_UINT)pOpData,
                 *pMultiplyStatus,
                 (LAC_ARCH_UINT)pXk,
                 (LAC_ARCH_UINT)pYk);

        return status;
#else
        /* Call synchronous mode function */
        return LacEcMontEdwds_PointMultiplySyn(
            instanceHandle, pOpData, pMultiplyStatus, pXk, pYk);
#endif
    }

#ifdef ICP_PARAM_CHECK
    /* Basic NULL Param Checking  */
    status = LacEcMontEdwds_PointMultiplyBasicParamCheck(
        instanceHandle, pOpData, pMultiplyStatus, pXk, pYk);
#endif

    if (CPA_STATUS_SUCCESS == status)
    {
        Cpa32U functionID = 0;
        icp_qat_fw_mmp_input_param_t in = { .flat_array = { 0 } };
        icp_qat_fw_mmp_output_param_t out = { .flat_array = { 0 } };
        lac_pke_op_cb_data_t cbData = { 0 };

        /* Zero the output buffers */
        osalMemSet(pXk->pData, 0, pXk->dataLenInBytes);
        if (CPA_CY_EC_MONTEDWDS_ED25519_TYPE == pOpData->curveType ||
            CPA_CY_EC_MONTEDWDS_ED448_TYPE == pOpData->curveType)
            osalMemSet(pYk->pData, 0, pYk->dataLenInBytes);

        /* populate callback data */
        cbData.pClientCb = pCb;
        cbData.pCallbackTag = pCallbackTag;
        cbData.pClientOpData = pOpData;
        cbData.pOpaqueData = NULL;
        cbData.pOutputData1 = pXk;
        cbData.pOutputData2 = pYk;

        switch (pOpData->curveType)
        {
            case CPA_CY_EC_MONTEDWDS_CURVE25519_TYPE:
                if (pOpData->generator)
                {
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.generator_multiplication_c25519.k, &pOpData->k);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.generator_multiplication_c25519.xr, pXk);
                    functionID = GENERATOR_MULTIPLICATION_C25519;
                }
                else
                {
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_c25519.k, &pOpData->k);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_c25519.xp, &pOpData->x);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.point_multiplication_c25519.xr, pXk);
                    functionID = POINT_MULTIPLICATION_C25519;
                }
                break;
            case CPA_CY_EC_MONTEDWDS_ED25519_TYPE:
                if (pOpData->generator)
                {
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.generator_multiplication_ed25519.k, &pOpData->k);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.generator_multiplication_ed25519.xr, pXk);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.generator_multiplication_ed25519.yr, pYk);

                    functionID = GENERATOR_MULTIPLICATION_ED25519;
                }
                else
                {
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_ed25519.k, &pOpData->k);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_ed25519.xp, &pOpData->x);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_ed25519.yp, &pOpData->y);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.point_multiplication_ed25519.xr, pXk);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.point_multiplication_ed25519.yr, pYk);
                    functionID = POINT_MULTIPLICATION_ED25519;
                }
                break;
            case CPA_CY_EC_MONTEDWDS_CURVE448_TYPE:
                if (pOpData->generator)
                {
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.generator_multiplication_c448.k, &pOpData->k);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.generator_multiplication_c448.xr, pXk);
                    functionID = GENERATOR_MULTIPLICATION_C448;
                }
                else
                {
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_c448.k, &pOpData->k);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_c448.xp, &pOpData->x);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.point_multiplication_c448.xr, pXk);
                    functionID = POINT_MULTIPLICATION_C448;
                }
                break;
            case CPA_CY_EC_MONTEDWDS_ED448_TYPE:
                if (pOpData->generator)
                {
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.generator_multiplication_ed448.k, &pOpData->k);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.generator_multiplication_ed448.xr, pXk);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.generator_multiplication_ed448.yr, pYk);
                    functionID = GENERATOR_MULTIPLICATION_ED448;
                }
                else
                {
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_ed448.k, &pOpData->k);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_ed448.xp, &pOpData->x);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        in.point_multiplication_ed448.yp, &pOpData->y);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.point_multiplication_ed448.xr, pXk);
                    LAC_MEM_SHARED_WRITE_FROM_PTR(
                        out.point_multiplication_ed448.yr, pYk);
                    functionID = POINT_MULTIPLICATION_ED448;
                }
                break;
            default:
                status = CPA_STATUS_INVALID_PARAM;
                break;
        }
        if (CPA_STATUS_SUCCESS == status)
        {
            /* Send the PKE request to the QAT */
            status = LacPkeEcMontEdwds_SendSingleRequest(
                functionID,
                &in,
                &out,
                LacEcMontEdwds_PointMultiplyCallback,
                &cbData,
                instanceHandle);
        }
    }

    pCryptoService = (sal_crypto_service_t *)instanceHandle;

    /* Increment stats */
    if (CPA_STATUS_SUCCESS == status)
    {
        LAC_EC_STAT_INC(numEcPointMultiplyRequests, pCryptoService);
    }
    else
    {
        LAC_EC_STAT_INC(numEcPointMultiplyRequestErrors, pCryptoService);
    }

    return status;
}
