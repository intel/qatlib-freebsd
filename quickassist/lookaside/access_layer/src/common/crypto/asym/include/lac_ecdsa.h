/******************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 *****************************************************************************/

/**
 *****************************************************************************
 * @file lac_ec.h
 *
 * @ingroup Lac_Ecdsa
 *
 * ECDH common include file
 *
 * @lld_start
 *
 * @lld_overview
 * This is the common include location for Elliptic Curve components
 *
 * @lld_initialisation
 *
 * @lld_module_algorithms
 *
 * @lld_process_context
 *
 * @lld_end
 *
 *****************************************************************************/

/*****************************************************************************/

#ifndef LAC_ECDSA_H
#define LAC_ECDSA_H

/**
 ******************************************************************************
 * @ingroup Lac_Ecdsa
 *      Check if there is optimised ecdsa SignRS MMP function id for the curve.
 *
 * @description
 *     If the there is optimised ecdsa SignRS MMP function for the curve it
 *     sends a request to firmware with optimised function ID.
 *
 * @param[in]  instanceHandle   Instance handle.
 * @param[in]  pCb              Callback function pointer. If this is set to a
 *                              NULL value the function will operate
 *                              synchronously.
 * @param[in]  pCallbackTag     User-supplied value to help identify request.
 * @param[in]  pOpData          Structure containing all the data needed to
 *                              perform the operation. The client code
 *                              allocates the memory for this structure. This
 *                              component takes ownership of the memory until
 *                              it is returned in the callback.
 * @param[out] pR               ECDSA message signature r.
 * @param[out] pS               ECDSA message signature s.
 *
 * @retval CPA_STATUS_SUCCESS       Function executed successfully.
 * @retval CPA_STATUS_FAIL          Function failed.
 * @retval CPA_STATUS_RETRY         Resubmit the request.
 * @retval CPA_STATUS_INVALID_PARAM Invalid parameter passed in.
 * @retval CPA_STATUS_RESOURCE      Error related to system resources.
 * @retval CPA_STATUS_RESTARTING    API implementation is restarting. Resubmit
 *                                  the request.
 * @retval CPA_STATUS_UNSUPPORTED   Optimised EcdsaSignRS is not supported for
 *                                  the platform or the curve.
 *
 *****************************************************************************/

CpaStatus LacEcdsa_OptimisedSignRS(const CpaInstanceHandle instanceHandle,
                                   const CpaCyEcdsaSignRSCbFunc pCb,
                                   void *pCallbackTag,
                                   const CpaCyEcdsaSignRSOpData *pOpData,
                                   CpaFlatBuffer *pR,
                                   CpaFlatBuffer *pS);
#endif /* LAC_ECDSA_H */