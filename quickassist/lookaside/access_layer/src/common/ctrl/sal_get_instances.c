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
 *****************************************************************************
 * @file sal_get_instances.c
 *
 * @defgroup SalCtrl Service Access Layer Controller
 *
 * @ingroup SalCtrl
 *
 * @description
 *      This file contains generic functions to get instances of a specified
 *      service type. Note these are complementary to the already existing
 *      service-specific functions.
 *
 *****************************************************************************/

/*
*******************************************************************************
* Include public/global header files
*******************************************************************************
*/

/* QAT-API includes */
#include "cpa.h"
#ifndef ICP_DC_ONLY
#include "cpa_cy_common.h"
#include "cpa_cy_im.h"
#endif
#include "cpa_dc.h"

/* Osal includes */
#include "Osal.h"

/* ADF includes */
#include "icp_accel_devices.h"
#include "icp_adf_accel_mgr.h"

/* SAL includes */
#include "lac_mem.h"
#include "lac_list.h"
#include "lac_sal_types.h"

#ifndef ICP_DC_ONLY
/**
 ******************************************************************************
 * @ingroup SalCtrl
 * @description
 *   Get the total number of either sym, asym or cy instances
 *****************************************************************************/
CpaStatus Lac_GetCyNumInstancesByType(
    const CpaAccelerationServiceType accelerationServiceType,
    Cpa16U *pNumInstances)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_accel_dev_t **pAdfInsts = NULL;
    icp_accel_dev_t *dev_addr = NULL;
    sal_t *base_addr = NULL;
    sal_list_t *list_temp = NULL;
    Cpa16U num_accel_dev = 0;
    Cpa16U num_inst = 0;
    Cpa16U i = 0;
    Cpa32U accel_capability = 0;
    char *service = NULL;

    LAC_CHECK_NULL_PARAM(pNumInstances);
    *pNumInstances = 0;

    switch (accelerationServiceType)
    {
        case CPA_ACC_SVC_TYPE_CRYPTO_ASYM:
            accel_capability = ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
            service = "asym";
            break;

        case CPA_ACC_SVC_TYPE_CRYPTO_SYM:
            accel_capability = ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC;
            service = "sym";
            break;

        case CPA_ACC_SVC_TYPE_CRYPTO:
            accel_capability = ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC |
                               ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
            service = "cy";
            break;

        default:
            LAC_LOG_ERROR("Invalid service type\n");
            return CPA_STATUS_INVALID_PARAM;
    }

    /* Get the number of accel_dev in the system */
    status = icp_amgr_getNumInstances(&num_accel_dev);
    LAC_CHECK_STATUS(status);

    /* Allocate memory to store addr of accel_devs */
    pAdfInsts = osalMemAlloc(num_accel_dev * sizeof(icp_accel_dev_t *));
    if (NULL == pAdfInsts)
    {
        LAC_LOG_ERROR("Failed to allocate dev instance memory");
        return CPA_STATUS_RESOURCE;
    }

    num_accel_dev = 0;
    status = icp_amgr_getAllAccelDevByCapabilities(
        accel_capability, pAdfInsts, &num_accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR_PARAMS("No support for service %s\n", service);
        osalMemFree(pAdfInsts);
        return status;
    }

    for (i = 0; i < num_accel_dev; i++)
    {
        dev_addr = pAdfInsts[i];
        if (NULL == dev_addr || NULL == dev_addr->pSalHandle)
        {
            continue;
        }
        base_addr = dev_addr->pSalHandle;

        if (CPA_ACC_SVC_TYPE_CRYPTO == accelerationServiceType)
        {
            list_temp = base_addr->crypto_services;
            while (NULL != list_temp)
            {
                num_inst++;
                list_temp = SalList_next(list_temp);
            }
        }

        if (CPA_ACC_SVC_TYPE_CRYPTO_ASYM == accelerationServiceType ||
            CPA_ACC_SVC_TYPE_CRYPTO == accelerationServiceType)
        {
            list_temp = base_addr->asym_services;
            if ((NULL == list_temp) &&
                (CPA_ACC_SVC_TYPE_CRYPTO != accelerationServiceType))
            {
                list_temp = base_addr->crypto_services;
            }
            while (NULL != list_temp)
            {
                num_inst++;
                list_temp = SalList_next(list_temp);
            }
        }

        if (CPA_ACC_SVC_TYPE_CRYPTO_SYM == accelerationServiceType ||
            CPA_ACC_SVC_TYPE_CRYPTO == accelerationServiceType)
        {
            list_temp = base_addr->sym_services;
            if ((NULL == list_temp) &&
                (CPA_ACC_SVC_TYPE_CRYPTO != accelerationServiceType))
            {
                list_temp = base_addr->crypto_services;
            }
            while (NULL != list_temp)
            {
                num_inst++;
                list_temp = SalList_next(list_temp);
            }
        }
    }

    *pNumInstances = num_inst;
    osalMemFree(pAdfInsts);

#ifdef ICP_TRACE
    if (NULL != pNumInstances)
    {
        LAC_LOG2("Called with params (0x%lx[%d])\n",
                 (LAC_ARCH_UINT)pNumInstances,
                 *pNumInstances);
    }
    else
    {
        LAC_LOG1("Called with params (0x%lx)\n", (LAC_ARCH_UINT)pNumInstances);
    }
#endif

    return status;
}

/**
 ******************************************************************************
 * @ingroup SalCtrl
 * @description
 *   Get either sym, asym or cy instance
 *****************************************************************************/
CpaStatus Lac_GetCyInstancesByType(
    const CpaAccelerationServiceType accelerationServiceType,
    Cpa16U numInstances,
    CpaInstanceHandle *pInstances)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    icp_accel_dev_t **pAdfInsts = NULL;
    icp_accel_dev_t *dev_addr = NULL;
    sal_t *base_addr = NULL;
    sal_list_t *list_temp = NULL;
    Cpa16U num_accel_dev = 0;
    Cpa16U num_allocated_instances = 0;
    Cpa16U index = 0;
    Cpa16U i = 0;
    Cpa32U accel_capability = 0;
    char *service = NULL;

#ifdef ICP_TRACE
    LAC_LOG3("Called with params (%d ,%d, 0x%lx)\n",
             accelerationServiceType,
             numInstances,
             (LAC_ARCH_UINT)pInstances);
#endif

    LAC_CHECK_NULL_PARAM(pInstances);
    if (0 == numInstances)
    {
        LAC_INVALID_PARAM_LOG("NumInstances is 0");
        return CPA_STATUS_INVALID_PARAM;
    }

    switch (accelerationServiceType)
    {
        case CPA_ACC_SVC_TYPE_CRYPTO_ASYM:
            accel_capability = ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
            service = "asym";
            break;

        case CPA_ACC_SVC_TYPE_CRYPTO_SYM:
            accel_capability = ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC;
            service = "sym";
            break;

        case CPA_ACC_SVC_TYPE_CRYPTO:
            accel_capability = ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC |
                               ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC;
            service = "cy";
            break;

        default:
            LAC_LOG_ERROR("Invalid service type\n");
            return CPA_STATUS_INVALID_PARAM;
    }

    /* Get the number of instances */
    status = Lac_GetCyNumInstancesByType(accelerationServiceType,
                                         &num_allocated_instances);
    if (CPA_STATUS_SUCCESS != status)
    {
        return status;
    }

    if (numInstances > num_allocated_instances)
    {
        LAC_LOG_ERROR1("Only %d instances available", num_allocated_instances);
        return CPA_STATUS_RESOURCE;
    }

    /* Get the number of accel devices in the system */
    status = icp_amgr_getNumInstances(&num_accel_dev);
    LAC_CHECK_STATUS(status);

    /* Allocate memory to store addr of accel_devs */
    pAdfInsts = osalMemAlloc(num_accel_dev * sizeof(icp_accel_dev_t *));
    if (NULL == pAdfInsts)
    {
        LAC_LOG_ERROR("Failed to allocate dev instance memory");
        return CPA_STATUS_RESOURCE;
    }

    num_accel_dev = 0;
    status = icp_amgr_getAllAccelDevByCapabilities(
        accel_capability, pAdfInsts, &num_accel_dev);
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR_PARAMS("No support for service %s\n", service);
        osalMemFree(pAdfInsts);
        return status;
    }

    for (i = 0; i < num_accel_dev; i++)
    {
        dev_addr = pAdfInsts[i];
        /* Note dev_addr cannot be NULL here as numInstances = 0
         * is not valid and if dev_addr = NULL then index = 0 (which
         * is less than numInstances and status is set to _RESOURCE
         * above
         */
        base_addr = dev_addr->pSalHandle;
        if (NULL == base_addr)
        {
            continue;
        }

        if (CPA_ACC_SVC_TYPE_CRYPTO == accelerationServiceType)
        {
            list_temp = base_addr->crypto_services;
            while (NULL != list_temp)
            {
                if (index > (numInstances - 1))
                    break;

                pInstances[index] = SalList_getObject(list_temp);
                list_temp = SalList_next(list_temp);
                index++;
            }
        }

        if (CPA_ACC_SVC_TYPE_CRYPTO_ASYM == accelerationServiceType ||
            CPA_ACC_SVC_TYPE_CRYPTO == accelerationServiceType)
        {
            list_temp = base_addr->asym_services;
            if ((NULL == list_temp) &&
                (CPA_ACC_SVC_TYPE_CRYPTO != accelerationServiceType))
            {
                list_temp = base_addr->crypto_services;
            }
            while (NULL != list_temp)
            {
                if (index > (numInstances - 1))
                    break;

                pInstances[index] = SalList_getObject(list_temp);
                list_temp = SalList_next(list_temp);
                index++;
            }
        }

        if (CPA_ACC_SVC_TYPE_CRYPTO_SYM == accelerationServiceType ||
            CPA_ACC_SVC_TYPE_CRYPTO == accelerationServiceType)
        {
            list_temp = base_addr->sym_services;
            if ((NULL == list_temp) &&
                (CPA_ACC_SVC_TYPE_CRYPTO != accelerationServiceType))
            {
                list_temp = base_addr->crypto_services;
            }
            while (NULL != list_temp)
            {
                if (index > (numInstances - 1))
                    break;

                pInstances[index] = SalList_getObject(list_temp);
                list_temp = SalList_next(list_temp);
                index++;
            }
        }
    }
    osalMemFree(pAdfInsts);

    return status;
}
#endif

/**
 ******************************************************************************
 * @ingroup SalCtrl
 *****************************************************************************/
CpaStatus cpaGetNumInstances(
    const CpaAccelerationServiceType accelerationServiceType,
    Cpa16U *pNumInstances)
{
    LAC_CHECK_NULL_PARAM(pNumInstances);

    switch (accelerationServiceType)
    {
#ifndef ICP_DC_ONLY
        case CPA_ACC_SVC_TYPE_CRYPTO_ASYM:
        case CPA_ACC_SVC_TYPE_CRYPTO_SYM:
        case CPA_ACC_SVC_TYPE_CRYPTO:
            return Lac_GetCyNumInstancesByType(accelerationServiceType,
                                               pNumInstances);

#endif
        case CPA_ACC_SVC_TYPE_DATA_COMPRESSION:
            return cpaDcGetNumInstances(pNumInstances);

        case CPA_ACC_SVC_TYPE_PATTERN_MATCH:
        case CPA_ACC_SVC_TYPE_RAID:
        case CPA_ACC_SVC_TYPE_XML:
            LAC_LOG_ERROR("Unsupported service type\n");
            return CPA_STATUS_UNSUPPORTED;

        default:
            LAC_LOG_ERROR("Invalid service type\n");
            *pNumInstances = 0;
            return CPA_STATUS_INVALID_PARAM;
    }
}

/**
 ******************************************************************************
 * @ingroup SalCtrl
 *****************************************************************************/
CpaStatus cpaGetInstances(
    const CpaAccelerationServiceType accelerationServiceType,
    Cpa16U numInstances,
    CpaInstanceHandle *pInstances)
{
    LAC_CHECK_NULL_PARAM(pInstances);

    switch (accelerationServiceType)
    {
#ifndef ICP_DC_ONLY
        case CPA_ACC_SVC_TYPE_CRYPTO_ASYM:
        case CPA_ACC_SVC_TYPE_CRYPTO_SYM:
        case CPA_ACC_SVC_TYPE_CRYPTO:
            return Lac_GetCyInstancesByType(
                accelerationServiceType, numInstances, pInstances);

#endif
        case CPA_ACC_SVC_TYPE_DATA_COMPRESSION:
            return cpaDcGetInstances(numInstances, pInstances);

        case CPA_ACC_SVC_TYPE_PATTERN_MATCH:
        case CPA_ACC_SVC_TYPE_RAID:
        case CPA_ACC_SVC_TYPE_XML:
            LAC_LOG_ERROR("Unsupported service type\n");
            return CPA_STATUS_UNSUPPORTED;

        default:
            LAC_LOG_ERROR("Invalid service type\n");
            return CPA_STATUS_INVALID_PARAM;
    }
}

/**
 ******************************************************************************
 * @ingroup SalCtrl
 *****************************************************************************/
CpaStatus cpaAllocInstance(const CpaAccelerationServiceType serviceType,
                           const CpaInstanceAllocPolicy policy,
                           CpaInstanceHandle *pInstanceHandle)
{
    return CPA_STATUS_UNSUPPORTED;
}

/**
 ******************************************************************************
 * @ingroup SalCtrl
 *****************************************************************************/
CpaStatus cpaFreeInstance(CpaInstanceHandle instanceHandle)
{
    return CPA_STATUS_UNSUPPORTED;
}
