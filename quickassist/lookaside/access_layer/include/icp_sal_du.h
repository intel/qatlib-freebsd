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
 * @file icp_sal_du.h
 *
 * @description
 *        This file includes device utilization APIs. It contains function
 *        prototypes for managing QAT device utilization on Intel(R)
 *        QuickAssist Technology.
 *
 ****************************************************************************/
#ifndef ICP_SAL_DU_H
#define ICP_SAL_DU_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 ******************************************************************
 * @ingroup SalUserDu
 *        Start the device utilization measurement
 *
 * @description
 *        This function is used to start the device utilization measurement
 *
 * @param[in]  pPf    Pointer to BDF address of physical function on which
 *                    to start the device utilization measurement
 *
 * @retval CPA_STATUS_SUCCESS         Operation successful
 * @retval CPA_STATUS_FAIL            Operation failed
 * @retval CPA_STATUS_INVALID_PARAM   Invalid/null arguments
 * @retval CPA_STATUS_UNSUPPORTED     Unsupported function
 *
 ******************************************************************
 */
CpaStatus icp_sal_userDuStart(struct adf_pci_address *pPf);

/*
 ******************************************************************
 * @ingroup SalUserDu
 *        Stop the device utilization measurement
 *
 * @description
 *        This function is used to stop the device utilization measurement.
 *        Should typically be called some time(eg., 5-10 seconds)
 *        after starting device utilization measurement
 *
 * @param[in]  pPf    Pointer to BDF address of physical function on which
 *                    to stop the device utilization measurement
 *
 * @retval CPA_STATUS_SUCCESS         Operation successful
 * @retval CPA_STATUS_FAIL            Operation failed
 * @retval CPA_STATUS_INVALID_PARAM   Invalid/null arguments
 * @retval CPA_STATUS_UNSUPPORTED     Unsupported function
 *
 ******************************************************************
 */
CpaStatus icp_sal_userDuStop(struct adf_pci_address *pPf);

/*
 ******************************************************************
 * @ingroup SalUserDu
 *        Query and get the overall device utilization for a
 *        specified service type
 *
 * @description
 *        This function is used to get the device utilization for
 *        a specified PF and service type, in SLA units.
 *        This data is reported based on the most recently completed
 *        iteration of device utilization, ie., between the last
 *        start and stop. If stop has never been called, the function
 *        will return CPA_STATUS_FAIL.
 *
 * @param[in]  pPf                 Pointer to BDF address of physical function
 * @param[in]  svcType             Svc service type - sym, asym or dc
 * @param[out] pCapacityInSlaUnit  Pointer to total capacity in sla units
 * @param[out] pUtilInSlaUnit      Pointer to utilization in sla units
 * @param[out] pUtilInPct          Pointer to utilization in percentage
 *
 * @retval CPA_STATUS_SUCCESS         Operation successful
 * @retval CPA_STATUS_FAIL            Operation failed
 * @retval CPA_STATUS_INVALID_PARAM   Invalid/null arguments
 * @retval CPA_STATUS_UNSUPPORTED     Unsupported function
 *
 ******************************************************************
 */
CpaStatus icp_sal_userDuQuery(struct adf_pci_address *pPf,
                              enum adf_svc_type svcType,
                              Cpa32U *pCapacityInSlaUnit,
                              Cpa32U *pUtilInSlaUnit,
                              Cpa16U *pUtilInPct);

/*
 ******************************************************************
 * @ingroup SalUserDu
 *        Query and get the overall device utilization for a
 *        specified service type for virtual function
 *
 * @description
 *        This function is used to get the device utilization for
 *        a specified VF and service type, in SLA units.
 *        This data is reported based on the most recently completed
 *        iteration of device utilization, ie., between the last
 *        start and stop. If stop has never been called, the function
 *        will return CPA_STATUS_FAIL.
 *
 * @param[in]  pPf                  Pointer to BDF address of physical function
 * @param[in]  pVf                  Pointer to BDF address of virtual function
 * @param[in]  svcType              Svc service type - sym, asym or dc
 * @param[out] pSlaInSlaUnit   Pointer to total capacity in sla units
 * @param[out] pUtilInSlaUnit       Pointer to utilization in sla units
 * @param[out] pUtilInPct           Pointer to utilization in percentage
 *
 * @retval CPA_STATUS_SUCCESS         Operation successful
 * @retval CPA_STATUS_FAIL            Operation failed
 * @retval CPA_STATUS_INVALID_PARAM   Invalid/null arguments
 * @retval CPA_STATUS_UNSUPPORTED     Unsupported function
 *
 ******************************************************************
 */
CpaStatus icp_sal_userDuQueryVf(struct adf_pci_address *pPf,
                                struct adf_pci_address *pVf,
                                enum adf_svc_type svcType,
                                Cpa32U *pSlaInSlaUnit,
                                Cpa32U *pUtilInSlaUnit,
                                Cpa16U *pUtilInPct);

#ifdef __cplusplus
} /* close the extern "C" { */
#endif

#endif
