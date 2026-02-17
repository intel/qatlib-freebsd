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
 * @file icp_sal_user.h
 *
 * @defgroup icpSalUser User Space Process API
 *
 * @ingroup icpSal
 *
 * @description
 *      User space process init and shutdown functions.
 *
 ***************************************************************************/

#ifndef ICP_SAL_USER_H
#define ICP_SAL_USER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cpa.h"

#ifdef ICP_HB_FAIL_SIM
#include "icp_sal_hb_fail_simulation.h"
#endif
#ifdef ICP_DC_ERROR_SIMULATION
#include "icp_sal_dc_error_simulation.h"
#endif
#ifdef ICP_RING_EXCEPTION_SIM
#include "icp_sal_ring_exception_simulation.h"
#endif

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function initialises and starts user space service access layer
 *    (SAL) - it registers SAL with ADF and initialises the ADF proxy.
 *    This function must only be called once per user space process.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[in] pProcessName           Process address space name described in
 *                                   the config file for this device
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 *
 *************************************************************************/
CpaStatus icp_sal_userStart(const char *pProcessName);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    Simple wrapper for the icp_sal_userStart() function
 * @deprecated
 *    This function is only for backwards compatibility.
 *    New users should use @ref icp_sal_userStart function directly.
 *
 *************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userStartMultiProcess(const char *pProcessName,
                              CpaBoolean limitDevAccess);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function stops and shuts down user space SAL
 *     - it deregisters SAL with ADF and shuts down ADF proxy
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 *
 ************************************************************************/
CpaStatus icp_sal_userStop(void);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function gets the number of the available dynamic allocated
 *    crypto instances.
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/

CpaStatus CPA_DEPRECATED
icp_sal_userCyGetAvailableNumDynInstances(Cpa32U *pNumCyInstances);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function gets the number of the available dynamic allocated
 *    compression instances
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/

CpaStatus CPA_DEPRECATED
icp_sal_userDcGetAvailableNumDynInstances(Cpa32U *pNumDcInstances);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function gets the number of the available dynamic allocated
 *    crypto instances which are from the specific device package.
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userCyGetAvailableNumDynInstancesByDevPkg(Cpa32U *pNumCyInstances,
                                                  Cpa32U devPkgID);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function gets the number of the available dynamic allocated
 *    crypto instances which are from the specific device package and specific
 *    accelerator.
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userCyGetAvailableNumDynInstancesByPkgAccel(Cpa32U *pNumCyInstances,
                                                    Cpa32U devPkgID,
                                                    Cpa32U accelerator_number);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function gets the number of the available dynamic allocated
 *    compression instances which are from the specific device package.
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userDcGetAvailableNumDynInstancesByDevPkg(Cpa32U *pNumDcInstances,
                                                  Cpa32U devPkgID);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function allocates crypto instances
 *    from dynamic crypto instance pool
 *     - it adds new allocated instances into crypto_services
 *     - it initializes new allocated instances
 *     - it starts new allocated instances
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userCyInstancesAlloc(Cpa32U numCyInstances,
                             CpaInstanceHandle *pCyInstances);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function allocates crypto instances
 *    from dynamic crypto instance pool
 *    which are from the specific device package.
 *     - it adds new allocated instances into crypto_services
 *     - it initializes new allocated instances
 *     - it starts new allocated instances
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userCyInstancesAllocByDevPkg(Cpa32U numCyInstances,
                                     CpaInstanceHandle *pCyInstances,
                                     Cpa32U devPkgID);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function allocates crypto instances
 *    from dynamic crypto instance pool
 *    which are from the specific device package and specific accelerator
 *     - it adds new allocated instances into crypto_services
 *     - it initializes new allocated instances
 *     - it starts new allocated instances
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userCyInstancesAllocByPkgAccel(Cpa32U numCyInstances,
                                       CpaInstanceHandle *pCyInstances,
                                       Cpa32U devPkgID,
                                       Cpa32U accelerator_number);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function frees crypto instances allocated
 *    from dynamic crypto instance pool
 *     - it stops the instances
 *     - it shutdowns the instances
 *     - it removes the instances from crypto_services
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userCyFreeInstances(Cpa32U numCyInstances,
                            CpaInstanceHandle *pCyInstances);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function allocates compression instances
 *    from dynamic compression instance pool
 *     - it adds new allocated instances into compression_services
 *     - it initializes new allocated instances
 *     - it starts new allocated instances
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userDcInstancesAlloc(Cpa32U numDcInstances,
                             CpaInstanceHandle *pDcInstances);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function allocates compression instances
 *    from dynamic compression instance pool
 *    which are from the specific device package.
 *     - it adds new allocated instances into compression_services
 *     - it initializes new allocated instances
 *     - it starts new allocated instances
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userDcInstancesAllocByDevPkg(Cpa32U numDcInstances,
                                     CpaInstanceHandle *pDcInstances,
                                     Cpa32U devPkgID);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function frees compression instances allocated
 *    from dynamic compression instance pool
 *     - it stops the instances
 *     - it shutdowns the instances
 *     - it removes the instances from compression_services
 *
 * @deprecated
 *    This DynInstances feature is obsolete and will be removed in future.
 *
 * @context
 *      This function is called from the user process context
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus CPA_DEPRECATED
icp_sal_userDcFreeInstances(Cpa32U numDcInstances,
                            CpaInstanceHandle *pDcInstances);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function checks if new devices have been started and if so
 *    starts to use them.
 *
 * @context
 *      This function is called from the user process context
 *      in threadless mode
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Deprecated function
 *
 ************************************************************************/
CpaStatus icp_sal_find_new_devices(void);

/*************************************************************************
 * @ingroup SalUser
 * @description
 *    This function polls device events.
 *
 * @context
 *      This function is called from the user process context
 *      in threadless mode
 *
 * @assumptions
 *      None
 * @sideEffects
 *      In case a device has been stopped or restarted the application
 *      will get restarting/stop/shutdown events
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 *
 ************************************************************************/
CpaStatus icp_sal_poll_device_events(void);

/*
 * icp_sal_check_device
 *
 * @description:
 *  This function checks the status of the firmware/hardware for a given device.
 *  This function is used as part of the heartbeat functionality.
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      In case a device is unresponsive the device will
 *      be restarted.
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[in] packageId              The package Id can be found by calling
 *                                   cpaCyInstanceGetInfo2() or
 *                                   cpaDcInstanceGetInfo2().
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported feature
 */
CpaStatus icp_sal_check_device(Cpa32U packageId);

/*
 * icp_sal_check_all_devices
 *
 * @description:
 *  This function checks the status of the firmware/hardware for all devices.
 *  This function is used as part of the heartbeat functionality.
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      In case a device is unresponsive the device will
 *      be restarted.
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported feature
 */
CpaStatus icp_sal_check_all_devices(void);

#define CPA_DEVICE_GEN_LEN 16
typedef struct _CpaPfInfo
{
    Cpa32U pkg_id;
    Cpa16U domain;
    Cpa16U bdf;
    char device_gen[CPA_DEVICE_GEN_LEN + 1];
} CpaPfInfo;

/*
 * icp_sal_get_num_pfs
 *
 * @description:
 *  Returns the number of PFs in the system, only returned if the process has
 *  privileges to access the QAT debugfs/sysfs entries.
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[out] pNumPFs               The number of PFs in the system.
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 */
CpaStatus icp_sal_get_num_pfs(Cpa16U *pNumPFs);

/*
 * icp_sal_get_pf_info
 *
 * @description:
 *  This function populates a pre-allocated list of PF info, only returned
 *  if the process has privileges to access the QAT debugfs/sysfs entries.
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[out] pPfInfo               Pre-allocated list of PF info, the size of
 *                                   this should match the number of PFs on
 *                                   the platform.
 * @retval CPA_STATUS_SUCCESS        No error
 * @retval CPA_STATUS_FAIL           Operation failed
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function
 */
CpaStatus icp_sal_get_pf_info(CpaPfInfo *pPf_info);

/*
 * @ingroup icp_sal_user
 * @description
 *      This is a stub function to reset the device
 *
 * @context
 *     None
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      Yes
 * @threadSafe
 *      Yes
 *
 */
CpaStatus icp_sal_reset_device(Cpa32U accelId);

/*
 * icp_sal_userIsQatAvailable
 *
 * @description:
 *  This function returns CPA_TRUE if a QAT device is present in the
 *  system and available to qatlib
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @retval CPA_TRUE     QAT device available
 * @retval CPA_FALSE    QAT device not available
 *
 */
CpaBoolean icp_sal_userIsQatAvailable(void);

/*
 * icp_sal_get_dc_error
 *
 * @description:
 *  This function returns the occurrences of compression errors specified
 *  in the input parameter
 *
 * @context
 *      This function is called from the user process context
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 * @param[in] dcError                DC Error Type
 *
 * returns                           Number of failing requests of type dcError
 */
Cpa64U icp_sal_get_dc_error(Cpa8S dcError);

/**
 *****************************************************************************
 * @ingroup icp_sal_user
 *      Updates the CSR with queued requests in the asymmetric tx ring.
 *
 * @description
 *      The function writes current shadow tail pointer of the asymmetric
 *      TX ring into ring's CSR. Updating the CSR will notify the HW that
 *      there are request(s) queued to be processed. The CSR is updated
 *      always, disregarding the current value of shadow tail pointer and
 *      the current CSR's tail value.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @param[in] instanceHandle         Crypto API instance handle.
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully.
 * @retval CPA_STATUS_FAIL           Function failed.
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function.
 * @pre
 *      None
 * @post
 *      None
 * @see
 *      None
 *
 *****************************************************************************/
CpaStatus icp_sal_AsymPerformOpNow(CpaInstanceHandle instanceHandle);

/**
 *****************************************************************************
 * @ingroup icp_sal_setForceAEADMACVerify
 *      Sets forceAEADMacVerify for particular instance to force HW MAC
 *      validation.
 *
 * @description
 * 	By default HW MAC verification is set to CPA_TRUE - this utility
 * 	function allows to change default behavior.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @blocking
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      No
 *
 * @param[in] instanceHandle         Crypto API instance handle.
 * @param[in] forceAEADMacVerify     new value
 *
 * @retval CPA_STATUS_SUCCESS        Function executed successfully.
 * @retval CPA_STATUS_FAIL           Function failed.
 * @retval CPA_STATUS_UNSUPPORTED    Unsupported function.
 * @pre
 *      None
 * @post
 *      None
 * @see
 *      None
 *
 *****************************************************************************/
CpaStatus icp_sal_setForceAEADMACVerify(CpaInstanceHandle instanceHandle,
                                        CpaBoolean forceAEADMacVerify);

#ifdef __cplusplus
} /* close the extern "C" { */
#endif

#endif
