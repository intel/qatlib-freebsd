/*****************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 *****************************************************************************/

/*****************************************************************************
 * @file icp_adf_user_proxy.h
 *
 * @description
 *      This file contains the function prototype to initialize ADF proxy
 *      in user space.
 *
 *****************************************************************************/
#ifndef ICP_ADF_USER_PROXY_H
#define ICP_ADF_USER_PROXY_H

/*
 * icp_adf_userProcessToStart
 *
 * Description:
 *  This function checks if a user space process with a given name has
 *  already been started.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_userProcessToStart(const char *const name_tml, char *name);

/*
 * icp_adf_userProxyInit
 *
 * Description:
 *  This function is used to initialize the ADF proxy in user space.
 *  It takes a process name as a parameter. Caller should check if
 *  such process name is not already started using
 *  icp_adf_userProcessStarted function.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_userProxyInit(const char *const name);

/*
 * icp_adf_userProxyShutdown
 *
 * Description:
 *  This function is used to shutdown the ADF proxy in user space.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_userProxyShutdown(void);

/*
 * icp_adf_userProcessStop
 *
 * Description:
 *  This function closes the processes info file.
 *  It should be called before a process exits if icp_adf_userProcessToStart()
 *  has been used to obtain unique user process name.
 *
 * Returns: void
 */
void icp_adf_userProcessStop(void);

/*
 * icp_adf_find_new_devices
 *
 * Description:
 *  Function tries to connect to devices.
 *  This function is used in threadless mode in user space.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_find_new_devices(void);

/*
 * icp_adf_poll_device_events
 *
 * Description:
 *  Function polls new device events.
 *  This function is used in threadless mode in user space.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_poll_device_events(void);

/*
 * icp_adf_check_device
 *
 * Description:
 *  Function checks the status of the firmware/hardware for a given device.
 *  This function is used as part of the heartbeat functionality.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_check_device(Cpa32U accelId);

/*
 * icp_adf_check_all_devices
 *
 * Description:
 *  Function checks the status of the firmware/hardware for all devices.
 *  This function is used as part of the heartbeat functionality.
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_check_all_devices(void);

/*
 * icp_adf_reset_device
 *
 * Description:
 *  reset device - calls the IOCTL in
 *  the driver which resets the device based on accelId
 * Returns:
 *   CPA_STATUS_SUCCESS on success
 *   CPA_STATUS_FAIL    on failure
 */

#ifdef ICP_HB_FAIL_SIM
/*
 * icp_adf_heartbeat_simulate_failure
 *
 * Description:
 *  Function simulates a heartbeat failure.
 *  If icp_adf_check_device is used along with this call, a heartbeat
 *  failure will be detected
 *
 * Returns:
 *   CPA_STATUS_SUCCESS   on success
 *   CPA_STATUS_FAIL      on failure
 */
CpaStatus icp_adf_heartbeat_simulate_failure(Cpa32U accelId);

#endif

CpaStatus icp_adf_reset_device(Cpa32U accelId);

#endif /* ICP_ADF_USER_PROXY_H */
