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
 * @file adf_transport.h
 *
 * @description
 *      File contains simple interface for components to get
 *      a handle for appropriate transport type.
 *
 *****************************************************************************/
#ifndef ADF_TRANSPORT_CTRL_H
#define ADF_TRANSPORT_CTRL_H

#include "icp_adf_transport.h"

/* Magic number used to indicated that this message is read, and the
 * status is empty now
 */
#define EMPTY_RING_ENTRY_SIG (0x7F7F7F7F)

/*
 * Structure of a dyn instance handle
 */
typedef struct icp_dyn_instance_handle_s
{
    Cpa32U instance_id;
    adf_service_type_t stype;
    struct icp_dyn_instance_handle_s *pNext;
    struct icp_dyn_instance_handle_s *pPrev;
} icp_dyn_instance_handle_t;

/*
 * Structure of a transport handle
 */
typedef struct icp_trans_handle_s
{
    /* Acceleration Handle for this transport */
    icp_accel_dev_t *accel_dev;

    /* Handle Identifier */
    icp_trans_identifier handle_id;

    /* Transport Type */
    icp_transport_type trans_type;

    /* Put a message onto the transport and wait for a response.
     * Note: Not all transports support sync messages.*/

    CpaStatus (*put_msg_sync)(struct icp_trans_handle_s *trans_handle,
                              Cpa32U *inBuf,
                              Cpa32U *outBuf,
                              Cpa32U buffsLen);

    /* Put a message onto the transport */
    CpaStatus (*put_msg)(struct icp_trans_handle_s *trans_handle,
                         Cpa32U *inBuf,
                         Cpa32U bufLen);

    /* register a callback for notification when something is available */
    CpaStatus (*reg_callback)(struct icp_trans_handle_s *trans_handle,
                              icp_trans_callback callback);

    /* notify handle to notify subscribed callbacks of messages */
    CpaStatus (*notify)(struct icp_trans_handle_s *trans_handle);

    /* notify handle for polling to notify subscribed callbacks of messages */
    CpaStatus (*polling_notify)(struct icp_trans_handle_s *trans_handle,
                                Cpa32U response_quota);
    /* Transport specific data to ease processing */
    void *trans_data;

    CpaBoolean is_dyn;

    struct icp_trans_handle_s *pNext;
    struct icp_trans_handle_s *pPrev;
} icp_trans_handle;

/*
 * Structure for dynamic instance resource management
 */
typedef struct adf_instancemgr_s
{
    Cpa32U serv_type;
    ICP_MUTEX instance_lock;
    Cpa32U max_instance;
    Cpa32U *instances;
    Cpa32U last_found;
    Cpa32U avail;
} adf_instancemgr_t;

/*
 * Function Pointer for creating the transport handle
 */
typedef CpaStatus (*mgr_create_handle)(icp_accel_dev_t *accel_dev,
                                       const char *section,
                                       Cpa32U accel_nr,
                                       Cpa32U bank_nr,
                                       icp_trans_handle **trans_handle,
                                       icp_adf_ringInfoService_t info,
                                       const char *service_name,
                                       const Cpa32U size,
                                       const Cpa32U msg_size,
                                       Cpa32U flags);

/*
 * Function Pointer for releasing the transport handle
 */
typedef CpaStatus (*mgr_release_handle)(icp_trans_handle *trans_handle);

/*
 * Function Pointer for finding an existing transport handle
 */
typedef CpaStatus (*mgr_find_handle)(icp_accel_dev_t *accel_dev,
                                     icp_trans_identifier trans_id,
                                     icp_trans_handle **trans_handle);

/*
 * Function Pointer for returning the ring number for a transport handle
 */
typedef CpaStatus (*mgr_get_ring_num)(icp_trans_handle *trans_handle,
                                      Cpa32U *ringNum);
/*
 * Struct representing a transport manager
 */
typedef struct icp_trans_mgr_s
{
    icp_transport_type trans_type;
    mgr_create_handle create_handle;
    mgr_release_handle release_handle;
    mgr_find_handle find_handle;
    mgr_get_ring_num get_ring_num;
    void *privData;
    struct icp_trans_mgr_s *pPrev;
    struct icp_trans_mgr_s *pNext;
} icp_trans_mgr;

/*
 * Register the ring manager with the ring factory
 */
CpaStatus adf_trans_registerTransMgr(icp_trans_mgr *trans_mgr);

/*
 * Remove the ring manager from the ring factory
 */
CpaStatus adf_trans_deregisterTransMgr(icp_trans_mgr *trans_mgr);

/*
 * Initialize dynamic instance pool
 */
CpaStatus adf_trans_initDynInstancePool(icp_accel_dev_t *accel_dev,
                                        Cpa32U crypto_num,
                                        Cpa32U compress_num);

/*
 * Destroy dynamic instance pool
 */
CpaStatus adf_trans_destroyDynInstancePool(icp_accel_dev_t *accel_dev);

/*
 * Get an available dynamic instance from the dynamic instance pool
 */
CpaStatus adf_trans_getDynInstance(icp_accel_dev_t *accel_dev,
                                   adf_service_type_t stype,
                                   Cpa32U *pinstance_id);

/*
 * Put back a dynamic instance into the dynamic instance pool
 */
CpaStatus adf_trans_putDynInstance(icp_accel_dev_t *accel_dev,
                                   adf_service_type_t stype,
                                   Cpa32U instance_id);

/*
 * Get the number of the available dynamic instances
 * in the dynamic instance pool
 */
CpaStatus adf_trans_getNumAvailDynInstance(icp_accel_dev_t *accel_dev,
                                           adf_service_type_t stype,
                                           Cpa32U *num);

#endif /* ADF_TRANSPORT_CTRL_H */
