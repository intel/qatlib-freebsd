/******************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 *****************************************************************************/

/******************************************************************************
 * @file adf_user_transport.h
 *
 * @description
 * User space transport functions
 *****************************************************************************/
#ifndef ADF_USER_TRANSPORT_H
#define ADF_USER_TRANSPORT_H

#include "adf_dev_ring_ctl.h"

/*
 * adf_user_put_msg
 *
 * Description
 * Function puts the message onto the ring
 */
CpaStatus adf_user_put_msg(adf_dev_ring_handle_t *pRingHandle, Cpa32U *inBuf);
/*
 * adf_user_notify_msgs
 *
 * Description
 * Function notifies client that there is response message
 */
CpaStatus adf_user_notify_msgs(adf_dev_ring_handle_t *pRingHandle);

/*
 * adf_user_check_ring_error
 *
 * Description
 * Function checks if the rp_exception bit is set in RINGSAT register
 *
 * Return value
 * -EINTR if the rp_exception bit is set
 * -EFAULT if the RINGSTAT rp_exception is not supported
 *  0 if the rp_exception bit is not set
 */
int32_t adf_user_check_ring_error(adf_dev_ring_handle_t *pRingHandle);

/*
 * adf_user_notify_msgs_poll
 *
 * Description
 * Function notifies client that there is response message on polling rings
 */
CpaStatus adf_user_notify_msgs_poll(adf_dev_ring_handle_t *pRingHandle);

/*
 * adf_user_unmap_rings
 *
 * Description
 * Function unmaps all rings allocated for a given device
 */
CpaStatus adf_user_unmap_rings(icp_accel_dev_t *accel_dev);

/*
 * adf_pollRing
 *
 * Description
 * Internal functions which polls
 * a polling ring. This function does not check
 * to see if the ring is a polling ring or
 * if the ring exists.
 *
 */
CpaStatus adf_pollRing(icp_accel_dev_t *accel_dev,
                       adf_dev_ring_handle_t *pRingHandle,
                       Cpa32U response_quota);

/*
 * adf_user_transport_init
 *
 * Description
 * Function initializes internal transport data
 */
CpaStatus adf_user_transport_init(icp_accel_dev_t *accel_dev);

/*
 * adf_user_transport_exit
 *
 * Description
 * Function deinitializes internal transport data
 */
CpaStatus adf_user_transport_exit(icp_accel_dev_t *accel_dev);

#endif /* ADF_USER_TRANSPORT_H */
