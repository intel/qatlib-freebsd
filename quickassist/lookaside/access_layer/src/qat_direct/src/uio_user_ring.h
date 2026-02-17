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
 * @file adf_user_uio_ring.h
 *
 * @description
 *      This file contains the ring related interfaces
 *
 *****************************************************************************/

#ifndef ADF_UIO_USER_RING_H
#define ADF_UIO_USER_RING_H

#include <adf_dev_ring_ctl.h>

int32_t adf_init_ring(adf_dev_ring_handle_t *ring,
                      adf_dev_bank_handle_t *bank,
                      uint32_t ring_num,
                      uint32_t *csr_base_addr,
                      uint32_t num_msgs,
                      uint32_t msg_size,
                      int nodeid);

void adf_cleanup_ring(adf_dev_ring_handle_t *ring);
int32_t adf_ring_freebuf(adf_dev_ring_handle_t *ring);

#ifndef USE_LEGACY_ETRINGMGR

int32_t adf_user_put_msg(adf_dev_ring_handle_t *ring, uint32_t *inBuf);
int32_t adf_user_notify_msgs(adf_dev_ring_handle_t *ring);
int32_t adf_user_notify_msgs_poll(adf_dev_ring_handle_t *ring);
int32_t adf_user_get_inflight_requests(adf_dev_ring_handle_t *ring,
                                       uint32_t *maxInflightRequests,
                                       uint32_t *numInflightRequests);
int32_t adf_user_flush_requests(adf_dev_ring_handle_t *ring);

#endif /* USE_LEGACY_ETRINGMGR */

#endif /*ADF_UIO_USER_RING_H*/
