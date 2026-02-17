/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#ifndef UIO_USER_SHARED_MUTEX_H
#define UIO_USER_SHARED_MUTEX_H

#ifndef ICP_WITHOUT_THREAD
#include <pthread.h>

int uio_shared_mutex_create(const char *name, pthread_mutex_t **mutex);
int uio_shared_mutex_lock(pthread_mutex_t *mutex);
int uio_shared_mutex_unlock(pthread_mutex_t *mutex);
int uio_shared_mutex_destroy(pthread_mutex_t *mutex);
#endif

#endif /* end of include guard: UIO_USER_SHARED_MUTEX_H */
