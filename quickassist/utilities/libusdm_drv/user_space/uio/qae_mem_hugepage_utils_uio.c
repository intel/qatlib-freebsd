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
 ****************************************************************************
 * @file qae_mem_hugepage_utils_uio.c
 *
 * This file provides dummy huge page utilities for FreeBSD user space memory
 * allocation with huge page not supported for uio.
 *
 ***************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/stat.h>
#include "qae_mem_hugepage_utils.h"
#include "qae_mem_user_utils.h"

API_LOCAL
int __qae_init_hugepages(const int fd)
{
    UNUSED(fd);
    return 0;
}

API_LOCAL
int __qae_hugepage_enabled()
{
    return 0;
}

API_LOCAL
dev_mem_info_t *__qae_hugepage_alloc_slab(const int fd,
                                          const size_t size,
                                          const int node,
                                          enum slabType type)
{
    UNUSED(fd);
    UNUSED(size);
    UNUSED(node);
    UNUSED(type);

    return NULL;
}

API_LOCAL
void __qae_hugepage_free_slab(const dev_mem_info_t *memInfo)
{
    UNUSED(memInfo);
}
