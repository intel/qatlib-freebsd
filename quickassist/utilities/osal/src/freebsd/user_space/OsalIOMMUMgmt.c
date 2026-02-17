/**
 * @file IOMMUMgmt.c (user space)
 *
 * @brief Stubs for NUMA
 *
 *
 * @par
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 */

#ifdef ICP_WITHOUT_IOMMU

#include "Osal.h"

int osalIOMMUMap(UINT64 phaddr, UINT64 iova, size_t size)
{
    return 0;
}

int osalIOMMUUnmap(UINT64 iova, size_t size)
{
    return 0;
}

UINT64 osalIOMMUVirtToPhys(UINT64 iova)
{
    return iova;
}

size_t osalIOMMUgetRemappingSize(size_t size)
{
    return size;
}

#endif
