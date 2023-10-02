/***************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2023 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2023 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *
 ***************************************************************************/
/**
 *****************************************************************************
 * @file qae_mem_utils.c
 *
 * This file provides Freebsd kernel memory allocation for quick assist API
 *
 *****************************************************************************/

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include "qae_mem.h"
#include "qae_mem_utils.h"

MALLOC_DEFINE(M_QAE_MEM, "usdm_drv", "usdm_drv driver");
SYSCTL_NODE(_debug, OID_AUTO, usdm_drv, CTLFLAG_RD, NULL, "usdm_drv driver");

/********************************************************************************
* @ingroup max_mem_numa
*       maximum amount of memory allocated in kernel space
* @description
*       This is a command line parameter that defines the maximum
*       amount of memory allocated by the driver in kernel space.
*       Measured in kilobytes.
*****************************************************************************/
uint32_t max_mem_numa = 0;
SYSCTL_UINT(_debug_usdm_drv,
            OID_AUTO,
            max_mem_numa,
            CTLFLAG_RDTUN,
            &max_mem_numa,
            0,
            "Maximum number of allocatable memory in 1k units");

#if __FreeBSD_version <= 1200000
TUNABLE_INT("debug.usdm_drv.max_mem_numa", &max_mem_numa);
#endif

/********************************************************************************
* @ingroup mem_allocated
*       amount of memory currently allocated in kernel space
* @description
*       This variable holds the overall
*       amount of memory allocated by the driver in kernel space.
*       Measured in bytes.
*****************************************************************************/
unsigned long mem_allocated = 0;
SYSCTL_ULONG(_debug_usdm_drv,
             OID_AUTO,
             mem_allocated,
             CTLFLAG_RD,
             &mem_allocated,
             0,
             "Amount of memory currently allocated in kernel space (in bytes)");

static uint32_t numaAllocations_g = 0;
SYSCTL_UINT(_debug_usdm_drv,
            OID_AUTO,
            numaAllocations,
            CTLFLAG_RD,
            &numaAllocations_g,
            0,
            "Number of NUMA allocations");
static uint32_t normalAllocations_g = 0;
SYSCTL_UINT(_debug_usdm_drv,
            OID_AUTO,
            normalAllocations,
            CTLFLAG_RD,
            &normalAllocations_g,
            0,
            "Number of normal allocations");


/**************************************
 * Memory functions
 *************************************/
void *qaeMemAlloc(size_t memsize)
{
    normalAllocations_g++;
    return (malloc(memsize, M_QAE_MEM, M_WAITOK));
}

void *qaeMemAllocNUMA(size_t size, int node, size_t alignment)
{
    void *ptr = NULL;
    void *phys_ptr = NULL;
    void *pRet = NULL;
    uint32_t alignment_offset = 0;
    qae_mem_alloc_info_t memInfo = {0};
    size_t totalInKBytes = (mem_allocated + size) / QAE_KBYTE;
    if ((mem_allocated + size) % QAE_KBYTE)
    {
        totalInKBytes += 1;
    }

    if (max_mem_numa && max_mem_numa < totalInKBytes)
    {
        printf("%s:%d Maximum NUMA allocation of %u kB reached "
               "currently allocated %lu bytes requested %zu bytes\n",
               __func__,
               __LINE__,
               max_mem_numa,
               mem_allocated,
               size);
        return NULL;
    }

    if (!size || alignment < 1)
    {
        printf("%s:%d Either size or alignment is zero - size = %zu, "
               "alignment = %zu \n",
               __func__,
               __LINE__,
               size,
               alignment);
        return NULL;
    }
    /* alignment should be 1,2,4,8.... */
    if (alignment & (alignment - 1))
    {
        printf("%s:%d Expecting alignment of a power of "
               "two but did not get one\n",
               __func__,
               __LINE__);
        return NULL;
    }
    /* add the alignment and the struct size to the buffer size */
    memInfo.mSize = icp_iommu_get_remapping_size(size + alignment +
                                            sizeof(qae_mem_alloc_info_t));
    if (memInfo.mSize > QAE_MEM_SIZE_LIMIT)
    {
        printf("%s:%d Total size needed for this " \
            "set of size and alignment (%zu) exceeds the OS " \
            "limit %d\n", __func__,__LINE__,memInfo.mSize,QAE_MEM_SIZE_LIMIT);
        return NULL;
    }
    /* allocate contigous memory */
    ptr = contigmalloc(memInfo.mSize, M_QAE_MEM, M_WAITOK, 0, ~0, 1, 0);
    if (!ptr)
    {
        printf("%s:%d failed to allocate memory\n", __func__, __LINE__);
        return NULL;
    }
    /* store the base address into the struct */
    memInfo.mAllocMemPtr = ptr;
    if (icp_iommu_map(&phys_ptr, ptr, memInfo.mSize))
    {
        printf("%s:%d failed to iommu map\n", __func__, __LINE__);
        contigfree(ptr, memInfo.mSize, M_QAE_MEM);
        return NULL;
    }
    /* add the size of the struct to the return pointer */
    pRet = (char *)memInfo.mAllocMemPtr + sizeof(qae_mem_alloc_info_t);
    /* compute the offset from the alignement */
    alignment_offset = (uintptr_t)pRet % alignment;
    /* in order to obtain the pointer to the buffer add the alignment and
     * subtract the offset, now we have the return pointer aligned
     */
    pRet = (char *)pRet + (alignment - alignment_offset);
    /* copy the struct immediately before the buffer pointer */
    memcpy((void *)((char *)pRet - sizeof(qae_mem_alloc_info_t)),
           (void *)(&memInfo),
           sizeof(qae_mem_alloc_info_t));
    /* increment the NUMA allocations counter */
    numaAllocations_g++;
    mem_allocated += memInfo.mSize;
    return pRet;
}

void qaeMemFreeNUMA(void **ptr)
{
    qae_mem_alloc_info_t *memInfo = NULL;
    if (!ptr || !(*ptr))
    {
        printf(
            "%s:%d Pointer to be freed cannot be NULL\n", __func__, __LINE__);
        return;
    }
    memInfo =
        (qae_mem_alloc_info_t *)((int8_t *)*ptr - sizeof(qae_mem_alloc_info_t));

    if (memInfo->mSize == 0 || memInfo->mAllocMemPtr == NULL)
    {
        printf("%s:%d Detected the corrupted data: memory leak!! \n",
               __func__,
               __LINE__);
        printf("%s:%d Size: %lu, memPtr: %p\n",
                __func__,
                __LINE__,
                memInfo->mSize,
                memInfo->mAllocMemPtr);
        return;
    }
    if (icp_iommu_unmap((void *)(uintptr_t)vtophys(memInfo->mAllocMemPtr),
                        memInfo->mSize))
    {
        printf("%s:%d failed to iommu unmap\n", __func__, __LINE__);
    }
    numaAllocations_g--;
    if (mem_allocated > memInfo->mSize)
    {
        mem_allocated -= memInfo->mSize;
    }
    else
    {
        mem_allocated = 0;
    }
    contigfree(memInfo->mAllocMemPtr, memInfo->mSize, M_QAE_MEM);
    *ptr = NULL;
}

void qaeMemFree(void **ptr)
{
    if (!ptr || !(*ptr))
    {
        printf("%s:%d Pointer to be freed cannot be NULL\n", __func__, __LINE__);
        return;
    }
    free(*ptr, M_QAE_MEM);
    normalAllocations_g--;
    *ptr = NULL;
}

uint64_t qaeVirtToPhysNUMA(void *ptr)
{
    if (!ptr)
    {
        printf("%s:%d Input parameter cannot be NULL \n", __func__, __LINE__);
        return 0;
    }
    return (uint64_t)(uintptr_t)vtophys(ptr);
}
