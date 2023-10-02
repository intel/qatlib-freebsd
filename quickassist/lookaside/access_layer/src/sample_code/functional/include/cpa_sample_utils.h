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
 * @file cpa_sample_utils.h
 *
 * @defgroup sampleUtils Macro and inline function definitions
 *
 * @ingroup sampleCode
 *
 * @description
 * Defines macros for printing and debugging, inline functions for memory
 * allocating and freeing and thread creation
 *
 ***************************************************************************/

#ifndef CPA_SAMPLE_UTILS_H
#define CPA_SAMPLE_UTILS_H

#include "cpa.h"
#include "cpa_cy_im.h"
#include "cpa_dc.h"


#ifdef DO_CRYPTO
#include "cpa_cy_sym.h"
#endif
#include "cpa_sample_cnv_utils.h"

#ifdef USER_SPACE
/* User space utils */
#include <semaphore.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
/* Performance sample code mem utils */

#include "qae_mem.h"

extern CpaDcHuffType huffmanType_g;
extern CpaStatus qaeMemInit(void);
extern void qaeMemDestroy(void);
/* Threads */
typedef pthread_t sampleThread;

/* Check for CY API version */
#define CY_API_VERSION_AT_LEAST(major, minor)                                  \
    (CPA_CY_API_VERSION_NUM_MAJOR > major ||                                   \
     (CPA_CY_API_VERSION_NUM_MAJOR == major &&                                 \
      CPA_CY_API_VERSION_NUM_MINOR >= minor))

/* Printing */
/**< Prints the name of the function and the arguments only if gDebugParam is
 * CPA_TRUE.
 */
#define PRINT_DBG(args...)                                                     \
    do                                                                         \
    {                                                                          \
        if (CPA_TRUE == gDebugParam)                                           \
        {                                                                      \
            printf("%s(): ", __func__);                                        \
            printf(args);                                                      \
            fflush(stdout);                                                    \
        }                                                                      \
    } while (0)

/**< Prints the arguments */
#ifndef PRINT
#define PRINT(args...)                                                         \
    do                                                                         \
    {                                                                          \
        printf(args);                                                          \
    } while (0)
#endif

/**< Prints the name of the function and the arguments */
#ifndef PRINT_ERR
#define PRINT_ERR(args...)                                                     \
    do                                                                         \
    {                                                                          \
        printf("%s(): ", __func__);                                            \
        printf(args);                                                          \
    } while (0)
#endif
/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      Completion definitions
 *
 ******************************************************************************/
struct completion_struct
{
    sem_t semaphore;
};
/* Use semaphores to signal completion of events */
#define COMPLETION_STRUCT completion_struct

#define COMPLETION_INIT(s) sem_init(&((s)->semaphore), 0, 0);

#define COMPLETION_WAIT(s, timeout) (sem_wait(&((s)->semaphore)) == 0)

#define COMPLETE(s) sem_post(&((s)->semaphore))

#define COMPLETION_DESTROY(s) sem_destroy(&((s)->semaphore))

#else

/* Kernel space utils */

#include <sys/param.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/sema.h>
#include <sys/systm.h>
#include <vm/vm.h>
#include <vm/pmap.h>

#ifdef __x86_64__
#define SAMPLE_ADDR_LEN uint64_t
#else
#define SAMPLE_ADDR_LEN uint32_t
#endif

extern CpaDcHuffType huffmanType_g;

struct cpa_freebsd_alloc_contig
{
    void *vaddr;
    unsigned long size;
};

/* Threads */
typedef struct thread *sampleThread;

/* Printing */
/**< Prints the name of the function and the arguments only if gDebugParam is
 * TRUE.
 */
#define PRINT_DBG(args...)                                                     \
    do                                                                         \
    {                                                                          \
        if (TRUE == gDebugParam)                                               \
        {                                                                      \
            printf("%s(): ", __func__);                                        \
            printf(args);                                                      \
        }                                                                      \
    } while (0)
/**< Prints the name of the function and the arguments */
#define PRINT_ERR(args...)                                                     \
    do                                                                         \
    {                                                                          \
        printf("%s(): ", __func__);                                            \
        printf(args);                                                          \
    } while (0)

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      Completion definitions
 *
 ******************************************************************************/
#define COMPLETION_STRUCT sema

#define COMPLETION_INIT(s) sema_init(s, 0, "cpa completion")

#define COMPLETION_WAIT(s, timeout) (sema_timedwait(s, timeout) == 0)

#define COMPLETE(s) sema_post(s)

#define COMPLETION_DESTROY(s) sema_destroy(s)

#endif

#ifndef BYTE_ALIGNMENT_8
#define BYTE_ALIGNMENT_8 (8)
#endif
#ifndef BYTE_ALIGNMENT_64
#define BYTE_ALIGNMENT_64 (64)
#endif

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      This function and associated macro sleeps for ms milliseconds
 *
 * @param[in] ms    sleep time in ms
 *
 * @retval none
 *
 ******************************************************************************/
static __inline CpaStatus sampleSleep(Cpa32U ms)
{
#ifdef USER_SPACE
    int ret = 0;
    struct timespec resTime, remTime;
    resTime.tv_sec = ms / 1000;
    resTime.tv_nsec = (ms % 1000) * 1000000;
    do
    {
        ret = nanosleep(&resTime, &remTime);
        resTime = remTime;
    } while ((ret != 0) && (errno == EINTR));

    if (ret != 0)
    {
        PRINT_ERR("nanoSleep failed with code %d\n", ret);
        return CPA_STATUS_FAIL;
    }
    else
    {
        return CPA_STATUS_SUCCESS;
    }
#else
    pause("cpaslp", ms * hz / 1000);
    return CPA_STATUS_SUCCESS;
#endif
}
/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      Macro from the sampleSleep function
 *
 ******************************************************************************/
#define OS_SLEEP(ms) sampleSleep((ms))

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      This function and associated macro allocates the memory for the given
 *      size and stores the address of the memory allocated in the pointer.
 *      Memory allocated by this function is NOT guaranteed to be physically
 *      contiguous.
 *
 * @param[out] ppMemAddr    address of pointer where address will be stored
 * @param[in] sizeBytes     the size of the memory to be allocated
 *
 * @retval CPA_STATUS_RESOURCE  Macro failed to allocate Memory
 * @retval CPA_STATUS_SUCCESS   Macro executed successfully
 *
 ******************************************************************************/
static __inline CpaStatus Mem_OsMemAlloc(void **ppMemAddr, Cpa32U sizeBytes)
{
#ifdef USER_SPACE
    *ppMemAddr = malloc(sizeBytes);
    if (NULL == *ppMemAddr)
    {
        return CPA_STATUS_RESOURCE;
    }
    return CPA_STATUS_SUCCESS;
#else
    *ppMemAddr = malloc(sizeBytes, M_DEVBUF, M_WAITOK);
    return CPA_STATUS_SUCCESS;
#endif
}

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      This function and associated macro allocates the memory for the given
 *      size for the given alignment and stores the address of the memory
 *      allocated in the pointer. Memory allocated by this function is
 *      guaranteed to be physically contiguous.
 *
 * @param[out] ppMemAddr    address of pointer where address will be stored
 * @param[in] sizeBytes     the size of the memory to be allocated
 * @param[in] alignment    the alignment of the memory to be allocated
 *(non-zero)
 *
 * @retval CPA_STATUS_RESOURCE  Macro failed to allocate Memory
 * @retval CPA_STATUS_SUCCESS   Macro executed successfully
 *
 ******************************************************************************/
static __inline CpaStatus Mem_Alloc_Contig(void **ppMemAddr,
                                           Cpa32U sizeBytes,
                                           Cpa32U alignment)
{
#ifdef USER_SPACE
    /* Use perf sample code memory allocator */

    /* In this sample all allocations are done from node=0
     * This might not be optimal in a dual processor system.
     */
    *ppMemAddr = qaeMemAllocNUMA(sizeBytes, 0, alignment);
    if (NULL == *ppMemAddr)
    {
        PRINT_ERR("Memory allocation Failed");
        return CPA_STATUS_RESOURCE;
    }
    return CPA_STATUS_SUCCESS;
#else
    struct cpa_freebsd_alloc_contig *ac;
    unsigned long size;
    void *pAlloc;

    /* contigfree() requires the size of the allocation being freed but
     * Mem_Free_Contig() does not accept the size.  Instead, extra room
     * is allocated at the start of the allocation to hold a
     * struct cpa_freebsd_alloc_contig to store the original pointer
     * and size.  The structure is located just before the returned
     * pointer so that it can be obtained from the pointer passed to
     * Mem_Free_Contig().
     */
    if (alignment < sizeof(*ac))
        size = sizeBytes + sizeof(*ac);
    else
        size = sizeBytes + alignment;
    pAlloc = contigmalloc(size, M_DEVBUF, M_WAITOK, 0, ~0UL, alignment, 0);
    if (alignment < sizeof(*ac))
    {
        ac = pAlloc;
    }
    else
    {
        ac = (void *)((char *)pAlloc + alignment - sizeof(*ac));
    }
    ac->vaddr = pAlloc;
    ac->size = size;
    *ppMemAddr = (void *)(ac + 1);
    KASSERT((uintptr_t)*ppMemAddr % alignment == 0,
            ("%s: not aligned", __func__));
    return CPA_STATUS_SUCCESS;

#endif
}

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      Macro from the Mem_OsMemAlloc function
 *
 ******************************************************************************/
#define OS_MALLOC(ppMemAddr, sizeBytes)                                        \
    Mem_OsMemAlloc((void *)(ppMemAddr), (sizeBytes))

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      Macro from the Mem_Alloc_Contig function
 *
 ******************************************************************************/
#define PHYS_CONTIG_ALLOC(ppMemAddr, sizeBytes)                                \
    Mem_Alloc_Contig((void *)(ppMemAddr), (sizeBytes), 1)

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *     Aligned version of PHYS_CONTIG_ALLOC() macro
 *
 ******************************************************************************/
#define PHYS_CONTIG_ALLOC_ALIGNED(ppMemAddr, sizeBytes, alignment)             \
    Mem_Alloc_Contig((void *)(ppMemAddr), (sizeBytes), (alignment))

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      This function and associated macro frees the memory at the given address
 *      and resets the pointer to NULL. The memory must have been allocated by
 *      the function Mem_OsMemAlloc()
 *
 * @param[out] ppMemAddr    address of pointer where mem address is stored.
 *                          If pointer is NULL, the function will exit silently
 *
 * @retval void
 *
 ******************************************************************************/
static __inline void Mem_OsMemFree(void **ppMemAddr)
{
#ifdef USER_SPACE
    if (NULL != *ppMemAddr)
    {
        free(*ppMemAddr);
        *ppMemAddr = NULL;
    }
#else
    free(*ppMemAddr, M_DEVBUF);
    *ppMemAddr = NULL;

#endif
}

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      This function and associated macro frees the memory at the given address
 *      and resets the pointer to NULL. The memory must have been allocated by
 *      the function Mem_Alloc_Contig().
 *
 * @param[out] ppMemAddr    address of pointer where mem address is stored.
 *                          If pointer is NULL, the function will exit silently
 *
 * @retval void
 *
 ******************************************************************************/
static __inline void Mem_Free_Contig(void **ppMemAddr)
{
#ifdef USER_SPACE
    if (NULL != *ppMemAddr)
    {
        qaeMemFreeNUMA(ppMemAddr);
        *ppMemAddr = NULL;
    }
#else
    if (NULL != *ppMemAddr)
    {
        struct cpa_freebsd_alloc_contig *ac;

        ac = (struct cpa_freebsd_alloc_contig *)*ppMemAddr - 1;
        contigfree(ac->vaddr, ac->size, M_DEVBUF);
        *ppMemAddr = NULL;
    }
#endif
}

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      Macro from the Mem_OsMemFree function
 *
 ******************************************************************************/
#define OS_FREE(pMemAddr) Mem_OsMemFree((void *)&pMemAddr)

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      Macro from the Mem_Free_Contig function
 *
 ******************************************************************************/
#define PHYS_CONTIG_FREE(pMemAddr) Mem_Free_Contig((void *)&pMemAddr)

#define _4K_PAGE_SIZE (4 * 1024)

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      This function returns the physical address for a given virtual address.
 *      In case of error 0 is returned.
 *
 * @param[in] virtAddr     Virtual address
 *
 * @retval CpaPhysicalAddr Physical address or 0 in case of error
 *
 ******************************************************************************/

static __inline CpaPhysicalAddr sampleVirtToPhys(void *virtAddr)
{
#ifdef USER_SPACE
    return (CpaPhysicalAddr)qaeVirtToPhysNUMA(virtAddr);
#else
    return (CpaPhysicalAddr)vtophys(virtAddr);
#endif
}

/**
 *******************************************************************************
 * @ingroup sampleUtils
 *      This function creates a thread
 *
 ******************************************************************************/

static __inline CpaStatus sampleThreadCreate(sampleThread *thread,
                                             void *funct,
                                             void *args)
{
#ifdef USER_SPACE
    if (pthread_create(thread, NULL, funct, args) != 0)
    {
        PRINT_ERR("Failed create thread\n");
        return CPA_STATUS_FAIL;
    }
    else
    {
        pthread_detach(*thread);
        return CPA_STATUS_SUCCESS;
    }
#else
    if (kthread_add(funct, args, &proc0, thread, 0, 0, "cpa sample") != 0)
    {
        PRINT_ERR("Failed create thread\n");
        return CPA_STATUS_SUCCESS;
    }
    return CPA_STATUS_SUCCESS;
#endif
}

static __inline void sampleThreadExit(void)
{
#ifdef USER_SPACE
    pthread_exit(NULL);
#else
    kthread_exit();
#endif
}

#ifdef DO_CRYPTO
void sampleCyGetInstance(CpaInstanceHandle *pCyInstHandle);

void sampleCyStartPolling(CpaInstanceHandle cyInstHandle);

void sampleCyStopPolling(void);

void symSessionWaitForInflightReq(CpaCySymSessionCtx pSessionCtx);
#endif // DO_CRYPTO

void sampleDcGetInstance(CpaInstanceHandle *pDcInstHandle);

void sampleDcStartPolling(CpaInstanceHandle dcInstHandle);

void sampleDcStopPolling(void);

Cpa64U sampleCoderdtsc(void);

void hexLog(Cpa8U *pData, Cpa32U numBytes, const char *caption);


#ifdef __x86_64__
#define SAMPLE_CODE_UINT Cpa64U
#define SAMPLE_CODE_INT Cpa64S
#else
#define SAMPLE_CODE_UINT Cpa32U
#define SAMPLE_CODE_INT Cpa32S
#endif // __x86_64__
/**
 *****************************************************************************
 * @ingroup sampleUtils
 *
 *      convert virtual address of a buffer to address that can be accessed by
 *      the owner of the instance from device point of view.
 *
 * @param[in]   pVirtAddr                virtual address of the buffer
 * @param[in]   instance                 crypto instance handle
 * @param[in]   type                     service type
 *
 * @retval CpaPhysicalAddr               address from device point of view,
 *                                       or NULL if failed to convert
 *
 ****************************************************************************/
CpaPhysicalAddr virtAddrToDevAddr(void *pVirtAddr,
                                  CpaInstanceHandle instance,
                                  CpaAccelerationServiceType type);
#endif
