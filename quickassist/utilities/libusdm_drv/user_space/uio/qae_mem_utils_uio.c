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
 * @file qae_mem_utils_uio.c
 *
 * This file provides for Linux user space memory allocation. It uses
 * a driver that allocates the memory in kernel memory space (to ensure
 * physically contiguous memory) and maps it to
 * user space for use by the  quick assist sample code
 *
 ***************************************************************************/
#include "qae_mem_utils_common.h"
#ifdef ICP_THREAD_SPECIFIC_USDM
#include "qae_mem_multi_thread.h"
#else
#include "qae_mem_lib_utils.h"
#endif
/**************************************************************************
                                   macro
**************************************************************************/
#define QAE_MEM "/dev/usdm_drv"

/**************************************************************************
    static variable
**************************************************************************/
int g_fd = -1;
#ifndef ICP_THREAD_SPECIFIC_USDM
int g_strict_node = 1;
#endif

#ifdef CACHE_PID
void *cache_pid = NULL;
#endif

/**************************************************************************
                                  function
**************************************************************************/

static inline void ioctl_free_slab(const int fd, dev_mem_info_t *memInfo)
{
    int ret = 0;

    ret = mem_ioctl(fd, DEV_MEM_IOC_MEMFREE, memInfo);
    if (unlikely(ret))
    {
        CMD_ERROR("%s:%d ioctl call for mem free failed, ret = %d\n",
                  __func__,
                  __LINE__,
                  ret);
    }
}

API_LOCAL
void __qae_finish_free_slab(const int fd, dev_mem_info_t *slab)
{
    {
        ioctl_free_slab(fd, slab);
    }
}

/**************************************
 * Memory functions
 *************************************/
static inline int qaeOpenFd(void)
{

    if (g_fd < 0 || is_new_process())
    {
#ifndef ICP_THREAD_SPECIFIC_USDM
        __qae_ResetControl();
#else
        free_page_table_fptr(&g_page_table);
        memset(&g_page_table, 0, sizeof(g_page_table));
#endif
        CMD_DEBUG("%s:%d Memory file handle is not initialized. "
                  "Initializing it now\n",
                  __func__,
                  __LINE__);

        /* if the code flow comes here with the reason that
         * it is a new process, then close the fd, if any,
         * that doesn't belong to the new process!
         */
        if (g_fd > 0)
        {
            CMD_DEBUG("%s:%d Closing the file handle that doesn't "
                      "belong to the process\n",
                      __func__,
                      __LINE__);
            close(g_fd);
        }

        g_fd = qae_open(QAE_MEM, O_RDWR);
        if (g_fd < 0)
        {
            CMD_ERROR("%s:%d Unable to initialize memory file handle %s\n",
                      __func__,
                      __LINE__,
                      QAE_MEM);
            return -ENOENT;
        }

#ifdef CACHE_PID
        cache_process_id();
#endif
        if (__qae_init_hugepages(g_fd))
        {
            close(g_fd);
            return -EIO;
        }
    }
    return 0;
}

int __qae_open(void)
{
    return qaeOpenFd();
}

int qaeMemInitAndReturnFd(int *mem_fd)
{
    int status = -1;
    if (NULL != mem_fd)
    {
        status = qaeMemInit();
    }
    if (status == 0)
    {
        *mem_fd = g_fd;
    }
    return status;
}

int __qae_free_special(void)
{
    int ret = 0;

#ifdef CACHE_PID
    uncache_process_id();
#endif

    /* Send ioctl to kernel space to remove block for this pid */
    if (g_fd > 0)
    {
        ret = mem_ioctl(g_fd, DEV_MEM_IOC_RELEASE, NULL);
        if (ret)
        {
            CMD_ERROR("%s:%d ioctl call for mem release failed, ret = %d\n",
                      __func__,
                      __LINE__,
                      ret);
        }
        close(g_fd);
        g_fd = -1;
    }

    return ret;
}

static inline void *mmap_phy_addr(const int fd,
                                  const uint64_t phy_addr,
                                  const size_t len)
{
    void *addr = NULL;

    addr =
        qae_mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phy_addr);
    if (MAP_FAILED == addr)
        return NULL;

    if (0 != mlock(addr, len))
    {
        munmap(addr, len);
        return NULL;
    }

    return addr;
}

#ifndef ICP_THREAD_SPECIFIC_USDM
static inline dev_mem_info_t *ioctl_alloc_slab(const int fd,
                                               const size_t size,
                                               const uint32_t alignment,
                                               const int node,
                                               enum slabType type)
{
    UNUSED(alignment);
    dev_mem_info_t params = { 0 };
    int ret = 0;
    dev_mem_info_t *slab = NULL;

    params.size = size;
    params.nodeId = node;
    params.type = type;

    ret = mem_ioctl(fd, DEV_MEM_IOC_MEMALLOC, &params);
    if (ret)
    {
        CMD_ERROR("%s:%d ioctl call for mem allocation failed, ret = %d\n",
                  __func__,
                  __LINE__,
                  ret);
        return NULL;
    }

    if (node != params.nodeId)
    {
        g_strict_node = 0;
    }

    if (SMALL == type)
        slab = mmap_phy_addr(fd, params.phy_addr, params.size);
    else
        slab = mmap_phy_addr(fd, params.phy_addr, getpagesize());

    if (NULL == slab)
    {
        CMD_ERROR("%s:%d mmap on memory allocated through ioctl failed\n",
                  __func__,
                  __LINE__);
        ret = mem_ioctl(fd, DEV_MEM_IOC_MEMFREE, &params);
        if (unlikely(ret))
        {
            CMD_ERROR("%s:%d ioctl call for mem free failed, ret = %d\n",
                      __func__,
                      __LINE__,
                      ret);
        }
        return NULL;
    }

    if (SMALL == type)
        slab->virt_addr = slab;
    else
    {
        slab->virt_addr = mmap_phy_addr(fd, params.phy_addr, params.size);

        if (NULL == slab->virt_addr)
        {
            CMD_ERROR("%s:%d mmap failed for large memory allocation\n",
                      __func__,
                      __LINE__);
            munmap(slab, getpagesize());
            ret = mem_ioctl(fd, DEV_MEM_IOC_MEMFREE, &params);
            if (unlikely(ret))
            {
                CMD_ERROR("%s:%d ioctl call for mem free failed, ret = %d\n",
                          __func__,
                          __LINE__,
                          ret);
            }
            return NULL;
        }
    }
    return slab;
}

API_LOCAL
dev_mem_info_t *__qae_alloc_slab(const int fd,
                                 const size_t size,
                                 const uint32_t alignment,
                                 const int node,
                                 enum slabType type)
{
    dev_mem_info_t *slab = NULL;

    if (HUGE_PAGE == type)
    {
    }
    else
    {
        slab = ioctl_alloc_slab(fd, size, alignment, node, type);
    }

    /* Store a slab into the hash table for a fast lookup. */
    if (slab)
        add_slab_to_hash(slab);

    return slab;
}
#else
static inline dev_mem_info_t *ioctl_alloc_slab(const int fd,
                                               const size_t size,
                                               const uint32_t alignment,
                                               const int node,
                                               enum slabType type,
                                               qae_mem_info_t *tls_ptr)
{
    UNUSED(alignment);
    dev_mem_info_t params = { 0 };
    int ret = 0;
    dev_mem_info_t *slab = NULL;

    params.size = size;
    params.nodeId = node;
    params.type = type;

    ret = mem_ioctl(fd, DEV_MEM_IOC_MEMALLOC, &params);
    if (ret)
    {
        CMD_ERROR("%s:%d ioctl call for mem allocation failed, ret = %d\n",
                  __func__,
                  __LINE__,
                  ret);
        return NULL;
    }

    if (node != params.nodeId)
    {
        tls_ptr->g_strict_node = 0;
    }

    if (SMALL == type)
        slab = mmap_phy_addr(fd, params.phy_addr, params.size);
    else
        slab = mmap_phy_addr(fd, params.phy_addr, getpagesize());

    if (NULL == slab)
    {
        CMD_ERROR("%s:%d mmap on memory allocated through ioctl failed\n",
                  __func__,
                  __LINE__);
        ret = mem_ioctl(fd, DEV_MEM_IOC_MEMFREE, &params);
        if (unlikely(ret))
        {
            CMD_ERROR("%s:%d ioctl call for mem free failed, ret = %d\n",
                      __func__,
                      __LINE__,
                      ret);
        }
        return NULL;
    }

    if (SMALL == type)
        slab->virt_addr = slab;
    else
    {
        slab->virt_addr = mmap_phy_addr(fd, params.phy_addr, params.size);

        if (NULL == slab->virt_addr)
        {
            CMD_ERROR("%s:%d mmap failed for large memory allocation\n",
                      __func__,
                      __LINE__);
            munmap(slab, getpagesize());
            ret = mem_ioctl(fd, DEV_MEM_IOC_MEMFREE, &params);
            if (unlikely(ret))
            {
                CMD_ERROR("%s:%d ioctl call for mem free failed, ret = %d\n",
                          __func__,
                          __LINE__,
                          ret);
            }
            return NULL;
        }
    }

    return slab;
}

API_LOCAL
dev_mem_info_t *__qae_alloc_slab(const int fd,
                                 const size_t size,
                                 const uint32_t alignment,
                                 const int node,
                                 enum slabType type,
                                 qae_mem_info_t *tls_ptr)
{
    dev_mem_info_t *slab = NULL;

    if (HUGE_PAGE == type)
    {
    }
    else
    {
        slab = ioctl_alloc_slab(fd, size, alignment, node, type, tls_ptr);
    }

    /* Store a slab into the hash table for a fast lookup. */
    if (slab)
        add_slab_to_hash(slab, tls_ptr);

    return slab;
}
#endif

/*
 * qaeMemMapContiguousIova - UIO stub implementation
 *
 * This API is only supported in VFIO mode. In UIO mode, it returns 0
 * to indicate failure since UIO does not support IOMMU-based IOVA mapping.
 */
uint64_t qaeMemMapContiguousIova(void *virt, size_t size)
{
    UNUSED(virt);
    UNUSED(size);

    CMD_DEBUG("%s:%d is not supported in this mode.\n", __func__, __LINE__);

    return 0;
}

/*
 * qaeMemUnmapContiguousIova - UIO stub implementation
 *
 * This API is only supported in VFIO mode. In UIO mode, it returns 1
 * to indicate failure since UIO does not support IOMMU-based IOVA mapping.
 */
int qaeMemUnmapContiguousIova(void *virt, size_t size)
{
    UNUSED(virt);
    UNUSED(size);

    CMD_DEBUG("%s:%d is not supported in this mode.\n", __func__, __LINE__);

    return 1;
}
