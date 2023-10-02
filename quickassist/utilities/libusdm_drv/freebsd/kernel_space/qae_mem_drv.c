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
 *
 * @file qae_mem_drv.c
 *
 * @brief Kernel-space support for user-space contiguous memory allocation
 *
 */
#include <sys/param.h>
#include <sys/conf.h>
#include <sys/ioccom.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rwlock.h>
#include <sys/sglist.h>
#include <sys/systm.h>
#include <sys/sx.h>
#include <sys/uio.h>
#include <machine/atomic.h>
#include <vm/vm.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>
#include "qae_mem_utils.h"

#define DEV_MEM_NAME "usdm_drv"
#define QAE_LOCAL_ENSURE(c, str, ret)                                        \
    if (!(c))                                                                \
    {                                                                        \
        printf("%s in file %s, ret = %d\n", str, __FILE__, ret);             \
        return ret;                                                          \
    }


/**
******************************************************************************
* @ingroup max_mem_numa
*       maximum amount of memory allocated in kernel space
* @description
*       This is a command line parameter that defines the maximum
*       amount of memory allocated by the driver in kernel space.
*       Measured in kilobytes.
*****************************************************************************/
extern uint32_t max_mem_numa;

/**
******************************************************************************
* @ingroup mem_allocated
*       amount of memory currently allocated in kernel space
* @description
*       This variable holds the overall
*       amount of memory allocated by the driver in kernel space.
*       Measured in bytes.
*****************************************************************************/
extern unsigned long mem_allocated;

static const char VERSION_STRING[] = "Version 0.7.1";

static struct sx dev_mem_lock;
SX_SYSINIT(dev_mem_lock, &dev_mem_lock, "usdm_drv");
static user_mem_dev_t *mem_dev_numa = NULL;

static volatile u_int ref_count;

static struct cdev *qae_dbg_slabs_file = NULL;

static void unregister_mem_device_driver(void);
STATIC void qae_mem_exit(void);
extern int handle_other_ioctls(uint32_t cmd);
/******************************************************************************
 * debug: /sys/kernel/debug/qae_mem_dbg directory
 * qae_mem_slabs file
 * cat qae_mem_slabs shows the allocated slabs for each process with the
 * physical and virtual start address
 * echo "d processid virt_addr" > qae_mem_slabs
 * echo "d processid phys_addr" > qae_mem_slabs
 * write dump command to debug file, the next cat qae_mem_slabs command
 * shows the 256 byte from address in hex and ascii format
 * echo "c processid slabid" > qae_mem_slabs
 * write dump command to debug file, the next cat qae_mem_slabs command
 * shows the 32 x 64 allocation bit map for small buffers allocations
 ******************************************************************************/

/*****************************************************************************
                             memory mgt code begin
*****************************************************************************/

/*
 * Find memory information
 */
static kdev_mem_info_t *kernelMemGetInfo(uint64_t id)
{
    user_proc_mem_list_t *list = NULL;
    kdev_mem_info_t *kmem = NULL;
    int error;

    error = devfs_get_cdevpriv((void **)&list);
    if (error)
    {
        printf("%s:%d Private data is missing\n", __func__, __LINE__);
        return NULL;
    }
    if (!list)
    {
        printf("%s:%d empty list\n",__func__,__LINE__);
        return NULL;
    }
    kmem = list->head;
    while (kmem)
    {
        if (kmem->phy_addr == id)
        {
            return kmem;
        }
        kmem = kmem->pNext_kernel;
    }
    return NULL;
}

/*
 * Allocate numa memory
 */
static dev_mem_info_t *userMemAlloc(uint32_t size,
                                    int node,
                                    uint32_t large_memory)
{
    dev_mem_info_t *mem_info = NULL;
    kdev_mem_info_t *kmem_info = NULL;
    user_proc_mem_list_t *list = NULL;
    uint32_t status = 0;
    block_ctrl_t *block_ctrl = NULL;
    uint32_t totalInKBytes = 0;
    void *phy_addr = NULL;
    int error =0;

    if (!size)
    {
        printf(
            "%s:%d Invalid parameter value [%u]\n", __func__, __LINE__, size);
        return NULL;
    }

#ifdef NUMA_SUPPORTED
    if (node != NUMA_NO_NODE)
    {
        /* node 0.. (MAX_NUMNODES-1) */
        if (node >= 0 && node < MAX_NUMNODES)
        {
            if (!node_online(node))
            {
                printf("%s:%d Requested node %d is not online. "
                       "Using node 0 as default\n",
                       __func__,
                       __LINE__,
                       node);
                node = 0;
            }
        }
        else
        {
            /* greater than MAX_NUMNODES  */
            printf("%s:%d Requested node %d not present. "
                   "Using node 0 as default\n",
                   __func__,
                   __LINE__,
                   node);
            node = 0;
        }
    }
#endif

    /* Find the process allocation list */
    error = devfs_get_cdevpriv((void **)&list);
    if (error)
    {
        printf("%s:%d Private data is missing\n", __func__, __LINE__);
        return NULL;
    }
    if (!list)
    {
        printf("%s:%d User process memory list is NULL \n", __func__, __LINE__);
        return NULL;
    }

    size = icp_iommu_get_remapping_size(size);
    totalInKBytes = (mem_allocated + size) / QAE_KBYTE;
    if ((mem_allocated + size) % QAE_KBYTE)
    {
        totalInKBytes += 1;
    }

    /* for request > 2M mem_info control block allocated separately */
    if (large_memory)
    {
        /* one page is used for block control information */
        const uint32_t pageSizeInKb = PAGE_SIZE / QAE_KBYTE;
        if (max_mem_numa && max_mem_numa < (totalInKBytes + pageSizeInKb))
        {
            printf("%s:%d Maximum NUMA allocation of %u kB reached "
                   "currently allocated %lu bytes requested %u bytes\n",
                   __func__,
                   __LINE__,
                   max_mem_numa,
                   mem_allocated,
                   (size + PAGE_SIZE));
            return NULL;
        }
        mem_info = malloc(PAGE_SIZE, M_QAE_MEM, M_WAITOK | M_ZERO);
        if (!mem_info)
        {
            printf("%s:%d Unable to allocate control block\n",
                    __func__,__LINE__);
            return NULL;
        }
        kmem_info = malloc(sizeof(kdev_mem_info_t), M_QAE_MEM, M_WAITOK | M_ZERO);
        if (!kmem_info)
        {
            printf("%s:%d Unable to allocate Kernel control block\n",
                   __func__,__LINE__);
            free(mem_info, M_QAE_MEM);
            return NULL;
        }
        kmem_info->kmalloc_ptr = contigmalloc(
            size, M_QAE_MEM, M_WAITOK | M_ZERO, 0, ~0, PAGE_SIZE, 0);
        if (!kmem_info->kmalloc_ptr)
        {
            printf("%s:%d Unable to allocate memory slab size %u \n",
                   __func__,
                   __LINE__,
                   size);
            free(kmem_info, M_QAE_MEM);
            free(mem_info, M_QAE_MEM);
            return NULL;
        }
        /* Initialize the huge page control */
        kmem_info->huge_mem_ctrl = mem_info;
        /* Update slab size */
        kmem_info->size = size;
        /* Update allocated size */
        mem_allocated += (size + PAGE_SIZE);
    }
    else
    {
        if (max_mem_numa && max_mem_numa < totalInKBytes)
        {
            printf("%s:%d Maximum NUMA allocation of %u kB reached"
                   " currently allocated %lu bytes requested %u bytes\n",
                   __func__,
                   __LINE__,
                   max_mem_numa,
                   mem_allocated,
                   size);
            return NULL;
        }

        /*
         * Allocating a physically contiguous block of memory here
         * without using M_ZERO flag.
         * A small part of the allocated memory is used to hold the
         * block_ctrl structure and the rest will be used by userspace.
         */
        block_ctrl =
            contigmalloc(size, M_QAE_MEM, M_WAITOK, 0, ~0, PAGE_SIZE, 0);
        if (!block_ctrl)
        {
            printf(
                "%s:%d Unable to allocate memory slab \n", __func__, __LINE__);
            return NULL;
        }

        /*
         * Since the control block is not zeroed so we zero it explicitly.
         * The returned control block values should mostly be initialised
         * to zero.
         */
#if __FreeBSD_version < 1100000
        memset(block_ctrl, 0, sizeof(block_ctrl_t));
#else
        explicit_bzero(block_ctrl, sizeof(block_ctrl_t));
#endif

        kmem_info = malloc(sizeof(kdev_mem_info_t), M_QAE_MEM, M_WAITOK | M_ZERO);
        if (!kmem_info)
        {
             printf("%s:%d Unable to allocate Kernel control block\n",
                    __func__,__LINE__);
             contigfree(block_ctrl, size, M_QAE_MEM);
             return NULL;
        }
        /*
         * block_ctrl structure is used to give info about allocated block.
         * mem_info strcture in the block_ctrl contains information on the
         * kernel virtual address to the allocated memory, physcial address,
         * the size of the memory and memory type.
         */
        mem_info = &block_ctrl->mem_info;
        kmem_info->kmalloc_ptr = block_ctrl;
        /* Huge page control block not applicable here for small slabs. */
        kmem_info->huge_mem_ctrl = NULL;
        kmem_info->size = size;
        mem_allocated += size;
    }
    mem_info->nodeId = node;
    mem_info->size = size;
    mem_info->type = large_memory;
    status = icp_iommu_map(&phy_addr, kmem_info->kmalloc_ptr, mem_info->size);
    if (status)
    {
        printf("%s:%d iommu map failed\n",__func__,__LINE__);
        if (LARGE == mem_info->type)
        {
            free(mem_info, M_QAE_MEM);
            mem_allocated -= PAGE_SIZE;
        }
        contigfree(kmem_info->kmalloc_ptr, size, M_QAE_MEM);
        free(kmem_info, M_QAE_MEM);
        mem_allocated -= size;
        return NULL;
    }

    mem_info->phy_addr = (uintptr_t)phy_addr;
    kmem_info->phy_addr = (uintptr_t)phy_addr;
    list->allocs_nr++;
    ADD_ELEMENT_TO_END_LIST(kmem_info, list->head, list->tail, _kernel);
    return mem_info;
}

/*
 * Free memory
 */
static void userMemFree(uint64_t id)
{
    kdev_mem_info_t *kmem_info = NULL;
    user_proc_mem_list_t *list = NULL;
    size_t size = 0;
    int error;

    error = devfs_get_cdevpriv((void **)&list);
    if (error)
    {
        printf("%s:%d Private data is missing\n", __func__, __LINE__);
        return;
    }
    if (!list)
    {
        printf("%s:%d No slab to free\n", __func__, __LINE__);
        return;
    }
    kmem_info = list->head;
    while (kmem_info)
    {
        if (kmem_info->phy_addr == id)
        {
            size = kmem_info->size;
            icp_iommu_unmap((void *)(uintptr_t)kmem_info->phy_addr, size);
            REMOVE_ELEMENT_FROM_LIST(kmem_info, list->head, list->tail, _kernel);

            if (kmem_info->huge_mem_ctrl)
            {
                free(kmem_info->huge_mem_ctrl, M_QAE_MEM);
                mem_allocated -= PAGE_SIZE;
            }
            contigfree(kmem_info->kmalloc_ptr, kmem_info->size, M_QAE_MEM);
            free(kmem_info, M_QAE_MEM);
            mem_allocated -= size;
            list->allocs_nr -= 1;
            return;
        }
        kmem_info = kmem_info->pNext_kernel;
    }
    printf("%s:%d Could not find slab with id: %ju \n",
           __func__,
           __LINE__,
           (uintmax_t)id);
    return;
}

/*
 * Clean all memory for a process
 */
static int userMemFreeSlabs(user_proc_mem_list_t *list)
{
    kdev_mem_info_t *kmem_info = NULL, *next = NULL;

    if (!list)
    {
        printf("%s:%d No slab to free\n", __func__, __LINE__);
        return EIO;
    }
    kmem_info = list->head;
    while (kmem_info)
    {
#ifdef ICP_DEBUG
        printf("%s:%d Potential memory leak, Process Id %d"
               " Virtual address %p "
               "Physical address %px has allocated block\n",
               __func__,
               __LINE__,
               list->pid,
               kmem_info->kmalloc_ptr,
               (void *)kmem_info->phy_addr);
#endif
        next = kmem_info->pNext_kernel;
        icp_iommu_unmap((void *)(uintptr_t)kmem_info->phy_addr, kmem_info->size);
        REMOVE_ELEMENT_FROM_LIST(kmem_info, list->head, list->tail, _kernel);
        if (kmem_info->huge_mem_ctrl)
        {
            free(kmem_info->huge_mem_ctrl, M_QAE_MEM);
        }
        contigfree(kmem_info->kmalloc_ptr, kmem_info->size, M_QAE_MEM);
        free(kmem_info, M_QAE_MEM);
        kmem_info = next;
    }
    mem_allocated = 0;
    return 0;
}

/*****************************************************************************
                              memory mgt code end
*****************************************************************************/
static int dev_mem_alloc(caddr_t arg)
{
    dev_mem_info_t *mem_info = NULL, *user_mem_info = NULL;
    user_mem_info = (dev_mem_info_t *)arg;
    if (user_mem_info->size > QAE_MEM_SIZE_LIMIT)
    {
        printf("%s:%d Invalid size value, size=%lu\n",
               __func__,
               __LINE__,
               user_mem_info->size);
        return -EIO;
    }
    mem_info = userMemAlloc(
        user_mem_info->size, user_mem_info->nodeId, user_mem_info->type);
    if (!mem_info)
    {
        printf("%s:%d userMemAlloc failed\n", __func__, __LINE__);
        return ENOMEM;
    }
    *user_mem_info = *mem_info;
    return 0;
}

static int dev_mem_free(caddr_t arg)
{
    dev_mem_info_t *user_mem_info = NULL;

    user_mem_info = (dev_mem_info_t *)arg;
    userMemFree(user_mem_info->phy_addr);
    return 0;
}

static int dev_release_pid(void)
{
    user_proc_mem_list_t *list = NULL;
    int error;

    error = devfs_get_cdevpriv((void **)&list);
    if (error)
        return (error);
    return userMemFreeSlabs(list);
}

static int mem_ioctl(struct cdev *dev,
                     u_long cmd,
                     caddr_t arg,
                     int flags,
                     struct thread *td)
{
    int ret = 0;
    switch (cmd)
    {
        case DEV_MEM_IOC_MEMALLOC:
            sx_xlock(&dev_mem_lock);
            ret = dev_mem_alloc(arg);
            sx_xunlock(&dev_mem_lock);
            break;

        case DEV_MEM_IOC_MEMFREE:
            sx_xlock(&dev_mem_lock);
            ret = dev_mem_free(arg);
            sx_xunlock(&dev_mem_lock);
            break;

        case DEV_MEM_IOC_RELEASE:
            sx_xlock(&dev_mem_lock);
            ret = dev_release_pid();
            sx_xunlock(&dev_mem_lock);
            break;

        default:
            ret = handle_other_ioctls(cmd);
    }
    return ret;
}

static int mem_mmap(struct cdev *dev,
                    vm_ooffset_t *offset,
                    vm_size_t size,
                    vm_object_t *object,
                    int nprot)
{
    vm_object_t obj;
    struct sglist *sg;
    uint64_t id = 0;
    kdev_mem_info_t *kmem_info = NULL;
    id = *offset;

    sx_slock(&dev_mem_lock);
    kmem_info = kernelMemGetInfo(id);
    if (!kmem_info)
    {
        printf("%s:%d cannot find kmeminfo\n", __func__, __LINE__);
        sx_sunlock(&dev_mem_lock);
        return ENOMEM;
    }

    /* Ensure memory mapping does not exceed the allocated memory region */
    if (size > kmem_info->size)
    {
        sx_sunlock(&dev_mem_lock);
        printf("%s:%d cannot map allocated memory\n", __func__, __LINE__);
        return ENOMEM;
    }

    sg = sglist_alloc(1, M_WAITOK);

    /* There is an agreement that mmap(PAGE_SIZE) means control block. */
    if (PAGE_SIZE == size)
    {
        sglist_append(sg, kmem_info->huge_mem_ctrl, PAGE_SIZE);
    }
    /* Any other size means memory block. */
    else
    {
        sglist_append(sg, kmem_info->kmalloc_ptr, kmem_info->size);
    }
    sx_sunlock(&dev_mem_lock);
    obj =
        vm_pager_allocate(OBJT_SG, sg, sglist_length(sg), VM_PROT_RW, 0, NULL);
    sglist_free(sg);
    if (obj == NULL)
    {
        printf("%s:%d failed to create VM object", __func__, __LINE__);
        return ENOMEM;
    }

    *offset = 0;
    *object = obj;
    return 0;
}

static void mem_release(void *arg);

static int mem_open(struct cdev *dev,
                    int oflags,
                    int devtype,
                    struct thread *td)
{
    user_proc_mem_list_t *list = NULL;
    int error;

    list = malloc(sizeof(user_proc_mem_list_t), M_QAE_MEM, M_WAITOK | M_ZERO);

    sx_xlock(&dev_mem_lock);
    error = devfs_set_cdevpriv(list, mem_release);
    if (error)
    {
        sx_xunlock(&dev_mem_lock);
        free(list, M_QAE_MEM);
        return error;
    }
    ADD_ELEMENT_TO_END_LIST(
        list, mem_dev_numa->head, mem_dev_numa->tail, _user);
    list->pid = td->td_proc->p_pid;
    sx_xunlock(&dev_mem_lock);

    atomic_add_int(&ref_count, 1);

    return 0;
}

static inline void remove_element(user_proc_mem_list_t *p)
{
    if (NULL == p)
        return;
    if (NULL != p->pPrev_user)
    {
        p->pPrev_user->pNext_user = p->pNext_user;
    }
    if (NULL != p->pNext_user)
    {
        p->pNext_user->pPrev_user = p->pPrev_user;
    }
}

static void mem_release(void *arg)
{
    user_proc_mem_list_t *list = NULL;

    list = arg;
    sx_xlock(&dev_mem_lock);
    if (list)
    {
        (void)userMemFreeSlabs(list);
        if (NULL != mem_dev_numa)
        {
            REMOVE_ELEMENT_FROM_LIST(
                list, mem_dev_numa->head, mem_dev_numa->tail, _user);
        }
        else
        {
            remove_element(list);
        }
        free(list, M_QAE_MEM);
    }
    sx_xunlock(&dev_mem_lock);

    atomic_subtract_int(&ref_count, 1);
}

static struct cdevsw mem_cdevsw = {.d_version = D_VERSION,
                                   .d_name = DEV_MEM_NAME,
                                   .d_open = mem_open,
                                   .d_ioctl = mem_ioctl,
                                   .d_mmap_single = mem_mmap};

static struct cdev *mem_cdev;

static int32_t register_mem_device_driver(void)
{
    mem_dev_numa = malloc(sizeof(user_mem_dev_t), M_QAE_MEM, M_WAITOK | M_ZERO);
    mem_cdev =
        make_dev(&mem_cdevsw, 0, UID_ROOT, GID_WHEEL, 0600, DEV_MEM_NAME);
    if (mem_cdev == NULL)
    {
        printf("failed to create mem numa device driver\n");
        free(mem_dev_numa, M_QAE_MEM);
        return ENODEV;
    }
    return 0;
}

/*
 * unregister the device driver
 */
static void unregister_mem_device_driver(void)
{
    if (mem_cdev != NULL)
    {
        destroy_dev(mem_cdev);
        mem_cdev = NULL;
    }
    free(mem_dev_numa, M_QAE_MEM);
    mem_dev_numa = NULL;
}

static inline char printable(char sym)
{
    if (sym >= 0x20 && sym <= 0x7E)
        /*check if printable ascii*/
        return sym;
    else
        /*else put out a dot*/
        return '.';
}

static char qae_dbg_ascii[128];
static char qae_dbg_command[128];
static char qae_dbg_slab_data[4096];

/*
 * dumps memory data in 16 8 hex bytes and
 * 8 ascii chars columns and 32 rows
 */
static int
dumpData(void *start, void *end)
{
    int row = 0;
    int col = 0;
    char *src = start;
    char *endaddr = end;
    size_t offs = 0;
    const int ROWS = 32;
    const int COLUMNS = 8;

    for (row = 0; row < ROWS; ++row)
    {
        size_t ascii = 0;

        for (col = 0; col < COLUMNS; ++col)
        {
            if (src > endaddr)
            {
                offs += snprintf(qae_dbg_slab_data + offs,
                        sizeof(qae_dbg_slab_data) - offs, "   ");
                ascii += snprintf(qae_dbg_ascii + ascii,
                        sizeof(qae_dbg_ascii) - ascii, "  ");
            }
            else
            {
                /*in the first 8 columns print bytes in hex with 2 nibbles*/
                offs += snprintf(qae_dbg_slab_data + offs,
                                 sizeof(qae_dbg_slab_data) - offs,
                                 "%02hhx ",
                                 (unsigned char)*src);
                /*in the last 8 columns print ascii char or dot*/
                ascii += snprintf(qae_dbg_ascii + ascii,
                        sizeof(qae_dbg_ascii) - ascii, "%c ", printable(*src));
                src++;
            }
        }
        offs += snprintf(qae_dbg_slab_data + offs,
                sizeof(qae_dbg_slab_data) - offs, "%.128s\n", qae_dbg_ascii);
        if (src > endaddr)
            return offs;
    }
    return offs;
}

/*
 * findSlabsForPid - find the link list of slabs for a given pid
 */
static kdev_mem_info_t *findSlabsForPid(pid_t pid)
{
    if (mem_dev_numa)
    {
        user_proc_mem_list_t *list = mem_dev_numa->head;
        while (list)
        {
            if (list->pid == pid)
                return list->head;
            list = list->pNext_user;
        }
    }
    return NULL;
}

/*
 * execute dump command
 * returns length of data in output buffer
 */
static int execDump(kdev_mem_info_t *slab, uintptr_t param, pid_t pid)
{
    uintptr_t endaddr = 0;
    uintptr_t startaddr = param;
    int offset = 0;
    int len = 0;
    printf("Process dump command \n");
    /* traverse thru slabs */
    while (slab)
    {
        uintptr_t phy_addr = (uintptr_t)slab->phy_addr;
        uintptr_t virt_addr = (uintptr_t)slab->kmalloc_ptr;
        /* calculate virtual address end of slab */
        endaddr = virt_addr + slab->size -1;

        /* check if this slab was sought after by virtual address */
        if (startaddr >= virt_addr && startaddr < endaddr)
        {
            offset = startaddr - virt_addr;
            printf("Block Found !!! "
                   "start %p block end %p dump addr %p offset %x\n",
                   (void *) virt_addr,
                   (void *) endaddr,
                   (void *) startaddr,
                   offset);

            break;
        }
        /* calculate physical address end of slab */
        endaddr = phy_addr + slab->size;
        /* check if this slab was sought after by phy address */
        if (startaddr >= phy_addr && startaddr < endaddr)
        {
            offset = startaddr - phy_addr;
            printf("Block Found(using phy_addr) !!! "
                   "start %p block end %p dump addr %p offset %x\n",
                   (void *) phy_addr,
                   (void *) endaddr,
                   (void *) startaddr,
                   offset);
            break;
        }
        /* take next slab if no hit */
        slab = slab->pNext_kernel;
    }
    /* log slab not found */
    if (!slab)
    {
        len = snprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                      "Slab not found PID %u Address %p\n",
                      pid, (void *)startaddr);
    }
    else /* dump 256 byte of slab data */
    {
        startaddr = (uintptr_t)slab + offset;
        endaddr = (uintptr_t)slab + slab->size - 1;
        len = dumpData((void *)startaddr, (void *)endaddr);
    }
    return len;
}

/*
 * execute dump control area command
 * returns length of data in output buffer
 */
static int32_t execDumpControl(kdev_mem_info_t *slab, uintptr_t param, pid_t pid)
{
    uint64_t id = param;
    uintptr_t endaddr = 0;
    int32_t len = 0;
    /* traverse thru slabs search by slab id */
    while (slab)
    {
        endaddr = slab->phy_addr + slab->size;
        if (id >= slab->phy_addr && id < endaddr)
        {
            break;
        }
        slab = slab->pNext_kernel;
    }
    if (!slab) /* log slab not found*/
    {
        len = snprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                      "Slab not found PID %u slab ID %ju\n",
                      pid, (uintmax_t)id);
    }
    else /* dump bitmap */
    {
        int row;
        uint64_t bitmap_row,mask;
        /* banner message */
        len = snprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                      "Small buffer allocation bitmap \n Slab id %ju \n",
                      (uintmax_t)id);
        /* display 0/1 in bitmap positions throughout the bitmap */
        for (row = 0; row < BITMAP_LEN; ++row)
        {
            bitmap_row = ((block_ctrl_t*)slab->kmalloc_ptr)->bitmap[row];
            for (mask = 1ULL << (QWORD_WIDTH - 1); mask; mask >>= 1)
            {
                char bit = '0';
                if (mask & bitmap_row)
                {
                    bit = '1';
                }
                len += snprintf(qae_dbg_slab_data + len,
                    sizeof(qae_dbg_slab_data) - len, "%c", bit);
            }
            len += snprintf(qae_dbg_slab_data + len,
                    sizeof(qae_dbg_slab_data) - len, "\n");
        }
    }
    return len;
}

/* processCommand
 * performs the command found in the command buffer
 * returns the number of characters the debug
 * buffer has after command was executed
 */
static int processCommand(void)
{
    char *arg = NULL;
    char *cmd = NULL;
    int8_t command = '\0';       /* command char c/d */
    uint64_t param = 0;          /* command parameter */
    pid_t pid = 0;               /* process id */
    kdev_mem_info_t *slab = NULL; /* slab the info is required for */
    int32_t len = 0;             /* length of string in output buffer */

    command = qae_dbg_command[0];
    if ('\0' == command) /* check if there is a command */
    {
        return 0;
    }
    /* get params */
    cmd = strstr(qae_dbg_command, " ");
    if (cmd) {
        cmd++;
        arg = strsep(&cmd, " ");
        if (NULL != arg) {
            pid = strtoul(arg, 0, 0);

            /* Find a next argument. */
            arg = strsep(&cmd, " ");
            if (NULL != arg)
            {
                param = strtouq(arg, 0, 0);
            }
        }
        printf("%s:%d Requested command %c Param %u %llux Buffer %s\n",
               __func__,
               __LINE__,
               command,
               pid,
               (unsigned long long)param,
               qae_dbg_command);
    }
    /* Destroy the original command. */
    qae_dbg_command[0] = '\0';

    switch (command)
    {
        case 'd':
            slab = findSlabsForPid(pid); /* find slab for process id */
            if (!slab)
            {
                printf("%s:%d "
                        "Could not find slab for process id: %d\n",
                        __func__,
                        __LINE__,
                        pid);
                return 0;
            }
            /* dump memory content */
            len = execDump(slab, param, pid);
            break;
        case 'c':
            slab = findSlabsForPid(pid); /* find slab for process id */
            if (!slab)
            {
                printf("%s:%d "
                        "Could not find slab for process id: %d\n",
                        __func__,
                        __LINE__,
                        pid);
                return 0;
            }
            /* control block data (bitmap) */
            len = execDumpControl(slab, param, pid);
            break;
        case 't':
            /* print total allocated NUMA memory */
            len = snprintf(qae_dbg_slab_data,
                           sizeof(qae_dbg_slab_data),
                           "Total allocated NUMA memory: %lu bytes\n",
                           mem_allocated);
            break;
        default:
            len = snprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                    "Invalid command %c\n", command);
            break;
    }
    return len;
}

/*
 * print info about a slab in debug buffer
 * return number of byte in buffer
 * 0 return value will end the file read operation
 * each time this function is called one slab data
 * is entered in the debug buffer
 * process and slab ptrs are saved in static variables
 * to traverse the linked list by file read until a 0
 * return value is received.
 */
static int getMemInfo(user_proc_mem_list_t **pmem_list)
{
    /* memory info for slab in memory list */
    static kdev_mem_info_t *kmem_info;
    /* memory list element of current process */
    user_proc_mem_list_t *mem_list = *pmem_list;
    int length = 0;
    /* initialise list of processes that allocated slabs */
    if (!kmem_info && !mem_list)
    {
        mem_list = mem_dev_numa->head;
        /* return if list is empty */
        if (!mem_list)
            return 0;
        kmem_info = mem_list->head;
    }
    /* iterate through all processes in the list */
    while(mem_list)
    {
        /* check if there is a valid slab entry */
        if (kmem_info)
        {
            length = snprintf(qae_dbg_slab_data, sizeof(qae_dbg_slab_data),
                "Pid %d, Slab Id %ju \n"
                "Virtual address %p, Physical Address %llux, Size %llu\n",
                mem_list->pid,
                kmem_info->phy_addr,
                kmem_info->kmalloc_ptr,
                (unsigned long long)kmem_info->phy_addr,
                (unsigned long long)kmem_info->size);
            /* advance slab pointer for next call */
            kmem_info = kmem_info->pNext_kernel;
            /* send slab info into read buffer */
            break;
        }
        else
        {
            /* null slab ptr in list of previous process
             * get next process from list
             */
            mem_list = mem_list->pNext_user;
            /* get first slab from next list element */
            if (mem_list)
                kmem_info = mem_list->head;
        }
    }
    /* if at the end of process list chain */
    if (!mem_list)
    {
        mem_list = mem_dev_numa->head;
        kmem_info = NULL;
    }
    /* save current process in list in a static for next call */
    *pmem_list = mem_list;
    return length;
}

/*
*qae_mem_update_slab_data
* updates data in debug buffer depending on last command
* open - non-null if called from debug file open routine
*       otherwise 0
*/
static int qae_mem_update_slab_data(int open)
{
    /* memory list of current process */
    static user_proc_mem_list_t *mem_list;
    static int count; /* number of chars in debug buffer */
    if (!mem_dev_numa)
        return 0;
    /* if file just opened initialise; make sure
     * list of slabs are generated from the top
     * if qae_dbg_command buffer is empty
     */
    if (open)
    {
        mem_list = NULL;
        count = 0;
        return 0;
    }
    /* last time a buffer with chars were sent in response to read operation
     * return 0 now to complete read operation.
     */
    if (count)
    {
        count = 0;
        return 0;
    }
    /* process command and report to read op if there is any result */
    count = processCommand();
    if (count)
        return count;
    /* get next slab info into debug data buffer
     * when 0 is returned it marks the end of buffer list
     * and will end the file read operation as well
     */
    return getMemInfo(&mem_list);
}

/*
 * read function for debug file
 */
static int qae_mem_slabs_data_read(struct cdev *dev,
                                   struct uio *uio,
                                   int ioflag)
{
    /* update data in debug buffer */
    int data_len = qae_mem_update_slab_data(0);
    /* check length and position */
    if (0 == data_len || uio->uio_offset >= data_len)
       return 0;
    return (uiomove(
       qae_dbg_slab_data + uio->uio_offset, data_len - uio->uio_offset, uio));
}

/*
 * write function for write operation of the debug file
 */
static int qae_mem_slabs_data_write(struct cdev *dev,
                                    struct uio *uio,
                                    int ioflag)
{
    ssize_t oresid;
    int error;

    /* write command to qae_dbg_command buffer
     * next read on debug file will parse the command string
     * and execute the requested command
     * if command buffer empty the next read
     * lists the allocated slabs
     * check count vs size of command buffer
     */
    if (uio->uio_resid >= sizeof(qae_dbg_command))
        return EFBIG;
    oresid = uio->uio_resid;
    error = uiomove(qae_dbg_command, sizeof(qae_dbg_command) - 1, uio);
    if (error)
    {
        qae_dbg_command[0] = '\0';
        return error;
    }
    /* terminating 0 */
    qae_dbg_command[oresid - uio->uio_resid] = '\0';
    return 0;
}

/*
 * called when debug file is opened
 * used for initialisation
 */
static int qae_mem_slabs_data_open(struct cdev *dev,
                                   int oflags,
                                   int devtype,
                                   struct thread *td)
{
    qae_mem_update_slab_data(1);
    return 0;
}

static struct cdevsw qae_mem_slabs_cdevsw = {.d_version = D_VERSION,
                                             .d_open = qae_mem_slabs_data_open,
                                             .d_read = qae_mem_slabs_data_read,
                                             .d_write =
                                                 qae_mem_slabs_data_write};

/*
 * Initialisation function to insmod device driver
 */
static inline void qae_debug_init(void)
{
    int slabs_enabled;

    slabs_enabled = 0;
    TUNABLE_INT_FETCH("debug.usdm_drv.enable_slabs", &slabs_enabled);
    if (slabs_enabled)
        qae_dbg_slabs_file = make_dev(&qae_mem_slabs_cdevsw,
                                      0,
                                      UID_ROOT,
                                      GID_WHEEL,
                                      0600,
                                      "qae_mem_slabs");
}

static int qae_mem_init(void)
{
    printf("Loading QAE MEM Module %s ...\n", VERSION_STRING);
    if (register_mem_device_driver())
    {
        printf("Error loading QAE MEM Module\n");
        return -1;
    }
    qae_debug_init();
    return 0;
}

/*
 * tear down function to rmmod device driver
 */
STATIC void qae_mem_exit(void)
{
    printf("Unloading QAE MEM Module %s...\n", VERSION_STRING);
    unregister_mem_device_driver();
    if (NULL != qae_dbg_slabs_file)
    {
        destroy_dev(qae_dbg_slabs_file);
        qae_dbg_slabs_file = NULL;
    }
}

static int qae_mem_modevent(module_t mod __unused,
                            int type,
                            void *data __unused)
{

    switch (type)
    {
        case MOD_LOAD:
            atomic_store_rel_int(&ref_count, 0);
            qae_mem_init();
            return (0);
        case MOD_UNLOAD:
            if (atomic_load_acq_int(&ref_count) == 0)
            {
                qae_mem_exit();
                return (0);
            } else {
                return (EBUSY);
            }
        default:
            return (EOPNOTSUPP);
    }
}

DEV_MODULE(usdm_drv, qae_mem_modevent, NULL);
MODULE_VERSION(usdm_drv, 1);
