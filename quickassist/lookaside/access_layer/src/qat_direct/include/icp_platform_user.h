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
 * @file icp_platform_user.h
 *
 * @description
 *      This file contains user space specific macros
 *
 *****************************************************************************/
#ifndef ICP_PLATFORM_USER_H
#define ICP_PLATFORM_USER_H
/* ***********************************************************
 * USER SPACE MACROS
 ************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/param.h>
#include <sys/cpuset.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "Osal.h"
#include <ctype.h>

#define VOLATILE volatile
#define KERN_ERR ""
#define KERN_INFO ""

#define ICP_MDELAY sleep
#define ICP_SSLEEP sleep

/* string conversion*/
#define ICP_STRTOL strtol
#define ICP_STRTOUL strtoul
#define ICP_STRTOULL strtoull

#define printk printf

/* memory */
#define ICP_MALLOC_GEN(size) osalMemAlloc(size)
#define ICP_FREE(ptr)                                                          \
    do                                                                         \
    {                                                                          \
        if (ptr)                                                               \
        {                                                                      \
            osalMemFree(ptr);                                                  \
            ptr = NULL;                                                        \
        }                                                                      \
    } while (0)
#define ICP_ZALLOC_GEN(size) calloc(1, size)
#define ICP_MALLOC_ATOMIC(size) malloc(size)

#ifndef STATIC
#define STATIC static
#endif

#ifndef INLINE
#define INLINE inline
#endif

#ifndef ICP_IOCTL
#define ICP_IOCTL ioctl
#endif

#ifndef ICP_OPEN
#define ICP_OPEN open
#endif

#ifndef ICP_CLOSE
#define ICP_CLOSE close
#endif

#ifndef ICP_WRITE
#define ICP_WRITE write
#endif

#ifndef ICP_ACCESS
#define ICP_ACCESS access
#endif

#define ICP_MMAP(addr, len, prot, flags, fd, offset)                           \
    mmap(addr, len, prot, flags, fd, offset)
#define ICP_MUNMAP(addr_v, len) munmap(addr_v, len)
#define ICP_READ(fd, buf, count) read(fd, buf, count)

#define ICP_MSLEEP(msecs)                                                      \
    do                                                                         \
    {                                                                          \
        if (msecs < 1000)                                                      \
        {                                                                      \
            usleep(1000 * msecs);                                              \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            sleep(msecs / 1000);                                               \
        }                                                                      \
    } while (0)

#define ICP_USLEEP(x) usleep(x)

#define ICP_ISDIGIT isdigit
#define ICP_ISSPACE isspace
#endif /* ICP_PLATFORM_USER_H */
