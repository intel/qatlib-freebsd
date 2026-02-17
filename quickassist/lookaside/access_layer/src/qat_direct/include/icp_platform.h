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
 * @file icp_platform.h
 *
 * @description
 *      This file contains the common macros and includes OS platform
 *      specific includes
 *
 *****************************************************************************/
#ifndef ICP_PLATFORM_H
#define ICP_PLATFORM_H

/* Including platform specific header */
#if   (defined(__freebsd) && defined(__KERNEL__))
#include "icp_platform_freebsd.h"
#elif (!(defined(__KERNEL__)))
#include "icp_platform_user.h"
#endif

/************************************************************
 * OS Agnostic MACROS
 ************************************************************/
#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FAIL
#define FAIL 1
#endif

#ifdef ICP_PARAM_CHECK
#define ICP_CHECK_FOR_NULL_PARAM(param)                                        \
    do                                                                         \
    {                                                                          \
        if (NULL == param)                                                     \
        {                                                                      \
            ADF_ERROR("%s(): invalid param: %s\n", __FUNCTION__, #param);      \
            return CPA_STATUS_INVALID_PARAM;                                   \
        }                                                                      \
    } while (0)

#define ICP_CHECK_FOR_NULL_PARAM_VOID(param)                                   \
    do                                                                         \
    {                                                                          \
        if (NULL == param)                                                     \
        {                                                                      \
            ADF_ERROR("%s(): invalid param: %s\n", __FUNCTION__, #param);      \
            return;                                                            \
        }                                                                      \
    } while (0)

#define ICP_CHECK_PARAM_RANGE(param, min, max)                                 \
    do                                                                         \
    {                                                                          \
        if (param >= max || param < min)                                       \
        {                                                                      \
            ADF_ERROR("%s(): invalid param: %s\n", __FUNCTION__, #param);      \
            return CPA_STATUS_INVALID_PARAM;                                   \
        }                                                                      \
    } while (0)
#define ICP_CHECK_PARAM_LT_MAX(param, max)                                     \
    do                                                                         \
    {                                                                          \
        if (param >= max)                                                      \
        {                                                                      \
            ADF_ERROR("%s(): invalid param: %s\n", __FUNCTION__, #param);      \
            return CPA_STATUS_INVALID_PARAM;                                   \
        }                                                                      \
    } while (0)
#define ICP_CHECK_PARAM_GT_MIN(param, min)                                     \
    do                                                                         \
    {                                                                          \
        if (param <= min)                                                      \
        {                                                                      \
            ADF_ERROR("%s(): invalid param: %s\n", __FUNCTION__, #param);      \
            return CPA_STATUS_INVALID_PARAM;                                   \
        }                                                                      \
    } while (0)
#else
#define ICP_CHECK_FOR_NULL_PARAM(param)
#define ICP_CHECK_FOR_NULL_PARAM_VOID(param)
#define ICP_CHECK_PARAM_RANGE(param, min, max)
#define ICP_CHECK_PARAM_LT_MAX(param, max)
#define ICP_CHECK_PARAM_GT_MIN(param, min)
#endif

#define ICP_CHECK_STATUS(status)                                               \
    do                                                                         \
    {                                                                          \
        if (CPA_STATUS_SUCCESS != (status))                                    \
        {                                                                      \
            return status;                                                     \
        }                                                                      \
    } while (0)

#define ICP_CHECK_STATUS_AND_LOG(status, format, args...)                      \
    do                                                                         \
    {                                                                          \
        if (CPA_STATUS_SUCCESS != status)                                      \
        {                                                                      \
            ADF_ERROR(format, args);                                           \
            return CPA_STATUS_FAIL;                                            \
        }                                                                      \
    } while (0)

#define ICP_CHECK_STATUS_AND_LOG_NORETURN(status, format, args...)             \
    do                                                                         \
    {                                                                          \
        if (CPA_STATUS_SUCCESS != status)                                      \
        {                                                                      \
            ADF_ERROR(format, args);                                           \
        }                                                                      \
    } while (0)

/*Macro to try to find an element in a linked list.
 *The Macro compares pointer values between the pointer and the list
 *to determine if the pointer is in the list. */
#define ICP_FIND_ELEMENT_IN_LIST(elementtofind, listhead, status)              \
    do                                                                         \
    {                                                                          \
        if (NULL == listhead)                                                  \
        {                                                                      \
            status = CPA_STATUS_FAIL;                                          \
        }                                                                      \
        while (listhead != NULL)                                               \
        {                                                                      \
            if (listhead == elementtofind)                                     \
            {                                                                  \
                status = CPA_STATUS_SUCCESS;                                   \
                break;                                                         \
            }                                                                  \
            else                                                               \
            {                                                                  \
                listhead = listhead->pNext;                                    \
            }                                                                  \
        }                                                                      \
    } while (0)

/*Macro for adding an element to the tail of a doubly linked list*/
/*The currentptr tracks the tail, and the headptr tracks the head.*/
#define ICP_ADD_ELEMENT_TO_END_OF_LIST(elementtoadd, currentptr, headptr)      \
    do                                                                         \
    {                                                                          \
        if (NULL == currentptr)                                                \
        {                                                                      \
            currentptr = elementtoadd;                                         \
            elementtoadd->pNext = NULL;                                        \
            elementtoadd->pPrev = NULL;                                        \
            headptr = currentptr;                                              \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            elementtoadd->pPrev = currentptr;                                  \
            currentptr->pNext = elementtoadd;                                  \
            elementtoadd->pNext = NULL;                                        \
            currentptr = elementtoadd;                                         \
        }                                                                      \
    } while (0)

/*currentptr is not used in this case since we don't track the tail. */
#define ICP_ADD_ELEMENT_TO_HEAD_OF_LIST(elementtoadd, currentptr, headptr)     \
    do                                                                         \
    {                                                                          \
        if (NULL == headptr)                                                   \
        {                                                                      \
            elementtoadd->pNext = NULL;                                        \
            elementtoadd->pPrev = NULL;                                        \
            headptr = elementtoadd;                                            \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            elementtoadd->pPrev = NULL;                                        \
            elementtoadd->pNext = headptr;                                     \
            headptr->pPrev = elementtoadd;                                     \
            headptr = elementtoadd;                                            \
        }                                                                      \
    } while (0)

#define ICP_REMOVE_ELEMENT_FROM_LIST(elementtoremove, currentptr, headptr)     \
    do                                                                         \
    {                                                                          \
        /*If the previous pointer is not NULL*/                                \
        if (NULL != elementtoremove->pPrev)                                    \
        {                                                                      \
            elementtoremove->pPrev->pNext = elementtoremove->pNext;            \
            if (elementtoremove->pNext)                                        \
            {                                                                  \
                elementtoremove->pNext->pPrev = elementtoremove->pPrev;        \
            }                                                                  \
            else                                                               \
            {                                                                  \
                /* Move the tail pointer backwards */                          \
                currentptr = elementtoremove->pPrev;                           \
            }                                                                  \
        }                                                                      \
        else if (NULL != elementtoremove->pNext)                               \
        {                                                                      \
            /*Remove the head pointer.*/                                       \
            elementtoremove->pNext->pPrev = NULL;                              \
            /*Hence move the head forward.*/                                   \
            headptr = elementtoremove->pNext;                                  \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            /*Remove the final entry in the list. */                           \
            currentptr = NULL;                                                 \
            headptr = NULL;                                                    \
        }                                                                      \
    } while (0)

/* Macro to free a linked list */
#define ICP_FREE_LIST(current_ptr)                                             \
    do                                                                         \
    {                                                                          \
        if (NULL != current_ptr)                                               \
        {                                                                      \
            while (NULL != current_ptr->pNext)                                 \
            {                                                                  \
                current_ptr = current_ptr->pNext;                              \
                ICP_FREE(current_ptr->pPrev);                                  \
            }                                                                  \
            ICP_FREE(current_ptr);                                             \
        }                                                                      \
    } while (0)

/* Logging macros */
extern char *icp_module_name;
#define xprintk(level, level_str, fmt, args...)                                \
    osalStdLog(level "%s %s: %s: " fmt,                                        \
               icp_module_name,                                                \
               level_str,                                                      \
               (__func__),                                                     \
               ##args)

#define ADF_PRINT(format, args...) printk(format, ##args)
#define ADF_ERROR(format, args...) xprintk(KERN_ERR, "err", format, ##args)
#ifdef _DEBUG_
#define ADF_DEBUG(format, args...)                                             \
    do                                                                         \
    {                                                                          \
        xprintk(KERN_INFO, "debug", format, ##args);                           \
    } while (0)
#else /*_DEBUG_*/
#define ADF_DEBUG(format, args...)                                             \
    do                                                                         \
    {                                                                          \
    } while (0)
#endif /*_DEBUG_*/

/* memory management */

#define ICP_MEMCPY(d, s, len) osalMemCopy(d, s, len)
#define ICP_MEMSET(d, s, len) osalMemSet(d, s, len)

/* string manipulation */
#define ICP_STRNCMP strncmp
#define ICP_STRNCPY strncpy
#define ICP_STRNLEN strnlen
#define ICP_MAX_STR_LEN (0x7F)

/* time */
#define ICP_GET_TIME osalTimeGet

#define ICP_VIRT_TO_PHYS(addr) OSAL_MMU_VIRT_TO_PHYS(addr)
/* locking  */
#define ICP_SPINLOCK OsalLock
#define ICP_SPINLOCK_INIT(ptr) osalLockInit(ptr, TYPE_IGNORE)
#define ICP_SPINLOCK_LOCK osalLock
#define ICP_SPINLOCK_UNLOCK osalUnlock
#define ICP_SPINLOCK_LOCK_BH osalLockBh
#define ICP_SPINLOCK_UNLOCK_BH osalUnlockBh
#define ICP_SPINLOCK_LOCK_IRQ osalLockIrqSave
#define ICP_SPINLOCK_UNLOCK_IRQ osalUnlockIrqRestore
#define ICP_SPINLOCK_UNINIT osalLockDestroy

#define ICP_MUTEX OsalMutex
#define ICP_MUTEX_INIT osalMutexInit
#define ICP_MUTEX_LOCK(ptr) osalMutexLock(ptr, OSAL_WAIT_FOREVER)
#define ICP_MUTEX_LOCK_INTERRUPTIBLE(ptr) osalMutexLock(ptr, OSAL_WAIT_FOREVER)
#define ICP_MUTEX_TRYLOCK osalMutexTryLock
#define ICP_MUTEX_UNLOCK osalMutexUnlock
#define ICP_MUTEX_UNINIT osalMutexDestroy

#endif /* ICP_PLATFORM_H */
