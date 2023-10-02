/**
 * @file OsalThread.c (user space)
 *
 * @brief POSIX thread implementation.
 *
 *
 * @par
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
 */

#include "Osal.h"

OSAL_PUBLIC OSAL_STATUS osalThreadCreate(OsalThread *thread,
                                         OsalThreadAttr *threadAttr,
                                         OsalVoidFnVoidPtr startRoutine,
                                         void *arg)
{
#ifndef ICP_WITHOUT_THREAD
    INT32 status;
    pthread_attr_t attr;
    struct sched_param param;

    OSAL_LOCAL_ENSURE(startRoutine,
                      "OsalThreadCreate(): Null start routine pointer",
                      OSAL_FAIL);

    if (threadAttr != NULL)
    {
        if (pthread_attr_init(&attr) != 0)
        {

            osalLog(OSAL_LOG_LVL_ERROR,
                    OSAL_LOG_DEV_STDOUT,
                    "\nosalThreadCreate:"
                    "Failed to initialize Thread Attributes!\n");

            return OSAL_FAIL;
        }

        /* Set scheduling policy based on value in threadAttr */

        if ((threadAttr->policy != SCHED_RR) &&
            (threadAttr->policy != SCHED_FIFO) &&
            (threadAttr->policy != SCHED_OTHER))
        {
            osalLog(OSAL_LOG_LVL_DEBUG3,
                    OSAL_LOG_DEV_STDOUT,
                    "\nosalThreadCreate: Invalid policy type! "
                    "Changing to OSAL_THREAD_DEFAULT_SCHED_POLICY\n");
            threadAttr->policy = OSAL_THREAD_DEFAULT_SCHED_POLICY;
        }

        if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) != 0)
        {
            osalLog(OSAL_LOG_LVL_ERROR,
                    OSAL_LOG_DEV_STDOUT,
                    "\nosalThreadCreate:"
                    "Failed to set inherit sched for thread!\n");
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }

        if (pthread_attr_setschedpolicy(&attr, threadAttr->policy) != 0)
        {
            osalLog(OSAL_LOG_LVL_ERROR,
                    OSAL_LOG_DEV_STDOUT,
                    "\nosalThreadCreate:"
                    "Failed to set scheduling policy for thread!\n");
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }

        /* Set priority based on value in threadAttr */
        osalMemSet(&param, 0, sizeof(param));
        param.sched_priority = threadAttr->priority;

        if (threadAttr->policy != SCHED_OTHER)
        {
            if (pthread_attr_setschedparam(&attr, &param) != 0)
            {
                osalLog(OSAL_LOG_LVL_ERROR,
                        OSAL_LOG_DEV_STDOUT,
                        "\nosalThreadCreate:"
                        "Failed to set the sched parameters attribute!\n");
                pthread_attr_destroy(&attr);
                return OSAL_FAIL;
            }
        }

        /* Always create detached thread as JOINABLE thread is not needed by
         * any of the OSAL users currently */

        if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0)
        {
            osalLog(OSAL_LOG_LVL_ERROR,
                    OSAL_LOG_DEV_STDOUT,
                    "\nosalThreadCreate:"
                    "Failed to set the dettachState attribute!\n");
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }

        if (pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM) != 0)
        {
            osalLog(OSAL_LOG_LVL_ERROR,
                    OSAL_LOG_DEV_STDOUT,
                    "\nosalThreadCreate: "
                    "Failed to set the system scope attribute !!!\n");
            pthread_attr_destroy(&attr);
            return OSAL_FAIL;
        }
    } /* end of if(threadAttr != NULL) */

    status = pthread_create(thread,
                            (threadAttr) ? &attr : NULL,
                            (void *(*)(void *))startRoutine,
                            arg);
    if (status)
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDOUT,
                "\nosalThreadCreate: Failed to create the thread!\n");
        pthread_attr_destroy(&attr);
        return OSAL_FAIL;
    }

    if (threadAttr != NULL)
    {
        pthread_attr_destroy(&attr);
    }
#endif
    return OSAL_SUCCESS;
} /* osalThreadCreate */

OSAL_PUBLIC void osalThreadBind(OsalThread *pTid, UINT32 cpu)
{
#ifndef ICP_WITHOUT_THREAD
    cpu_set_t cpuSet;

    /* Initialize the cpuSet to zero */
    CPU_ZERO(&cpuSet);
    /* Set given cpu in cpuSet */
    CPU_SET(cpu, &cpuSet);
    /* Bind given thread to requested CPU core */
    if (pthread_setaffinity_np(*pTid, sizeof(cpuSet), &cpuSet) != 0)
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDOUT,
                "\nosalThreadBind: Failed to bind the thread to requested "
                "core!\n");
        return;
    }
    /* Obtain actual thread CPU affinity */
    if (pthread_getaffinity_np(*pTid, sizeof(cpuSet), &cpuSet) != 0)
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDOUT,
                "\nosalThreadBind: Failed to obtain bounded thread "
                "affinity!\n");
        return;
    }
    /* Check if thread is bound as requested */
    if (!CPU_ISSET(cpu, &cpuSet))
    {
        osalLog(OSAL_LOG_LVL_WARNING,
                OSAL_LOG_DEV_STDOUT,
                "\nosalThreadBind: thread is not bound with requested core!\n");
        return;
    }
#endif
}

OSAL_PUBLIC OSAL_STATUS osalThreadStart(OsalThread *thread)
{
#ifndef ICP_WITHOUT_THREAD
    osalLog(OSAL_LOG_LVL_DEBUG3,
            OSAL_LOG_DEV_STDOUT,
            "\nosalThreadStart: Nothing to be done!\n");
#endif
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS osalThreadKill(OsalThread *thread)
{
#ifndef ICP_WITHOUT_THREAD
    INT32 status;

    OSAL_LOCAL_ENSURE(
        thread, "osalThreadKill(): Null thread pointer", OSAL_FAIL);

    status = pthread_cancel(*thread);

    if (status)
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDOUT,
                "\nosalThreadKill: Failed to cancel the thread!\n");
        return OSAL_FAIL;
    }
#endif
    return OSAL_SUCCESS;
} /* osalThreadKill */

OSAL_PUBLIC OSAL_STATUS osalThreadPrioritySet(OsalThread *thread,
                                              UINT32 priority)
{
#ifndef ICP_WITHOUT_THREAD
    osalLog(OSAL_LOG_LVL_ERROR,
            OSAL_LOG_DEV_STDOUT,
            "\nosalThreadPrioritySet(): not implemented on this OS \n !");
#endif
    return OSAL_SUCCESS;
}

OSAL_PUBLIC void osalThreadExit(void)
{
#ifndef ICP_WITHOUT_THREAD
    pthread_exit(NULL);
#endif
}

/* API to set scheduling policy and priority of an executing thread */

OSAL_PUBLIC OSAL_STATUS osalThreadSetPolicyAndPriority(OsalThread *thread,
                                                       UINT32 policy,
                                                       UINT32 priority)
{
#ifndef ICP_WITHOUT_THREAD
    int ret = 0;
    struct sched_param param;
    int minPrio, maxPrio, policy1;

    OSAL_LOCAL_ENSURE(thread,
                      "osalThreadSetPolicyAndPriority(): Null thread pointer",
                      OSAL_FAIL);

    /* check for a valid value for 'policy' */
    if ((policy != SCHED_RR) && (policy != SCHED_FIFO) &&
        (policy != SCHED_OTHER))
    {
        osalLog(
            OSAL_LOG_LVL_ERROR,
            OSAL_LOG_DEV_STDOUT,
            "osalThreadSetPolicyAndPriority(): invalid value for policy!\n");

        return OSAL_FAIL;
    }

    osalMemSet(&param, 0, sizeof(param));

    if (pthread_getschedparam(*thread, &policy1, &param) != 0)
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDOUT,
                "osalThreadSetPolicyAndPriority(): could not get sched param"
                "!\n");
        return OSAL_FAIL;
    }

    minPrio = sched_get_priority_min(policy);
    maxPrio = sched_get_priority_max(policy);

    if ((priority < minPrio) || (priority > maxPrio))
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDOUT,
                "osalThreadPrioritySet(): Erroneous priority!\n");

        return OSAL_FAIL;
    }

    param.sched_priority = priority;
    ret = pthread_setschedparam(*thread, policy, &param);

    if (ret != 0)
    {
        osalLog(OSAL_LOG_LVL_ERROR,
                OSAL_LOG_DEV_STDOUT,
                "osalThreadSetPolicyAndPriority:"
                "Policy and Priority Set operation failed!\n");
        return OSAL_FAIL;
    }
#endif
    return OSAL_SUCCESS;
}
