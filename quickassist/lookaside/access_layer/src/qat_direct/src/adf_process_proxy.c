/******************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 *****************************************************************************/

/******************************************************************************
 * @file adf_process_proxy.c
 *
 * @description
 * User space interface to ADF in kernel space
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/param.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#ifndef ICP_WITHOUT_THREAD
#include <pthread.h>
#endif
#include "cpa.h"
#include "icp_accel_devices.h"
#include "icp_platform.h"
#include "adf_platform.h"
#include "adf_kernel_types.h"
#include "icp_adf_init.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_transport.h"
#include "adf_transport_ctrl.h"
#include "adf_user_init.h"
#include "adf_user_transport.h"
#include "adf_init.h"
#include "uio_user.h"
#include "uio_user_utils.h"
#include "uio_user_cfg.h"
#include "lac_log.h"
#include <sys/types.h>
#include <sys/event.h>
#include <sys/time.h>

#define ADF_DEV_STATE_PATH "/dev/qat_dev_state"
#define ADF_DEV_PROCESSES_PATH "/dev/qat_dev_processes"
/*
 * Value returned by open function in case of failure
 */
#define OPEN_ERROR -1

/*
 * Number of microsecond
 * that the main thread needs to wait for the reading
 * thread to receive init and start events
 */

#define TIME_DELAY 10
/*
 * Time to sleep between each loop iteration in monitor devices func
 */
#define SLEEP_TIME 2000

/*
 * Proxy init counter
 */
Cpa16U init_ctr = 0;

static struct kevent k_event;
static int kq;
static int state_fd;

/*
 * Mutex guarding serialized access to icp_dev_processes
 */
#ifndef ICP_WITHOUT_THREAD
pthread_mutex_t processes_lock = PTHREAD_MUTEX_INITIALIZER;
#endif
STATIC int process_info_file = -1;

/*
 * Process proxy running state
 */
OsalAtomic process_proxy_status = 0;

/*
 * adf_reset_userProxy
 *
 * Description:
 *  Function to reset the ADF proxy status in user space.
 *  It resets proxy status and related parameters.
 *
 * Returns: void
 */
void adf_reset_userProxy(void);

/*
 * adf_process_proxy_stop
 * Sets the process proxy running state to stopped
 */
STATIC inline void adf_process_proxy_stop(void)
{
    osalAtomicSet(0, &process_proxy_status);
}

/*
 * adf_process_proxy_start
 * Sets the process proxy running state to started
 */
STATIC inline void adf_process_proxy_start(void)
{
    osalAtomicSet(1, &process_proxy_status);
}

STATIC inline Cpa32U adf_process_proxy_running(void)
{
    return (Cpa32U)osalAtomicGet(&process_proxy_status);
}

/*
 * adf_process_proxy_init
 * Init process proxy and connect to kernel space.
 * For every acceleration device in the system open
 * events and rings interface and start event listening threads
 */
STATIC CpaStatus adf_process_proxy_init(void)
{
    if (adf_process_proxy_running())
    {
        ADF_ERROR("Proxy already running\n");
        return CPA_STATUS_FAIL;
    }

    adf_process_proxy_start();
    if (adf_init_devices())
    {
        ADF_ERROR("Error initializing devices\n");
        return CPA_STATUS_FAIL;
    }

    kq = kqueue();
    state_fd = ICP_OPEN(ADF_DEV_STATE_PATH, O_RDONLY);
    EV_SET(&k_event,
           state_fd,
           EVFILT_READ,
           EV_ADD | EV_CLEAR,
           0,
           0,
           (void *)&state_fd);
    kevent(kq, &k_event, 1, NULL, 0, NULL);

    return adf_proxy_get_devices();
}

/*
 * adf_process_proxy_shutdown
 * User space proxy is shutting down. Close and clean all opened devices
 */
static __inline__ CpaStatus adf_process_proxy_shutdown()
{
    kq = 0;
    if (-1 != state_fd)
    {
        ICP_CLOSE(state_fd);
    }

    return adf_cleanup_devices();
}

/*
 * icp_adf_userProxyInit
 * This function is called by the application to bring the proxy up & running
 * Every userspace process has to call it to be able to create rings
 * and receive events.
 */
CpaStatus icp_adf_userProxyInit(char const *const name)
{
    CpaStatus status = CPA_STATUS_FAIL;

    ICP_CHECK_FOR_NULL_PARAM(name);
    /* Allow the user to call init just once */
    if (init_ctr)
    {
        ADF_ERROR("User proxy already initialized\n");
        return status;
    }
    init_ctr = 1;
    /* Connect to kernel space. */
    status = adf_process_proxy_init();
    if (CPA_STATUS_SUCCESS != status)
    {
        ADF_ERROR("adf_process_proxy_init failed\n");
        return CPA_STATUS_FAIL;
    }

    return status;
}

/*
 * icp_adf_userProxyShutdown
 * This function is called by the application to shutdown the proxy
 */
CpaStatus icp_adf_userProxyShutdown(void)
{
    CpaStatus status = CPA_STATUS_SUCCESS;

    adf_process_proxy_stop();
    status = adf_process_proxy_shutdown();
    init_ctr = 0;

    return status;
}

/*
 * icp_adf_userProcessToStart
 *
 *  This function checks if an user space process with a given name has
 *  already been started.
 *
 *  Input:
 *  name_tml           - process name which is also the name of input
 *                       buffer.
 *  name               - output buffer.
 *
 *  Return values:
 *  CPA_STATUS_FAIL    - process with a given name is not started.
 *  CPA_STATUS_SUCCESS - process is started, could open, write and
 *                       read to process file.
 */
CpaStatus icp_adf_userProcessToStart(char const *const name_tml, char *name)
{
    int res = 0, name_len = 0;

    /* Validate process name */
    if (!name_tml || !name)
    {
        ADF_ERROR("Invalid pointer\n");
        return CPA_STATUS_FAIL;
    }

    name_len = strnlen(name_tml, ADF_CFG_MAX_VAL_LEN_IN_BYTES);
    if (name_len + 1 > ADF_CFG_MAX_SECTION_LEN_IN_BYTES || 0 == name_len)
    {
        return CPA_STATUS_FAIL;
    }

#ifndef ICP_WITHOUT_THREAD
    res = pthread_mutex_lock(&processes_lock);
    if (res != 0)
    {
        ADF_ERROR("Mutex lock error %d\n", res);
        return CPA_STATUS_FAIL;
    }
#endif

    if (process_info_file != -1)
    {
        ADF_ERROR("File " ADF_DEV_PROCESSES_PATH " already opened\n");
        goto fail_process_start;
    }

    process_info_file = ICP_OPEN(ADF_DEV_PROCESSES_PATH, O_RDWR);
    if (process_info_file < 0)
    {
        ADF_ERROR("Opening " ADF_DEV_PROCESSES_PATH " failed, errno = %d\n",
                  errno);
        goto fail_process_start;
    }

    res = ICP_WRITE(process_info_file, name_tml, name_len);
    if (res != name_len)
    {
        ADF_ERROR("Error writing " ADF_DEV_PROCESSES_PATH " file. "
                  "Expected bytes written %d vs %d, Errno = %d\n",
                  res,
                  name_len,
                  res < 0 ? errno : 0);
        goto fail_process_start;
    }

    res = ICP_READ(process_info_file, name, ADF_CFG_MAX_SECTION_LEN_IN_BYTES);
    if (res != -1)
    {
#ifndef ICP_WITHOUT_THREAD
        res = pthread_mutex_unlock(&processes_lock);
        if (res != 0)
        {
            ICP_CLOSE(process_info_file);
            process_info_file = -1;
            ADF_ERROR("Mutex unlock error %d\n", res);
            return CPA_STATUS_FAIL;
        }
#endif
        return CPA_STATUS_SUCCESS;
    }

    /* Read error */
    ADF_ERROR("Error reading " ADF_DEV_PROCESSES_PATH " file. Errno = %d\n",
              res < 0 ? errno : 0);
fail_process_start:

    if (process_info_file != -1)
        ICP_CLOSE(process_info_file);
    process_info_file = -1;

#ifndef ICP_WITHOUT_THREAD
    res = pthread_mutex_unlock(&processes_lock);
    if (res != 0)
        ADF_ERROR("Mutex unlock error %d\n", res);
#endif
    return CPA_STATUS_FAIL;
}

void icp_adf_userProcessStop(void)
{
#ifndef ICP_WITHOUT_THREAD
    int res = 0;
    res = pthread_mutex_lock(&processes_lock);
    if (res != 0)
        ADF_ERROR("Mutex lock error %d\n", res);
#endif
    if (process_info_file != -1)
    {
        ICP_CLOSE(process_info_file);
        process_info_file = -1;
    }
#ifndef ICP_WITHOUT_THREAD
    res = pthread_mutex_unlock(&processes_lock);
    if (res != 0)
        ADF_ERROR("Mutex unlock error %d\n", res);
#endif

    return;
}

/*
 * icp_adf_get_busAddress
 */
Cpa16U icp_adf_get_busAddress(Cpa16U packageId)
{
    return icp_adf_cfgGetBusAddress(packageId);
}

/*
 * adf_reset_userProxy
 *
 *  Function to reset the ADF proxy status in user space.
 */
void adf_reset_userProxy(void)
{
    init_ctr = 0;
#ifndef ICP_WITHOUT_THREAD
    if (pthread_mutex_init(&processes_lock, NULL))
        ADF_ERROR("Mutex init failed\n");
#endif
    osalAtomicSet(0, &process_proxy_status);
    icp_adf_userProcessStop();
}

int adf_proxy_poll_event(Cpa32U *dev_id, enum adf_event *event)
{
    struct kevent tevent;
    struct timespec timeout;
    int ret = 0;
    int fd = -1;
    int *fd_ptr = NULL;
    struct adf_state state;

    timeout.tv_sec = 0;
    timeout.tv_nsec = 0;
    ret = kevent(kq, NULL, 0, &tevent, 1, &timeout);

    if (ret <= 0)
    {
        return 0;
    }
    else
    {
        fd_ptr = (int *)tevent.udata;
        if (NULL == fd_ptr)
            return 0;
        fd = *fd_ptr;
        if (fd == -1)
            return 0;
        if (sizeof(state) != ICP_READ(fd, &state, sizeof(state)))
            return 0;
        if (state.dev_state > ADF_EVENT_ERROR)
        {
            ADF_ERROR("Unknown event received\n");
            return 0;
        }
        else
        {
            *dev_id = state.dev_id;
            *event = state.dev_state;
        }
    }
    return 1;
}
