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
 *****************************************************************************
 * @file sal_user.c
 *
 * @defgroup SalUser
 *
 * @description
 *    This file contains implementation of functions to start/stop user process
 *
 *****************************************************************************/
#include <unistd.h>

/* QAT-API includes */
#include "cpa.h"

/* ADF includes */
#include "icp_adf_init.h"
#include "icp_accel_devices.h"
#include "icp_adf_accel_mgr.h"
#include "icp_adf_user_proxy.h"
#include "icp_adf_transport.h"
#include "icp_adf_cfg.h"
#include "icp_adf_debug.h"

/* FW includes */
#include "icp_qat_fw_la.h"

/* SAL includes */
#include "icp_sal_user.h"
#include "lac_log.h"
#include "lac_mem.h"
#include "lac_mem_pools.h"
#include "lac_list.h"
#ifndef ICP_DC_ONLY
#include "lac_sal_types_crypto.h"
#endif
#include "sal_types_compression.h"
#include "lac_sal.h"
#include "lac_sal_ctrl.h"
#include "dc_session.h"
#include "dc_error_counter.h"
#include "dc_err_sim.h"

#define START_REF_COUNT_MAX 64

#ifndef ICP_WITHOUT_THREAD
static pthread_mutex_t sync_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sync_multi_lock = PTHREAD_MUTEX_INITIALIZER;
#endif
static char multi_section_name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES] = { 0 };

/* Start reference count to keep track of multiple calls to
 * icp_sal_userStartMulti() and icp_sal_userStop() from the same application.
 * Only the first call to start will map the instances and
 * the last call to stop will free them.
 * This is added to support co-existence scenario (two libraries using
 * QAT in same application).
 */
static int start_ref_count = 0;
static pid_t start_ref_pid = -1;

void adf_reset_userProxy(void);

static CpaStatus do_userReset()
{

#ifndef ICP_WITHOUT_THREAD
    if (pthread_mutex_init(&sync_lock, NULL))
    {
        LAC_LOG_ERROR("Mutex init failed\n");
        return CPA_STATUS_FAIL;
    }
    if (pthread_mutex_init(&sync_multi_lock, NULL))
    {
        LAC_LOG_ERROR("Mutex init failed\n");
        return CPA_STATUS_FAIL;
    }
    if (pthread_mutex_lock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex lock failed\n");
        pthread_mutex_destroy(&sync_lock);
        return CPA_STATUS_FAIL;
    }
#endif
    start_ref_count = 0;
#ifndef ICP_WITHOUT_THREAD
    if (pthread_mutex_unlock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex unlock failed\n");
        pthread_mutex_destroy(&sync_lock);
        return CPA_STATUS_FAIL;
    }
#endif
    adf_reset_userProxy();
    reset_adf_subsystemTable();

    return CPA_STATUS_SUCCESS;
}

CpaStatus icp_sal_userStartMultiProcess(const char *pProcessName,
                                        CpaBoolean limitDevAccess)
{
    return icp_sal_userStart(pProcessName);
}

static CpaStatus do_userStart(const char *pProcessName)
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    status = icpSetProcessName(pProcessName);
    LAC_CHECK_STATUS(status);
    status = SalCtrl_AdfServicesRegister();
    LAC_CHECK_STATUS(status);

    status = icp_adf_userProxyInit(pProcessName);

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to initialize proxy\n");
        SalCtrl_AdfServicesUnregister();
        return status;
    }
    status = SalCtrl_AdfServicesStartedCheck();
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to start services\n");
        SalCtrl_AdfServicesUnregister();
    }
    return status;
}

CpaStatus icp_sal_userStart(const char *process_name)
{
    char name[ADF_CFG_MAX_SECTION_LEN_IN_BYTES + 1] = { 0 };
    CpaStatus status = CPA_STATUS_SUCCESS;
    pid_t pid = getpid();

    if (start_ref_pid != pid)
    {
        status = do_userReset();
        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_ERROR("do_userReset failed\n");
            return CPA_STATUS_FAIL;
        }
    }

#ifndef ICP_WITHOUT_THREAD
    if (pthread_mutex_lock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex lock failed\n");
        pthread_mutex_destroy(&sync_lock);
        return CPA_STATUS_FAIL;
    }
#endif

    if (0 == start_ref_count)
    {
        status = icp_adf_userProcessToStart(process_name, name);

        if (CPA_STATUS_SUCCESS != status)
        {
            LAC_LOG_DEBUG("icp_adf_userProcessToStart failed\n");
#ifndef ICP_WITHOUT_THREAD
            if (pthread_mutex_unlock(&sync_lock))
                LAC_LOG_ERROR("Mutex unlock failed\n");
            else
                pthread_mutex_destroy(&sync_lock);
#endif
            return CPA_STATUS_FAIL;
        }
        status = do_userStart(name);
    }
    if (CPA_STATUS_SUCCESS == status)
    {
        /* To handle overflow case */
        if (start_ref_count >= START_REF_COUNT_MAX)
        {
            LAC_LOG_ERROR("start_ref_count overflow!\n");
#ifndef ICP_WITHOUT_THREAD
            if (pthread_mutex_unlock(&sync_lock))
                LAC_LOG_ERROR("Mutex unlock failed\n");
            else
                pthread_mutex_destroy(&sync_lock);
#endif
            return CPA_STATUS_FAIL;
        }
        else
        {
            start_ref_count += 1;
        }
    }
#ifndef ICP_WITHOUT_THREAD
    if (pthread_mutex_unlock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex unlock failed\n");
        return CPA_STATUS_FAIL;
    }
#endif
    if (CPA_STATUS_SUCCESS == status)
    {
        start_ref_pid = pid;
    }
    return status;
}

static CpaStatus do_userStop()
{
    CpaStatus status = SalCtrl_AdfServicesUnregister();

    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to unregister\n");
        return status;
    }

    status = icp_adf_userProxyShutdown();
    if (CPA_STATUS_SUCCESS != status)
    {
        LAC_LOG_ERROR("Failed to shutdown proxy\n");
        return status;
    }
    icp_adf_userProcessStop();
    return status;
}

CpaStatus icp_sal_userStop()
{
    CpaStatus status = CPA_STATUS_SUCCESS;
    CpaBoolean destroy_mutex = CPA_FALSE;

    pid_t pid = getpid();

    if (start_ref_pid != pid)
    {
        LAC_LOG_DEBUG("Process id mismatch\n");
        return CPA_STATUS_FAIL;
    }
#ifndef ICP_WITHOUT_THREAD
    if (pthread_mutex_lock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex lock failed\n");
        return CPA_STATUS_FAIL;
    }
#endif
    if (start_ref_count == 1)
    {
        status = do_userStop();
    }
    if (start_ref_count > 0)
    {
        start_ref_count -= 1;
    }
    osalMemSet(multi_section_name, '\0', ADF_CFG_MAX_SECTION_LEN_IN_BYTES);

    if (0 == start_ref_count)
    {
        start_ref_pid = -1;
        destroy_mutex = CPA_TRUE;
    }

#ifndef ICP_WITHOUT_THREAD
    if (pthread_mutex_unlock(&sync_lock))
    {
        LAC_LOG_ERROR("Mutex unlock failed\n");
        return CPA_STATUS_FAIL;
    }
#endif
    if (CPA_TRUE == destroy_mutex)
    {
#ifndef ICP_WITHOUT_THREAD
        pthread_mutex_destroy(&sync_lock);
#endif
    }

    return status;
}

CpaStatus icp_sal_find_new_devices(void)
{
    return icp_adf_find_new_devices();
}

CpaStatus icp_sal_poll_device_events(void)
{
    return icp_adf_poll_device_events();
}

CpaStatus icp_sal_check_device(Cpa32U accelId)
{
    return icp_adf_check_device(accelId);
}

CpaStatus icp_sal_check_all_devices(void)
{
    return icp_adf_check_all_devices();
}

CpaStatus icp_sal_reset_device(Cpa32U accelId)
{
    return icp_adf_reset_device(accelId);
}

#ifdef ICP_HB_FAIL_SIM
CpaStatus icp_sal_heartbeat_simulate_failure(Cpa32U accelId)
{
    return icp_adf_heartbeat_simulate_failure(accelId);
}

#endif /* QAT_HB_FAIL_SIM */

CpaStatus icp_sal_get_num_pfs(Cpa16U *pNumPFs)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus icp_sal_get_pf_info(CpaPfInfo *pPf_info)
{
    return CPA_STATUS_UNSUPPORTED;
}

#ifdef ICP_RING_EXCEPTION_SIM
CpaStatus icp_sal_ring_exception_simulate(void *ring)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus icp_sal_ring_exception_stop_simulate(void *ring)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus icp_sal_rp_exception_is_set(void *ring)
{
    return CPA_STATUS_UNSUPPORTED;
}

Cpa32S icp_sal_check_ring_error(void *ring)
{
    return CPA_STATUS_UNSUPPORTED;
}

#endif /* ICP_RING_EXCEPTION_SIM */

CpaStatus icp_sal_ns_cnv_simulate_error(CpaInstanceHandle dcInstance)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaStatus icp_sal_ns_cnv_reset_error(CpaInstanceHandle dcInstance)
{
    return CPA_STATUS_UNSUPPORTED;
}

CpaBoolean icp_sal_userIsQatAvailable(void)
{
    return icp_adf_isDeviceAvailable();
}

