/*-
 *****************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ******************************************************************************/

/**
 *******************************************************************************
 * @file  cpa_dc_dp_sample_freebsd_kernel_module.c
 *
 ******************************************************************************/

#include "cpa_sample_utils.h"

int gDebugParam = 1;

static int event_handler(struct module *m, int what, void *arg);

TUNABLE_INT("cpa_dc_dp.debugParam", &gDebugParam); /* sets verbosity level */

extern CpaStatus dcDpSample(void);

static int dc_dp_mod_init(void)
{
    CpaStatus stat = CPA_STATUS_SUCCESS;
    PRINT_DBG("Loading Compression Data Plane Sample Code Module ...\n");

    stat = dcDpSample();
    if (CPA_STATUS_SUCCESS != stat)
    {
        PRINT_ERR("\nData Plane Compression Sample Code App failed\n");
    }
    else
    {
        PRINT_DBG("\nData Plane Compression Sample Code App finished\n");
    }

    PRINT_DBG("\nRemoving Module - ignore insmod error\n");

    /* module does not have any runtime functionality so remove it */
    return -EAGAIN;
}

static void dc_dp_mod_uninit(void)
{
    PRINT_DBG("Unloading Compression Data Plane Sample Code Module ...\n");
}

static int event_handler(struct module *m, int what, void *arg)
{
    switch (what)
    {
        case MOD_LOAD:
            return dc_dp_mod_init();
            break;
        case MOD_UNLOAD:
            dc_dp_mod_uninit();
            break;
        default:
            return EOPNOTSUPP;
            break;
    }
    return 0;
}

static moduledata_t dc_dp_mod = {"dc_dp_mod", event_handler, NULL};

DECLARE_MODULE(cpa_dc_dp, dc_dp_mod, SI_SUB_KLD, SI_ORDER_ANY);
MODULE_VERSION(cpa_dc_dp, 1);

MODULE_DEPEND(cpa_dc_dp, qat_api, 1, 1, 1);
