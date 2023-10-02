/*-
 *****************************************************************************
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
