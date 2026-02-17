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
 * @file adf_platform.h
 *
 * @description
 *      The file includes platform specific headers
 *
 *****************************************************************************/
#ifndef ADF_PLATFORM_H
#define ADF_PLATFORM_H

#include "adf_platform_common.h"
#include "adf_platform_acceldev_common.h"

#ifndef ADF_PLATFORM
#ifdef ADF_PLATFORM_ACCELDEV
#define ADF_PLATFORM 1
#endif
#ifdef ADF_PLATFORM_ACCELDEVVF
#define ADF_PLATFORM 1
#endif

#ifndef ADF_PLATFORM
#error "Define ADF target platform in adf_platform.mk file"
#endif
#else /* ifndef ADF_PLATFORM */
/* Build flag contains -DADF_PLATFORM */
#error "ADF_PLATFORM should not be defined"
#endif

#endif /* ADF_PLATFORM_H */
