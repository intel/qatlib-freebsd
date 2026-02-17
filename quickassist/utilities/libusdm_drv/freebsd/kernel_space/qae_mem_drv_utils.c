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
 * @file qae_mem_drv_utils.c
 *
 * This file handles ioctls from user space to kernel space for quick assist API
 *
 *****************************************************************************/

#include <sys/types.h>
#include <sys/errno.h>
#include <sys/systm.h>

int handle_other_ioctls(uint32_t cmd);

int handle_other_ioctls(uint32_t cmd)
{
    printf("Invalid IOCTL command specified(0x%x)\n", cmd);
    return ENOTTY;
}
