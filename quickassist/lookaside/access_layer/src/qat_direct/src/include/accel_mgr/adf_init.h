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
 * @file adf_init.h
 *
 * @description
 *      This header file that contains the prototypes and definitions required
 *      for ADF initialisation.
 *
 *****************************************************************************/
#ifndef ADF_INIT_H
#define ADF_INIT_H

#include "icp_accel_devices.h"

/*
 * Version numbers for ADF Version Registration
 *
 * Major Version Number is updated upon significant feature change(s)
 * Significant POR changes
 *
 * Minor Version Number is updated upon less significant feature change(s)
 *
 * Patch Version Number is updated upon bug fixes, release candidate updates
 */
#define ADF_MAJOR_VERSION 0
#define ADF_MINOR_VERSION 0
#define ADF_PATCH_VERSION 1

/*
 * Initialisation value for the adfModuleId, a value of 0 implies ADF has
 * not registered the version information with the DCC component.
 */
#define VERSION_INFO_UNREGISTERED 0

/*
 * This macro sets the specified bit in status to 1
 * i.e. set the bit
 */
#define SET_STATUS_BIT(status, bit) status |= (1 << bit)

/*
 * This macro sets the specified bit in status to 0
 * i.e. clears the bit
 */
#define CLEAR_STATUS_BIT(status, bit) status &= ~(1 << bit)

/*
 * This macro checks if the specified bit in status is set or not.
 */
#define BIT_IS_SET(status, bit) (status & (1 << bit))

/*
 * ADF system status bits
 */
#define ADF_STATUS_ISR_RES_ALLOCATED 0  /* ISR Service res are allocated*/
#define ADF_STATUS_AE_INITIALISED 1     /* AccelEngines are initialised */
#define ADF_STATUS_AE_UCODE_MAPPED 2    /* AccelEngine microcode is mapped*/
#define ADF_STATUS_AE_UCODE_LOADED 3    /* AccelEngines are loaded */
#define ADF_STATUS_AE_STARTED 4         /* AccelEngines are started */
#define ADF_STATUS_ORPHAN_TH_RUNNING 5  /* Orphan thread running on dev*/
#define ADF_STATUS_AE_UOF_LOADED 6      /* UOF FW is not released*/
#define ADF_STATUS_SRIOV_ENABLED 7      /* SRIOV is enabled*/
#define ADF_STATUS_ARB_INITIALISED 8    /* HW Arbiter initialised */
#define ADF_STATUS_SYSTEM_RESTARTING 28 /* ADF is restarting device */
#define ADF_STATUS_SYSTEM_STARTING 29   /* ADF is starting device */
#define ADF_STATUS_SYSTEM_CONFIGURED 30 /* ADF has configured device */
#define ADF_STATUS_SYSTEM_STARTED 31    /* ADF has started device */

/*
 * Pending time in ms that ADF will sleep before sending shutdown
 * when a subsystem return pending on stop
 */
#define PENDING_DELAY 100

/**
 *****************************************************************************
 * @description
 *      This function will initialise the AEs, map the firmware,
 *      send an init event to the subservice and call adf_subsystemStart.
 *
 *****************************************************************************/
CpaStatus adf_subsystemInit(icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This function will download the Ucode, start the AEs and send
 *      a start event to the subservices
 *
 *****************************************************************************/
CpaStatus adf_subsystemStart(icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This function will load the firmware, patch the previously loaded
 *      symbols and send a start event to the subservices
 *
 *****************************************************************************/
CpaStatus adf_subsystemResume(icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This function will stop the subcomponents in the system,
 *      and free resources for ISR and firmware loading that have been allocated
 *
 *****************************************************************************/
CpaStatus adf_subsystemStop(icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This is a wrapper function to adf_subsystemStop called when the OS
 *      issues a suspend.
 *
 *****************************************************************************/
CpaStatus adf_subsystemSuspend(icp_accel_dev_t *accel_dev);

/**
 *****************************************************************************
 * @description
 *      This function will shutdown the subcomponents in the system.
 *
 *****************************************************************************/
CpaStatus adf_subsystemShutdown(icp_accel_dev_t *accel_dev);

#endif /* ADF_INIT_H */
