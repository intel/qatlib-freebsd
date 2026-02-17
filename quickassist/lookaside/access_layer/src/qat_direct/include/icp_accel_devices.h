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
 * @file icp_accel_devices.h
 *
 * @defgroup Acceleration Driver Framework
 *
 * @ingroup icp_Adf
 *
 * @description
 *      This is the top level header file that contains the layout of the ADF
 *      icp_accel_dev_t structure and related macros/definitions.
 *      It can be used to dereference the icp_accel_dev_t *passed into upper
 *      layers.
 *
 *****************************************************************************/

#ifndef ICP_ACCEL_DEVICES_H_
#define ICP_ACCEL_DEVICES_H_

#include "cpa.h"
#include "Osal.h"

#define ADF_CFG_NULL_TERM_SIZE 1
#define ADF_CFG_BASE_DEC 10
#define ADF_CFG_BASE_HEX 16
#define ADF_CFG_ALL_DEVICES 0xFFFE
#define ADF_CFG_NO_DEVICE 0xFFFF

#define ADF_CFG_MAX_STR_LEN 128
#define MAX_DEVICE_NAME_SIZE 32
#define ADF_DEVICE_NAME_LENGTH 32
#define ADF_CFG_MAX_STR_LEN 128
#define ADF_CFG_MAX_KEY_LEN_IN_BYTES ADF_CFG_MAX_STR_LEN
#define ADF_CFG_MAX_VAL_LEN_IN_BYTES ADF_CFG_MAX_STR_LEN
#define ADF_CFG_MAX_SECTION_LEN_IN_BYTES ADF_CFG_MAX_STR_LEN
#define ADF_MAX_DEVICES (32 * 32)

enum dev_sku_info
{
    DEV_SKU_1 = 0,
    DEV_SKU_2,
    DEV_SKU_3,
    DEV_SKU_4,
    DEV_SKU_VF,
    DEV_SKU_UNKNOWN,
};

enum adf_event
{
    ADF_EVENT_INIT = 0,
    ADF_EVENT_START,
    ADF_EVENT_STOP,
    ADF_EVENT_SHUTDOWN,
    ADF_EVENT_RESTARTING,
    ADF_EVENT_RESTARTED,
    ADF_EVENT_ERROR,
    ADF_EVENT_NOP,
};

struct adf_state
{
    enum adf_event dev_state;
    int dev_id;
};

#define ADF_CFG_NO_INSTANCE 0xFFFFFFFF

#define ICP_DC_TX_RING_0 6
#define ICP_DC_TX_RING_1 7
#define ICP_RX_RINGS_OFFSET 8
#define ICP_RINGS_PER_BANK 16

/* Number of worker threads per AE */
#define ICP_ARB_WRK_THREAD_TO_SARB 12
#define MAX_ACCEL_NAME_LEN 16
#define ADF_DEVICE_NAME_LENGTH 32
#define ADF_DEVICE_TYPE_LENGTH 16

#define ADF_CTL_DEVICE_NAME "/dev/qat_adf_ctl"

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Accelerator capabilities
 *
 *****************************************************************************/
typedef enum
{
    ICP_ACCEL_CAPABILITIES_CRYPTO_SYMMETRIC = 0x01,
    ICP_ACCEL_CAPABILITIES_CRYPTO_ASYMMETRIC = 0x02,
    ICP_ACCEL_CAPABILITIES_CIPHER = 0x04,
    ICP_ACCEL_CAPABILITIES_AUTHENTICATION = 0x08,
    ICP_ACCEL_CAPABILITIES_RESERVED_1 = 0x10,
    ICP_ACCEL_CAPABILITIES_COMPRESSION = 0x20,
    ICP_ACCEL_CAPABILITIES_DEPRECATED = 0x40,
    ICP_ACCEL_CAPABILITIES_RANDOM_NUMBER = 0x80,
    ICP_ACCEL_CAPABILITIES_CRYPTO_ZUC = 0x100,
    ICP_ACCEL_CAPABILITIES_SHA3 = 0x200,
    ICP_ACCEL_CAPABILITIES_KPT = 0x400,
    ICP_ACCEL_CAPABILITIES_RL = 0x800,
    ICP_ACCEL_CAPABILITIES_HKDF = 0x1000,
    ICP_ACCEL_CAPABILITIES_ECEDMONT = 0x2000,
    ICP_ACCEL_CAPABILITIES_EXT_ALGCHAIN = 0x4000,
    ICP_ACCEL_CAPABILITIES_SHA3_EXT = 0x8000,
    ICP_ACCEL_CAPABILITIES_AESGCM_SPC = 0x10000,
    ICP_ACCEL_CAPABILITIES_CHACHA_POLY = 0x20000,
    ICP_ACCEL_CAPABILITIES_SM2 = 0x40000,
    ICP_ACCEL_CAPABILITIES_SM3 = 0x80000,
    ICP_ACCEL_CAPABILITIES_SM4 = 0x100000,
    ICP_ACCEL_CAPABILITIES_INLINE = 0x200000,
    ICP_ACCEL_CAPABILITIES_CNV_INTEGRITY = 0x400000,
    ICP_ACCEL_CAPABILITIES_CNV_INTEGRITY64 = 0x800000,
    ICP_ACCEL_CAPABILITIES_LZ4_COMPRESSION = 0x1000000,
    ICP_ACCEL_CAPABILITIES_LZ4S_COMPRESSION = 0x2000000,
    ICP_ACCEL_CAPABILITIES_AES_V2 = 0x4000000,
    ICP_ACCEL_CAPABILITIES_KPT2 = 0x8000000,
} icp_accel_capabilities_t;

/**
 *****************************************************************************
 * @ingroup icp_AdfAccelHandle
 *
 * @description
 *      Device Configuration Data Structure
 *
 *****************************************************************************/

typedef enum device_type_e
{
    DEVICE_UNKNOWN = 0,
    DEVICE_DH895XCC,
    DEVICE_DH895XCCVF,
    DEVICE_C62X,
    DEVICE_C62XVF,
    DEVICE_C3XXX,
    DEVICE_C3XXXVF,
    DEVICE_200XX,
    DEVICE_200XXVF,
    DEVICE_C4XXX,
    DEVICE_C4XXXVF,
    DEVICE_D15XX,
    DEVICE_D15XXVF,
    DEVICE_4XXX,
    DEVICE_4XXXVF
} device_type_t;

/*
 * Enumeration on Service Type
 */
typedef enum adf_service_type_s
{
    ADF_SERVICE_CRYPTO,
    ADF_SERVICE_COMPRESS,
    ADF_SERVICE_MAX /* this is always the last one */
} adf_service_type_t;

typedef struct accel_dev_s
{
    /* Some generic information */
    Cpa32U accelId;
    Cpa8U *pAccelName;        /* Name given to accelerator */
    Cpa32U aeMask;            /* Acceleration Engine mask */
    device_type_t deviceType; /* Device Type              */
    /* Device name for SAL */
    char deviceName[ADF_DEVICE_TYPE_LENGTH + 1];
    Cpa32U accelCapabilitiesMask; /* Accelerator's capabilities
                                     mask */
    Cpa32U dcExtendedFeatures;    /* bit field of features */
    OsalAtomic usageCounter;      /* Usage counter. Prevents
                                     shutting down the dev if not 0*/
    Cpa32U deviceMemAvail;        /* Device memory for intermediate buffers */
    /* Component specific fields - cast to relevent layer */
    void *pRingInflight;       /* For offload optimization */
    void *pSalHandle;          /* For SAL*/
    void *pQatStats;           /* For QATAL/SAL stats */
    void *ringInfoCallBack;    /* Callback for user space
                                  ring enabling */
    void *pShramConstants;     /* Virtual address of Shram constants page */
    Cpa64U pShramConstantsDma; /* Bus address of Shram constants page */

    /* Status of ADF and registered subsystems */
    Cpa32U adfSubsystemStatus;
    /* Physical processor to which the dev is connected */
    Cpa8U pkg_id;
    enum dev_sku_info sku;
    Cpa32U pciDevId;
    Cpa8U devFileName[ADF_DEVICE_NAME_LENGTH];
    Cpa32S csrFileHdl;
    Cpa32S devCtlFileHdl;
    void *accel;

    Cpa32U maxNumBanks;
    Cpa32U maxNumRingsPerBank;

    /* pointer to dynamic instance resource manager */
    void *pInstMgr;
    void *banks; /* banks information */
#ifdef KERNEL_SPACE
    struct adf_accel_dev *accel_dev;
    struct accel_dev_s *pPrev;
    struct accel_dev_s *pNext;
#endif
} icp_accel_dev_t;

#endif /* ICP_ACCEL_HANDLE_H */
