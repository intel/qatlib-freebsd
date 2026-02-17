/***************************************************************************
 *
 *   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright(c) 2007-2026 Intel Corporation
 * 
 *   These contents may have been developed with support from one or more
 *   Intel-operated generative artificial intelligence solutions.
 *
 ***************************************************************************/
#ifndef LAC_SAL_TYPES_PATTERNMATCH_H_
#define LAC_SAL_TYPES_PATTERNMATCH_H_

/**
 *****************************************************************************
 * @ingroup SalCtrl
 *      PatternMatch specific Service Container
 *
 * @description
 *      Contains information required per pattern match service instance.
 *
 *****************************************************************************/
typedef struct sal_patternmatch_service_s
{
    sal_service_t generic_service_info;
    /**< An instance of the Generic Service Container */

    lac_memory_pool_id_t patternmatch_mem_pool;
    /**< Memory pool ID used for patternmatch */

    OsalAtomic *pPmStatsArr;
    /**< Pointer to an array of atomic stats (Cpa64U) for pattern_match */

    icp_comms_trans_handle trans_handle_patternmatch_tx;

    icp_comms_trans_handle trans_handle_patternmatch_rx;
    Cpa32U pmResponseRingId;

    /* pointers to head and tail of list of matchContexts returned if the
     * callback is set to NULL for the matchContext, this builds a list of
     * results and is cleared by calling function cpaPmPollResults.
     */
    CpaPmMatchCtx *pPolledMatchCtxListHead;
    CpaPmMatchCtx *pPolledMatchCtxListTail;

    /* Resources for handling the polled list:
     * List lock for granting/revoking access to the list (while list is
     * being updated.
     * Semaphore is posted if cpaPmPollResults is blocking (set to true). If it
     * is blocking and the polled list is empty, it waits until a polled match
     * is added to the list.
     * Boolean isWaiting used for controlling the semaphore (it is a check to
     * see if cpaPmPollResults is blocking and if true, the semaphore is posted)
     */
    lac_lock_t pPolledMatchCtxListLock;
    OsalSemaphore sid;
    CpaBoolean isWaiting;

    lac_lock_t activePDBLock;
    /* pointer to the currently active database for this instance */
    pm_pdb_desc_t *pActiveDB;

    /* Config info */
    Cpa16U acceleratorNum;
    Cpa16U bankNum;
    Cpa16U pkgID;
    Cpa16U isPolled;
    Cpa32U coreAffinity;
    Cpa32U nodeAffinity;
    /* Statistics handler */
    debug_file_info_t *debug_file;
} sal_patternmatch_service_t;

/*************************************************************************
 * @ingroup SalCtrl
 * @description
 *  This function returns a valid patternmatch instance handle for the system
 *  if it exists.
 *
 *  @performance
 *    To avoid calling this function the user of the QA api should not use
 *    instanceHandle = CPA_INSTANCE_HANDLE_SINGLE.
 *
 * @context
 *    This function is called whenever instanceHandle =
 *                                                  CPA_INSTANCE_HANDLE_SINGLE
 *    at the QA Patternmatch api.
 *
 * @assumptions
 *      None
 * @sideEffects
 *      None
 * @reentrant
 *      No
 * @threadSafe
 *      Yes
 *
 * @retval   Pointer to first patternmatch instance handle or NULL if no
 *           patternmatch instances in the system.
 *
 *************************************************************************/
CpaInstanceHandle Pm_GetFirstHandle(void);

/******************************************************************************
 * Pattern match quick assist helper function implementations
 ******************************************************************************/

void PmPdb_CrcTableInit(Cpa32U *pCrcTable);

#endif /*LAC_SAL_TYPES_PATTERNMATCH_H_*/
