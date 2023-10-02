/***************************************************************************
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
 ***************************************************************************/
/*
 *This file is about ring/bank debug function.
 *out put string debug info about ring/bank
 */

#include <icp_accel_devices.h>
#include <icp_adf_init.h>
#include <icp_adf_transport.h>
#include <adf_dev_ring_ctl.h>
#include <adf_platform.h>
#include <adf_platform_acceldev_common.h>
#include <adf_platform_acceldev_gen4.h>

#include <icp_adf_debug.h> /* just for stub functions */
#include <icp_platform.h>
#include <uio_user_arbiter.h>

#define UIO_DBG_CHECK_BUFFER_SIZE(len, size)                                   \
    do                                                                         \
    {                                                                          \
        if ((len) >= (size))                                                   \
            return (len);                                                      \
    } while (0)

/* ring debug info function */
uint32_t uio_ring_base_name(adf_dev_ring_handle_t *ring,
                            char *data,
                            uint32_t size)
{
    uint32_t len = 0;

    if (NULL == ring || NULL == data || 0 == size)
    {
        return 0;
    }

    len = snprintf(data,
                   size,
                   " Service Name:\t%s\n",
                   (NULL == ring->service_name) ? "null" : ring->service_name);

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len,
                    size - len,
                    " Accelerator Number:\t%d, Bank Number:\t %d,"
                    " Ring Number:\t %d\n",
                    ring->accel_num,
                    ring->bank_num,
                    ring->ring_num);
    return len;
}

uint32_t uio_ring_cfg_info(adf_dev_ring_handle_t *ring,
                           char *data,
                           uint32_t size)
{
    uint32_t len = 0;
    uint32_t space = 0, head = 0, tail = 0;
    uint32_t empty_stat, nearly_empty_stat;
    uint32_t *csr_base_addr;
    uint64_t ring_config;

    if (NULL == ring || NULL == data || 0 == size)
    {
        return 0;
    }

    csr_base_addr = ring->csr_addr;
    if (NULL == csr_base_addr)
    {
        len = snprintf(
            data, size, " Ring %d is not initialized\n", ring->ring_num);
        return len;
    }

    ring_config = READ_CSR_RING_CONFIG(ring->bank_offset, ring->ring_num);
    head = READ_CSR_RING_HEAD(ring->bank_offset, ring->ring_num);
    tail = READ_CSR_RING_TAIL(ring->bank_offset, ring->ring_num);

    if (head == tail)
    {
        empty_stat = READ_CSR_E_STAT(ring->bank_offset);
        space = ((empty_stat) & (1 << ring->ring_num)) ? ring->ring_size : 0;
    }
    else if (tail > head)
    {
        space = ring->ring_size - (tail - head);
    }
    else
    {
        space = head - tail;
    }

    len += snprintf(data,
                    size,
                    " Ring Config: 0x%lx %s, Base Address: 0x%p, Physical Base "
                    "Address: 0x%lx"
                    " Head: %x, Tail: %x, Space: %x\n",
                    (unsigned long)ring_config,
                    (ring->resp == ICP_RESP_TYPE_NONE) ? "Tx" : "Rx",
                    ring->ring_virt_addr,
                    (unsigned long)ring->ring_phys_base_addr,
                    head,
                    tail,
                    space);

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    if (ring->ring_size)
    {
        len += snprintf(
            data + len,
            size - len,
            " Message size: %d, Max messages: %d, Current messages: %d\n",
            ring->message_size,
            ring->ring_size / ring->message_size,
            ring->ring_size / ring->message_size - space / ring->message_size);
    }

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    nearly_empty_stat = READ_CSR_NE_STAT(ring->bank_offset);
    nearly_empty_stat = (nearly_empty_stat & (1 << ring->ring_num)) ? 1 : 0;
    empty_stat = READ_CSR_E_STAT(ring->bank_offset);
    empty_stat = (empty_stat & (1 << ring->ring_num)) ? 1 : 0;
    len += snprintf(data + len,
                    size - len,
                    " Ring Empty flag: %d, Ring Nearly Empty flag: %d\n",
                    empty_stat,
                    nearly_empty_stat);

    return len;
}

#define ADF_DEBUG_LINE_LEN 60
static uint32_t uio_read_io_memory(uint32_t *ptr,
                                   uint32_t io_size,
                                   char *data,
                                   uint32_t size,
                                   uint32_t offset)
{
    uint32_t i = 0;
    uint32_t len = 0;

    for (i = offset; i < io_size;)
    {

        int x = 0;
        if (len + ADF_DEBUG_LINE_LEN > size)
        {
            goto out;
        }

        len += snprintf(data + len, size - len, "%p:", ptr);
        for (x = 0; x < sizeof(uint32_t) && i < io_size;
             x++, i += sizeof(uint32_t))
        {
            len += snprintf(data + len, size - len, " %08X", *ptr);
            ptr++;
            offset += sizeof(uint32_t);
        }
        len += snprintf(data + len, size - len, "\n");
    }

out:
    return len;
}

/* get the content of the ring buffer*/
uint32_t uio_ring_get_queue(adf_dev_ring_handle_t *ring,
                            char *data,
                            uint32_t size,
                            uint32_t offset)
{
    if (NULL == ring || NULL == data || 0 == size)
    {
        return 0;
    }

    if (NULL == ring->csr_addr)
    {
        uint32_t len = 0;
        len = snprintf(
            data, size, " Ring %d is not initialized\n", ring->ring_num);
        return len;
    }

    return uio_read_io_memory(
        ring->ring_virt_addr, ring->ring_size, data, size, offset);
}

/*debug for bank*/
uint32_t uio_bank_get_cfg_info(adf_dev_bank_handle_t *bank,
                               char *data,
                               uint32_t size)
{
    int i = 0;
    uint32_t len = 0;
    uint32_t bits_to_show = 0;

    if (NULL == bank || NULL == data || 0 == size)
    {
        return 0;
    }

    len = snprintf(data + len,
                   size - len,
                   "------- Bank %d Configuration -------\n",
                   bank->bank_number);
    UIO_DBG_CHECK_BUFFER_SIZE(len, size);

    if (NULL == bank->csr_addr)
    {
        len += snprintf(data + len,
                        size - len,
                        " Bank %d is not initialized",
                        bank->bank_number);
        return len;
    }

    if (bank->timed_coalesc_enabled)
    {
        len +=
            snprintf(data + len, size - len, " Interrupt Coalescing Enabled\n");
    }
    else
    {
        len += snprintf(
            data + len, size - len, " Interrupt Coalescing Disabled\n");
    }

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    if (bank->flag_interrupts_enabled)
    {
        len += snprintf(data + len, size - len, " Interrupt Enabled\n");
    }
    else
    {
        len += snprintf(data + len, size - len, " Interrupt Disabled\n");
    }

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    if (bank->number_msg_coalesc_enabled)
    {
        len += snprintf(
            data + len,
            size - len,
            " Interrupt Coalescing based on number of messages enabled\n");
    }

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    bits_to_show = bank->max_num_rings - 1;
    len += snprintf(data + len, size - len, " Interrupt mask:\t");

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    for (i = bits_to_show; i >= 0; i--)
    {
        if (bank->interrupt_mask & (1 << i))
        {
            len += snprintf(data + len, size - len, " 1");
        }
        else
        {
            len += snprintf(data + len, size - len, " 0");
        }
    }

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len, size - len, "\n");

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len, size - len, " Polling mask:\t\t");

    for (i = bits_to_show; i >= 0; i--)
    {
        UIO_DBG_CHECK_BUFFER_SIZE(len, size);
        if (bank->pollingMask & (1 << i))
        {
            len += snprintf(data + len, size - len, " 1");
        }
        else
        {
            len += snprintf(data + len, size - len, " 0");
        }
    }

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len, size - len, "\n Ring mask:\t\t");

    for (i = bits_to_show; i >= 0; i--)
    {
        UIO_DBG_CHECK_BUFFER_SIZE(len, size);
        if (bank->ring_mask & (1 << i))
        {
            len += snprintf(data + len, size - len, " 1");
        }
        else
        {
            len += snprintf(data + len, size - len, " 0");
        }
    }
    return len;
}

uint32_t uio_bank_get_csr_info(adf_dev_bank_handle_t *bank,
                               char *data,
                               uint32_t size)
{
    if (NULL == bank || NULL == data || 0 == size)
    {
        return 0;
    }

    int i = 0;
    uint32_t len = 0, irq_csr = 0;
    uint32_t *csr_base_addr = bank->csr_addr;
    uint32_t empty_stat, nearly_empty_stat;
    uint32_t bits_to_show = bank->max_num_rings - 1;
    uint32_t arbsrven = 0;

    len = snprintf(data + len,
                   size - len,
                   "------- Bank %d Csr Info------------\n",
                   bank->bank_number);
    UIO_DBG_CHECK_BUFFER_SIZE(len, size);

    if (NULL == bank->csr_addr)
    {
        len += snprintf(data + len, size - len, "Csr address is NULL");
        return len;
    }

    if (bank->timed_coalesc_enabled)
    {
        irq_csr = READ_CSR_INT_COL_EN(bank->bank_offset);
        len += snprintf(data + len, size - len, "\n");
        len += snprintf(data + len, size - len, " Coalesc reg:\t\t");

        for (i = bits_to_show; i >= 0; i--)
        {
            UIO_DBG_CHECK_BUFFER_SIZE(len, size);
            if (irq_csr & (1 << i))
            {
                len += snprintf(data + len, size - len, " 1");
            }
            else
            {
                len += snprintf(data + len, size - len, " 0");
            }
        }
    }

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    if (!bank->timed_coalesc_enabled || bank->number_msg_coalesc_enabled)
    {
        irq_csr = READ_CSR_INT_EN(bank->bank_offset);
        len += snprintf(data + len, size - len, "\n");

        UIO_DBG_CHECK_BUFFER_SIZE(len, size);
        len += snprintf(data + len, size - len, " Interrupt reg:\t\t");

        for (i = bits_to_show; i >= 0; i--)
        {
            UIO_DBG_CHECK_BUFFER_SIZE(len, size);
            if (irq_csr & (1 << i))
            {
                len += snprintf(data + len, size - len, " 1");
            }
            else
            {
                len += snprintf(data + len, size - len, " 0");
            }
        }
    }

    if (!bank->ring_mask)
    {
        UIO_DBG_CHECK_BUFFER_SIZE(len, size);
        len += snprintf(
            data + len, size - len, "\nThere are no rings allocated.\n");
        return len;
    }

    empty_stat = READ_CSR_E_STAT(bank->bank_offset);
    nearly_empty_stat = READ_CSR_NE_STAT(bank->bank_offset);

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len, size - len, "\n");
    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len, size - len, " Bank empty stat:\t");

    for (i = bits_to_show; i >= 0; i--)
    {
        UIO_DBG_CHECK_BUFFER_SIZE(len, size);
        if (empty_stat & (1 << i))
        {
            len += snprintf(data + len, size - len, " 1");
        }
        else
        {
            len += snprintf(data + len, size - len, " 0");
        }
    }

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len, size - len, "\n");

    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len, size - len, " Bank nempty stat:\t");

    for (i = bits_to_show; i >= 0; i--)
    {
        UIO_DBG_CHECK_BUFFER_SIZE(len, size);
        if (nearly_empty_stat & (1 << i))
        {
            len += snprintf(data + len, size - len, " 1");
        }
        else
        {
            len += snprintf(data + len, size - len, " 0");
        }
    }

    arbsrven = READ_CSR_ARB_RINGSRVARBEN(csr_base_addr, 0);
    UIO_DBG_CHECK_BUFFER_SIZE(len, size);
    len += snprintf(data + len, size - len, "\n Arbiter enable:\t");
    for (i = bits_to_show; i >= 0; i--)
    {
        UIO_DBG_CHECK_BUFFER_SIZE(len, size);
        if (arbsrven & (1 << i))
        {
            len += snprintf(data + len, size - len, " 1");
        }
        else
        {
            len += snprintf(data + len, size - len, " 0");
        }
    }
    return len;
}

/*get ring cfg info in a bank */
uint32_t uio_bank_get_ring_info(adf_dev_bank_handle_t *bank,
                                char *data,
                                uint32_t size)
{
    int i;
    uint32_t len = 0;
    /* adf_dev_ring_handle_t *ring = NULL; */

    if (NULL == bank || NULL == data || 0 == size)
    {
        return 0;
    }

    len = snprintf(data + len,
                   size - len,
                   "------- Bank %d Rings Info -------\n",
                   bank->bank_number);
    UIO_DBG_CHECK_BUFFER_SIZE(len, size);

    if (!bank->ring_mask)
    {
        len +=
            snprintf(data + len, size - len, "There are no rings allocated.");
        return len;
    }

    for (i = 0; i < bank->max_num_rings; i++)
    {
        if (bank->ring_mask & (1 << i))
        {
        }
    }

    return len;
}

/* stub functions for legacy functions */
int icp_adf_debugAddDir(icp_accel_dev_t *accel_dev, debug_dir_info_t *dir_info)

{
    return 0;
}

int icp_adf_debugAddFile(icp_accel_dev_t *accel_dev,
                         debug_file_info_t *file_info)

{
    return 0;
}

void icp_adf_debugRemoveDir(debug_dir_info_t *dir_info)
{
    return;
}

void icp_adf_debugRemoveFile(debug_file_info_t *file_info)
{
    return;
}

void adf_dump_bank_csr_reg(const uint8_t *pre, adf_dev_bank_handle_t *bank)
{
    uint32_t *csr_base_addr = bank->csr_addr;

    if (!csr_base_addr)
    {
        ADF_ERROR("csr_base_addr is NULL!\n");
        return;
    }

    ADF_PRINT(
        "@%s:bank %02d "
        "status(e=0x%x,ne=0x%x,f=0x%x,nf=0x%x,intm=0x%x),cfg(arbsrven=0x%x)\n",
        pre,
        bank->bank_number,
        READ_CSR_E_STAT(0),
        READ_CSR_NE_STAT(0),
        READ_CSR_F_STAT(0),
        READ_CSR_NF_STAT(0),
        READ_CSR_INT_EN(0),
        READ_CSR_ARB_RINGSRVARBEN(csr_base_addr, 0));
}

void adf_dump_ring_sta_reg(const uint8_t *pre, adf_dev_ring_handle_t *ring)
{
    uint32_t *csr_base_addr = ring->csr_addr;

    ADF_PRINT("@%s:status for ring %02d:%02d(head=0x%x,tail=0x%x)\n",
              pre,
              ring->bank_data->bank_number,
              ring->ring_num,
              READ_CSR_RING_HEAD(ring->bank_offset, ring->ring_num),
              READ_CSR_RING_TAIL(ring->bank_offset, ring->ring_num));
}

void adf_dump_ring_cfg_reg(const uint8_t *pre, adf_dev_ring_handle_t *ring)
{
    Cpa64U ring_base_addr = 0;
    Cpa32U ring_config = 0;
    uint32_t *csr_base_addr = ring->csr_addr;
    device_type_t deviceType;

    ICP_CHECK_FOR_NULL_PARAM_VOID(ring->accel_dev);
    deviceType = ring->accel_dev->deviceType;

    if (IS_SAL_GEN4(deviceType))
    {
        ring_config =
            READ_CSR_RING_CONFIG_GEN4(ring->bank_offset, ring->ring_num),
        ring_base_addr =
            read_base_gen4(csr_base_addr, ring->bank_offset, ring->ring_num);
    }
    else
    {
        ring_config = READ_CSR_RING_CONFIG(ring->bank_offset, ring->ring_num),
        ring_base_addr =
            read_base(csr_base_addr, ring->bank_offset, ring->ring_num);
    }

    ADF_PRINT("@%s:cfg for ring %02d:%02d(cfg=0x%x,base=0x%lx)\n",
              pre,
              ring->bank_data->bank_number,
              ring->ring_num,
              ring_config,
              ring_base_addr);
}
