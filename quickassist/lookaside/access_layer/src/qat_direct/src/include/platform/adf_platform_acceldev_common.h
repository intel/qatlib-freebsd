/*****************************************************************************
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
 *****************************************************************************/

/*****************************************************************************
 * @file adf_platform_acceldev_common.h
 *
 * @description
 *      This file contains the platform specific macros for DH89xxCC that are
 *      common for PF and VF
 *
 *****************************************************************************/
#ifndef ADF_PLATFORM_ACCELDEV_COMMON_H
#define ADF_PLATFORM_ACCELDEV_COMMON_H

/*****************************************************************************
 * Define Constants and Macros
 *****************************************************************************/

/* Coalesced Interrupt Enable */
#define ETR_CSR_INTR_COL_CTL_ENABLE 0x80000000

/* Initial bank Interrupt Source mask */
#define BANK_INT_SRC_SEL_MASK_0 0x4444444CUL
#define BANK_INT_SRC_SEL_MASK_X 0x44444444UL

/* VF2PF communication masks and offsets */
#define ICP_PMISCBAR_VF2PFMSGMASK 0x3FE0000
#define ICP_PMISCBAR_VF2PFMSGMASKOP 0x20000
#define ICP_PMISCBAR_VF2PFMSGMASKNR 0x3C0000
#define ICP_PMISCBAR_VF2PFMSGMASKINFO 0x3C00000
#define ICP_PMISCBAR_VF2PFMSGMASKFLAG 0x4000000
#define ICP_PMISCBAR_VF2PFMSGMASKMSGTYPE 0xE0000000
#define ICP_PMISCBAR_VF2PFMSGSHIFTRINGIRQ 0x10
#define ICP_PMISCBAR_VF2PFMSGSHIFTRINGOP 0x11
#define ICP_PMISCBAR_VF2PFMSGSHIFTRINGNR 0x12
#define ICP_PMISCBAR_VF2PFMSGSHIFTRINGINFO 0x16
#define ICP_PMISCBAR_VF2PFMSGSHIFTRINGFLAG 0x1A
#define ICP_PMISCBAR_VF2PFMSGSHIFTMSGTYPE 0x1D

/* VF2PF return codes */
#define ICP_PMISCBAR_VF2PFMSGRINGINFOACK 0x2
#define ICP_PMISCBAR_VF2PFMSGRINGINFOERR 0xFFFC
#define ICP_PMISCBAR_VF2PFMSGRINGINFOACKMSK 0x2
#define ICP_PMISCBAR_VF2PFMSGRINGINFOACKSHIFT 0x2

/* VF2PF message types */
#define ICP_PMISCBAR_VF2PFMSGTYPERINGOP 0x1
#define ICP_PMISCBAR_VF2PFMSGTYPERINGOFFSET 0x2
#define ICP_PMISCBAR_VF2PFMSGTYPEESRAM 0x3

/* Ring Csrs offsets */
#define ICP_RING_CSR_RING_CONFIG 0x000
#define ICP_RING_CSR_RING_LBASE 0x040
#define ICP_RING_CSR_RING_UBASE 0x080
#define ICP_RING_CSR_RING_HEAD_OFFSET 0x0C0
#define ICP_RING_CSR_RING_TAIL_OFFSET 0x100
#define ICP_RING_CSR_RING_STAT 0x140
#define ICP_RING_CSR_UO_STAT 0x148
#define ICP_RING_CSR_E_STAT 0x14C
#define ICP_RING_CSR_NE_STAT 0x150
#define ICP_RING_CSR_NF_STAT 0x154
#define ICP_RING_CSR_F_STAT 0x158
#define ICP_RING_CSR_C_STAT 0x15C
#define ICP_RING_CSR_INT_EN 0x16C
#define ICP_RING_CSR_INT_REG 0x170
#define ICP_RING_CSR_INT_SRCSEL 0x174
#define ICP_RING_CSR_INT_SRCSEL_2 0x178
#define ICP_RING_CSR_INT_COL_EN 0x17C
#define ICP_RING_CSR_INT_COL_CTL 0x180
/*This value is included here as we dont yet need to create device specific
  header files for userspace*/
#define ICP_DH895xCC_RING_CSR_FLAG_AND_COL_EN 0x184
#define DEFAULT_BUNDLE_SIZE 0x200

/* RingConfig CSR Parameter Watermark Offsets */
#define RING_CONFIG_NEAR_FULL_WM 0x0A
#define RING_CONFIG_NEAR_EMPTY_WM 0x05
#define RING_CONFIG_LATE_HEAD_POINTER_MODE 0x80000000

/* Default RingConfig is Nearly Full = Full and Nearly Empty = Empty */
#define BUILD_RING_CONFIG(size)                                                \
    ((ICP_RING_NEAR_WATERMARK_0 << RING_CONFIG_NEAR_FULL_WM) |                 \
     (ICP_RING_NEAR_WATERMARK_0 << RING_CONFIG_NEAR_EMPTY_WM) | size)

/* Response Ring Configuration */
#define BUILD_RESP_RING_CONFIG(size, watermark_nf, watermark_ne)               \
    ((watermark_nf << RING_CONFIG_NEAR_FULL_WM) |                              \
     (watermark_ne << RING_CONFIG_NEAR_EMPTY_WM) | size)

/* All Ring Base Addresses are 64 byte aligned, thus
 * bits[43:0] of the RingBase register correspond to
 * bits[49:6] of the Rings Memory Address. */

#define BUILD_RING_BASE_ADDR(addr, size)                                       \
    ((addr >> 6) & (0xFFFFFFFFFFFFFFFFULL << size))

#define IRQ_READ_SINT_OFFSET(dev, val)                                         \
    pci_read_config_dword(dev, SINT_OFFSET, val)

/* CSR read/write macros */
#define READ_CSR_RING_CONFIG(bank_offset, ring)                                \
    ICP_ADF_CSR_RD(csr_base_addr,                                              \
                   bank_offset + ICP_RING_CSR_RING_CONFIG + (ring << 2))

#define READ_CSR_RING_HEAD(bank_offset, ring)                                  \
    ICP_ADF_CSR_RD(csr_base_addr,                                              \
                   bank_offset + ICP_RING_CSR_RING_HEAD_OFFSET + (ring << 2))

#define READ_CSR_RING_TAIL(bank_offset, ring)                                  \
    ICP_ADF_CSR_RD(csr_base_addr,                                              \
                   bank_offset + ICP_RING_CSR_RING_TAIL_OFFSET + (ring << 2))

#define READ_CSR_E_STAT(bank_offset)                                           \
    ICP_ADF_CSR_RD(csr_base_addr, bank_offset + ICP_RING_CSR_E_STAT)

#define READ_CSR_NE_STAT(bank_offset)                                          \
    ICP_ADF_CSR_RD(csr_base_addr, bank_offset + ICP_RING_CSR_NE_STAT)

#define READ_CSR_NF_STAT(bank_offset)                                          \
    ICP_ADF_CSR_RD(csr_base_addr, bank_offset + ICP_RING_CSR_NF_STAT)

#define READ_CSR_F_STAT(bank_offset)                                           \
    ICP_ADF_CSR_RD(csr_base_addr, bank_offset + ICP_RING_CSR_F_STAT)

#define READ_CSR_INT_EN(bank_offset)                                           \
    ICP_ADF_CSR_RD(csr_base_addr, bank_offset + ICP_RING_CSR_INT_EN)

#define READ_CSR_INT_REG(bank_offset)                                          \
    ICP_ADF_CSR_RD(csr_base_addr, bank_offset + ICP_RING_CSR_INT_REG)

#define READ_CSR_INT_COL_EN(bank_offset)                                       \
    ICP_ADF_CSR_RD(csr_base_addr, bank_offset + ICP_RING_CSR_INT_COL_EN)

#define READ_CSR_INT_COL_CTL(bank_offset)                                      \
    ICP_ADF_CSR_RD(csr_base_addr, bank_offset + ICP_RING_CSR_INT_COL_CTL)

#define WRITE_CSR_RING_CONFIG(bank_offset, ring, value)                        \
    ICP_ADF_CSR_WR(csr_base_addr,                                              \
                   bank_offset + ICP_RING_CSR_RING_CONFIG + (ring << 2),       \
                   value)

#define WRITE_CSR_RING_BASE(bank_offset, ring, value)                          \
    do                                                                         \
    {                                                                          \
        Cpa32U l_base = 0, u_base = 0;                                         \
        l_base = (Cpa32U)(value & 0xFFFFFFFF);                                 \
        u_base = (Cpa32U)((value & 0xFFFFFFFF00000000ULL) >> 32);              \
        ICP_ADF_CSR_WR(csr_base_addr,                                          \
                       bank_offset + ICP_RING_CSR_RING_LBASE + (ring << 2),    \
                       l_base);                                                \
        ICP_ADF_CSR_WR(csr_base_addr,                                          \
                       bank_offset + ICP_RING_CSR_RING_UBASE + (ring << 2),    \
                       u_base);                                                \
    } while (0)

static inline Cpa64U read_base(Cpa32U *csr_base_addr,
                               Cpa32U bank_offset,
                               Cpa32U ring)
{
    Cpa32U l_base = ICP_ADF_CSR_RD(
        csr_base_addr, bank_offset + ICP_RING_CSR_RING_LBASE + (ring << 2));
    Cpa32U u_base = ICP_ADF_CSR_RD(
        csr_base_addr, bank_offset + ICP_RING_CSR_RING_UBASE + (ring << 2));
    Cpa64U addr = (l_base & 0xFFFFFFFF);
    addr |= ((((Cpa64U)u_base) << 32) & 0xFFFFFFFF00000000ULL);
    return addr;
}

#define READ_CSR_RING_BASE(bank_offset, ring)                                  \
    read_base(csr_base_addr, bank_offset, ring)

#define WRITE_CSR_RING_HEAD(bank_offset, ring, value)                          \
    ICP_ADF_CSR_WR(csr_base_addr,                                              \
                   bank_offset + ICP_RING_CSR_RING_HEAD_OFFSET + (ring << 2),  \
                   value)

#define WRITE_CSR_RING_TAIL(bank_offset, ring, value)                          \
    ICP_ADF_CSR_WR(csr_base_addr,                                              \
                   bank_offset + ICP_RING_CSR_RING_TAIL_OFFSET + (ring << 2),  \
                   value)

#define WRITE_CSR_INT_EN(bank_offset, value)                                   \
    ICP_ADF_CSR_WR(csr_base_addr, bank_offset + ICP_RING_CSR_INT_EN, value)

#define WRITE_CSR_INT_SRCSEL(bank_offset)                                      \
    do                                                                         \
    {                                                                          \
        ICP_ADF_CSR_WR(csr_base_addr,                                          \
                       bank_offset + ICP_RING_CSR_INT_SRCSEL,                  \
                       BANK_INT_SRC_SEL_MASK_0);                               \
        ICP_ADF_CSR_WR(csr_base_addr,                                          \
                       bank_offset + ICP_RING_CSR_INT_SRCSEL_2,                \
                       BANK_INT_SRC_SEL_MASK_X);                               \
    } while (0)

#define UPDATE_CSR_INT_SRCSEL_RING(bank_offset, ring, val)                     \
    do                                                                         \
    {                                                                          \
        if (ring > IAINTSRCSEL_REG_MAX_RING_IN_REG)                            \
        {                                                                      \
            ICP_ADF_CSR_WR(                                                    \
                csr_base_addr, bank_offset + ICP_RING_CSR_INT_SRCSEL_2, val);  \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            ICP_ADF_CSR_WR(                                                    \
                csr_base_addr, bank_offset + ICP_RING_CSR_INT_SRCSEL, val);    \
        }                                                                      \
    } while (0)

#define WRITE_CSR_INT_REG(bank_offset, value)                                  \
    ICP_ADF_CSR_WR(csr_base_addr, bank_offset + ICP_RING_CSR_INT_REG, value)

#define WRITE_CSR_INT_COL_EN(bank_offset, value)                               \
    ICP_ADF_CSR_WR(csr_base_addr, bank_offset + ICP_RING_CSR_INT_COL_EN, value)

#define WRITE_CSR_INT_COL_CTL(bank_offset, value)                              \
    ICP_ADF_CSR_WR(csr_base_addr,                                              \
                   bank_offset + ICP_RING_CSR_INT_COL_CTL,                     \
                   ETR_CSR_INTR_COL_CTL_ENABLE | value)

#define WRITE_CSR_INT_COL_CTL_CLR(bank_offset)                                 \
    ICP_ADF_CSR_WR(csr_base_addr, bank_offset + ICP_RING_CSR_INT_COL_CTL, 0)

#endif /* ADF_PLATFORM_ACCELDEV_COMMON_H */
