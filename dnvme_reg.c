/*
 * NVM Express Compliance Suite
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_reg.h"
#include "definitions.h"
#include "sysdnvme.h"

#ifdef QEMU
/* QEMU doesn't suppport writeq() or readq(), thus make our own */
inline u64 READQ(const volatile void __iomem *addr)
{
    return (((u64)readl(addr + 4) << 32) | (u64)readl(addr));
}
inline void WRITEQ(u64 val, volatile void __iomem *addr)
{
    writel(val, addr);
    writel(val >> 32, addr + 4);
}

#else

inline void WRITEQ(u64 val, volatile void __iomem *addr)
{
    writeq(val, addr);
}
inline u64 READQ(const volatile void __iomem *addr)
{
    return readq(addr);
}
#endif

/*
 * read_nvme_reg_generic  - Function to read the controller registers located in
 * the MLBAR/MUBAR (PCI BAR 0 and 1) that are mapped to memory area which
 * supports in-order access.
 */
int read_nvme_reg_generic(u8 __iomem *bar0,
    u8 *udata, u32 nbytes, u32 offset, enum nvme_acc_type acc_type)
{
    u32 index = 0;
    u32 u32data;
    u64 u64data;
    u16 u16data;
    u8  u8data;

    bar0 += offset;

    /* loop until user requested nbytes are read. */
    for (index = 0; index < nbytes; ) {

        /* Read data from NVME space */
        if (acc_type == BYTE_LEN) {
            u8data = readb(bar0);

            /* Copy data to user buffer. */
            memcpy((u8 *)&udata[index], &u8data, sizeof(u8));
            LOG_DBG("NVME Read byte at 0x%llX:0x%0X", (u64)bar0, u8data);

            bar0 += 1;
            index += 1;

        } else if (acc_type == WORD_LEN) {
            u16data = readw(bar0);

            /* Copy data to user buffer. */
            memcpy((u8 *)&udata[index], &u16data, sizeof(u16));
            LOG_DBG("NVME Read WORD at 0x%llX:0x%X", (u64)bar0, u16data);

            bar0 += 2;
            index += 2;

        } else if (acc_type == DWORD_LEN) {
            u32data = readl(bar0);

            /* Copy data to user buffer. */
            memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
            LOG_DBG("NVME Read DWORD at 0x%llX:0x%X", (u64)bar0, u32data);

            bar0 += 4;
            index += 4;

        } else if (acc_type == QUAD_LEN) {
            u64data = READQ(bar0);

            /* Copy data to user buffer. */
            memcpy((u8 *)&udata[index], &u64data, sizeof(u64));
            LOG_DBG("NVME Read QUAD 0x%llX:0x%llX",
                (u64)bar0, u64data);

            bar0 += 8;
            index += 8;

        } else {
            LOG_ERR("Use only BYTE/WORD/DWORD/QUAD access type");
            return -EINVAL;
        }
    }

    return 0;
}

/*
 * write_nvme_reg_generic  - Function to write the controller registers
 * located in the MLBAR/MUBAR (PCI BAR 0 and 1) that are mapped to
 * memory area which supports in-order access.
 */
int write_nvme_reg_generic(u8 __iomem *bar0,
    u8 *udata, u32 nbytes, u32 offset, enum nvme_acc_type acc_type)
{
    u32 index = 0;
    u32 u32data;
    u64 u64data;
    u16 u16data;
    u8  u8data;

    bar0 += offset;

    /* loop until user requested nbytes are written. */
    for (index = 0; index < nbytes;) {

        /* Check the acc_type and do write as per the access type  */
        if (acc_type == BYTE_LEN) {
            memcpy((u8 *)&u8data, &udata[index], sizeof(u8));
            writeb(u8data, bar0);
            LOG_DBG("NVME Writing BYTE at Addr:Val::0x%llX:0x%X",
                (u64)bar0, u8data);

            bar0 += 1;
            index += 1;

        } else if (acc_type == WORD_LEN) {
            memcpy((u8 *)&u16data, &udata[index], sizeof(u16));
            writew(u16data, bar0);
            LOG_DBG("NVME Writing WORD at Addr:Val::0x%llX:0x%X",
                (u64)bar0, u16data);

            bar0 += 2;
            index += 2;

        } else if (acc_type == DWORD_LEN) {
            memcpy((u8 *)&u32data, &udata[index], sizeof(u32));
            writel(u32data, bar0);
            LOG_DBG("NVME Writing DWORD at Addr:Val::0x%llX:0x%X",
                (u64)bar0, u32data);

            bar0 += 4;
            index += 4;

        } else if (acc_type == QUAD_LEN) {
            memcpy((u8 *)&u64data, &udata[index], sizeof(u64));
            WRITEQ(u64data, bar0);
            LOG_DBG("NVME Writing QUAD at Addr:Val::0x%llX:0x%llX",
                (u64)bar0, u64data);

            bar0 += 8;
            index += 8;

        } else {
            LOG_ERR("use only BYTE/WORD/DWORD/QUAD");
            return -EINVAL;
        }
    }

    return 0;
}
