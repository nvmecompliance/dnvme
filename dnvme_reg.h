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

#ifndef _DNVME_REG_H_
#define _DNVME_REG_H_

#include "dnvme_interface.h"


/**
 * nvme_ctrl_reg defines the register space for the
 * nvme controller registers as defined in NVME Spec 1.0b.
 */
struct nvme_ctrl_reg {
    u64    cap;    /* Controller Capabilities */
    u32    vs;     /* Version */
    u32    intms;  /* Interrupt Mask Set */
    u32    intmc;  /* Interrupt Mask Clear */
    u32    cc;     /* Controller Configuration */
    u32    rsvd1;  /* Reserved */
    u32    csts;   /* Controller Status */
    u32    rsvd2;  /* Reserved */
    u32    aqa;    /* Admin Queue Attributes */
    u64    asq;    /* Admin SQ Base Address */
    u64    acq;    /* Admin CQ Base Address */
};


#define REGMASK_CAP_CQR     (1 << 16)


/**
 * read_nvme_reg_generic function is a generic function which
 * reads data from the controller registers of the nvme with
 * user specified offset and bytes. Copies data back to udata
 * pointer which points to user space buffer.
 *
 */
int read_nvme_reg_generic(u8 __iomem *bar0,
        u8 *udata, u32 nbytes, u32 offset, enum nvme_acc_type acc_type);

/**
 * write_nvme_reg_generic function is a generic function which
 * writes data to the controller registers of the nvme with
 * user specified offset and bytes.
 */
int write_nvme_reg_generic(u8 __iomem *bar0,
        u8 *udata, u32 nbytes, u32 offset, enum nvme_acc_type acc_type);

#endif
