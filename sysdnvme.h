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

#ifndef _SYSDNVME_H_
#define _SYSDNVME_H_

#define APPNAME         "dnvme"
#define LEVEL           APPNAME

/* LOG_NRM() macro should be used with caution. It was originally peppered
 * throughout the code and enough latency was introduced while running within
 * QEMU that tnmve would sometimes miss CE's arriving from the simulated hdw.
 * It was discovered that syslogd was running causing QEMU to hold off
 * important threads. Converting almost all LOG_NRM's to LOG_DBG's and realizing
 * that the dnvme should be as efficient as possible made this issue disappear.
 */
#define LOG_NRM(fmt, ...)    \
    printk("%s: " fmt "\n", LEVEL, ## __VA_ARGS__)
#define LOG_ERR(fmt, ...)    \
    printk("%s-err:%s:%d: " fmt "\n", \
    LEVEL, __FILE__, __LINE__, ## __VA_ARGS__)
#ifdef DEBUG
#define LOG_DBG(fmt, ...)    \
    printk("%s-dbg:%s:%d: " fmt "\n", \
    LEVEL, __FILE__, __LINE__, ## __VA_ARGS__)
#else
#define LOG_DBG(fmt, ...)
#endif

/* Debug flag for IOCT_SEND_64B module */
#define TEST_PRP_DEBUG

/**
 * Absract the differences in trying to make this driver run within QEMU and
 * also within real world 64 bit platforms agaisnt real hardware.
 */
inline u64 READQ(const volatile void __iomem *addr);
inline void WRITEQ(u64 val, volatile void __iomem *addr);


#endif /* sysdnvme.h */
