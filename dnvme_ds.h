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

#ifndef _DNVME_DS_H_
#define _DNVME_DS_H_

#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#include "dnvme_interface.h"

/* 0.0.01 */
#define    DRIVER_VERSION           0x00000001
#define    DRIVER_VERSION_STR(VER)  #VER

/* To store the max vector locations */
#define    MAX_VEC_SLT              2048
/*
 * Strucutre used to define all the essential parameters
 * related to PRP1, PRP2 and PRP List
 */
struct nvme_prps {
    u32 npages; /* No. of pages inside the PRP List */
    u32 type; /* refers to types of PRP Possible */
    /* List of virtual pointers to PRP List pages */
    __le64 **vir_prp_list;
    u8 *vir_kern_addr; /* K.V.A for pinned down pages */
    __le64 prp1; /* Physical address in PRP1 of command */
    __le64 prp2; /* Physical address in PRP2 of command */
    dma_addr_t first_dma; /* First entry in PRP List */
    /* Size of data buffer for the specific command */
    u32 data_buf_size;
    /* SG table use sg_next(nvme_prps.st.sgl) */
    struct sg_table st;
    /* Number of pages mapped to DMA area */
    u32 num_map_pgs;
    /* Address of data buffer for the specific command */
    u64 data_buf_addr;
    enum dma_data_direction data_dir;
};

/*
 * structure for the CQ tracking params with virtual address and size.
 */
struct nvme_trk_cq {
    u8          *vir_kern_addr;  /* phy addr ptr to the q's alloc to kern mem */
    dma_addr_t   cq_dma_addr;    /* dma mapped address using dma_alloc */
    u32          size;           /* length in bytes of the alloc Q in kernel */
    u32 __iomem *dbs;            /* Door Bell stride  */
    u8           contig;         /* Indicates if prp list is contig or not */
    u8           bit_mask;       /* bitmask added for unique ID creation */
    struct nvme_prps  prp_persist; /* PRP element in CQ */
};

/*
 *    Structure definition for tracking the commands.
 */
struct cmd_track {
    u16 unique_id;      /* driver assigned unique id for a particular cmd */
    u16 persist_q_id;   /* target Q ID used for Create/Delete Q's, never == 0 */
    u8  opcode;         /* command opcode as per spec */
    struct list_head cmd_list_hd; /* link-list using the kernel list */
    struct nvme_prps prp_nonpersist; /* Non persistent PRP entries */
};

/*
 * structure definition for SQ tracking parameters.
 */
struct nvme_trk_sq {
    void        *vir_kern_addr;     /* virtual kernal address using kmalloc */
    dma_addr_t   sq_dma_addr;       /* dma mapped address using dma_alloc */
    u32          size;              /* len in bytes of allocated Q in kernel */
    u32 __iomem *dbs;               /* Door Bell stride */
    u16          unique_cmd_id;     /* unique counter for each comand in SQ */
    u8           contig;            /* Indicates if prp list is contig or not */
    u8           bit_mask;          /* bitmask added for unique ID creation */
    struct nvme_prps prp_persist;   /* PRP element in CQ */
    struct list_head cmd_track_list;/* link-list head for cmd_track list */
};

/*
 * Structure with Metrics of CQ. Has a node which makes it work with
 * kernel linked lists.
 */
struct metrics_cq {
    struct list_head    cq_list_hd; /* link-list using the kernel list  */
    struct nvme_gen_cq  public_cq;  /* parameters in nvme_gen_cq */
    struct nvme_trk_cq  private_cq; /* parameters in nvme_trk_cq */
};

/*
 * Structure with Metrics of SQ. Has a node which makes it work with
 * kernel linked lists.
 */
struct metrics_sq {
    struct list_head    sq_list_hd;  /* link-list using the kernel list */
    struct nvme_gen_sq  public_sq;   /* parameters in nvme_gen_sq */
    struct nvme_trk_sq  private_sq;  /* parameters in nvme_trk_sq */
};

/*
 * Structure with cq track parameters for interrupt related functionality.
 * Note:- Struct used for u16 for future additions
 */
struct irq_cq_track {
    struct list_head irq_cq_head;    /* linked list head for irq CQ trk */
    u16              cq_id;          /* Completion Q id */
};

/*
 * Structure with parameters of IRQ vector, CQ track linked list and irq_no
 */
struct irq_track {
    struct  list_head irq_list_hd;    /* list head for irq track list */
    struct  list_head irq_cq_track;   /* linked list of IRQ CQ nodes */
    u16               irq_no;         /* idx in list; always 0 based */
    u32               int_vec;        /* vec number; assigned by OS */
    u8                isr_fired;      /* flag to indicate if irq has fired */
    u32               isr_count;      /* total no. of times irq fired */
    u32               outstanding_cmd_count; /* count of outstanding cmds */
};

/*
 * structure for meta data per device parameters.
 */
struct metrics_meta_data {
    struct list_head meta_trk_list;
    struct dma_pool *meta_dmapool_ptr;
    u32              meta_buf_size;
};

/*
 * Structure for meta data buffer allocations.
 */
struct metrics_meta {
    struct list_head meta_list_hd;
    u32              meta_id;
    void *           vir_kern_addr;
    dma_addr_t       meta_dma_addr;
};

/*
 * Structure for Nvme device private parameters. These parameters are
 * device specific and populated while the nvme device is being opened
 * or during probe.
 */
struct private_metrics_dev {
    struct pci_dev *pdev;           /* Pointer to the PCIe device */
    struct device *spcl_dev;        /* Special device file */
    struct nvme_ctrl_reg __iomem *ctrlr_regs;  /* Pointer to reg space */
    u8 __iomem *bar0;               /* 64 bit BAR0 memory mapped ctrlr regs */
    u8 __iomem *bar1;               /* 64 bit BAR1 I/O mapped registers */
    u8 __iomem *bar2;               /* 64 bit BAR2 memory mapped MSIX table */
    struct dma_pool *prp_page_pool; /* Mem for PRP List */
    struct device *dmadev;          /* Pointer to the dma device from pdev */
    int minor_no;                   /* Minor no. of the device being used */
    u8 open_flag;                   /* Allows device opening only once */
};

/*
 * Structure with nvme device related public and private parameters.
 */
struct nvme_device {
    struct private_metrics_dev private_dev;
    struct public_metrics_dev  public_dev;
};

/*
 * Work container which holds vectors and scheduled work queue item
 */
struct work_container {
    struct list_head wrk_list_hd;
    struct work_struct sched_wq; /* Work Struct item used in bh */
    u16 irq_no; /* 0 based irq_no */
    u32 int_vec; /* Interrupt vectors assigned by the kernel */
    /* Pointer to the IRQ_processing strucutre of the device */
    struct irq_processing *pirq_process;
};

/*
 * Irq Processing structure to hold all the irq parameters per device.
 */
struct irq_processing {
    /* irq_track_mtx is used only while traversing/editing/deleting the
     * irq_track_list
     */
    struct list_head irq_track_list; /* IRQ list; sorted by irq_no */
    struct mutex irq_track_mtx; /* Mutex for access to irq_track_list */

    /* To resolve contention for ISR's getting scheduled on different cores */
    spinlock_t isr_spin_lock;

    /* Mask pointer for ISR (read both in ISR and BH) */
    /* Pointer to MSI-X table offset or INTMS register */
    u8 __iomem *mask_ptr;
    /* Will only be read by ISR and set once per SET/DISABLE of IRQ scheme */
    u8 irq_type; /* Type of IRQ set */

    /* Used by ISR to enqueue work to BH */
    struct workqueue_struct *wq; /* Wq per device */

    /* Used by BH to dequeue work and process on it */
    /* Head of work_container's list */
    /* Remains static throughout the lifetime of the interrupts */
    struct list_head wrk_item_list;
};

/*
 * Structure which defines the device list for all the data structures
 * that are defined.
 */
struct metrics_device_list {
    struct  list_head    metrics_device_hd; /* metrics linked list head */
    struct  list_head    metrics_cq_list;   /* CQ linked list */
    struct  list_head    metrics_sq_list;   /* SQ linked list */
    struct  nvme_device *metrics_device;    /* Pointer to this nvme device */
    struct  mutex        metrics_mtx;       /* Mutex for locking per device */
    struct  metrics_meta_data metrics_meta; /* Pointer to meta data buff */
    struct  irq_processing irq_process;     /* IRQ processing structure */
};

/* Global linked list for the entire data structure for all devices. */
extern struct list_head metrics_dev_ll;

#endif
