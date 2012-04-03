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

#ifndef _SYSFUNCPROTO_H_
#define _SYSFUNCPROTO_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>

#include "dnvme_ds.h"
#include "dnvme_reg.h"
#include "sysdnvme.h"

/**
 * pci probe function prototype. This function mainly
 * probe the pci bus and returns if is finds a PCIe
 * nvme based card in the system.
 * It provides the base address and bus, dev, fun no.s
 * for the device it found.
 * @param pdev
 * @param id
 * @return whether probing was successful or not.
 */
int dnvme_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id);

/**
 * This function prototype specifies the device ioctl entry point
 * for block type of device.
 * The ioctl main entry starts from this function which takes
 * argument specific to block device.
 * @param bdev
 * @param mode
 * @param cmd
 * @param arg
 * @return whether successful or not.
 */
int dnvme_ioctl(struct block_device *bdev, fmode_t mode, unsigned int cmd,
    unsigned long arg);

/**
 * This is the main entry point for IOCTL char device type.
 * The user selection of IOCTL required is specified in the
 * ioctl_num parameter based on which corresponding IOCTL
 * call is made. The data supplied by used is in ioctl_param.
 * @param filp
 * @param ioctl_num
 * @param ioctl_param
 * @return whether successful or not.
 */
long dnvme_ioctl_device(struct file *filp, unsigned int ioctl_num,
        unsigned long ioctl_param);
/**
 * driver_generic_read is a function that is called from
 * driver IOCTL when user want to read data from the
 * NVME card. The read parameter like offset and length
 * etc are specified from the struct rw_generic
 * @param nvme_data pointer to the device opened.
 * @param pmetrics_device_element
 * @return read success or failure.
 */
int driver_generic_read(struct rw_generic *nvme_data,
    struct  metrics_device_list *pmetrics_device_element);
/**
 * driver_generic_write is a function that is called from
 * driver IOCTL when user want to write data to the
 * NVME card. The write parameters offset and length
 * etc are specified from the struct nvme_write_generic
 * @param nvme_data pointer to the device opened.
 * @param pmetrics_device_element
 * @return write success or failure.
 */
int driver_generic_write(struct rw_generic *nvme_data,
        struct  metrics_device_list *pmetrics_device_element);

/**
 * device_status_chk  - Generic error checking function
 * which checks error registers and set kernel
 * alert if a error is detected.
 * @param pmetrics_device_element
 * @param status
 * @return device status fail or success.
 */
int device_status_chk(struct  metrics_device_list *pmetrics_device_element,
        int *status);

/**
 * driver_create_asq - Driver Admin Submission Queue creation routine
 * @param create_admn_q
 * @param pmetrics_device_element
 * @return ASQ creation SUCCESS or FAIL
 */
int driver_create_asq(struct nvme_create_admn_q *create_admn_q,
        struct  metrics_device_list *pmetrics_device_element);

/*
 * driver_iotcl_init - Driver Initialization routine before starting to
 * issue  ioctls.
 * @param pdev
 * @param pmetrics_device_list
 * @return init SUCCESS or FAIL
 */
int driver_ioctl_init(struct pci_dev *pdev, void __iomem *ctrlrRegs,
        struct metrics_device_list *pmetrics_device_list);

/**
 * driver_create_acq - Driver Admin completion  Queue creation routine
 * @param create_admn_q
 * @param pmetrics_device_list
 * @return ACQ creation SUCCESS or FAIL
 */
int driver_create_acq(struct nvme_create_admn_q *create_admn_q,
        struct metrics_device_list *pmetrics_device_list);

/**
 * driver_nvme_prep_sq - Driver routine to set up user parameters into metrics
 * for prepating the IO SQ.
 * @param prep_sq
 * @param pmetrics_device_element
 * @return allocation of contig mem SUCCESS or FAIL.
 */
int driver_nvme_prep_sq(struct nvme_prep_sq *prep_sq,
        struct  metrics_device_list *pmetrics_device_element);

/**
 * driver_nvme_prep_cq - Driver routine to set up user parameters into metrics
 * for prepating the IO CQ.
 * @param prep_cq
 * @param pmetrics_device_element
 * @return allocation of contig mem SUCCESS or FAIL.
 */
int driver_nvme_prep_cq(struct nvme_prep_cq *prep_cq,
        struct  metrics_device_list *pmetrics_device_element);

/**
 * driver_send_64b - Routine for sending 64 bytes command into
 * admin/IO SQ/CQ's
 * @param pmetrics_device
 * @param nvme_64b_send
 * @return Error Codes
 */
int driver_send_64b(struct  metrics_device_list *pmetrics_device,
    struct nvme_64b_send *nvme_64b_send);

/**
 * driver_log - Driver routine to log data into file from metrics
 * @param n_file
 * @return allocation of contig mem SUCCESS or FAIL.
 */
int driver_log(struct nvme_file *n_file);

/**
 * deallocate_all_queues - This function will start freeing up the memory for
 * the queues (SQ and CQ) allocated during the prepare queues. This function
 * takes a parameter, ST_DISABLE or ST_DISABLE_COMPLETELY, which identifies if
 * you need to clear Admin or not. Also while driver exit call this function
 * with ST_DISABLE_COMPLETELY.
 * @param pmetrics_device
 * @param nstate
 */
void deallocate_all_queues(struct  metrics_device_list *pmetrics_device,
        enum nvme_state nstate);

/**
 * driver_reap_inquiry - This function will traverse the metrics device list
 * for the given cq_id and return the number of commands that are to be reaped.
 * This is only function apart from initializations, that will modify tail_ptr
 * for the corresponding CQ.
 * @param pmetrics_device
 * @param usr_reap_inq
 * @return success or failure based on reap_inquiry
 */
int driver_reap_inquiry(struct  metrics_device_list *pmetrics_device,
    struct nvme_reap_inquiry *usr_reap_inq);

/**
 * dnvme_device_open - This operation is always the first operation performed
 * on the device file.
 * @param inode
 * @param filp
 * @return success or failure based on device open
 */
int dnvme_device_open(struct inode *inode, struct file *filp);

/**
 * dnvme_device_release - This operation is invoked when the file structure
 * is being released.
 * @param inode
 * @param filp
 * @return success or failure based on device clean up.
 */
int dnvme_device_release(struct inode *inode, struct file *filp);

/**
 * dnvme_device_mmap - This mmap will do the linear mapping to device memory
 * into user space.
 * The parameter vma holds all the required mapping and return the caller with
 * virtual address.
 * @param filp
 * @param vma
 * @return success or failure depending on mapping.
 */
int dnvme_device_mmap(struct file *filp, struct vm_area_struct *vma);

/**
 * driver_reap_cq - Reap the number of elements specified for the given CQ id.
 * Return the CQ entry data in the buffer specified.
 * @param pmetrics_device
 * @param usr_reap_data
 * @return Success of Failure based on Reap Success or failure.
 */
int driver_reap_cq(struct  metrics_device_list *pmetrics_device,
    struct nvme_reap *usr_reap_data);

/**
 * Create a dma pool for the requested size. Initialize the DMA pool pointer
 * with DWORD alignment and associate it with the active device.
 * @param pmetrics_device_element
 * @param alloc_size
 * @return SUCCESS or FAIL based on dma pool creation.
 */
int metabuff_create(struct metrics_device_list *pmetrics_device_element,
        u16 alloc_size);

/**
 * Create a meta buffer node when user request and allocate a consistent
 * dma memory from the meta dma pool. Add this node into the meta data
 * linked list.
 * @param pmetrics_device_element
 * @param meta_id
 * @return Success of Failure based on dma alloc Success or failure.
 */
int metabuff_alloc(struct metrics_device_list *pmetrics_device_element,
        u32 meta_id);

/**
 * Delete a meta buffer node when user requests and deallocate a consistent
 * dma memory. Delete this node from the meta data linked list.
 * @param pmetrics_device_element
 * @param meta_id
 * @return Success of Failure based on metabuff delete
 */
int metabuff_del(struct metrics_device_list *pmetrics_device_element,
        u32 meta_id);

/*
 * deallocate_mb will free up the memory and nodes for the meta buffers
 * that were allocated during the alloc and create meta. Finally
 * destroys the dma pool and free up the metrics meta node.
 * @param pmetrics_device
 */
void deallocate_mb(struct  metrics_device_list *pmetrics_device);

#endif
