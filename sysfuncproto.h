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
* This File is mainly for creating the function prototypes
*/
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
* This function prototype is for assigning disk
* parameters in case if the device pdev is of block type
* device.
* @param pdev
* @param which
* @return whether successful or not.
*/
int dnvme_blk_gendisk(struct pci_dev *pdev, int which);

/**
* This is the main entry point for IOCTL char device type.
* The user selection of IOCTL required is specified in the
* ioctl_num parameter based on which corresponding IOCTL
* call is made. The data supplied by used is in ioctl_param.
* @param inode
* @param file
* @param ioctl_num
* @param ioctl_param
* @return whether successful or not.
*/
int dnvme_ioctl_device(struct inode *inode, struct file *file,
    unsigned int ioctl_num, unsigned long ioctl_param);

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
int driver_ioctl_init(struct pci_dev *pdev, struct metrics_device_list
        *pmetrics_device_list);

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
* @param pmetrics_device_element
* @param nvme_64b_send
* @return Error Codes
*/
int driver_send_64b(struct  metrics_device_list *pmetrics_device_element,
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
 * @return success or failure based on deallocation.
 */
int deallocate_all_queues(struct  metrics_device_list *pmetrics_device,
        enum nvme_state nstate);

/**
 * driver_reap_inquiry - This function will traverse the metrics device list
 * for the given cq_id and return the number of commands that are to be reaped.
 * This is only function apart from initializations, that will modify tail_ptr
 * for the corresponding CQ.
 * @param pmetrics_device
 * @param reap_inq
 * @return success or failure based on reap_inquiry
 */
int driver_reap_inquiry(struct  metrics_device_list *pmetrics_device,
        struct nvme_reap_inquiry *reap_inq);

#endif
