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
* @param file Pass the file descriptor of the device opened.
* @param data_usr Structure with different user required parameters.
* @param pdev pointer to the device opened.
* @return read success or failure.
*/
int driver_generic_read(struct file *file, struct rw_generic *data_usr,
    struct pci_dev *pdev);

/**
* driver_generic_write is a function that is called from
* driver IOCTL when user want to write data to the
* NVME card. The write parameters offset and length
* etc are specified from the struct nvme_write_generic
* @param file Pass the file descriptor of the device opened.
* @param data_usr Structure with different user required parameters.
* @param pdev pointer to the device opened.
* @return read success or failure.
*/
int driver_generic_write(struct file *file,
    struct rw_generic *data_usr, struct pci_dev *pdev);

/**
* device_status_chk  - Generic error checking function
* which checks error registers and set kernel
* alert if a error is detected.
* @param pdev
* @param status
*/
int device_status_chk(struct pci_dev *pdev, int *status);


/**
* driver_create_asq - Driver Admin Submission Queue creation routine
* @param create_admn_q
* @param nvme_dev
* @return ASQ creation SUCCESS or FAIL
*/
int driver_create_asq(struct nvme_create_admn_q *create_admn_q,
        struct nvme_device *nvme_dev);

/*
* driver_iotcl_init - Driver Initialization routine before starting to
* issue  ioctls.
* @param nvme_dev
* @param pdev
* @param pmetrics_device_list
* @return init SUCCESS or FAIL
*/
int driver_ioctl_init(struct nvme_dev_entry *nvme_dev,
    struct pci_dev *pdev, struct metrics_device_list *pmetrics_device_list);

/**
* driver_create_acq - Driver Admin completion  Queue creation routine
* @param create_admn_q
* @param pnvme_dev
* @return ACQ creation SUCCESS or FAIL
*/
int driver_create_acq(struct nvme_create_admn_q *create_admn_q,
        struct nvme_device *pnvme_dev);

/**
* driver_nvme_prep_sq - Driver routine to set up user parameters into metrics
* for prepating the IO SQ.
* @param prep_sq
* @param pnvme_dev
* @return allocation of contig mem SUCCESS or FAIL.
*/
int driver_nvme_prep_sq(struct nvme_prep_sq *prep_sq,
        struct nvme_device *pnvme_dev);

/**
* driver_nvme_prep_cq - Driver routine to set up user parameters into metrics
* for prepating the IO CQ.
* @param prep_cq
* @param pnvme_dev
* @return allocation of contig mem SUCCESS or FAIL.
*/
int driver_nvme_prep_cq(struct nvme_prep_cq *prep_cq,
        struct nvme_device *pnvme_dev);

/**
 * free_allqs - Q deallocation routine for freeing up the kernel
 * memory.
 */
void free_allqs(void);

/**
* driver_send_64b - Routine for sending 64 bytes command into
* admin/IO SQ/CQ's
* @param nvme_dev
* @param nvme_64b_send
* @return Error Codes
*/
int driver_send_64b(struct nvme_dev_entry *nvme_dev,
    struct nvme_64b_send *nvme_64b_send);

/**
* driver_log - Driver routine to log data into file from metrics
* @param n_file
* @return allocation of contig mem SUCCESS or FAIL.
*/
int driver_log(struct nvme_file *n_file);

#endif
