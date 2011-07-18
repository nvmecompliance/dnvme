#ifndef _DFUNCPROTO_H_
#define _DFUNCPROTO_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>

/**
* This File is mainly for creating the function prototypes
*/
/**
* pci probe function prototype. This function mainly
* probe the pci bus and returns if is finds a PCIe
* nvme based card in the system.
* It provides the base address and bus, dev, fun no.s
* for the device it found.
*/
static int dnvme_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id);

/**
* This function protoype specifies the device ioctl entry point
* for block type of device.
* The ioctl main entry starts from this function whcih takes
* argument specific to block device.
*/
static int dnvme_ioctl(
			struct block_device *bdev,
			fmode_t mode, unsigned int cmd,
			unsigned long arg);

/**
* This function prototype is for assigning disk
* parameters in case if the device pdev is of block type
* device.
*/
static int dnvme_blk_gendisk(
			struct pci_dev *pdev,
			int which);

/**
* This is the main entry point for IOCTL char device type.
* The user selection of IOCTL required is specified in the
* ioctl_num parameter based on which corresponing IOCTL
* call is made. The data supplied by used is in ioctl_param.
*/
static int dnvme_ioctl_device(
			struct inode *inode,
			struct file *file,
			unsigned int ioctl_num,
			unsigned long ioctl_param);
/**
* This is default Ioctl that gets called when the user
* ioctl_num does not
* match any valid ioctl number in the driver.
*/
int driver_default_ioctl(
			struct file *file,
			unsigned long buffer,
			size_t length);

#endif
