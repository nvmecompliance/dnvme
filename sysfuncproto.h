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
* @param pdev
* @param id
* @return whether probing was successful or not.
*/
int dnvme_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id);

/**
* This function protoype specifies the device ioctl entry point
* for block type of device.
* The ioctl main entry starts from this function whcih takes
* argument specific to block device.
* @param bdev
* @param mode
* @param cmd
* @param arg
* @return whether successful or not.
*/
int dnvme_ioctl(
			struct block_device *bdev,
			fmode_t mode, unsigned int cmd,
			unsigned long arg);

/**
* This function prototype is for assigning disk
* parameters in case if the device pdev is of block type
* device.
* @param pdev
* @param which
* @return whether successful or not.
*/
int dnvme_blk_gendisk(
			struct pci_dev *pdev,
			int which);

/**
* This is the main entry point for IOCTL char device type.
* The user selection of IOCTL required is specified in the
* ioctl_num parameter based on which corresponing IOCTL
* call is made. The data supplied by used is in ioctl_param.
* @param inode
* @param file
* @param ioctl_num
* @param ioctl_param
* @return whether successful or not.
*/
int dnvme_ioctl_device(
			struct inode *inode,
			struct file *file,
			unsigned int ioctl_num,
			unsigned long ioctl_param);
/**
* This is default Ioctl that gets called when the user
* ioctl_num does not
* match any valid ioctl number in the driver.
* @param file
* @param buffer
* @param length
* @return whether successful or not.
*/
int driver_default_ioctl(
			struct file *file,
			unsigned long buffer,
			size_t length);

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
int driver_generic_read(struct file *file,
			struct rw_generic *data_usr,
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
			struct rw_generic *data_usr,
			struct pci_dev *pdev);

/**
* device_status_chk  - Generic error checking function
* which checks error registers and set kernel
* alert if a error is detected.
* @param pdev
* @param status
*/
int device_status_chk(struct pci_dev *pdev,
			int *status);


#endif
