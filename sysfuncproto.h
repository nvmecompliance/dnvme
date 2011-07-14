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

static int dnvme_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id);
static int dnvme_ioctl(struct block_device *bdev,
			fmode_t mode, unsigned int cmd,
			unsigned long arg);
static int dnvme_blk_gendisk(struct pci_dev *pdev, int which);

static int dnvme_ioctl_device(
			struct inode *inode,
			struct file *file,
			unsigned int ioctl_num,
			unsigned long ioctl_param);
/*
static int dnvme_pic_remove();
static int dnvme_pci_suspend();
static int dnvme_pci_resume();
*/

int driver_default_ioctl(struct file *file,
                        unsigned long buffer,
                        size_t length
);


#endif
