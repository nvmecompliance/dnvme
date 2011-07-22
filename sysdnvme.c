/*
* sysdnvme.c
* NMVE Express Device Driver for Test Complinace.
*
* Copyright (c) 2011, Intel Corporation.
*
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include "dnvme_interface.h"
#include "definitions.h"
#include "sysfuncproto.h"
#include "sysdnvme.h"
#include "dnvme_ioctls.h"
#include "version.h"

#define	DRV_NAME		"dnvme"
#define	NVME_DEVICE_NAME	"qnvme"
#define DRV_VERSION		"NVME_1.0a"
#define NVME_N_DEVICES		1
#define _CLASSIC_		1
#define NVME_BLOCK_SIZE		512
#define NVME_BUFFER_SIZE	1024

/*
* Define the PCI storage express as
* 0x1FFFFF
*/
static DEFINE_PCI_DEVICE_TABLE(dnvme_pci_tbl) = {
    { PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xFFFF00) },
    { 0, }
    };

/*
* PCI dnvme driver structure definition
*/
static struct pci_driver dnvme_pci_driver = {
    .name           = DRV_NAME,
    .id_table       = dnvme_pci_tbl,
    .probe          = dnvme_pci_probe,
};

/*
*   This is the main ioctl for char type device
*   this ioctl invoke the dnmve device ioctls.
*/
static const struct file_operations dnvme_fops_f = {
      .owner = THIS_MODULE,
      .ioctl = dnvme_ioctl_device,
};

/*
*   struct nvme_dev_char will be basic structure for list
*   of nvme devices
*/
struct nvme_dev_char {
	struct cdev cdev;
};

/*
* Block deivce ioctls when necessary in future.
*/
static const struct block_device_operations dnvme_fops = {
      .owner = THIS_MODULE,
      .ioctl = dnvme_ioctl,
};

/* local parameters */
static int NVME_MAJOR;
static struct class *class_nvme;
static struct device *nvme_devices;

LIST_HEAD(nvme_devices_llist);
module_param(NVME_MAJOR, int, 0);
MODULE_LICENSE("GPL");

/**
*   First initialization for Driver code.
*   dnvme_init - Perform early initialization of the host
*   host: dnvme host to initialize
*   @return returns 0 if iniitalization was successful.
 *  @author T.Sravan Kumar
*/
static int dnvme_init(void)
{
   int retCode = -ENODEV;
   int nvme_major = -EBUSY;
   int nvme_minor = 0;
   int err = -EINVAL;
   dev_t nvme_dev_c = 0;
   dev_t devno = 0;
   struct device *device = NULL;
   int nvme_ndevices = NVME_N_DEVICES;
   struct nvme_dev_char *dev = NULL;

    LOG_NRM("version: %d.%d", VER_MAJOR, VER_MINOR);
   LOG_DBG("Init module - dnvme init");

   if (!_CLASSIC_) {
	nvme_major = register_blkdev(NVME_MAJOR, DRV_NAME);
	if (nvme_major < 0) {
		/*Unable to register the PCI device */
		LOG_ERROR("NVME Blk Registration failed\n");
		return nvme_major;
	} else {

		NVME_MAJOR = nvme_major;
		 LOG_DEBUG("Major Number = %d\n", NVME_MAJOR);
	}
   }

   /*
   *  Unable to register block device for the moment in QEMU
   *  using it as char dev instead
   */
   /* This is classic way to register a char device */
   if (_CLASSIC_ == 1) {
	nvme_major = register_chrdev
				(
				NVME_MAJOR,
				NVME_DEVICE_NAME,
				&dnvme_fops_f
				);

	if (nvme_major < 0) {
		LOG_ERR("NVME Char Registration failed");
		return -ENODEV;
		}
	LOG_DEBUG("NVME Char type registered..\n");
   } else {
	err = alloc_chrdev_region
			(
				&nvme_dev_c,
				0,
				nvme_ndevices,
				NVME_DEVICE_NAME
			);
	if (err < 0) {
		LOG_ERR("Allocation region failed and stopping");
		return err;
	}
	/* Get the Major number for all the NVME devices */
	nvme_major = MAJOR(nvme_dev_c);
   }

   /* Allocate Major and create class */
   NVME_MAJOR = nvme_major;

   class_nvme = class_create(THIS_MODULE, NVME_DEVICE_NAME);

   /* Check if class_nvme creation has any issues */
   if (IS_ERR(class_nvme)) {
	err = PTR_ERR(class_nvme);
	LOG_ERR("Nvme class creation failed and stopping");
	return err;
   }

   /* Allocate kernel mem for each of the device using one now */
   nvme_devices = kzalloc
			(nvme_ndevices * sizeof(struct device),
			GFP_KERNEL);

   if (nvme_devices == NULL) {
	LOG_ERR("Allocation failed in nvme_device");
	err = -ENOMEM;
	return err;
   }

   /* Allocate kernel mem for each of the device using one now */
   dev = kzalloc
		(
			nvme_ndevices * sizeof(struct nvme_dev_char),
			GFP_KERNEL
		);

   /* Commented as of now.. but for multiple card support */
   /*   while (n_devices > 0) {*/
   devno = MKDEV(nvme_major, nvme_minor);
   /* nvme_dev_c->data = NULL;
      nvme_dev_c->buffer_size = NVME_BUFFER_SIZE;
      nvme_dev_c->block_size = NVME_BLOCK_SIZE;
   */
   cdev_init(&dev->cdev, &dnvme_fops_f);
   dev->cdev.owner = THIS_MODULE;
   dev->cdev.ops = &dnvme_fops_f;
   err = cdev_add(&dev->cdev, devno, 1);

   if (err) {
	LOG_ERR("Adding device to kernel failed");
	return err;
   }

   device = device_create(class_nvme, NULL, devno, NULL,
				NVME_DEVICE_NAME"%d", nvme_minor);
   if (IS_ERR(device)) {
	err = PTR_ERR(device);
	LOG_ERR("Device Createion failed");
	return err;
   }

   nvme_minor = nvme_minor + 1;
   /*} //while n_device*/

   retCode = pci_register_driver(&dnvme_pci_driver);

   if (retCode < 0) {
	/*Unable to register the PCI device */
	LOG_ERR("PCI Driver Registration unsuccessful");
	return retCode;
   }

   LOG_DBG("PCI Registration Success return code = %d", retCode);
   return 0;
}

/**
* dnvme_pci_probe - Probe the NVME PCIe device for BARs.
* this function is called when the driver invokes the fops
* after basic initialization is performed.
*/
int __devinit dnvme_pci_probe(struct pci_dev *pdev,
				     const struct pci_device_id *id)
{
   int retCode = -ENODEV; /* retCode is set to no devices */
   int bars = 0; /* initialize bars to 0 */
   struct nvme_device_entry *nvme_dev_list = NULL;
   u32  BaseAddress0 = 0;
   u32  *bar;

   /*
   *	Following the Iniitalization steps from LDD 3 and pci.txt.
   *	Before touching any device registers, the driver needs to enable
   *	the PCI device by calling pci_enable_device().
   */

   LOG_DBG("Start probing for NVME PCI Express Device");

   if ((retCode == pci_enable_device(pdev)) < 0) {
	LOG_ERR("PciEnable not successful");
	return retCode;
   }

   /* Why does retcode is negative here and still success? TSK */
   LOG_DBG("PCI enable Success!. Return Code = %d\n", retCode);

   if (pci_enable_device_mem(pdev)) {
	LOG_ERR("pci_enalbe_device_mem not successful");
	return -1;
   }

   LOG_DBG("NVME Probing... Dev = 0x%x Vendor = 0x%x",
			pdev->device, pdev->vendor);
   LOG_DBG("Bus No = 0x%x, Dev Slot = 0x%x",
			pdev->bus->number, PCI_SLOT(pdev->devfn));
   LOG_DBG("Dev Func = 0x%x, Class = 0x%x",
			PCI_FUNC(pdev->devfn), pdev->class);

    /* Return void, Enables bus mastering and calls pcibios_set_master */
   pci_set_master(pdev);

    /* MAke BAR mask fromt eh resource */
   bars = pci_select_bars(pdev, IORESOURCE_MEM);

   if (pci_request_selected_regions(pdev, bars, DRV_NAME)) {
	LOG_ERR("Can't select regions");
	return -EINVAL;
   } else {
	LOG_DBG("Select regions success");
   }

   LOG_DEBUG("Mask for PCI BARS = %d\n", bars);
   LOG_DEBUG("PCI Probe Success!. Return Code = %d\n", retCode);

   /**
   *  Try Allocating the device memory in the host and check
   *  for success.
   */

   nvme_dev_list = kzalloc((int)sizeof(struct nvme_device_entry), GFP_KERNEL);

   if (nvme_dev_list == NULL) {
	LOG_ERR("allocate Host Memory for Device Failed!!...nvme_dev_list");
	return -ENOMEM;
   }

   bar = ioremap(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0));

   if (bar != NULL) {
	LOG_DEBUG("Bar 0 Address:\n");
	LOG_DEBUG("Remap value.\n");
   } else {
	LOG_ERROR("allocate Host Memory for Device Failed!!\n");
	return -EINVAL;
   }

   /*
   * Only debug because the above remap should give BAR's
   */
   pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &BaseAddress0);
   LOG_DBG("PCI BAR 0 = 0x%x", BaseAddress0);

   /*
   * Call function to get the NVME Allocated to IOCTLs.
   * Using the generic nvme fops we will allocate the required
   * ioctl entry function.
   */
   dnvme_blk_gendisk(pdev, 0);

   nvme_dev_list->pdev = pdev;
   memcpy(&nvme_dev_list->bar, &BaseAddress0, sizeof(u32));
   nvme_dev_list->bus =  pdev->bus->number;
   nvme_dev_list->slot = PCI_SLOT(pdev->devfn);
   nvme_dev_list->func = PCI_FUNC(pdev->devfn);
   list_add_tail(&nvme_dev_list->list, &nvme_devices_llist);
   return retCode;
}

/*
* dnvme_blk_gendisk - Creates Block Disk to add to the the kernel
* This function helps in setting up the block device
* with required parameters for inserting into disk.
*/
int dnvme_blk_gendisk(struct pci_dev *pdev, int which)
{
   struct gendisk *disk;
   int size = 14096;

    disk = alloc_disk(NVME_MINORS);
    if (!disk) {
	LOG_ERR("Disk Allocation %d Failed", disk->major);
	return -EINVAL;
    } else {
	LOG_DBG("Major Allocation %d", disk->major);
	LOG_DBG("Minor Allocation %d", disk->minors);
	LOG_DBG("First Minor Allocation %d", disk->first_minor);
    }

   disk->major = NVME_MAJOR;
   disk->first_minor = which * NVME_MINORS;
   disk->fops = &dnvme_fops;

   snprintf(disk->disk_name, 32, "blkqnvme%c", which+'a');
   LOG_DBG("Disk Name = %s", disk->disk_name);
   if (disk->disk_name == NULL) {
	LOG_ERR("Disk name is empty");
	return -EINVAL;
   }

   set_capacity(disk, size);

   /* Driver Fails when tried add disk to kernel, for now using as char dev */
   /* add_disk(disk); */

   return 0;
}

/*
* dnvme_ioctl  - Call correcponding ioctl functions from Blk driver.
* This function is used only when the device is initialized as block
* device otherwise the char type ioctl is used.
*/
int dnvme_ioctl(struct block_device *bdev, fmode_t mode,
			unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
    case NVME_IOCTL_IDENTIFY_NS:
	LOG_DBG("IOCTL Identify NS Command");
	break;
    case NVME_IOCTL_IDENTIFY_CTRL:
	LOG_DBG("IOCTL Identify CTRL Command");
	break;
    case NVME_IOCTL_GET_RANGE_TYPE:
	LOG_DBG("IOCTL  NVME_IOCTL_GET_RANGE_TYPE Get Range Command");
	break;
    case NVME_IOCTL_SUBMIT_IO:
	LOG_DBG("IOCTL NVME_IOCTL_SUBMIT_IO Command");
	break;
    case NVME_IOCTL_DOWNLOAD_FW:
	LOG_DBG("IOCTL CNVME_IOCTL_DOWNLOAD_FW Command");
	break;
    case NVME_IOCTL_ACTIVATE_FW:
	LOG_DBG("IOCTL INVME_IOCTL_ACTIVATE_FW Command");
	break;
    default:
	return -ENOTTY;
     }
   return 0;
}

/*
 * This function is called whenever a process tries to do an ioctl on our
 * device file. We get two extra parameters (additional to the inode and file
 * structures, which all device functions get): the number of the ioctl called
 * and the parameter given to the ioctl function.
 *
 * If the ioctl is write or read/write (meaning output is returned to the
 * calling process), the ioctl call returns the output of this function.
 *
*/
int dnvme_ioctl_device(
		struct inode *inode,	/* see include/linux/fs.h */
		struct file *file,	/* ditto */
		unsigned int ioctl_num,	/* number and param for ioctl */
		unsigned long ioctl_param)
{
   int ret_val = -EINVAL;
   struct nvme_read_generic *nvme_rd_data;
   struct nvme_write_generic *nvme_wr_data;
   struct nvme_device_entry *nvme_dev_entry;
   int *nvme_dev_err_sts;
   struct pci_dev *pdev = NULL;

   /* Get the device from the linked list */
   list_for_each_entry(nvme_dev_entry, &nvme_devices_llist, list) {
	pdev = nvme_dev_entry->pdev;
	LOG_DBG("[Nvme_Drv] device [%02x:%02x.%02x]",
	nvme_dev_entry->bus,
	nvme_dev_entry->slot, nvme_dev_entry->func);
	}

   /*
   * Given a ioctl_num invoke corresponding function
   */
   switch (ioctl_num) {
   case NVME_IOCTL_ERR_CHK:
	/*
	* check if the device has any errors set in its status
	* register. And report errors.
	*/
	nvme_dev_err_sts = (int *)ioctl_param;
	LOG_DBG("Checking device Status");
	ret_val = device_status_chk(pdev, nvme_dev_err_sts);
	break;

   case NVME_IOCTL_READ_GENERIC:

	LOG_DBG("Invoking User App request to read  the PCI Header Space");
	nvme_rd_data = (struct nvme_read_generic *)ioctl_param;

	ret_val = driver_generic_read(file, nvme_rd_data, pdev);
	break;

   case NVME_IOCTL_WRITE_GENERIC:

	LOG_DBG("Invoke IOCTL Generic Write Funtion");
	nvme_wr_data = (struct nvme_write_generic *)ioctl_param;

	ret_val = driver_generic_write(file, nvme_wr_data, pdev);
	break;

   case NVME_IOCTL_CREATE_ADMN_Q:
	LOG_DBG("IOCTL for Create Admin Q\n");
	break;

   case NVME_IOCTL_DEL_ADMN_Q:
	LOG_DBG("IOCTL NVME_IOCTL_DEL_ADMN_Q Command");
	break;

   case NVME_IOCTL_SEND_ADMN_CMD:
	LOG_DBG("IOCTL NVME_IOCTL_SEND_ADMN_CMD Command");
	break;

   default:
	LOG_DBG("Cannot find IOCTL going to default case");
	ret_val = driver_default_ioctl(file, ioctl_param, 80);
	break;
   }

   return ret_val;
}

/*
*  Module Exit code.
*  dnvme_exit -sTODO  Perform clean exit
*/
static void __exit dnvme_exit(void)
{
   unregister_chrdev(NVME_MAJOR, NVME_DEVICE_NAME);
   pci_unregister_driver(&dnvme_pci_driver);

   LOG_DBG("dnvme exited...Bye");
}

/*
*  Driver Module Calls.
*/
MODULE_DESCRIPTION("Kernel Device Driver for NMVE 1.0a Spec PCI Express");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:"DRV_NAME);

module_init(dnvme_init);
module_exit(dnvme_exit);
