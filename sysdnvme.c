/**
* \file sysdnvme.c
* \brief NMVE Express Device Driver for Test Complinace.
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

#include "dnvme_ioctls.h"
#include "definitions.h"
#include "sysfuncproto.h"
#include "sysdnvme.h"

#define	DRV_NAME		"dnvme"
#define	NVME_DEVICE_NAME	"qnvme"
#define DRV_VERSION		"NVME_1.0a"
#define NVME_N_DEVICES		1
#define _CLASSIC_		0
#define NVME_BLOCK_SIZE		512
#define NVME_BUFFER_SIZE	1024

static DEFINE_PCI_DEVICE_TABLE(dnvme_pci_tbl) = {
    { PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0x1FFFFF) },
    { 0, }
    };


/**
* PCI dnvme driver structure definition
*/
static struct pci_driver dnvme_pci_driver = {
    .name           = DRV_NAME,
    .id_table       = dnvme_pci_tbl,
    .probe          = dnvme_pci_probe,
  /* .remove         = dnvme_pic_remove,
    .suspend        = dnvme_pci_suspend,
    .resume         = dnvme_pci_resume, */
};

/**
*   This is the main ioctl for char type device
*   this ioctl invoke the dnmve device ioctls.
*/
static const struct file_operations dnvme_fops_f = {
      .owner = THIS_MODULE,
      .ioctl = dnvme_ioctl_device,
};

/**
*   struct nvme_dev_char will be basic structure for list
*   of nvme devices
*/
struct nvme_dev_char {
	struct cdev cdev;
};

/**
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
static struct nvme_dev_char *nvme_dev_char;

static unsigned long nvme_block_size = NVME_BLOCK_SIZE;

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
     /**
      * desc no parameters, void
     */
  LOG_DEBUG("Init module - dnvme init\n");

  nvme_major = register_blkdev(NVME_MAJOR, DRV_NAME);
  if (nvme_major < 0) {
	/*Unable to register the PCI device */
       LOG_ERROR("NVME Blk Registration failed\n");
       return nvme_major;
     } else {

       NVME_MAJOR = nvme_major;

       LOG_DEBUG("Major Number = %d\n", NVME_MAJOR);
     }

    /* Unable to register block device for the moment in QEMU
     * using it as char dev instead
    */
   /* This is classic way to register a char device */
   if (_CLASSIC_) {
	nvme_major = register_chrdev(NVME_MAJOR, "nvme_ssd", &dnvme_fops_f);

	if (nvme_major < 0) {
		LOG_ERROR("NVME Char Registration failed...\n");
		return -ENODEV;
		}
   } else {
	err = alloc_chrdev_region
			(
				&nvme_dev_c,
				0,
				nvme_ndevices,
				NVME_DEVICE_NAME
			);
	if (err < 0) {
		LOG_ERROR("Allocation region failed and stoppin!!..\n");
		return err;
	}
	/* Get the Major number for all the NVME devices */
	nvme_major = MAJOR(nvme_dev_c);
   }

   /* Allocate Manor and create class */
   NVME_MAJOR = nvme_major;

   class_nvme = class_create(THIS_MODULE, NVME_DEVICE_NAME);

   /* Check if class_nvme creation has any issues */
   if (IS_ERR(class_nvme)) {
	err = PTR_ERR(class_nvme);
	LOG_ERROR("Nvme class creation failed and stopping!!...\n");
	return err;
   }

   /* Allocate kernel mem for each of the device using one now */
   nvme_devices = kzalloc
			(nvme_ndevices * sizeof(struct device),
			GFP_KERNEL);

   if (nvme_devices == NULL) {
	LOG_ERROR("Allocation failed in nvme_device..\n");
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
	LOG_ERROR("Adding device to kernel failed...\n");
	return err;
   }

   device = device_create(class_nvme, NULL, devno, NULL,
				NVME_DEVICE_NAME"%d", nvme_minor);
   if (IS_ERR(device)) {
	err = PTR_ERR(device);
	LOG_ERROR("Device Createion failed...\n");
	return err;
   }

   nvme_minor = nvme_minor + 1;
/*} //while n_device*/

  retCode = pci_register_driver(&dnvme_pci_driver);

   if (retCode < 0) {
	/*Unable to register the PCI device */
	LOG_ERROR("PCI Driver Registration unsuccessful..\n");
	return retCode;
   }

      LOG_DEBUG("PCI Registration Success\n\
	PCI Register driver return code = %d\n", retCode);
    return 0;
}

/**
*  dnvme_pci_probe - Probe the NVME PCIe device for BARs.
*  @param pdev
*  @param id
*  @author T.Sravan Kumar
*  @return whether probing was successful or not.
*/
static int __devinit dnvme_pci_probe(struct pci_dev *pdev,
				     const struct pci_device_id *id)
{
   int retCode = -ENODEV;
   /*int err = -EINVAL;*/
   int bars = 0;
   struct nvme_device_entry *nvme_dev_list = NULL;
   u32  BaseAddress0 = 0x0;
   u32  *bar;

   /**
   *	Following the Iniitalization steps from LDD 3 and pci.txt.
   *	Before touching any device registers, the driver needs to enable
   *	the PCI device by calling pci_enable_device().
   */

   LOG_DEBUG("Start probing for NVME PCI Express Device...\n");
   LOG_NORMAL("Logging using Log Normal...\n");

     if ((retCode == pci_enable_device(pdev)) < 0) {
	LOG_NORMAL("Error!! PciEnable not successful...\n");
	return retCode;
   }

   /* Why does retcode is negative here and still success? TSK */
   LOG_DEBUG("PCI enable Success!. Return Code = %d\n", retCode);

   if (pci_enable_device_mem(pdev)) {
	LOG_NORMAL("Error!! pci_enalbe_device_mem not successful...\n");
	return -1;
   }

   LOG_DEBUG("NVME Probing... Dev = 0x%x Vendor = 0x%x\n\
		Bus No = 0x%x, Dev Slot = 0x%x\n\
		Dev Func = 0x%x, Class = 0x%x\n",
		pdev->device, pdev->vendor,
		pdev->bus->number, PCI_SLOT(pdev->devfn),
		PCI_FUNC(pdev->devfn), pdev->class);

    /* Return void, Enables bus mastering and calls pcibios_set_master */
   pci_set_master(pdev);

    /* MAke BAR mask fromt eh resource */
   bars = pci_select_bars(pdev, IORESOURCE_MEM);

   if (pci_request_selected_regions(pdev, bars, DRV_NAME)) {
	LOG_ERROR("Can't select regions...\n");
	return -EINVAL;
   } else {
	LOG_DEBUG("Select regions success!!\n");
   }

   LOG_DEBUG("PCI bars return code = %d\n", bars);
   LOG_DEBUG("PCI Probe Success!. Return Code = %d\n", retCode);
  /**
   *  Try Allocating the device memory in the host and check
   *  for success.
   */

   nvme_dev_list = kzalloc((int)sizeof(struct nvme_device_entry), GFP_KERNEL);

   if (nvme_dev_list == NULL) {
	LOG_ERROR("allocate Host Memory for Device Failed!!...nvme_dev_list\n");
	return -ENOMEM;
   }

   bar = ioremap(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0));

   if (bar != NULL) {
	LOG_DEBUG("Bar 0 Address remap: 0x%08x\n", *bar);
	LOG_NORMAL("Bar 0 Address remap done.\n");
   } else {
	LOG_ERROR("allocate Host Memory for Device Failed!!...\n");
   }

#ifdef DEBUG
   /*Only debug because the above remap should give BAR's */
   pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &BaseAddress0);
   LOG_DEBUG("PCI BAR 0 = 0x%08x\n", BaseAddress0);
#endif

   /*! Call function to get the NVME Allocated to IOCTLs.
   * Using the generic nvme fops we will allocate the required
   * ioctl entry function.
   */
   dnvme_blk_gendisk(pdev, 0);

   nvme_dev_list->pdev = pdev;
   nvme_dev_list->bar = BaseAddress0;
   nvme_dev_list->bus = pdev->bus->number;
   nvme_dev_list->slot = PCI_SLOT(pdev->devfn);
   nvme_dev_list->func = PCI_FUNC(pdev->devfn);
   list_add_tail(&nvme_dev_list->list, &nvme_devices_llist);
   return retCode;
}

/**
*  dnvme_blk_gendisk  -Creates Block Disk to add to the the kernel
*  @param *pdev
*  @param which
*  @return returns 0 if block disk adding to kernel success.
*  @author T.Sravan Kumar
*/
static int dnvme_blk_gendisk(struct pci_dev *pdev, int which)
{
   struct gendisk *disk;
   int size = 14096;

    disk = alloc_disk(NVME_MINORS);
    if (!disk) {
	LOG_ERROR("Disk Allocation %d Failed...\n", disk->major);
	return -EINVAL;
    } else {
	LOG_DEBUG("Major Allocation %d\n", disk->major);
	LOG_DEBUG("Minor Allocation %d.\n", disk->minors);
	LOG_DEBUG("First Minor Allocation %d\n", disk->first_minor);
    }

   disk->major = NVME_MAJOR;
   disk->first_minor = which * NVME_MINORS;
   disk->fops = &dnvme_fops;

   snprintf(disk->disk_name, 32, "blkqnvme%c", which+'a');
   LOG_DEBUG("Disk Name = %s\n", disk->disk_name);
   if (disk->disk_name == NULL) {
	LOG_ERROR("Disk name is empty\n");
	return -EINVAL;
   }

   set_capacity(disk, size);

   /* Driver Fails when tried add disk to kernel, for now using as char dev */
   /* add_disk(disk); */

   return 0;
}

/**
*  dnvme_ioctl  - Call correcponding ioctl functions from Blk driver.
*  @param *bdev
*  @param  mode
*  @param cmd
*  @param arg
*  @return if ioctl call succesful or not.
*  @author T.Sravan Kumar
*/
static int dnvme_ioctl(struct block_device *bdev, fmode_t mode,
			unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
    case NVME_IOCTL_IDENTIFY_NS:
	LOG_DEBUG("IOCTL Identify NS Command...");
	break;
    case NVME_IOCTL_IDENTIFY_CTRL:
	LOG_DEBUG("IOCTL Identify CTRL Command...");
	break;
    case NVME_IOCTL_GET_RANGE_TYPE:
	LOG_DEBUG("IOCTL  NVME_IOCTL_GET_RANGE_TYPE Get Range Command...");
	break;
    case NVME_IOCTL_SUBMIT_IO:
	LOG_DEBUG("IOCTL NVME_IOCTL_SUBMIT_IO Command...");
	break;
    case NVME_IOCTL_DOWNLOAD_FW:
	LOG_DEBUG("IOCTL CNVME_IOCTL_DOWNLOAD_FW Command...");
	break;
    case NVME_IOCTL_ACTIVATE_FW:
	LOG_DEBUG("IOCTL INVME_IOCTL_ACTIVATE_FW Command...");
	break;
    default:
	return -ENOTTY;
     }
return 0;
}

/**
 * This function is called whenever a process tries to do an ioctl on our
 * device file. We get two extra parameters (additional to the inode and file
 * structures, which all device functions get): the number of the ioctl called
 * and the parameter given to the ioctl function.
 *
 * If the ioctl is write or read/write (meaning output is returned to the
 * calling process), the ioctl call returns the output of this function.
 *
 *  @param *inode
 *  @param *file
 *  @param ioctl_num
 *  @param ioctl_param
 *  @author T.Sravan Kumar
 *  @return if ioctl call succesful or not.
 */
int dnvme_ioctl_device(
		struct inode *inode,	/* see include/linux/fs.h */
		struct file *file,	/* ditto */
		unsigned int ioctl_num,	/* number and param for ioctl */
		unsigned long ioctl_param)
{
    int retVal = -EINVAL;
    int len;
    struct nvme_read_generic *nvme_data;
    struct nvme_device_entry *nvme_dev_entry;
    struct pci_dev *pdev;

   len = 80;
   switch (ioctl_num) {
   case NVME_IOCTL_READ_GENERIC:

	LOG_DEBUG("IOCTL called from user land..\n\
		Invoking generic read function...\n");
	nvme_data = (struct nvme_read_generic *)ioctl_param;
	LOG_DEBUG("Invoking User App request to read  the PCI Header Space\n");

	/* Get the device from the linked list */
	list_for_each_entry(nvme_dev_entry, &nvme_devices_llist, list) {
		pdev = nvme_dev_entry->pdev;
		LOG_DEBUG("[Nvme_Drv] device [%02x:%02x.%02x]",
		nvme_dev_entry->bus,
		nvme_dev_entry->slot, nvme_dev_entry->func);
		LOG_DEBUG("[Nvme_Drv] Bar 0 [0x%x]", nvme_dev_entry->bar);
		}
	retVal = driver_generic_read(file, ioctl_param, nvme_data, pdev);
	break;

   case NVME_IOCTL_CREATE_ADMN_Q:
	LOG_DEBUG("IOCTL for Create Admin Q\n");
	break;
   case NVME_IOCTL_DEL_ADMN_Q:
	LOG_DEBUG("IOCTL NVME_IOCTL_DEL_ADMN_Q Command...");
	break;
   case NVME_IOCTL_SEND_ADMN_CMD:
	LOG_DEBUG("IOCTL NVME_IOCTL_SEND_ADMN_CMD Command...");
	break;
   default:
	LOG_DEBUG("Cannot find IOCTL going to default case...\n");
	retVal = driver_default_ioctl(file, ioctl_param, len);
	break;
   }

return 0;
}

/**
*  Module Exit code.
*  dnvme_exit -sTODO  Perform clean exit
*  @author T.Sravan Kumar
*/
static void __exit dnvme_exit(void)
{
    unregister_chrdev(NVME_MAJOR, NVME_DEVICE_NAME);
    pci_unregister_driver(&dnvme_pci_driver);

    LOG_DEBUG("dnvme exited...Bye!\n");
}

/*
*  Driver Module Calls.
*/
MODULE_DESCRIPTION("Low Level Device Driver for NMVE 1.0a Spec PCI Device");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:"DRV_NAME);

module_init(dnvme_init);
module_exit(dnvme_exit);
