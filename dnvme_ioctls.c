#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/delay.h>

#include "dnvme_ioctls.h"
#include "dnvme_interface.h"
#include "definitions.h"
#include "dnvme_reg.h"
#include "sysfuncproto.h"
#include "sysdnvme.h"
#include "dnvme_sts_chk.h"

/*
*  device_status_chk  - Generic error checking function
*  which checks error registers and set kernel
*  alert if a error is detected.
*/
int device_status_chk(struct pci_dev *pdev,
			int *status)
{
   /* Local variable declaration. */
   u16 data; /* unsinged 16 bit data. */
   int ret_code = EINVAL; /* initialize ret code to invalid */
   /*
   * Pointer for user data to be copied to user space from
   * kernel space. Initialize with user passed data pointer.
   */
   int __user *datap = (int __user *)status;

   /*
   * Read a word (16bit value) from the configuration register
   * and pass it to user.
   */
   ret_code = pci_read_config_word(pdev, PCI_DEVICE_STATUS, &data);
   /*
   * Check the return code to know if pci read is succes.
   */
   if (ret_code < 0) {
	LOG_ERR("pci_read_config failed in driver error check\n");
	return -EINVAL;
   }

   /*
   * Get the Device Status from the PCI Header.
   */
   *status = device_status_pci(data);

   /* Print out to kernel log the device status */
   if (*status == SUCCESS) {
	LOG_DBG("PCI Device Status SUCCESS = %d\n", *status);
	ret_code = 0;
   } else {
	LOG_ERR("PCI Device Status FAIL = %d\n", *status);
	ret_code = -EINVAL;
   }

   *status = device_status_next(pdev);
   /* Print out to kernel log the device status */
   if (*status == SUCCESS) {
	LOG_DBG("NEXT Capability Status SUCCESS = %d\n", *status);
	ret_code = 0;
   } else {
	LOG_ERR("NEXT Capabilty status FAIL!!!\n");
	LOG_ERR("Check individual error messages for cause = %d\n", *status);
	ret_code = -EINVAL;
   }

   *status = nvme_controller_status(pdev);
   /*
   *  Efficient way to copying data to user buffer datap
   *  using in a single copy function call.
   *  First parameter is copy to user buffer,
   *  second parameter is copy from location,
   *  third parameter give the number of bytes to copy.
   */
   ret_code = copy_to_user(status, datap, sizeof(status));

   return ret_code;
}

/*
*   driver_genric_read - Generic Read functionality for reading
*   NVME PCIe registers and memory mapped addres
*/
int driver_generic_read(struct file *file,
			struct rw_generic *nvme_data,
			struct pci_dev *pdev)
{
   /* Local variable declaration. */
   u8 data; /*  data read from PCI space. */
   u16 index; /* index for looping till the end. */
   int ret_code = -EINVAL; /* to verify if return code is success. */
   struct nvme_space nvme_ctrl_reg_space;
   struct nvme_dev_entry *nvme = NULL;
   unsigned char __user *datap = (unsigned char __user *)nvme_data->buffer;

   LOG_DBG("Inside Generic Read Funtion of the IOCTLs");

   nvme = kzalloc(sizeof(struct nvme_dev_entry), GFP_KERNEL);
   if (nvme == NULL) {
	LOG_ERR("Unable to allocate kernel mem in generic read\n");
	LOG_ERR("Exiting from here...");
	return -ENOMEM;
   }

   /*
   *  Switch based on the user passed read type using the
   *  enum type specified in struct rw_generic.
   */
   switch (nvme_data->type) {
   case NVMEIO_PCI_HDR: /* Switch case for NVME PCI Header type. */

	LOG_DBG("User App request to read  the PCI Header Space\n");
	LOG_DBG("Read request for bytes = %x\n", nvme_data->nBytes);
	/*
	* Loop through the number of bytes that are specified in the
	* bBytes parameter.
	*/
	for (index = 0; index < nvme_data->nBytes; index++) {
		/* Check where you are reading */
		LOG_DBG("Reading for index = %d\n", index);
		LOG_DBG("PCI Offset = %d\n", nvme_data->offset + index);

		if ((nvme_data->offset + index) > MAX_PCI_EXPRESS_CFG) {
			LOG_ERR("Offset is more than the PCI Express ");
			LOG_ERR("Extended config space...\n");
			return -EINVAL;
		}
		/*
		* Read a byte from the configuration register
		* and pass it to user.
		*/
		ret_code = pci_read_config_byte
			(pdev, (nvme_data->offset + index), &data);

		if (ret_code < 0) {
			LOG_ERR("pci_read_config failed\n");
			return ret_code;
		}

		LOG_DBG("Reading PCI header from offset = %d, data = 0x%x\n",
					(nvme_data->offset + index), data);

		/*
		* copy each data read from pci space to user pointer.
		* Index points to the next data location.
		*/
		datap[index] = data;

		LOG_DBG("Index = %d data 2 user = %d\n", index, datap[index]);
	}

	/*
	* done required reading then break and return.
	*/
	break;

   case NVMEIO_BAR01:
	/* Registers are aligned and so */
	LOG_DBG("Invoking User App request for to read from NVME space\n");

	/* Remap io mem for this device. */
	nvme->bar0mapped = ioremap(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));

	/* Check if remap was success */
	if (!nvme->bar0mapped) {
		LOG_ERR("Unable to map io region nmve..exit\n");
		return -EINVAL;
	}

	LOG_NRM("[Nvme_Drv]Using Bar0 address: %llx, length %d\n",
		(uint64_t)nvme->bar0mapped, (int) pci_resource_len(pdev, 0));

	/* Assign the BAR remapped into nvme space control register */
	nvme_ctrl_reg_space.bar_dev = (void __iomem *)nvme->bar0mapped;

	/* Read NVME register space. */
	read_nvme_reg_generic(
			nvme_ctrl_reg_space,
			datap,
			nvme_data->nBytes,
			nvme_data->offset
			);

	/* done with nvme space reading break from this case .*/
	break;

   default:
	LOG_DBG("Could not find switch case using defuult");
   }

   /*
   *  Efficient way to copying data to user buffer datap
   *  using in a single copy function call.
   *  First parameter is copy to user buffer,
   *  second parameter is copy from location,
   *  third parameter give the number of bytes to copy.
   */
   ret_code = copy_to_user(&nvme_data->buffer[0], datap,
				nvme_data->nBytes * sizeof(u8));
   if (ret_code < 0)
	LOG_ERR("Error copying to user buffer returning");

   return ret_code;
}

/*
*   driver_generic_write - Generic write function for
*   NVME PCIe registers and memory mapped address
*/
int driver_generic_write(struct file *file,
			struct rw_generic *nvme_data,
			struct pci_dev *pdev)
{
   u16 index = 0; /* Index to loop */
   int ret_code = -EINVAL; /* return code to verify if written success. */

   struct nvme_space nvme_ctrl_reg_space;
   struct nvme_dev_entry *nvme = NULL;
   /*
   * Pointer for user data to be copied to user space from
   * kernel space. Initialize with user passed data pointer.
   */
   unsigned char __user *datap = (unsigned char __user *)nvme_data->buffer;

   LOG_DBG("Inside Generic write Funtion of the IOCTLs");

   nvme = kzalloc(sizeof(struct nvme_dev_entry), GFP_KERNEL);
   if (nvme == NULL) {
	LOG_ERR("Unable to allocate kernel mem in generic read\n");
	LOG_ERR("Exiting from here...");
	return -ENOMEM;
   }
   /* allocate kernel memory to datap that is requested from user app */
   datap = kzalloc(sizeof(u8) * nvme_data->nBytes, GFP_KERNEL);

   /*
   * Check if allocation of memory is not null else return
   * no memory.
   */
   if (!datap) {
	LOG_ERR("Unable to allocate kernel memory in driver generic write");
	return -ENOMEM;
   }

   /*
   * copy from user data buffer to kernel data buffer at single place
   * using copy_from_user for efficiency.
   */
   copy_from_user((u8 *)datap, (u8 *)nvme_data->buffer,
				nvme_data->nBytes * sizeof(u8));

   /*
   * Switch based on the type of requested write determined by nvme_data->data
   */
   switch (nvme_data->type) {
   case NVMEIO_PCI_HDR: /* Switch case for NVME PCI Header type. */

	LOG_DBG("Invoking User App request to write the PCI Header Space");
	/*
	* Loop through the number of bytes that are specified in the
	* bBytes parameter.
	*/
	for (index = 0; index < nvme_data->nBytes; index++) {
		/*
		* write user data to pci config space at location
		* indicated by (offset + index).
		*/
		ret_code = pci_write_config_byte
			(pdev, (nvme_data->offset + index), datap[index]);

		if (ret_code < 0) {
			LOG_ERR("Unable to write to location = %d data = %x",
				(nvme_data->offset + index), datap[index]);
			return ret_code;
		}

		LOG_NRM("Writing to PCI header offset,data = %d, %x\n",
				(nvme_data->offset + index), datap[index]);
	}
	/* Done writing user requested data, returning. */
	break;
   case NVMEIO_BAR01:
	LOG_DBG("Invoking User App request to write NVME Space using BAR01");

	/* Remap io mem for this device. */
	nvme->bar0mapped = ioremap(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));

	/* Check if remap was success */
	if (!nvme->bar0mapped) {
		LOG_ERR("Unable to map io region nmve..exit\n");
		return -EINVAL;
	}

	LOG_NRM("[Nvme_Drv]Using Bar0 address: %llx, length %d\n",
		(uint64_t)nvme->bar0mapped, (int) pci_resource_len(pdev, 0));

	/* Assign the BAR remapped into nvme space control register */
	nvme_ctrl_reg_space.bar_dev = (void __iomem *)nvme->bar0mapped;

	/*
	* Write NVME register space with datap from offset until
	* nBytes are written.
	*/
	ret_code = write_nvme_reg_generic(
			nvme_ctrl_reg_space,
			(u8 *)datap,
			nvme_data->nBytes,
			nvme_data->offset
			);

	/* done with nvme space writing break from this case .*/
	break;

   default:
	LOG_DBG("Could not find switch case using default");
   }

   return ret_code;
}


/*
*   driver_default_ioctl - Default if none of the switch
*   in ioctl gets called.
*/
int driver_default_ioctl(struct file *file,
			unsigned long buffer,
			size_t length
			)
{
    unsigned long __user *datap = (unsigned long __user *)buffer;
    unsigned long tmp;
    LOG_DBG("Inside Default IOCTL Function");
    tmp = 0xa5a5;
    put_user(tmp, datap);

    return 0;
}

