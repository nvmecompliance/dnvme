#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/unistd.h>

#include "dnvme_ioctls.h"
#include "definitions.h"
#include "sysfuncproto.h"
#include "sysdnvme.h"

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
   u16 err_data;
   int ret_code = EINVAL;

   /*
   * Pointer for user data to be copied to user space from
   * kernel space. Initialize with user passed data pointer.
   */
   int __user *datap = (int __user *)status;

   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   *status = SUCCESS;

   /*
   * Read a word (16bit value) from the configuration register
   * and pass it to user.
   */
   ret_code = pci_read_config_word(pdev, PCI_DEVICE_STATUS, &data);

   LOG_DEBUG("PCI Device Status read = %x\n", data);
   /*
   * Check the retrun code to know if pci read is succes.
   */
   if (ret_code < 0)
	LOG_ERROR("pci_read_config failed in driver error check\n");

   /*
   * Store the data into err_data. Making a data copy. As we will
   * be modifying the data bits.
   */
   err_data = data;

   LOG_DEBUG(KERN_CRIT "PCI Device Status crit = %x\n", data);
   LOG_DEBUG(KERN_EMERG "PCI Device Status Emerg = %x\n", data);

   if (data & DEV_ERR_MASK) {
	*status = FAIL;

	if (data & DPE) {
		LOG_ERROR("Device Status - DPE Set\n");
		LOG_ERROR("Detected Data parity Error!!!\n");
	}
	if (data & SSE) {
		LOG_ERROR("Device Status - SSE Set\n");
		LOG_ERROR("Detected Signaled System Error!!!\n");
	}
	if (data & DPD) {
		LOG_ERROR("Device Status - DPD Set\n");
		LOG_ERROR("Detected Master Data Parity Error!!!\n");
	}
   }

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
			struct nvme_read_generic *nvme_data,
			struct pci_dev *pdev)
{
   /* Local variable declaration. */
   u8 offset; /* Offset to read data from. */
   u8 data; /*  data read from PCI space. */
   u8 index; /* index for looping till the end. */
   int ret_code = -EINVAL; /* to verify if return code is success. */

   /*
   * Pointer for user data to be copied to user space from
   * kernel space. Initialize with user passed data pointer.
   */
   unsigned char __user *datap = (unsigned char __user *)nvme_data->rdBuffer;

   LOG_DEBUG("Inside Generic Read Funtion of the IOCTLs\n");

   /*
   *  Switch based on the user passed read type using the
   *  enum type specified in struct nvme_read_generic.
   */
   switch (nvme_data->type) {
   case NVME_PCI_HEADER: /* Switch case for NVME PCI Header type. */

	LOG_DEBUG("User App request to read  the PCI Header Space\n");

	/*
	* Check here if any invalid data is passed and return from here.
	* if not valid.
	*/
	if ((nvme_data->offset < 0) || (nvme_data->nBytes < 0)) {
		LOG_ERROR("invalid params to IOCTL generic function...\n");
		return -EINVAL;
	}

	/*
	* Copy offset to local variable to increase speed.
	*/
	offset = nvme_data->offset;

	/*
	* Loop through the number of bytes that are specified in the
	* bBytes parameter.
	*/
	for (index = 0; index < nvme_data->nBytes; index++) {
		/*
		* Read a byte from the configuration register
		* and pass it to user.
		*/
		ret_code = pci_read_config_byte(pdev, offset + index, &data);

		if (ret_code < 0) {
			LOG_ERROR("pci_read_config failed\n");
			return ret_code;
		}

		LOG_DEBUG("Reading PCI header from offset = %d, data = %x\n",
					(offset + index), data);

		/*
		* copy each data read from pci space to user pointer.
		* Index points to the next data location.
		*/
		datap[index] = data;

	}

	/*
	*  Efficient way to copying data to user buffer datap
	*  using in a single copy function call.
	*  First parameter is copy to user buffer,
	*  second parameter is copy from location,
	*  third parameter give the number of bytes to copy.
	*/
	ret_code = copy_to_user(&nvme_data->rdBuffer[0], datap,
					nvme_data->nBytes * sizeof(u8));

	/*
	* Check to see if copy to user buffer is successful,
	* else send error message and continue.
	*/
	if (ret_code < 0) {
		LOG_ERROR("Copy to user failed at index\n");
		return ret_code;
	}

	/*
	* done required reading then break and return.
	*/
	break;

   case NVME_PCI_BAR01:
	LOG_DEBUG("Invoking User App request to read  the PCI Header Space\n");
	break;

   default:
	LOG_DEBUG("Could not find switch case using defuult\n");
   }

   return ret_code;
}

/*
*   driver_generic_write - Generic write function for
*   NVME PCIe registers and memory mapped address
*/
int driver_generic_write(struct file *file,
			struct nvme_write_generic *nvme_data,
			struct pci_dev *pdev)
{
   u8 offset; /* offset where data to be written. */
   u8 data; /* data to be written. */
   u8 index; /* Index to loop */
   int ret_code = -EINVAL; /* return code to verify if written success. */

   /*
   * Pointer for user data to be copied to user space from
   * kernel space. Initialize with user passed data pointer.
   */
   unsigned char __user *datap = (unsigned char __user *)nvme_data->wrBuffer;

   LOG_DEBUG("Inside Generic write Funtion of the IOCTLs\n");

   /* allocate kernel memory to datap that is requested from user app */
   datap = kzalloc(sizeof(u8) * nvme_data->nBytes, GFP_KERNEL);

   /*
   * Check if allocation of memory is not null else return
   * no memory.
   */
   if (!datap) {
	LOG_ERROR("Unable to allocate kernel memory in driver generic write\n");
	return -ENOMEM;
   }

   /*
   * copy from user data buffer to kernel data buffer at single place
   * using copy_from_user for efficiency.
   */
   copy_from_user(datap, nvme_data->wrBuffer, nvme_data->nBytes * sizeof(u8));

   /*
   * Switch based on the type of requested write determined by nvme_data->data
   */
   switch (nvme_data->type) {
   case NVME_PCI_HEADER: /* Switch case for NVME PCI Header type. */

	LOG_DEBUG("Invoking User App request to write the PCI Header Space\n");

	/*
	* Check here if any invalid data is passed and retrun from here.
	*/
	if ((nvme_data->offset < 0) || (nvme_data->nBytes < 0)) {
		LOG_ERROR("invalid params to IOCTL write function...\n");
		return -EINVAL;
	}

	/*
	* Copy offset to local variable.
	*/
	offset = nvme_data->offset;

	/*
	* Loop through the number of bytes that are specified in the
	* bBytes parameter.
	*/
	for (index = 0; index < nvme_data->nBytes; index++) {
		/*
		* Read a byte from the user buffer to local variable.
		*/
		data = datap[index];

		/*
		* write user data to pci config space at location
		* indicated by (offset + index).
		*/
		ret_code = pci_write_config_byte(pdev, offset + index, data);

		if (ret_code < 0) {
			LOG_ERROR("Unable to write to location = %d data = %x",
				(offset + index), data);
			return ret_code;
		}

		LOG_DEBUG("Writing to PCI header offset,data = %d, %x\n",
					(offset + index), data);
	}
	/* Done writing user requested data, returning. */
	break;

   case NVME_PCI_BAR01:
	LOG_DEBUG("Invoking User App request to write PCI BAR01\n");
	break;

   default:
	LOG_DEBUG("Could not find switch case using defuult\n");
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
    LOG_DEBUG("Inside Default IOCTL Funtion\n");
    tmp = 0xa5a5;
    put_user(tmp, datap);

    return 0;
}

