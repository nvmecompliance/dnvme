#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/unistd.h>

#include "dnvme_ioctls.h"
#include "dnvme_interface.h"
#include "definitions.h"
#include "sysfuncproto.h"
#include "sysdnvme.h"

uint32_t read_reg(int index, u32 *bar)
{
   uint32_t u32reg;

   /* reg.u32[0] = readl((u32 *)(bar + offset));*/
   u32reg = ioread32((u32 *)(bar + index));

   LOG_DBG("Read Operation = 0x%x", u32reg);
   return u32reg;
}

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

   LOG_DBG("PCI Device Status read = %x", data);
   /*
   * Check the retrun code to know if pci read is succes.
   */
   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check");

   /*
   * Store the data into err_data. Making a data copy. As we will
   * be modifying the data bits.
   */
   err_data = data;

   LOG_DBG(KERN_CRIT "PCI Device Status crit = %x", data);
   LOG_DBG(KERN_EMERG "PCI Device Status Emerg = %x", data);

   if (data & DEV_ERR_MASK) {
	*status = FAIL;

	if (data & DPE) {
		LOG_ERR("Device Status - DPE Set");
		LOG_ERR("Detected Data parity Error");
	}
	if (data & SSE) {
		LOG_ERR("Device Status - SSE Set\n");
		LOG_ERR("Detected Signaled System Error");
	}
	if (data & DPD) {
		LOG_ERR("Device Status - DPD Set\n");
		LOG_ERR("Detected Master Data Parity Error");
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
   u64 u64_bar01 = 0;
   u32 bar0 = 0;
   u32 bar1 = 0;
   u32 u32data = 0;
   /*
   * Pointer for user data to be copied to user space from
   * kernel space. Initialize with user passed data pointer.
   */
   unsigned char __user *datap = (unsigned char __user *)nvme_data->rdBuffer;

   LOG_DBG("Inside Generic Read Funtion of the IOCTLs");

   /*
   * Check here if any invalid data is passed and return from here.
   * if not valid.
   */
   if ((nvme_data->offset < 0) || (nvme_data->nBytes < 0)) {
	LOG_ERR("invalid params to IOCTL generic function");
	return -EINVAL;
   }

   /*
   * Copy offset to local variable to increase speed.
   */
   offset = nvme_data->offset;

   /*
   *  Switch based on the user passed read type using the
   *  enum type specified in struct nvme_read_generic.
   */
   switch (nvme_data->type) {
   case NVME_PCI_HEADER: /* Switch case for NVME PCI Header type. */

	LOG_DBG("User App request to read  the PCI Header Space");

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
			LOG_ERR("pci_read_config failed");
			return ret_code;
		}

		LOG_DBG("Reading PCI header from offset = %d, data = 0x%x",
					(offset + index), data);

		/*
		* copy each data read from pci space to user pointer.
		* Index points to the next data location.
		*/
		datap[index] = data;

	}

	/*
	* done required reading then break and return.
	*/
	break;

   case NVME_PCI_BAR01:
	/* Registers are aligned and so */
	LOG_DBG("Invoking User App request for BAR01");
	/*
	* Get the Bar01 value for the current device pdev.
	*/
	pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &bar0);

	LOG_DBG("BAR 01 = 0x%x", bar1);
	LOG_DBG("BAR 00 = 0x%x", bar0);

	/*
	* Upper 32 bit of the memory register base address.
	*/
	memcpy((u32 *)&u64_bar01, (u32 *)(void __iomem *)&bar1, sizeof(u32));

	/*
	* Lower 32 bit of the memory register base address.
	*/
	u64_bar01 = ((u64_bar01 << 32) | bar0);

	LOG_DBG("BAR 64 01 = 0x%x", (u32)u64_bar01);
	/*
	* Compute the required offset from the BAR01.
	*/
	u64_bar01 = u64_bar01 + offset;

	index = 0;
	/*
	* Loop through the number of bytes that are specified in the
	* bBytes parameter.
	*/
	for (index = 0; index < nvme_data->nBytes; index += 4) {
		/*
		* Read a 32 byte data from the configuration register.
		*/
		u32data = read_reg(index, (u32 *)&u64_bar01);

		memcpy((char *)&datap[index], (char *)&u32data, sizeof(u32));

		LOG_DBG("Reading NVME Space offset = 0x%x, data = 0x%x",
				offset + index, u32data);

	}

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
   ret_code = copy_to_user(&nvme_data->rdBuffer[0], datap,
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
			struct nvme_write_generic *nvme_data,
			struct pci_dev *pdev)
{
   u8 offset; /* offset where data to be written. */
   u8 data; /* data to be written. */
   u8 index = 0; /* Index to loop */
   int ret_code = -EINVAL; /* return code to verify if written success. */

   /*
   * Pointer for user data to be copied to user space from
   * kernel space. Initialize with user passed data pointer.
   */
   unsigned char __user *datap = (unsigned char __user *)nvme_data->wrBuffer;

   LOG_DBG("Inside Generic write Funtion of the IOCTLs");

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
   copy_from_user(datap, nvme_data->wrBuffer, nvme_data->nBytes * sizeof(u8));

   /*
   * Switch based on the type of requested write determined by nvme_data->data
   */
   switch (nvme_data->type) {
   case NVME_PCI_HEADER: /* Switch case for NVME PCI Header type. */

	LOG_DBG("Invoking User App request to write the PCI Header Space");

	/*
	* Check here if any invalid data is passed and retrun from here.
	*/
	if ((nvme_data->offset < 0) || (nvme_data->nBytes < 0)) {
		LOG_ERR("invalid params to IOCTL write function");
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
			LOG_ERR("Unable to write to location = %d data = %x",
				(offset + index), data);
			return ret_code;
		}

		LOG_DBG("Writing to PCI header offset,data = %d, %x\n",
					(offset + index), data);
	}
	/* Done writing user requested data, returning. */
	break;

   case NVME_PCI_BAR01:
	LOG_DBG("Invoking User App request to write PCI BAR01");
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

