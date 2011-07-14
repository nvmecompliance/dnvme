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

/**
*   driver_genric_read - Generic Read functionality for reading
*   NVME PCIe registers and memory mapped addres
*   @param file
*   @param buffer
*   @param nvme_data
*   @param pdev
*   @return if successfully read returns 0 else returns -ERR.
*/
int driver_generic_read(struct file *file,
			unsigned long buffer,
			struct nvme_read_generic *nvme_data,
			struct pci_dev *pdev)
{
unsigned long __user *datap = (unsigned long __user *)buffer;
u8 offset;
u8 data;
u8 index;

    LOG_DEBUG("Inside Generic Read Funtion fo the IOCTLs\n");

   switch (nvme_data->type) {
   case NVME_PCI_HEADER:

	LOG_DEBUG("Invoking User App request to read  the PCI Header Space\n");

	/*
	* Check here if any invalid data is passed and retrun from here.
	*/
	if ((nvme_data->offset < 0) || (nvme_data->nBytes < 0)) {
		LOG_ERROR("invalid parameters are to \
			IOCTL generic function...\n");
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
		* Read a byte from the configuration register
		* and pass it to user.
		*/
		pci_read_config_byte(pdev, offset + index, &data);

		LOG_DEBUG("Reading PCI header from offset = %d, data = %x\n",
					(offset + index), data);
	}
	break;

   case NVME_PCI_BAR0:
	LOG_DEBUG("Invoking User App request to read  the PCI Header Space\n");
	break;

   default:
	LOG_DEBUG("Could not find switch case using defuult\n");
   }

return 0;
}


int driver_default_ioctl(struct file *file,
			unsigned long buffer,
			size_t length
			)
{
unsigned long __user *datap = (unsigned long __user *)buffer;
unsigned long tmp;
/*
char temp[100] = "Message from Kernel generic read..";
int len = 0;
int i = 0;
*/
    LOG_DEBUG("Inside Default IOCTL Funtion\n");
    tmp = 0xa5a5;
/*
    while(temp[i]) {
	put_user(temp[i], buffer++);
	len--;
	i++;
  }
*/
    put_user(tmp, datap);

return 0;
}

