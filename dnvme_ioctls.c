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

/*----------------------------------------------------------------------------*/
/**
*   driver_genric_read - Generic Read functionality for reading 
*   NVME PCIe registers and memory mapped addres
*   @param file
*   @param buffer
*   @param length
*   @return if successfully read returns 0 else returns -ERR.
*/
/*----------------------------------------------------------------------------*/
int driver_generic_read(struct file *file,
			unsigned long buffer,
			struct nvme_read_generic *nvme_data, 
			struct pci_dev *pdev)
{
unsigned long __user *datap = (unsigned long __user *)buffer;

    LOG_DEBUG("Inside Generic Read Funtion fo the IOCTLs\n");

   switch(nvme_data->type) {
   case NVME_PCI_HEADER:
	LOG_DEBUG("Invoking User App request to read  the PCI Header Space\n");
	/* Get the device from the linked list */
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
    put_user(tmp,datap);

return 0;
}

