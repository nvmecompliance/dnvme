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
#include "dnvme_queue.h"

/*
*  device_status_chk  - Generic error checking function
*  which checks error registers and set kernel
*  alert if a error is detected.
*/
int device_status_chk(struct pci_dev *pdev,
			int *status)
{
   /* Local variable declaration. */
   u16 data; /* Unsigned 16 bit data. */
   int ret_code = -EINVAL; /* initialize ret code to invalid */
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
   * Check the return code to know if pci read is success.
   */
   if (ret_code < 0) {
	LOG_ERR("pci_read_config failed in driver error check");
	return -EINVAL;
   }

   /*
   * Get the Device Status from the PCI Header.
   */
   *status = device_status_pci(data);

   /* Print out to kernel log the device status */
   if (*status == SUCCESS)
	LOG_NRM("PCI Device Status SUCCESS (STS)");
   else
	LOG_ERR("PCI Device Status FAIL (STS)");

   *status = (*status == SUCCESS) ? device_status_next(pdev) : FAIL;

   /* Print out to kernel log the device status */
   if (*status == SUCCESS)
	LOG_NRM("NEXT Capability Status SUCCESS.");
   else
	LOG_ERR("NEXT Capabilty status FAIL");

   *status = (*status == SUCCESS) ? nvme_controller_status(pdev) : FAIL;
   /* Print out to kernel log the device status */
   if (*status == SUCCESS)
	LOG_NRM("NVME Controller Status SUCCESS (CSTS)");
   else
	LOG_ERR("iNVME Controller Status FAIL (CSTS)");
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
*   NVME PCIe registers and memory mapped address
*/
int driver_generic_read(struct file *file,
			struct rw_generic *nvme_data,
			struct pci_dev *pdev)
{
   /* Local variable declaration. */
   u16 index; /* index for looping till the end. */
   int ret_code = -EINVAL; /* to verify if return code is success. */
   struct nvme_space nvme_ctrl_reg_space;
   struct nvme_dev_entry *nvme = NULL;
   unsigned char __user *datap = (unsigned char __user *)nvme_data->buffer;

   LOG_DBG("Inside Generic Read Function of the IOCTLs");

   nvme = kzalloc(sizeof(struct nvme_dev_entry), GFP_KERNEL);
   if (nvme == NULL) {
	LOG_ERR("Unable to allocate kernel mem in generic read");
	return -ENOMEM;
   }

   /*
   *  Switch based on the user passed read type using the
   *  enum type specified in struct rw_generic.
   */
   switch (nvme_data->type) {
   case NVMEIO_PCI_HDR: /* Switch case for NVME PCI Header type. */

	LOG_DBG("User App request to read  the PCI Header Space");
	LOG_DBG("Read request for bytes = 0x%x", nvme_data->nBytes);
	LOG_DBG("off:acc= 0x%x:0x%2x", nvme_data->offset, nvme_data->acc_type);
	/*
	* Loop through the number of bytes that are specified in the
	* bBytes parameter.
	*/
	for (index = 0; index < nvme_data->nBytes;) {
		/* Check where you are reading */
		LOG_DBG("Reading for index = 0x%x", index);
		LOG_DBG("PCI Offset = 0x%x", nvme_data->offset + index);

		if ((nvme_data->offset + index) > MAX_PCI_EXPRESS_CFG) {
			LOG_ERR("Offset is more than the PCI Express");
			LOG_ERR("Extended config space...");
			return -EINVAL;
		}
		/*
		* Check the access width and access the PCI space as per
		* the requested width.
		*/
		if ((nvme_data->acc_type == DWORD_LEN)
			&& (nvme_data->nBytes % 4 == 0)) {
			/* Read dword from the PCI register space. */
			ret_code = pci_read_config_dword(pdev,
			(nvme_data->offset + index), (u32 *)&datap[index]);
			LOG_NRM("Reading PCI offset, data = 0x%x, 0x%x",
				(nvme_data->offset + index), (u32)datap[index]);

			/* increment by dword size */
			index += 4;
		} else if ((nvme_data->acc_type == WORD_LEN)
			&& (nvme_data->nBytes % 2 == 0)) {
			/* Read a word from the PCI register space. */
			ret_code = pci_read_config_word(pdev,
			(nvme_data->offset + index), (u16 *)&datap[index]);
			LOG_NRM("Reading PCI offset, data = 0x%x, 0x%x",
				(nvme_data->offset + index), (u16)datap[index]);

			/* increment by word size */
			index += 2;
		} else if (nvme_data->acc_type == BYTE_LEN) {
			/* Read a byte from the PCI register space. */
			ret_code = pci_read_config_byte(pdev,
			(nvme_data->offset + index), (u8 *) &datap[index]);
			LOG_NRM("Reading PCI offset, data = 0x%x, 0x%x",
				(nvme_data->offset + index), (u8)datap[index]);

			/* increment by byte size */
			index++;
		} else {
			LOG_ERR("PCI space accessed by DWORD, WORD or BYTE");
			LOG_ERR("Wrong PCI access width specified or");
			LOG_ERR("Wrong no. of bytes specified.");
			return -EINVAL;
		}

		/* Check if reading is successful */
		if (ret_code < 0) {
			LOG_ERR("pci_read_config failed");
			return ret_code;
		}
	}
	/* done required reading then break and return.	*/
	break;

   case NVMEIO_BAR01:
	/* Registers are aligned and so */
	LOG_DBG("Invoking User App request to read from NVME space");

	/*
	* Checking for 4 bytes boundary. If either nBytes or offset is not
	* 4 bytes aligned return error.
	*/
	if ((nvme_data->acc_type == DWORD_LEN) &&
		(((nvme_data->nBytes % 4) != 0) ||
		((nvme_data->offset % 4) != 0))
		) {
			LOG_ERR("Offset or nBytes is not DWORD Aligned");
			LOG_ERR("Provide them on 4 bytes Boundaray");
			return -EINVAL;
		} else if ((nvme_data->acc_type == QUAD_LEN) &&
			(((nvme_data->nBytes % 8) != 0) ||
			((nvme_data->offset % 4) != 0))
			) {
			/*
			* Only nbytes need to be QUAD aligned and offset may be
			* DWORD or QUAD aligned.
			*/
			LOG_ERR("Offset is not DWORD Aligned");
			LOG_ERR("nbytes is not QUAD Aligned");
			LOG_ERR("Provide them on 8 bytes Boundaray");
			return -EINVAL;
		} else if ((nvme_data->acc_type == WORD_LEN) &&
			((nvme_data->nBytes % 2) != 0)) {
			LOG_ERR("nBytes is not WORD aligned");
			return -EINVAL;
		}

	/* Remap io mem for this device. */
	nvme->bar0mapped = ioremap(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));

	/* Check if remap was success */
	if (!nvme->bar0mapped) {
		LOG_ERR("Unable to map io region nvme..exit");
		return -EINVAL;
	}

	LOG_NRM("Using Bar0 address: 0x%llx, length 0x%x",
		(uint64_t)nvme->bar0mapped, (int) pci_resource_len(pdev, 0));

	/* Assign the BAR remapped into nvme space control register */
	nvme_ctrl_reg_space.bar_dev = (void __iomem *)nvme->bar0mapped;

	/* Read NVME register space. */
	ret_code = read_nvme_reg_generic(
			nvme_ctrl_reg_space,
			datap,
			nvme_data->nBytes,
			nvme_data->offset,
			nvme_data->acc_type
			);

	if (ret_code < 0) {
		LOG_ERR("Read NVME Space failed");
		return -EINVAL;
	}
	/* done with nvme space reading break from this case .*/
	break;

   default:
	LOG_DBG("Could not find switch case using default");
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

   struct nvme_space l_ctrl_reg_space;
   struct nvme_dev_entry *nvme_dev = NULL;
   /*
   * Pointer for user data to be copied to user space from
   * kernel space. Initialize with user passed data pointer.
   */
   unsigned char __user *datap = (unsigned char __user *)nvme_data->buffer;

   LOG_DBG("Inside Generic write Funtion of the IOCTLs");

   nvme_dev = kzalloc(sizeof(struct nvme_dev_entry), GFP_KERNEL);
   if (nvme_dev == NULL) {
	LOG_ERR("Unable to allocate kernel mem in generic read");
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
		* indicated by (offset + index) as per access width.
		*/
		if ((nvme_data->acc_type == DWORD_LEN)
			&& (nvme_data->nBytes % 4 == 0)) {
			/* Write a word to PCI register space. */
			ret_code = pci_write_config_dword(pdev,
				(nvme_data->offset + index), datap[index]);
			LOG_NRM("Writing to PCI offset, data = 0x%x, 0x%x",
				(nvme_data->offset + index), (u32)datap[index]);
			/* increment by dword size */
			index += 4;
		} else if ((nvme_data->acc_type == WORD_LEN)
			&& (nvme_data->nBytes % 2 == 0)) {
			/* Write a word to PCI register space. */
			ret_code = pci_write_config_word(pdev,
				(nvme_data->offset + index), datap[index]);
			LOG_NRM("Writing to PCI offset, data = 0x%x, 0x%x",
				(nvme_data->offset + index), (u16)datap[index]);
			/* increment by word size */
			index += 2;
		} else if (nvme_data->acc_type == BYTE_LEN) {
			/* Write a byte from to PCI register space. */
			ret_code = pci_write_config_byte(pdev,
				(nvme_data->offset + index), datap[index]);

			LOG_NRM("Writing to PCI offset, data = 0x%x, 0x%x",
				(nvme_data->offset + index), (u8)datap[index]);
			/* increment by byte size */
			index++;
		} else {
			LOG_ERR("PCI space acccessed by DWORD, WORD or BYTE");
			LOG_ERR("Wrong PCI acccess width specified or ");
			LOG_ERR("Wrong no of bytes specified");
			return -EINVAL;
		}
		/* Check if reading is successful */
		if (ret_code < 0) {
			LOG_ERR("pci_read_config failed");
			LOG_ERR("Unable to write to location = 0x%x data = 0%x",
				(nvme_data->offset + index), datap[index]);
			return ret_code;
		}
	}
	/* Done writing user requested data, returning. */
	break;
   case NVMEIO_BAR01:
	LOG_DBG("Invoking User App request to write NVME Space using BAR01");

	/*
	* Checking for 4 bytes boundary. If either nBytes or offset is not
	* 4 bytes aligned return error.
	*/
	if ((nvme_data->acc_type == DWORD_LEN) &&
		(((nvme_data->nBytes % 4) != 0) ||
		((nvme_data->offset % 4) != 0))
		) {
			LOG_ERR("Either Offset or nBytes is not DWORD Aligned");
			LOG_ERR("Provide them on 4 bytes Boundaray");
			return -EINVAL;
		} else if ((nvme_data->acc_type == QUAD_LEN) &&
			(((nvme_data->nBytes % 8) != 0) ||
			((nvme_data->offset % 4) != 0))
			) {
			LOG_ERR("Either Offset or nBytes is not QUAD Aligned");
			LOG_ERR("Provide them on 8 bytes Boundaray");
			return -EINVAL;
		} else if ((nvme_data->acc_type == WORD_LEN) &&
			((nvme_data->nBytes % 2) != 0)) {
			LOG_ERR("nBytes is not WORD aligned");
			return -EINVAL;
		}

	/* Remap io mem for this device. */
	nvme_dev->bar0mapped = ioremap(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));

	/* Check if remap was success */
	if (!nvme_dev->bar0mapped) {
		LOG_ERR("Unable to map io region nvme..exit");
		return -EINVAL;
	}

	LOG_NRM("Using Bar0 address: 0x%llx, length 0x%x",
	(uint64_t)nvme_dev->bar0mapped, (int) pci_resource_len(pdev, 0));

	/* Assign the BAR remapped into nvme space control register */
	l_ctrl_reg_space.bar_dev = (void __iomem *)nvme_dev->bar0mapped;

	/*
	* Write NVME register space with datap from offset until
	* nBytes are written.
	*/
	ret_code = write_nvme_reg_generic(
			l_ctrl_reg_space,
			(u8 *)datap,
			nvme_data->nBytes,
			nvme_data->offset,
			nvme_data->acc_type
			);

	/* done with nvme space writing break from this case .*/
	break;

   default:
	LOG_DBG("Could not find switch case using default");
   }

   return ret_code;
}

/*
*   driver_create_asq - Driver Admin Submission Queue creation routine
*   from ASQ create ioctl.
*/
int driver_create_asq(
		struct nvme_asq_gen *nvme_asq_cr,
		struct nvme_dev_entry *nvme_dev
		)
{
   int ret_code = -EINVAL; /* ret code to verify if ASQ creation succeeded */

#ifdef DEBUG
   /* Check if the nvme_dev init flag is set */
   if (!nvme_dev->init_flag) {
	LOG_ERR("The device send for ASQ creation is not initialized");
	return ret_code;
   }
#endif

   /* Call routine to create admin Submission queue */
   ret_code = create_admn_sq(nvme_dev, nvme_asq_cr->asq_size);

   return ret_code;
}

/*
*   driver_create_acq - Driver Admin Completion Queue creation routine
*   from ACQ create ioctl.
*/
int driver_create_acq(
		struct nvme_acq_gen *nvme_acq_cr,
		struct nvme_dev_entry *nvme_dev
		)
{
   int ret_code = -EINVAL; /* ret code to verify if ACQ creation succeeded */

#ifdef DEBUG
   /* Check if the nvme_dev init flag is set */
   if (!nvme_dev->init_flag) {
	LOG_ERR("The device send for ACQ creation is not initialized");
	return ret_code;
   }
#endif

   /* As we are doing polling based so irq feild is not used for now */
   /* Call routine to create admin Submission queue */
   ret_code = create_admn_cq(nvme_dev, nvme_acq_cr->acq_size);

   return ret_code;
}

/*
*   driver_iotcl_init - Driver Initialization routine before starting to
*   issue  ioctls.
*/
int driver_ioctl_init(
		struct nvme_dev_entry *nvme_dev,
		struct pci_dev *pdev
		)
{
   int ret_code = -EINVAL; /* ret code to verify if ASQ creation succeeded */

   LOG_DBG("Inside driver IOCTL init function");
   LOG_NRM("Initializing the BAR01 and NVME Controller Space");

   /* Remap io mem for this device. */
   nvme_dev->bar0mapped = ioremap(pci_resource_start(pdev, 0),
					pci_resource_len(pdev, 0));
   /* Check if remap was success */
   if (!nvme_dev->bar0mapped) {
	LOG_ERR("IOCTL init failed");
	return -EINVAL;
   }

   /* Assign the 64bit address mapped to controller space in nvme_dev */
   nvme_dev->nvme_ctrl_space = (void __iomem *)nvme_dev->bar0mapped;

   /* Set the init flag to initialization done flag. */
   nvme_dev->init_flag = NVME_DEV_INIT;

   /* Set the pci device of the nvme_dev to point to pdev from ioctl */
   nvme_dev->pdev = pdev;

   LOG_NRM("IOCTL Init Success:Reg Space Location:  0x%llx",
	(uint64_t)nvme_dev->nvme_ctrl_space);

   ret_code = SUCCESS;

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
