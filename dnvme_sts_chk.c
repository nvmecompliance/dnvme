#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_sts_chk.h"
#include "definitions.h"
#include "sysdnvme.h"
#include "dnvme_reg.h"
/*
*  device_status_pci  - PCI device status check function
*  which checks error registers and set kernel
*  alert if a error is detected.
*/
int device_status_pci(u16 device_data)
{
   int status;

   LOG_DBG("PCI Device Status Data = %x\n", device_data);

   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   if (device_data & DEV_ERR_MASK) {
	status = FAIL;

	if (device_data & DPE) {
		LOG_ERR("Device Status - DPE Set\n");
		LOG_ERR("Detected Data parity Error\n");
	}
	if (device_data & DPD) {
		LOG_ERR("Device Status - DPD Set\n");
		LOG_ERR("Detected Master Data Parity Error\n");
	}
	if (device_data & RMA) {
		LOG_ERR("Device Status - RMA Set\n");
		LOG_ERR("Received Master Abort...\n");
	}
	if (device_data & RTA) {
		LOG_ERR("Device Status - RTA Set\n");
		LOG_ERR("Received Target Abort...\n");
	}
   }
   return status;
}
/*
* nvme_controller_status - This function checks the controller status
* register CSTS at offset 0x1C from the BAR01 offset.
*/
int nvme_controller_status(struct pci_dev *pdev)
{
   int status;
   u32 u32data;
   u32 tmp;
   struct nvme_space nvme_ctrl_reg_space;

   nvme_ctrl_reg_space.bar_dev = ioremap(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0));

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->csts);
   tmp = u32data;

   LOG_NRM("NVME Controller Status = 0x%X", u32data);
   u32data &= 0x3;

   if (u32data != 0x1 || u32data == 0x2) {
	status = FAIL;
	LOG_NRM("NVME Controller Status Failed!!!\n");

	if ((u32data & 0x1) == 0x0)
		LOG_NRM("NVME Controller is not ready...\n");

	if ((u32data & 0x2) == 0x2)
		LOG_NRM("NVME Controller Fatal Status is set...\n");
   } else {
	LOG_NRM("NVME Controller Status Success\n");
	status = SUCCESS;
   }

   u32data = tmp;
   u32data &= 0xC;
   u32data >>= 2;

   switch (u32data) {
	LOG_NRM("The Shutdown Status of the NVME Controller:\n");
   case 0:
	LOG_NRM("No Shutdown requested\n");
	break;
   case 1:
	LOG_NRM("Shutdown Processing occuring\n");
	break;
   case 2:
	LOG_NRM("Shutdown Process Complete\n");
	break;
   case 3:
	LOG_NRM("Reserved Bits set\n");
	break;
   }

   return status;
}

/*
* device_status_next  - This function will check if the NVME device supports
* NEXT capability item in the linked list. If the device supports the NEXT
* capabilty then it goes into each of the status registers and checks the
* device current state. It reports back to the caller wither SUCCESS or FAIL.
* Print out to the kernel message details of the status.
*/
int device_status_next(struct pci_dev *pdev)
{
   int status;
   int ret_code;
   u16 pci_offset;
   u32 cap_aer;
   u16 next_item = 1;
   u16 capability;
   u16 data; /* unsinged 16 bit data. */


   /*
   * Check if CAP pointer points to next available
   * linked list resgisters in the PCI Header.
   */
   ret_code = pci_read_config_word(pdev, CAP_REG, &pci_offset);

   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check\n");

   /*
   * Read 32 bits of data from the Next pointer as AER is 32 bit
   * which is maximum.
   */
   ret_code = pci_read_config_dword(pdev, pci_offset, &cap_aer);
   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check\n");

   /*
   * Enter into loop if cap_aer has non zero value.
   * next_item is set to 1 for entering this loop for first time.
   */
   while (cap_aer != 0 || next_item != 0) {

	LOG_DBG("AER CAP Value = %x\n", cap_aer);
	/*
	* AER error mask is used for checking if it is not AER type.
	*/
	if (!(cap_aer & AER_ERR_MASK)) {
		capability = (u16)(cap_aer & LOWER_16BITS);
		/* Mask out higher bits */
		next_item = (capability & NEXT_MASK) >> 8;
		/* Get next item offset */
		cap_aer = 0; /* Reset cap_aer */
		LOG_DBG("Capability Value = %x\n", capability);

		/* Switch based on which ID Capabilty indicates */
		switch (capability & ~NEXT_MASK) {
		case PMCAP_ID:
			LOG_DBG("Entering PCI Pwr Mgmt Capabilities\n");

			if ((0x0 != pci_offset) && (MAX_PCI_HDR < pci_offset)) {
				/* Compute the PMCS offset from CAP data */
				pci_offset = pci_offset + PMCS;

				ret_code = pci_read_config_word
				(pdev, pci_offset, &data);
				if (ret_code < 0)
					LOG_ERR("pci_read_config failed\n");

				status = device_status_pmcs(data);
			} else {
				LOG_DBG("Invalid offset = %d\n", pci_offset);
			}
			break;
		case MSICAP_ID:
			LOG_DBG("Entering into MSI Capabilities\n");
			status = device_status_msicap(pdev, pci_offset);
			break;
		case MSIXCAP_ID:
			LOG_DBG("Entering into MSI-X Capabilities\n");
			status = device_status_msixcap(pdev, pci_offset);
			break;
		case PXCAP_ID:
			LOG_DBG("Entering into PCI Express Capabilities\n");
			status = device_status_pxcap(pdev, pci_offset);
			break;
		default:
			break;
		} /* end of switch case */

	} else {
		LOG_DBG("All Advaned Error Reporting..\n");
		/* Advanced Error capabilty function */
		next_item = (cap_aer >> 20) & MAX_PCI_EXPRESS_CFG;

		switch (cap_aer & AER_ID_MASK) {
		case AER_CAP_ID:
			LOG_DBG("Advanced Error Reporting Capability\n");
			status = device_status_aercap(pdev, pci_offset);
			break;
		}

		/* Check if more items are there*/
		if (next_item == 0) {
			LOG_DBG("No NEXT item in the list Exiting..1\n");
			break;
		}
	} /* end of else if cap_aer */

	/* If item exists then read else break here */
	if (next_item != 0 && pci_offset != 0) {
		ret_code = pci_read_config_word(pdev, next_item, &pci_offset);
		ret_code = pci_read_config_dword(pdev, pci_offset, &cap_aer);
	} else {
		LOG_DBG("There is no NEXT item in the list so exiting...2\n");
		break;
	}
   } /* end of while loop */

   return status;
}

/*
* device_status_pmcs: This function checks the pci power management
* control and status.
* PMCAP + 4h --> PMCS.
*/
int device_status_pmcs(u16 device_data)
{
   int status;
   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   LOG_DBG("PCI Power Management Control and Status = %x\n", device_data);

   return status;
}

/*
* device_status_msicap: This function checks the Message Signalled Interrupt
* control and status bits.
*/
int device_status_msicap(struct pci_dev *pdev, u16 device_data)
{
   int status;
   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   LOG_DBG("PCI MSI Cap= %x\n", device_data);

   return status;
}
/*
* device_status_msixcap: This func checks the Message Signalled Interrupt - X
* control and status bits.
*/
int device_status_msixcap(struct pci_dev *pdev, u16 device_data)
{
   int status;
   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   LOG_DBG("PCI MSI-X Cap= %x\n", device_data);

   return status;
}
/*
* device_status_pxcap: This func checks the PCI Express
* capabiltiy status register
*/
int device_status_pxcap(struct pci_dev *pdev, u16 device_data)
{
   int status;
   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   LOG_DBG("PXCap= %x\n", device_data);

   return status;
}
/*
* device_status_aerap: This func checks the status register of
* Advanced error reporting capabiltiy of PCI express device.
*/
int device_status_aercap(struct pci_dev *pdev, u16 device_data)
{
   int status;
   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   LOG_DBG("AER Cap= %x\n", device_data);

   return status;
}
