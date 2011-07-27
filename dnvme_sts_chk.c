#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_sts_chk.h"
#include "definitions.h"
#include "sysdnvme.h"

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
	if (device_data & SSE) {
		LOG_ERR("Device Status - SSE Set\n");
		LOG_ERR("Detected Signaled System Error\n");
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
	if (device_data & STA) {
		LOG_ERR("Device Status - STA Set\n");
		LOG_ERR("Signalled Target Abort...\n");
	}
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
			break;
		case PXCAP_ID:
			LOG_DBG("Entering into PCI Express Capabilities\n");
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
