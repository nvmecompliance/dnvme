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

   LOG_DBG("PCI Device Status (STS) Data = 0x%X", device_data);
   LOG_NRM("Checking all the PCI register error bits");
   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   if (device_data & DEV_ERR_MASK) {
	status = FAIL;

	if (device_data & DPE) {
		LOG_ERR("Device Status - DPE Set");
		LOG_ERR("Detected Data parity Error");
	}
	if (device_data & DPD) {
		LOG_ERR("Device Status - DPD Set");
		LOG_ERR("Detected Master Data Parity Error");
	}
	if (device_data & RMA) {
		LOG_ERR("Device Status - RMA Set");
		LOG_ERR("Received Master Abort...");
	}
	if (device_data & RTA) {
		LOG_ERR("Device Status - RTA Set");
		LOG_ERR("Received Target Abort...");
	}
   }

   if ((device_data & CL_MASK) != CL_MASK) {
	LOG_ERR("In STS, the CL bit indicates empty Capabilites list.");
	LOG_ERR("The controller should support PCI Power Mnmt as a min.");
	status = FAIL;
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

   LOG_NRM("Checking the NVME Controller Status (CSTS)...");

   nvme_ctrl_reg_space.bar_dev = ioremap(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0));

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->csts);
   tmp = u32data;

   LOG_NRM("NVME Controller Status CSTS = 0x%X", u32data);
   u32data &= NVME_CSTS_RSVD;

   status = SUCCESS;
   if (u32data != NVME_CSTS_RDY || u32data == NVME_CSTS_CFS) {
	if ((u32data & NVME_CSTS_RDY) == 0x0)
		LOG_NRM("NVME Controller is not ready (RDY)...");

	if ((u32data & NVME_CSTS_CFS) == NVME_CSTS_CFS) {
		status = FAIL;
		LOG_ERR("NVME Controller Fatal Status (CFS) is set...");
	}
   } else {
	LOG_NRM("NVME Controller Status (CSTS) Success");
   }

   u32data = tmp;
   u32data &= NVME_CSTS_SHST_MASK;

   /* Right shift by 2 bits. */
   u32data >>= 2;

   switch (u32data) {
	LOG_DBG("The Shutdown Status of the NVME Controller (SHST):");
   case NVME_CSTS_NRML_OPER:
	LOG_DBG("No Shutdown requested");
	break;
   case NVME_CSTS_SHT_OCC:
	LOG_DBG("Shutdown Processing occuring");
	break;
   case NVME_CSTS_SHT_COMP:
	LOG_DBG("Shutdown Process Complete");
	break;
   case NVME_CSTS_SHT_RSVD:
	LOG_DBG("Reserved Bits set");
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
   u16 pci_offset = 0;
   u32 cap_aer;
   u16 next_item = 1;
   u16 capability;
   u16 data; /* unsinged 16 bit data. */
   u8 power_management_feature = 0;

   LOG_NRM("Checking NEXT Capabilities of the NVME Controller");
   LOG_NRM("Checks if PMCS is supported as a minimum");

   /*
   * Check if CAP pointer points to next available
   * linked list resgisters in the PCI Header.
   */
   ret_code = pci_read_config_byte(pdev, CAP_REG, (u8 *)&pci_offset);

   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check");

   LOG_DBG("CAP_REG Contents = 0x%X", pci_offset);
   /*
   * Read 16 bits of data from the Next pointer as PMS bit
   * which is must
   */
   ret_code = pci_read_config_word(pdev, pci_offset, (u16 *)&cap_aer);
   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check");

   cap_aer = (u16)(cap_aer & LOWER_16BITS);
   /*
   * Enter into loop if cap_aer has non zero value.
   * next_item is set to 1 for entering this loop for first time.
   */
   while (cap_aer != 0 || next_item != 0) {

	LOG_DBG("CAP Value 16/32 Bits = 0x%X", cap_aer);
	/*
	* AER error mask is used for checking if it is not AER type.
	* Right Shift bby 16 bits.
	*/
	if (((cap_aer & AER_ERR_MASK) >> 16) != NVME_AER_CVER) {

		capability = (u16)(cap_aer & LOWER_16BITS);

		/* Mask out higher bits */
		next_item = (capability & NEXT_MASK) >> 8;

		/* Get next item offset */
		cap_aer = 0; /* Reset cap_aer */
		LOG_DBG("Capability Value = 0x%X", capability);

		/* Switch based on which ID Capabilty indicates */
		switch (capability & ~NEXT_MASK) {
		case PMCAP_ID:
			LOG_NRM("PCI Pwr Mgmt is Supported (PMCS Exists)");
			LOG_NRM("Checking PCI Pwr Mgmt Capabilities Status");

			/* Set power management is supported */
			power_management_feature = 1;

			if ((0x0 != pci_offset) && (MAX_PCI_HDR < pci_offset)) {
				/* Compute the PMCS offset from CAP data */
				pci_offset = pci_offset + PMCS;

				ret_code = pci_read_config_word
						(pdev, pci_offset, &data);
				if (ret_code < 0)
					LOG_ERR("pci_read_config failed");

				status = device_status_pmcs(data);
			} else {
				LOG_DBG("Invalid offset = 0x%x", pci_offset);
			}
			break;
		case MSICAP_ID:
			LOG_NRM("Checing MSI Capabilities");
			status = device_status_msicap(pdev, pci_offset);
			break;
		case MSIXCAP_ID:
			LOG_NRM("Checking MSI-X Capabilities");
			status = device_status_msixcap(pdev, pci_offset);
			break;
		case PXCAP_ID:
			LOG_NRM("Checking PCI Express Capabilities");
			status = device_status_pxcap(pdev, pci_offset);
			break;
		default:
			break;
		} /* end of switch case */

	} else {
		LOG_DBG("All Advaned Error Reporting..");

		/*
		* Advanced Error capabilty function.
		* Right shift by 20 bits to get the next item.
		*/
		next_item = (cap_aer >> 20) & MAX_PCI_EXPRESS_CFG;

		switch (cap_aer & AER_ID_MASK) {
		case AER_CAP_ID:
			LOG_NRM("Checking Advanced Error Reporting Capability");
			status = device_status_aercap(pdev, pci_offset);
			break;
		}

		/* Check if more items are there*/
		if (next_item == 0) {
			LOG_NRM("No NEXT item in the list Exiting..1");
			break;
		}
	} /* end of else if cap_aer */

	/* If item exists then read else break here */
	if (next_item != 0 && pci_offset != 0) {
		/* Get the next item in the linked lint */
		ret_code = pci_read_config_word(pdev, next_item, &pci_offset);
		/* Read 32 bits as next item could be AER cap */
		ret_code = pci_read_config_dword(pdev, pci_offset, &cap_aer);
	} else {
		LOG_NRM("No NEXT item in the list exiting...2");
		break;
	}
   } /* end of while loop */

   /* Check if PCI Power Management cap is supported as a min */
   if (power_management_feature == 0) {
	LOG_ERR("The controller should support PCI Pwr management as a min");
	LOG_ERR("PCI Power Management Capability is not Supported.");
	status = FAIL;
   }

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

   LOG_DBG("PCI Power Management Control and Status = %x", device_data);

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

   LOG_DBG("PCI MSI Cap= %x", device_data);

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

   LOG_DBG("PCI MSI-X Cap= %x", device_data);

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

   LOG_DBG("PXCap= %x", device_data);

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

   LOG_DBG("AER Cap= %x", device_data);

   return status;
}
