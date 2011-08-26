#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_sts_chk.h"
#include "dnvme_queue.h"
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
   int status     = 0;
   int ret_code   = 0;
   u16 pci_offset = 0;
   u32 cap_aer    = 0;
   u16 next_item  = 1;
   u16 capability = 0;
   u16 data       = 0; /* unsinged 16 bit data. */
   u8 power_management_feature = 0;

   LOG_NRM("Checking NEXT Capabilities of the NVME Controller");
   LOG_NRM("Checks if PMCS is supported as a minimum");

   /*
   * Set status success when you enter this function and
   * determine if it ever fails subsequently when it checking the
   * status bits.
   */
   status = SUCCESS;

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

				status = (status == SUCCESS) ?
						device_status_pmcs(data) : FAIL;
			} else {
				LOG_DBG("Invalid offset = 0x%x", pci_offset);
			}
			break;
		case MSICAP_ID:
			LOG_NRM("Checing MSI Capabilities");
			status = (status == SUCCESS) ?
				device_status_msicap(pdev, pci_offset) : FAIL;
			break;
		case MSIXCAP_ID:
			LOG_NRM("Checking MSI-X Capabilities");
			status = (status == SUCCESS) ?
				device_status_msixcap(pdev, pci_offset) : FAIL;
			break;
		case PXCAP_ID:
			LOG_NRM("Checking PCI Express Capabilities");
			status = (status == SUCCESS) ?
				device_status_pxcap(pdev, pci_offset) : FAIL;
			break;
		default:
			LOG_ERR("Next Device Status check in default case!!!");
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
		if (ret_code < 0)
			LOG_ERR("pci_read_config failed in driver error check");

		/* Read 32 bits as next item could be AER cap */
		ret_code = pci_read_config_dword(pdev, pci_offset, &cap_aer);
		if (ret_code < 0)
			LOG_ERR("pci_read_config failed in driver error check");
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
int device_status_pxcap(struct pci_dev *pdev, u16 base_offset)
{
   int status;
   u16 pxcap_sts_reg;
   int ret_code;
   u16 offset;

   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   LOG_DBG("Offset Value of PXCAP= %x", base_offset);

   /*
   * Print out on kern.log that driver is checking the PCI
   * express device status register.
   */
   LOG_NRM("Checking the PCI Express Device Status (PXDS)...");
   LOG_NRM("Offset PXCAP + Ah: PXDS");

   /* Compute the PXDS offset from the PXCAP */
   offset = base_offset + NVME_PXCAP_PXDS;

   LOG_DBG("PXDS Offset = 0x%X", offset);

   /* Read the device status register value into pxcap_sts_reg */
   ret_code = pci_read_config_word(pdev, offset, &pxcap_sts_reg);
   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check");

   LOG_DBG("PXDS Device status register data = 0x%X\n", pxcap_sts_reg);

   /* Mask off the reserved bits. */
   pxcap_sts_reg &= ~(NVME_PXDS_RSVD);

   /* Check in device status register if any error bit is set */
   /*
   * Each Bit position has different stauts indication as indicated
   * in NVME Spec 1.0b.
   */
   if (pxcap_sts_reg) {
	/* Check if Correctable error is detected */
	if (pxcap_sts_reg & NVME_PXDS_CED) {
		status = FAIL;
		LOG_ERR("Correctalbe Error Detected (CED) in PXDS");
	}
	/* Check if Non fatal error is detected */
	if ((pxcap_sts_reg & NVME_PXDS_NFED) >> 1) {
		status = FAIL;
		LOG_ERR("Non-Fatal Error Detected (NFED) in PXDS");
	}
	/* Check if fatal error is detected */
	if ((pxcap_sts_reg & NVME_PXDS_FED) >> 2) {
		status = FAIL;
		LOG_ERR("Fatal Error Detected (FED) in PXDS");
	}
	/* Check if Unsupported Request detected */
	if ((pxcap_sts_reg & NVME_PXDS_URD) >> 3) {
		status = FAIL;
		LOG_ERR("Unsupported Request Detected (URD) in PXDS");
	}
	/* Check if AUX Power detected */
	if ((pxcap_sts_reg & NVME_PXDS_APD) >> 4)
		LOG_NRM("AUX POWER Detected (APD) in PXDS");

	/* Check if Transactions Pending */
	if ((pxcap_sts_reg & NVME_PXDS_TP) >> 5)
		LOG_NRM("Transactions Pending (TP) in PXDS");

   }
   return status;
}

/*
* device_status_aerap: This func checks the status register of
* Advanced error reporting capabiltiy of PCI express device.
*/
int device_status_aercap(struct pci_dev *pdev, u16 base_offset)
{
   int status; /* Status indicating SUCCESS or FAIL */
   u16 offset; /* Offset 16 bit for PCIE space */
   u32 u32aer_sts = 0; /* AER Cap Status data */
   u32 u32aer_msk = 0; /* AER Mask bits data */
   int ret_code = 0; /* Reurn code for pci reads */
   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   LOG_DBG("Offset in AER CAP= 0x%X", base_offset);
   LOG_NRM("Checking Advanced Err Capability Status Regs (AERUCES and AERCS)");

   /* Compute the offset of AER Uncorrectable error status */
   offset = base_offset + NVME_AERUCES_OFFSET;

   /* Read the aer status bits */
   ret_code = pci_read_config_dword(pdev, offset, &u32aer_sts);
   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check");

   /* Mask the reserved bits */
   u32aer_sts &= ~NVME_AERUCES_RSVD;

   /* compute the mask offset */
   offset = base_offset + NVME_AERUCEM_OFFSET;

   /* get the mask bits */
   ret_code = pci_read_config_dword(pdev, offset, &u32aer_msk);
   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check");

   /* zero out the reserved bits */
   u32aer_msk &= ~NVME_AERUCES_RSVD;

   /*
   * Complement the Mask Registers and check if unmasked
   * bits have a error set
   */
   if (u32aer_sts & ~u32aer_msk) {
	/* Data Link Protocol Error check */
	if ((u32aer_sts & NVME_AERUCES_DLPES) >> 4) {
		status = FAIL;
		LOG_ERR("Data Link Protocol Error Status is Set (DLPES)");
	}
	/* Poinsed TLP status, not an error. */
	if ((u32aer_sts & NVME_AERUCES_PTS) >> 12)
		LOG_ERR("Poisoned TLP Status (PTS)");

	/* Check if Flow control Protocol error is set */
	if ((u32aer_sts & NVME_AERUCES_FCPES) >> 13) {
		status = FAIL;
		LOG_ERR("Flow Control Protocol Error Status (FCPES)");
	}
	/* check if completion time out status is set */
	if ((u32aer_sts & NVME_AERUCES_CTS) >> 14) {
		status = FAIL;
		LOG_ERR("Completion Time Out Status (CTS)");
	}
	/* check if completer Abort Status is set */
	if ((u32aer_sts & NVME_AERUCES_CAS) >> 15) {
		status = FAIL;
		LOG_ERR("Completer Abort Status (CAS)");
	}
	/* Check if Unexpected completion status is set */
	if ((u32aer_sts & NVME_AERUCES_UCS) >> 16) {
		status = FAIL;
		LOG_ERR("Unexpected Completion Status (UCS)");
	}
	/* Check if Receiver Over Flow status is set, status not error */
	if ((u32aer_sts & NVME_AERUCES_ROS) >> 17)
		LOG_ERR("Receiver Overflow Status (ROS)");
	/* Check if Malformed TLP Status is set, not an error */
	if ((u32aer_sts & NVME_AERUCES_MTS) >> 18)
		LOG_ERR("Malformed TLP Status (MTS)");

	/* ECRC error status check */
	if ((u32aer_sts & NVME_AERUCES_ECRCES) >> 19) {
		status = FAIL;
		LOG_ERR("ECRC Error Status (ECRCES)");
	}
	/* Unsupported Request Error Status*/
	if ((u32aer_sts & NVME_AERUCES_URES) >> 20) {
		status = FAIL;
		LOG_ERR("Unsupported Request Error Status (URES)");
	}
	/* Acs violation status check */
	if ((u32aer_sts & NVME_AERUCES_ACSVS) >> 21) {
		status = FAIL;
		LOG_ERR("ACS Violation Status (ACSVS)");
	}
	/* uncorrectable error status check */
	if ((u32aer_sts & NVME_AERUCES_UIES) >> 22) {
		status = FAIL;
		LOG_ERR("Uncorrectable Internal Error Status (UIES)");
	}
	/* MC blocked TLP status check, not an error*/
	if ((u32aer_sts & NVME_AERUCES_MCBTS) >> 23)
		LOG_ERR("MC Blocked TLP Status (MCBTS)");

	/* Atomic Op Egress blocked status, not an error */
	if ((u32aer_sts & NVME_AERUCES_AOEBS) >> 24)
		LOG_ERR("AtomicOp Egress Blocked Status (AOEBS)");

	/* TLP prefix blocked error status. */
	if ((u32aer_sts & NVME_AERUCES_TPBES) >> 25) {
		status = FAIL;
		LOG_ERR("TLP Prefix Blocked Error Status (TPBES)");
	}
   }

   /* Compute the offset for AER Correctable Error Status Register */
   offset = base_offset + NVME_AERCS_OFFSET;

   /* Read data from pcie space into u32aer_sts */
   ret_code = pci_read_config_dword(pdev, offset, &u32aer_sts);
   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check");

   /* zero out Reserved Bits*/
   u32aer_sts &= ~NVME_AERCS_RSVD;

   /* Compute the offser for AER correctable Mask register */
   offset = base_offset + NVME_AERCM_OFFSET;

   /*
   * Read the masked bits. When they are set to 1 it means that the s/w
   * should ignore those errors
   */
   ret_code = pci_read_config_dword(pdev, offset, &u32aer_msk);
   if (ret_code < 0)
	LOG_ERR("pci_read_config failed in driver error check");

   /* Zero out any reserved bits if they are set */
   u32aer_msk &= ~NVME_AERCS_RSVD;

   /*
   * Complement the mask so that any value which is 1 is not be tested
   * so it becomes zero. Remaining unmasked bits are bit wise anded to
   * check if any error is set which is not masked.
   */
   if (u32aer_sts & ~u32aer_msk) {
	/* Checked if reciever error status is set */
	if (u32aer_sts & NVME_AERCS_RES) {
		status = FAIL;
		LOG_ERR("Receiver Error Status (RES)");
	}
	/* check if Bad TLP status is set */
	if ((u32aer_sts & NVME_AERCS_BTS) >> 6) {
		status = FAIL;
		LOG_ERR("BAD TLP Status (BTS)");
	}
	/* check if BAD DLP is set */
	if ((u32aer_sts & NVME_AERCS_BDS) >> 7) {
		status = FAIL;
		LOG_ERR("BAD DLLP Status (BDS)");
	}
	/* Check if RRS is set, status not an error */
	if ((u32aer_sts & NVME_AERCS_RRS) >> 8)
		LOG_ERR("REPLAY_NUM Rollover Status (RRS)");

	/* Check if RTS is set */
	if ((u32aer_sts & NVME_AERCS_RTS) >> 12) {
		status = FAIL;
		LOG_ERR("Replay Timer Timeout Status (RTS)");
	}
	/* Check if non fatatl error is set */
	if ((u32aer_sts & NVME_AERCS_ANFES) >> 13) {
		status = FAIL;
		LOG_ERR("Advisory Non Fatal Error Status (ANFES)");
	}
	/* Check if CIES is set */
	if ((u32aer_sts & NVME_AERCS_CIES) >> 14) {
		status = FAIL;
		LOG_ERR("Corrected Internal Error Status (CIES)");
	}
	/* check if HLOS is set, Status not an error */
	if ((u32aer_sts & NVME_AERCS_HLOS) >> 15)
		LOG_ERR("Header Log Overflow Status (HLOS)");

   }
   return status;
}
