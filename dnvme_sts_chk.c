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
