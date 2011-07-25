#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_sts_chk.h"
#include "definitions.h"


int device_status_pci(u16 device_data)
{
   int status;

   LOG_DEBUG("PCI Device Status Data = %x\n", device_data);

   /*
   * Set the status to SUCCESS and eventually verify any error
   * really got set.
   */
   status = SUCCESS;

   if (device_data & DEV_ERR_MASK) {
        status = FAIL;

        if (device_data & DPE) {
                LOG_ERROR("Device Status - DPE Set\n");
                LOG_ERROR("Detected Data parity Error\n");
        }
        if (device_data & SSE) {
                LOG_ERROR("Device Status - SSE Set\n");
                LOG_ERROR("Detected Signaled System Error\n");
        }
        if (device_data & DPD) {
                LOG_ERROR("Device Status - DPD Set\n");
                LOG_ERROR("Detected Master Data Parity Error\n");
        }
        if (device_data & RMA) {
                LOG_ERROR("Device Status - RMA Set\n");
                LOG_ERROR("Received Master Abort...\n");
        }
        if (device_data & RTA) {
                LOG_ERROR("Device Status - RTA Set\n");
                LOG_ERROR("Received Target Abort...\n");
        }
        if (device_data & STA) {
                LOG_ERROR("Device Status - STA Set\n");
                LOG_ERROR("Signalled Target Abort...\n");
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

   LOG_DEBUG("PCI Power Management Control and Status = %x\n", device_data);

   return status;
}
