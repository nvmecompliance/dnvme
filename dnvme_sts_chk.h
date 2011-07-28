#ifndef _DNVME_STS_CHK_H_
#define _DNVME_STS_CHK_H_

/**
* @def PCI_DEVICE_STATUS
* define the offset for STS register
* from the start of PCI config space as specified in the
* NVME_Comliance 1.0a. offset 06h:STS - Device status.
* This register has error status for NVME PCI Exress
* Card. After reading data from this reagister, the driver
* will identify if any error is set during the operation and
* report as kernel alert message.
*/
#define PCI_DEVICE_STATUS               0x6

/**
* @def DEV_ERR_MASK
* The bit positions that are set in this 16 bit word
* implies that the error is defined for those poistionis in
* STS register. The bits that are 0 are non error positions.
*/
#define DEV_ERR_MASK                    0xB100

/**
* @def DPE
* This bit position indicates data parity error.
* Set to 1 by h/w when the controlller detects a
* parity error on its interface.
*/
#define DPE                             0x8000


/**
* @def DPD
* This bit position indicates Master data parity error.
* Set to 1 by h/w if parity error is set or parity
* error line is asserted and parity error response bit
* in CMD.PEE is set to 1.
*/
#define DPD                             0x0100

/**
* @def RMA
* This bit position indicates Received Master Abort.
* Set to 1 by h/w if when the controller receives a
* master abort to a cycle it generated.
*/
#define RMA                             0x2000

/**
* @def RTA
* This bit position indicates Received Target Abort.
* Set to 1 by h/w if when the controller receives a
* target abort to a cycle it generated.
*/
#define RTA                             0x1000

/**
* @def CL_MASK
* This bit position indicates Capabilities List of the controller
* The controller should support the PCI Power Management cap as a
* minimum.
*/
#define CL_MASK                             0x0010

/**
* @def NEXT_MASK
* This indicates the location of the next capability item
* in the list.
*/
#define NEXT_MASK			0xFF00

#define AER_ERR_MASK			0x20000
/**
* @def PMCAP_ID
* This bit indicates if the pointer leading to this position
* is a PCI power management capability.
*/
#define PMCAP_ID			0x1

/**
* @def MSICAP_ID
* This bit indicates if the pointer leading to this position
* is a capability.
*/
#define MSICAP_ID			0x5

/**
* @def MSIXCAP_ID
* This bit indicates if the pointer leading to this position
* is a capability.
*/
#define MSIXCAP_ID			0x11

/**
* @def PXCAP_ID
* This bit indicates if the pointer leading to this position
* is a capability.
*/
#define PXCAP_ID			0x10

/**
* @def AERCAP_ID
* This bit indicates if the pointer leading to this position
* is a capability.
*/
#define AERCAP_ID			0x0001

/**
* device_status_pci function returns the device status of
* the PCI Device status register set in STS register. The offset for this
* register is 0x06h as specified in NVME Expres 1.0a spec.
* @param device_data
* @return SUCCESS or FAIL
*/
int device_status_pci(u16 device_data);

/**
* device_status_next function checks if the next capability of the NVME
* Express device exits and if it exists then gets its status.
* @param pdev
* @return SUCCESS or FAIL
*/
int device_status_next(struct pci_dev *pdev);

/**
* nvme_controller_status - This function checks the controller status
* register CSTS at offset 0x1C from the BAR01 offset.
* @param pdev
* @return SUCCESS or FAIL
*/
int nvme_controller_status(struct pci_dev *pdev);

/**
* device_status_pci function returns the device status of
* the PCI Power Management status register set in PMCS register.
* @param device_data
* @return SUCCESS or FAIL
*/
int device_status_pmcs(u16 device_data);

/**
* device_status_msicap function returns the device status of
* Message signalled Interrupt status register in MC and MPEND
* @param pdev
* @param device_data
* @return SUCCESS or FAIL
*/
int device_status_msicap(struct pci_dev *pdev, u16 device_data);

/**
* device_status_msixcap function returns the device status of
* Message signalled Interrupt-X status register in MXC
* @param pdev
* @param device_data
* @return SUCCESS or FAIL
*/
int device_status_msixcap(struct pci_dev *pdev, u16 device_data);

/**
* device_status_pxcap function returns the device status of
* PCI express capabilty device status register in PXDS.
* @param pdev
* @param device_data
* @return SUCCESS or FAIL
*/
int device_status_pxcap(struct pci_dev *pdev, u16 device_data);

/**
* device_status_aercap function returns the device status of
* Advanced Error Reporting AER capabilty device status registers
* The register checked are AERUCES, AERCS and AERCC
* @param pdev
* @param device_data
* @return SUCCESS or FAIL
*/
int device_status_aercap(struct pci_dev *pdev, u16 device_data);

#endif
