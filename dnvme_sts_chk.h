/*
* NVM Express Compliance Suite
* Copyright (c) 2011, Intel Corporation.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef _DNVME_STS_CHK_H_
#define _DNVME_STS_CHK_H_

/**
* @def PCI_DEVICE_STATUS
* define the offset for STS register
* from the start of PCI config space as specified in the
* NVME_Comliance 1.0b. offset 06h:STS - Device status.
* This register has error status for NVME PCI Exress
* Card. After reading data from this reagister, the driver
* will identify if any error is set during the operation and
* report as kernel alert message.
*/
#define PCI_DEVICE_STATUS               0x6

/**
* @def DEV_ERR_MASK
* The bit positions that are set in this 16 bit word
* implies that the error is defined for those positions in
* STS register. The bits that are 0 are non error positions.
*/
#define DEV_ERR_MASK                    0xB100

/**
* @def DPE
* This bit position indicates data parity error.
* Set to 1 by h/w when the controller detects a
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
#define RTA                  0x1000

/**
* @def CL_MASK
* This bit position indicates Capabilities List of the controller
* The controller should support the PCI Power Management cap as a
* minimum.
*/
#define CL_MASK              0x0010

/**
* @def NEXT_MASK
* This indicates the location of the next capability item
* in the list.
*/
#define NEXT_MASK            0xFF00

#define AER_ERR_MASK         0x20000
/**
* @def PMCAP_ID
* This bit indicates if the pointer leading to this position
* is a PCI power management capability.
*/
#define PMCAP_ID            0x1

/**
* @def MSICAP_ID
* This bit indicates if the pointer leading to this position
* is a capability.
*/
#define MSICAP_ID            0x5

/**
* @def MSIXCAP_ID
* This bit indicates if the pointer leading to this position
* is a capability.
*/
#define MSIXCAP_ID            0x11

/**
* @def PXCAP_ID
* This bit indicates if the pointer leading to this position
* is a capability.
*/
#define PXCAP_ID            0x10


/**
* PCI Express Device Status- PXDS
* The below enums are for PCI express device status reister
* individual status bits and offset position.
*/
enum {
    NVME_PXDS_CED   = 0x1 << 0, /* Correctable Error */
    NVME_PXDS_NFED  = 0x1 << 1, /* Non Fatal Error */
    NVME_PXDS_FED   = 0x1 << 2, /* Fatal Error */
    NVME_PXDS_URD   = 0x1 << 3, /* Unsupported Request*/
    NVME_PXDS_APD   = 0x1 << 4, /* AUX Power */
    NVME_PXDS_TP    = 0x1 << 5, /* Transactions Pending */
    NVME_PXDS_RSVD  = 0xFFE0,   /* Reserved Bits in PXDS */
    NVME_PXCAP_PXDS = 0xA,      /* Device Status offset from PXCAP */
};

/**
* @def AERCAP_ID
* This bit indicates if the pointer leading to this position
* is a capability.
*/
#define AERCAP_ID            0x0001

/**
* enums for bit positions specified in NVME Controller Status
* offset 0x1Ch CSTS register.
*/
enum {
    NVME_CSTS_SHST      = 0x3,
    NVME_CSTS_SHST_MASK = 0xC,
    NVME_CSTS_RSVD      = 0xF,
};

/**
* enums for bit positions specified in NVME controller status
* offset 1C CSTS in bits 02:03
*/
enum {
    NVME_CSTS_NRML_OPER = 0x0,
    NVME_CSTS_SHT_OCC   = 0x1,
    NVME_CSTS_SHT_COMP  = 0x2,
    NVME_CSTS_SHT_RSVD  = 0x3,
};

/**
* enums for capability version indicated in the AER capability ID
* Offset AERCAP:AERID
*/
enum {
    NVME_AER_CVER = 0x2,
};

/**
* enums for Advanced Error reporting Status and Mask Registers offsets
*/
enum {
    NVME_AERUCES_OFFSET   = 0x4,
    NVME_AERUCEM_OFFSET   = 0x8,
    NVME_AERUCESEV_OFFSET = 0xC,
    NVME_AERCS_OFFSET     = 0x10,
    NVME_AERCM_OFFSET     = 0x14,
    NVME_AERCC_OFFSET     = 0x14,
};

/**
* enums for AER Uncorrectable Error Status and Mask bits.
* The bit positions for status and Mask are same in the NVME Spec 1.0b
* so here we have only this bit positions defined for both AERUCES
* and AERUCEM, mask register.
*/
enum {
    NVME_AERUCES_RSVD   = 0xFC00002F,
    NVME_AERUCES_DLPES  = 0x1 << 4,
    NVME_AERUCES_PTS    = 0x1 << 12,
    NVME_AERUCES_FCPES  = 0x1 << 13,
    NVME_AERUCES_CTS    = 0x1 << 14,
    NVME_AERUCES_CAS    = 0x1 << 15,
    NVME_AERUCES_UCS    = 0x1 << 16,
    NVME_AERUCES_ROS    = 0x1 << 17,
    NVME_AERUCES_MTS    = 0x1 << 18,
    NVME_AERUCES_ECRCES = 0x1 << 19,
    NVME_AERUCES_URES   = 0x1 << 20,
    NVME_AERUCES_ACSVS  = 0x1 << 21,
    NVME_AERUCES_UIES   = 0x1 << 22,
    NVME_AERUCES_MCBTS  = 0x1 << 23,
    NVME_AERUCES_AOEBS  = 0x1 << 24,
    NVME_AERUCES_TPBES  = 0x1 << 25,
};

/**
* enums for AER Correctable Error Status and Mask bits.
* The bit positions for status and Mask are same in the NVME Spec 1.0b
* so here we have only this bit positions defined for both AERCS
* and AERCEM, mask register.
*/
enum {
    NVME_AERCS_RSVD  = 0xFFFF0E3E,
    NVME_AERCS_HLOS  = 0x1 << 15,
    NVME_AERCS_CIES  = 0x1 << 14,
    NVME_AERCS_ANFES = 0x1 << 13,
    NVME_AERCS_RTS   = 0x1 << 12,
    NVME_AERCS_RRS   = 0x1 << 8,
    NVME_AERCS_BDS   = 0x1 << 7,
    NVME_AERCS_BTS   = 0x1 << 6,
    NVME_AERCS_RES   = 0x1 << 0,
};

/**
* device_status_pci function returns the device status of
* the PCI Device status register set in STS register. The offset for this
* register is 0x06h as specified in NVME Express 1.0b spec.
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
* Message signaled Interrupt status register in MC and MPEND
* @param pdev
* @param device_data
* @return SUCCESS or FAIL
*/
int device_status_msicap(struct pci_dev *pdev, u16 device_data);

/**
* device_status_msixcap function returns the device status of
* Message signaled Interrupt-X status register in MXC
* @param pdev
* @param device_data
* @return SUCCESS or FAIL
*/
int device_status_msixcap(struct pci_dev *pdev, u16 device_data);

/**
* device_status_pxcap function returns the device status of
* PCI express capability device status register in PXDS.
* @param pdev
* @param base_offset
* @return SUCCESS or FAIL
*/
int device_status_pxcap(struct pci_dev *pdev, u16 base_offset);

/**
* device_status_aercap function returns the device status of
* Advanced Error Reporting AER capability device status registers
* The register checked are AERUCES, AERCS and AERCC
* @param pdev
* @param base_offset
* @return SUCCESS or FAIL
*/
int device_status_aercap(struct pci_dev *pdev, u16 base_offset);

#endif
