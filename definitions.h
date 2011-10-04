#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#define SUCCESS              0
#define FAIL                -1
#define DEVICE_LIST_SIZE     20
#define CONFIG_PCI           1
#define NVME_DEV_INIT        0x3
/**
* @def PCI_CLASS_STORAGE_EXPRESS
* Set to value matching with NVME HW
*/
#define PCI_CLASS_STORAGE_EXPRESS    0x010802

#define NVME_MINORS            16

/**
* @def MAX_PCI_EXPRESS_CFG
* Maximum pcie config space.
*/
#define MAX_PCI_EXPRESS_CFG        0xFFF

/**
* @def MAX_PCI_CFG
* Maximum pci config space available
*/
#define MAX_PCI_CFG               0xFF

/**
* @def MAX_PCI_HDR
* Maximum pci header offset.
*/
#define MAX_PCI_HDR              0x3F

#define LOWER_16BITS             0xFFFF

/**
* @def CAP_REG
* Set to offset defined in NVME Spec 1.0b.
*/
#define CAP_REG                0x34

/**
* @def PMCS
* Set to offset defined in NVME Spec 1.0b.
*/
#define PMCS                   0x4

/**
* @def AER_ID_MASK
* Mask bits will extract last bit and that will help
* in determining what this bit corresponds to in terms of
* AER capability.
*/
#define AER_ID_MASK            0x1

/**
* @def AER_CAP_ID
* Indicate that this capability structure is an Advanced Error
* reporting capability.
*/
#define AER_CAP_ID             0x1

/**
* This is a structure that defines all the PCI
* header information laid out in NVME SPec 1.0b
* PCI Header section.
*/
struct nvme_pci_header {
    u32 ID;
    u16 CMD;
    u16 STS;
    u8  RID;
    u32 CC;
    u8  CLS;
    u8  MLT;
    u8  HTYPE;
    u8  BIST;
    u32 BAR0;
    u32 BAR1;
    u32 BAR2;
    u32 BAR3;
    u32 BAR4;
    u32 BAR5;
    u32 CCPTR;
    u32 SS;
    u32 EPROM;
    u64 CAP;
    u16 INTR;
    u8  MGNT;
    u8  MLAT;
};

#endif
