#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#define SUCCESS				0
#define FAIL				-1
#define DEVICE_LIST_SIZE		20
#define CONFIG_PCI			1
#define NVME_DEV_INIT			0x3
/**
* @def PCI_CLASS_STORAGE_EXPRESS
* Set to value matching with NVME HW
*/
#define PCI_CLASS_STORAGE_EXPRESS	0x010802

#define NVME_MINORS			16

/**
* @def MAX_PCI_EXPRESS_CFG
* Maximum pcie config space.
*/
#define MAX_PCI_EXPRESS_CFG		0xFFF

/**
* @def MAX_PCI_CFG
* Maximum pci config space available
*/
#define MAX_PCI_CFG			0xFF

/**
* @def MAX_PCI_HDR
* Maximum pci header offset.
*/
#define MAX_PCI_HDR			0x3F

#define LOWER_16BITS			0xFFFF

/**
* @def CAP_REG
* Set to offset defined in NVME Spec 1.0b.
*/
#define CAP_REG				0x34

/**
* @def PMCS
* Set to offset defined in NVME Spec 1.0b.
*/
#define PMCS				0x4

/**
* @def AER_ID_MASK
* Mask bits will extract last bit and that will help
* in determining what this bit corresponds to in terms of
* AER capability.
*/
#define AER_ID_MASK			0x1

/**
* @def AER_CAP_ID
* Indicate that this capability structure is an Advanced Error
* reporting capabilty.
*/
#define AER_CAP_ID			0x1

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

/**
*  ToDo: remove these enums and add to dnvme_ioctls.h
*  when new function is added.
*  enums are for add different switch cases
*  when the IOCTL's from the driver get called.
*/
enum {
     NVME_IDENTIFY_NS = 0,
     NVME_IDENTIFY_CTRL,
     NVME_GET_RANGE_TYPE,
     NVME_SUBMIT_IO,
     NVME_DOWNLOAD_FW, /** < enum to Download Firmware. */
     NVME_ACTIVATE_FW, /** < enum to activate the downloaded Firmware. */
     NVME_DEL_ADMN_Q, /** < enum Delete aprev create admin Queue. */
     NVME_SEND_ADMN_CMD, /** < enum Send and admin command. */
};

/**
* Definition for IOCTL driver code with parameters and size
* ioctl uses _IOWR for device control operations.
*/

/**
* @def NVME_IOCTL_IDENTIFY_NS
* define a unique value for identify NS
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_IDENTIFY_NS _IOWR('A', NVME_IDENTIFY_NS, int)

/**
* @def NVME_IOCTL_IDENTIFY_CTRL
* define a unique value for identify controller
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_IDENTIFY_CTRL _IOWR('A', NVME_IDENTIFY_CTRL, int)

/**
* @def NVME_IOCTL_GET_RANGE_TYPE
* define a unique value for get nvme range type
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_GET_RANGE_TYPE _IOWR('A', NVME_GET_RANGE_TYPE, int)

/**
* @def NVME_IOCTL_SUBMIT_IO
* define a unique value for submit input output command
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_SUBMIT_IO _IOWR('A', NVME_SUBMIT_IO, int)

/**
* @def NVME_IOCTL_DOWNLOAD_FW
* define a unique value for download firmware
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_DOWNLOAD_FW	_IOWR('A', NVME_DOWNLOAD_FW, int)

/**
* @def NVME_IOCTL_ACTIVATE_FW
* define a unique value for active firmware switch
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_ACTIVATE_FW	_IOWR('A', NVME_ACTIVATE_FW, int)

/**
* @def NVME_IOCTL_DEL_ADMN_Q
* define unique value for deleting admin queue.
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_DEL_ADMN_Q _IOWR('A', NVME_DEL_ADMN_Q, int)

/**
* @def NVME_IOCTL_SEND_ADMN_CMD
* define unique ioctl for sending admin queue command.
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_SEND_ADMN_CMD _IOWR('A', NVME_SEND_ADMN_CMD, int)

#endif
