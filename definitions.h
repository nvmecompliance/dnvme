#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#ifdef __KERNEL__

#define LOG_DEBUG	printk
#define LOG_NORMAL(msg, ...)	\
	printk(KERN_ERR "%s\n", msg, __FILE__, __LINE__);
#define LOG_ERROR	printk

#endif

#define SUCCESS				0
#define FAIL				-1
#define DEVICE_LIST_SIZE		20
#define CONFIG_PCI			1
#define PCI_CLASS_STORAGE_EXPRESS	0x010802
#define NVME_MINORS			16

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
#define DEV_ERR_MASK                    0xC100

/**
* @def DPE
* This bit position indicates data parity error.
* Set to 1 by h/w when the controlller detects a
* parity error on its interface.
*/
#define DPE                             0x8000


/**
* @def SSE
* This bit position indicates Signaled System Error.
* Not Supported vy NVM Express.
*/
#define SSE                             0x4000

/**
* @def DPD
* This bit position indicates Master data parity error.
* Set to 1 by h/w if parity error is set or parity
* error line is asserted and parity error response bit
* in CMD.PEE is set to 1.
*/
#define DPD                             0x0100

/**
* This is a structure that defines all the PCI
* header information laid out in NVME SPec 1.0a
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
*   enums are for add different switch cases
*   when the IOCTL's from the driver get called.
*/
enum {
     NVME_IDENTIFY_NS = 0,
     NVME_IDENTIFY_CTRL,
     NVME_GET_RANGE_TYPE,
     NVME_SUBMIT_IO,
     NVME_DOWNLOAD_FW, /** < enum to Download Firmware. */
     NVME_ACTIVATE_FW, /** < enum to activate the downloaded Firmware. */
     NVME_CREATE_ADMN_Q, /** < enum Create a new Admin Queue. */
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
* @def NVME_IOCTL_CREATE_ADMN_Q
* define unique value for creating admin queue.
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_CREATE_ADMN_Q _IOWR('A', NVME_CREATE_ADMN_Q, int)

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
