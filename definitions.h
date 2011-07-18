#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#ifdef __KERNEL__

#define LOG_DEBUG	printk
#define LOG_NORMAL	printk
#define LOG_ERROR	printk

#endif

#define DEVICE_LIST_SIZE		20
#define CONFIG_PCI			1
#define PCI_CLASS_STORAGE_EXPRESS	0x010802
#define NVME_MINORS			16

/**
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
     NVME_READ_GENERIC, /** < enum to invoke read generic function call. */
     NVME_WRITE_GENERIC, /** < enum to invoke write generic function call. */
     NVME_CREATE_ADMN_Q, /** < enum Create a new Admin Queue. */
     NVME_DEL_ADMN_Q, /** < enum Delete aprev create admin Queue. */
     NVME_SEND_ADMN_CMD, /** < enum Send and admin command. */
};

/**
* Definition for IOCTL driver code with parameters and size
* ioctl uses _IOWR for device control operations.
*/

/**
* @def define a unique value for identify NS
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_IDENTIFY_NS _IOWR('A', NVME_IDENTIFY_NS, int)

/**
* @def define a unique value for identify controller
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_IDENTIFY_CTRL _IOWR('A', NVME_IDENTIFY_CTRL, int)

/**
* @def define a unique value for get nvme range type
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_GET_RANGE_TYPE _IOWR('A', NVME_GET_RANGE_TYPE, int)

/**
* @def define a unique value for submit input output command
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_SUBMIT_IO _IOWR('A', NVME_SUBMIT_IO, int)

/**
* @def define a unique value for download firmware
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_DOWNLOAD_FW	_IOWR('A', NVME_DOWNLOAD_FW, int)

/**
* @def define a unique value for active firmware switch
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_ACTIVATE_FW	_IOWR('A', NVME_ACTIVATE_FW, int)

/**
* @def define a unique value for Generic read capabiltiy
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_READ_GENERIC	_IOWR('A', NVME_READ_GENERIC,\
					struct nvme_read_generic)

/**
* @def define a unique value for Generic read capabiltiy
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_WRITE_GENERIC _IOWR('A', NVME_WRITE_GENERIC,\
					struct nvme_write_generic)
/**
* @def define unique value for creating admin queue.
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_CREATE_ADMN_Q _IOWR('A', NVME_CREATE_ADMN_Q, int)

/**
* @def define unique value for deleting admin queue.
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_DEL_ADMN_Q _IOWR('A', NVME_DEL_ADMN_Q, int)

/**
* @def define unique ioctl for sending admin queue command.
* the first parameter is the group to which this
* IOCTL type belongs to, genererally from (0-255)
* the second parameter is type within the group.
* the third parameter give the size of data and
* type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_SEND_ADMN_CMD _IOWR('A', NVME_SEND_ADMN_CMD, int)

#endif
