#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#ifndef __KERNEL__
#include <stdint.h>

#define LOG_DEBUG	printf
#define LOG_NORMAL	printf
#define LOG_ERROR	printf

#else

#define LOG_DEBUG	printk
#define LOG_NORMAL	printk
#define LOG_ERROR	printk

#endif

#define DEVICE_LIST_SIZE		20
#define TEST_MAJOR			140
#define CONFIG_PCI			1
#define PCI_CLASS_STORAGE_EXPRESS	0x010802
#define NVME_MINORS			16	

/* IOCTL Definitions used from user app call. */
enum {
     NVME_IDENTIFY_NS = 0,
     NVME_IDENTIFY_CTRL,
     NVME_GET_RANGE_TYPE,
     NVME_SUBMIT_IO,
     NVME_DOWNLOAD_FW,
     NVME_ACTIVATE_FW,
     NVME_READ_GENERIC,
     NVME_CREATE_ADMN_Q,
     NVME_DEL_ADMN_Q,
     NVME_SEND_ADMN_CMD,
};

/* Definition for IOCTL driver code with parameters and size */
#define NVME_IOCTL_IDENTIFY_NS _IOWR('A', NVME_IDENTIFY_NS, int)
#define NVME_IOCTL_IDENTIFY_CTRL _IOWR('A', NVME_IDENTIFY_CTRL, int)
#define NVME_IOCTL_GET_RANGE_TYPE _IOWR('A', NVME_GET_RANGE_TYPE, int)
#define NVME_IOCTL_SUBMIT_IO _IOWR('A', NVME_SUBMIT_IO, int)
#define NVME_IOCTL_DOWNLOAD_FW	_IOWR('A', NVME_DOWNLOAD_FW, int)
#define NVME_IOCTL_ACTIVATE_FW	_IOWR('A', NVME_ACTIVATE_FW, int)
#define NVME_IOCTL_READ_GENERIC	_IOWR('A', NVME_READ_GENERIC, struct nvme_read_generic)
#define NVME_IOCTL_CREATE_ADMN_Q _IOWR('A', NVME_CREATE_ADMN_Q, int)
#define NVME_IOCTL_DEL_ADMN_Q _IOWR('A', NVME_DEL_ADMN_Q, int)
#define NVME_IOCTL_SEND_ADMN_CMD _IOWR('A', NVME_SEND_ADMN_CMD, int)

#endif
