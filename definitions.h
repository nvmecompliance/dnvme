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

#define DEVICE_LIST_SIZE	20
#define TEST_MAJOR 		140
#define NVME_NODE_FILE		"/dev/nvme_node"
#define NODE_NAME		"nvme_node"

enum test_grp {
	TEST_ALL = 0,
	TEST_REG_MAP,
	TEST_CMD_SET,
	TEST_GRP_NUM,
};

enum {
	TEST_ALL_CMDS = 0,
	CMD_IDENTIFY,
	CMD_CREATE_DUP_CQ,
	CMD_DELETE_CQ,
	CMD_GET_NON_EXT_FEAT,
	CMD_SET_GET_FEATS,
	CMD_CREATE_TOO_MANY_CQS,
	CMD_ABORT,
	ALL_CMDS
};

enum {
	STATUS_OK = 0,
	STATUS_NOK,
	STATUS_WRONG_ARG,
	STATUS_IOCTL_FAILED,
	STATUS_IN_PROGRESS,
	STATUS_TIMEOUT,
	STATUS_INIT_FAILED,
	STATUS_TEST_ABORTED,
	STATUS_TEST_NOT_DEFINED,
};

/* flags to test_sel */
enum {
	TEST_MMIO_INITAL_VALUE = 1,
	TEST_MMIO_RW = 2,
};

/* ioctls defines */
enum {
	TEST_GET_DEVICE_LIST = 0,
	TEST_SELECT_DEVICE,
	TEST_START,
	TEST_GET_STATUS,
	TEST_GET_RESULT,
	TEST_ABORT,
	TEST_RESET,
};

/* ioctls definitions */

#define TEST_IOC_GET_DEVICE_LIST \
	_IOWR('A', TEST_GET_DEVICE_LIST, struct dev_data)
#define TEST_IOC_SELECT		_IOWR('A', TEST_SELECT_DEVICE, struct dev_data)
#define TEST_IOC_START		_IOWR('A', TEST_START, struct test_sel)
#define TEST_IOC_GET_STATUS	_IOWR('A', TEST_GET_STATUS, struct test_status)
#define TEST_IOC_RESET		_IOWR('A', TEST_RESET, int)
/* currently not used by app */
#define TEST_IOC_GET_RESULT	_IOWR('A', TEST_GET_RESULT, int)
#define TEST_IOC_ABORT		_IOWR('A', TEST_ABORT, int)

struct dev_data {
	int bus;
	int slot;
	int func;
};

struct test_sel {
	int test_grp;
	int test_num;
	unsigned int flags;
};

struct test_status {
	int state;
};

#endif
