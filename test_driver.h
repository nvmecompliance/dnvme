#ifndef _TEST_DRIVER_H_
#define _TEST_DRIVER_H_

#include "nvme.h"
#include "cmd_test.h"
#include "reg_test.h"

struct test_ctx {
	struct pci_dev *test_dev;
	struct msix_entry *entry;
	struct nvme_bar __iomem *bar_dev;
	struct nvme_queue **queues;
	u32 __iomem *dbs;
	u32 ctrl_config;
	int test_grp;
	int test_num;
	unsigned int test_flags;
};

#endif
