#include <linux/module.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/kthread.h>

/* project specific includes */
#include "test_driver.h"
#include "definitions.h"

#define PCI_CLASS_STORAGE_EXPRESS	0x010802

/* to handle multiple outstanding requesta,
need array or list of at least these 2 variables */
struct test_ctx test_ctx;
int test_result = STATUS_NOK;

struct task_struct *test_thread = (void *)-1;

struct nvme_dev_entry {
	struct pci_dev *pdev;
	u8 *bar0mapped;
	struct list_head list;
};
LIST_HEAD(nvme_dev);

struct nvme_dev_entry *get_nvme_dev(struct dev_data *device)
{
	struct pci_dev *pdev;
	struct nvme_dev_entry *nvme_dev_entry;

	list_for_each_entry(nvme_dev_entry, &nvme_dev, list) {
		pdev = nvme_dev_entry->pdev;
		if ((pdev->bus->number == device->bus) &&
			(PCI_SLOT(pdev->devfn) == device->slot) &&
			(PCI_FUNC(pdev->devfn) == device->func)) {
			LOG_DEBUG("[Nvme_Drv]Found test device [%02x:%02x.%02x]"
				"\n", device->bus, device->slot,
				device->func);
			return nvme_dev_entry;
		}
	}
	LOG_ERROR("[Nvme_Drv]Can't find device [%02x:%02x.%02x]\n", device->bus,
		device->slot, device->func);
	return NULL;
}

int run_test(void *data)
{
	struct test_ctx *test = (struct test_ctx *) data;
	LOG_ERROR("[Nvme_Drv]Wait 3 seconds for fun\n");
	msleep(3000);

	if (test->test_grp == TEST_REG_MAP)
		test_result = verify_reg_map(test);
	else if (test->test_grp == TEST_CMD_SET)
		test_result = test_cmd_set(test);
	else
		test_result = STATUS_WRONG_ARG;

	return test_result;
}

int get_test_dev(struct dev_data *device)
{
	struct pci_dev *test_dev;
	struct nvme_dev_entry *nvme;

	nvme = get_nvme_dev(device);
	if (!nvme)
		return STATUS_NOK;

	test_dev = nvme->pdev;
	if (!test_dev)
		return STATUS_NOK;

	test_ctx.test_dev = test_dev;

	/* map BAR0 to access configurations registers */
	if (nvme->bar0mapped == 0) {
		nvme->bar0mapped = ioremap(pci_resource_start(test_dev, 0),
			pci_resource_len(test_dev, 0));
		if (!nvme->bar0mapped) {
			LOG_ERROR("[Nvme_Drv]Error mapping bar 0\n");
			return -1;
		}
	}

	LOG_DEBUG("[Nvme_Drv]Using Bar0 address: %llx, length %d\n",
		(uint64_t)nvme->bar0mapped, (int) pci_resource_len(test_dev, 0));

	test_ctx.bar_dev = (void __iomem *)nvme->bar0mapped;

	return 0;
}

int start_test(struct test_sel *test_sel)
{
	int res = 0;

	test_ctx.test_grp = test_sel->test_grp;
	test_ctx.test_num = test_sel->test_num;
	test_ctx.test_flags = test_sel->flags;

	/* set status to in_progress prior to starting thread, 
	   who knows if app may call immediately prior to thread actuallly running  */
	test_result = STATUS_IN_PROGRESS;

	test_thread = kthread_run(run_test, (void *)&test_ctx, "test_thread");
	if (IS_ERR(test_thread)) {
		test_result = STATUS_NOK;
		LOG_ERROR("[Nvme_Drv]Can't create thread\n");
	}
	return res;
}

int get_device_list(struct dev_data *devices, int num)
{
	int i = 0;
	struct nvme_dev_entry *nvme_dev_entry;

	list_for_each_entry(nvme_dev_entry, &nvme_dev, list) {
		if (i >= num)
			break;
		devices[i].bus = nvme_dev_entry->pdev->bus->number;
		devices[i].slot = PCI_SLOT(nvme_dev_entry->pdev->devfn);
		devices[i].func = PCI_FUNC(nvme_dev_entry->pdev->devfn);
		i++;
	}

	return i;
}

long test_driver_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	int res = 0;
	struct test_sel *test_sel;
	struct test_status *test_status;
	struct dev_data *dev_data;

	/* FIXME check usage copy_to_user */
	switch (cmd) {
	case TEST_IOC_GET_DEVICE_LIST:
		res = get_device_list((struct dev_data *)data,
			DEVICE_LIST_SIZE);
		break;
	case TEST_IOC_SELECT:
		/* get pci_dev selected for test */
		dev_data = (struct dev_data *)data;
		if (get_test_dev(dev_data))
			return STATUS_WRONG_ARG;
		break;
	case TEST_IOC_START:
		/* make sure TEST_IOC_SELECT has been previously called */
		if (test_ctx.bar_dev  == 0) {
			res = STATUS_IOCTL_FAILED;
			break;
		}	
		test_sel = (struct test_sel *)data;
		test_result = 0;
		res = start_test(test_sel);
		break;
	case TEST_IOC_GET_STATUS:
		/* make sure TEST_IOC_SELECT has been previously called */
		if (test_ctx.bar_dev  == 0) {
			res = STATUS_IOCTL_FAILED;
			break;
		}	
		test_status = (struct test_status *)data;
		test_status->state = test_result;
		break;
	case TEST_IOC_GET_RESULT:
		res = STATUS_IOCTL_FAILED;
		break;
	case TEST_IOC_ABORT:
		if (IS_ERR(test_thread))
			kthread_stop(test_thread);
		test_thread = (void *)-1;
		res = STATUS_TEST_ABORTED;
		break;
	case TEST_IOC_RESET:
		/* make sure TEST_IOC_SELECT has been previously called */
		if (test_ctx.bar_dev  == 0) {
			res = STATUS_IOCTL_FAILED;
			break;
		}	
		if (IS_ERR(test_thread))
			kthread_stop(test_thread);
		test_thread = (void *)-1;
		reset(&test_ctx);
		res = STATUS_OK;
		break;
	}
	return res;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = test_driver_ioctl,
};

static int __devinit test_probe(struct pci_dev *pdev,
	const struct pci_device_id *id)
{
	int bars;
	struct nvme_dev_entry *nvme_dev_entry = NULL;

	LOG_DEBUG("[Nvme_Drv]Probing - %04X %04X [%02x:%02x.%02x c:%06x]\n",
		pdev->device, pdev->vendor,
		pdev->bus->number, PCI_SLOT(pdev->devfn),
		PCI_FUNC(pdev->devfn), pdev->class);
	/* FIXME return correct status if allocation fails */
	nvme_dev_entry = kzalloc(sizeof(*nvme_dev_entry), GFP_KERNEL);
	nvme_dev_entry->pdev = pdev;
	nvme_dev_entry->bar0mapped = 0;
	list_add_tail(&nvme_dev_entry->list, &nvme_dev);


	/* Moved from  get_test_dev(. . .) above */
	if (pci_enable_device(pdev)) {
		LOG_ERROR("[Nvme_Drv]Can't enable PCI device\n");
		return -1;
	}

	if (pci_enable_device_mem(pdev)) {
		LOG_ERROR("[Nvme_Drv]Can't enable PCI device memory\n");
		return -1;
	}

	pci_set_master(pdev);

	bars = pci_select_bars(pdev, IORESOURCE_MEM);

	if (pci_request_selected_regions(pdev, bars,
		"test_driver")) {
		LOG_ERROR("[Nvme_Drv]Can't select regions\n");
	}

	return 0;
}

static void __devexit test_remove(struct pci_dev *pdev)
{
	return;
}

static DEFINE_PCI_DEVICE_TABLE(test_id_table) = {
	{ PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xFFFF00) },
	{ 0, }
};

static struct pci_driver test_driver = {
	.name           = "test_driver",
	.id_table       = test_id_table,
	.probe          = test_probe,
	.remove         = __devexit_p(test_remove),
};

static int test_driver_init(void)
{
	int res;

	register_chrdev(TEST_MAJOR, NODE_NAME, &fops);
	res = pci_register_driver(&test_driver);
	if (res) {
		LOG_NORMAL("[Nvme_Drv]PCI driver registration failed\n");
		return 0;
	}

	return 0;
}

static void test_driver_exit(void)
{
	unregister_chrdev(TEST_MAJOR, NODE_NAME);
	pci_unregister_driver(&test_driver);
}

MODULE_AUTHOR("Intel Corporation");
MODULE_LICENSE("GPL");
module_init(test_driver_init);
module_exit(test_driver_exit);
