/*
* sysdnvme.c
* NVME Express Device Driver for Test Compliance.
*
* Copyright (c) 2011, Intel Corporation.
*
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include "dnvme_interface.h"
#include "definitions.h"
#include "sysfuncproto.h"
#include "dnvme_reg.h"
#include "sysdnvme.h"
#include "dnvme_ioctls.h"
#include "dnvme_queue.h"
#include "dnvme_ds.h"
#include "version.h"
#include "dnvme_cmds.h"
#include "ut_reap_inq.h"

#define    DRV_NAME            "dnvme"
#define    NVME_DEVICE_NAME    "qnvme"
#define    DRV_VERSION         "1.0"

/*
* Define the PCI storage express as
* 0xFFFFF00 to be used while informing to kernel.
*/
static DEFINE_PCI_DEVICE_TABLE(dnvme_pci_tbl) = {
    { PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xFFFF00) },
    { 0, }
};

/*
* PCI dnvme driver structure definition.
*/
static struct pci_driver dnvme_pci_driver = {
    .name           = DRV_NAME,
    .id_table       = dnvme_pci_tbl,
    .probe          = dnvme_pci_probe,
};

/*
*   This is the main ioctl for char type device
*   this ioctl invoke the dnvme device ioctls.
*/
static const struct file_operations dnvme_fops_f = {
    .owner = THIS_MODULE,
    .ioctl = dnvme_ioctl_device,
    .open  = dnvme_device_open,
    .release = dnvme_device_release,
    .mmap = dnvme_device_mmap,
};

/* local functions static declarations */
static struct  metrics_device_list *find_device(struct inode *inode);

/* char device specific parameters */
static int NVME_MAJOR;
static struct class *class_nvme;
struct cdev *char_dev;
struct device *device;
LIST_HEAD(metrics_dev_ll);
int nvme_minor_x;
module_param(NVME_MAJOR, int, 0);

/*
*   First initialization for Driver code.
*   dnvme_init - Perform early initialization of the host
*   host: dnvme host to initialize
*   @return returns 0 if initialization was successful.
*   @author T.Sravan Kumar
*/
static int dnvme_init(void)
{
    int retCode = -ENODEV;
    int err = -EINVAL;

    LOG_NRM("version: %d.%d", VER_MAJOR, VER_MINOR);
    LOG_DBG("Init module - dnvme init");

    /* This is classic way to register a char device */
    NVME_MAJOR = register_chrdev(NVME_MAJOR, NVME_DEVICE_NAME, &dnvme_fops_f);
    if (NVME_MAJOR < 0) {
        LOG_ERR("NVME Char Registration failed");
        return -ENODEV;
    }
    LOG_DBG("NVME Char type registered..");

   class_nvme = class_create(THIS_MODULE, NVME_DEVICE_NAME);

    /* Check if class_nvme creation has any issues */
    if (IS_ERR(class_nvme)) {
        err = PTR_ERR(class_nvme);
        LOG_ERR("Nvme class creation failed and stopping");
        return err;
    }

    /* Register this device as pci device */
    retCode = pci_register_driver(&dnvme_pci_driver);
    if (retCode < 0) {
        /*Unable to register the PCI device */
        LOG_ERR("PCI Driver Registration unsuccessful");
        return retCode;
    }

    LOG_DBG("PCI Registration Success return code = 0x%x", retCode);
    return 0;
}

/*
* dnvme_pci_probe - Probe the NVME PCIe device for BARs. This function is
* called when the driver invokes the fops after basic initialization is
* performed.
*/
int __devinit dnvme_pci_probe(struct pci_dev *pdev,
        const struct pci_device_id *id)
{
    int retCode = -ENODEV;  /* retCode is set to no devices */
    int bars = 0;           /* initialize bars to 0         */
    u32  BaseAddress0 = 0;
    u32  *bar;
    dev_t devno = 0;
    int err;
    struct metrics_device_list *pmetrics_device_element;

    /* Following the Iniitalization steps from LDD 3 */
    LOG_DBG("Start probing for NVME PCI Express Device");
    if ((retCode == pci_enable_device(pdev)) < 0) {
        LOG_ERR("PciEnable not successful");
        return retCode;
    }

    /* Why does retcode is negative here and still success? TSK */
    LOG_DBG("PCI enable Success!. Return Code = 0x%x", retCode);
    if (pci_enable_device_mem(pdev)) {
        LOG_ERR("pci_enalbe_device_mem not successful");
        return -1;
    }

    LOG_DBG("NVME Probing... Dev = 0x%x Vendor = 0x%x", pdev->device,
            pdev->vendor);
    LOG_DBG("Bus No = 0x%x, Dev Slot = 0x%x", pdev->bus->number,
            PCI_SLOT(pdev->devfn));
    LOG_DBG("Dev Func = 0x%x, Class = 0x%x", PCI_FUNC(pdev->devfn),
            pdev->class);

    /* Allocate kernel mem for each of the device using one now */
    char_dev = kzalloc(sizeof(struct cdev), GFP_KERNEL);
    if (char_dev == NULL) {
        LOG_ERR("Allocation for char device failed!!");
        return -ENOMEM;
    }
    /* Make device with nvme major and minor number */
    devno = MKDEV(NVME_MAJOR, nvme_minor_x);
    /* initialize the device and char device */
    cdev_init(char_dev, &dnvme_fops_f);
    /* Set the owner */
    char_dev->owner = THIS_MODULE;
    /* assign the fops to char device */
    char_dev->ops = &dnvme_fops_f;
    /* Add this char device to kernel */
    err = cdev_add(char_dev, devno, 1);
    if (err) {
        LOG_ERR("Adding device to kernel failed");
        return err;
    }
    /* Create device with class name class_nvme */
    device = device_create(class_nvme, NULL, devno, NULL, NVME_DEVICE_NAME"%d",
            nvme_minor_x);
    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        LOG_ERR("Device Creation failed");
        return err;
    }

    /* Return void, Enables bus mastering and calls pcibios_set_master */
    pci_set_master(pdev);

    /* Make BAR mask from the resource */
    bars = pci_select_bars(pdev, IORESOURCE_MEM);

    if (pci_request_selected_regions(pdev, bars, DRV_NAME)) {
        LOG_ERR("Can't select regions, Exiting!!!");
        return -EINVAL;
    } else {
        LOG_DBG("Select regions success");
    }

    LOG_DBG("Mask for PCI BARS = 0x%x", bars);
    LOG_DBG("PCI Probe Success!. Return Code = 0x%x", retCode);

    bar = ioremap(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
    if (bar == NULL) {
        LOG_ERR("allocate Host Memory for Device Failed!!");
        return -EINVAL;
    }

    /* Only debug because the above remap should give BAR's  */
    pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &BaseAddress0);
    LOG_DBG("PCI BAR 0 = 0x%x", BaseAddress0);

    /* Allocate mem fo nvme device with kernel memory */
    pmetrics_device_element = kmalloc(sizeof(struct metrics_device_list),
            GFP_KERNEL);
    if (pmetrics_device_element == NULL) {
        LOG_ERR("failed mem allocation for device in device list.");
        return -ENOMEM;
    }
    /* Initialize the current device found */
    retCode = driver_ioctl_init(pdev, pmetrics_device_element);
    if (retCode != SUCCESS) {
        LOG_ERR("Failed driver ioctl initializations!!");
        return -EINVAL;
    }
    /* Update info in the metrics list */
    pmetrics_device_element->metrics_device->minor_no = nvme_minor_x;
    /* update the device minor number */
    nvme_minor_x = nvme_minor_x + 1;

    /* set device open status to false when initialized */
    pmetrics_device_element->metrics_device->open_flag = 0;

    /* Initialize the mutex state. */
    mutex_init(&pmetrics_device_element->metrics_mtx);

    /* Add the device to the linked list */
    list_add_tail(&pmetrics_device_element->metrics_device_hd, &metrics_dev_ll);
    return retCode;
}

/*
 * find device from the device linked list. Returns pointer to the
 * device if found otherwise returns NULL.
 */
static struct  metrics_device_list *find_device(struct inode *inode)
{
    struct  metrics_device_list *pmetrics_device_element; /* Metrics device  */
    int dev_found = 1;
    /* Loop through the devices available in the metrics list */
    list_for_each_entry(pmetrics_device_element, &metrics_dev_ll,
            metrics_device_hd) {
        LOG_DBG("Minor Number in the List = %d", pmetrics_device_element->
                metrics_device->minor_no);
        if (iminor(inode) == pmetrics_device_element->metrics_device->
                minor_no) {
            LOG_DBG("Found device in the metrics list");
            return pmetrics_device_element;
        } else {
            dev_found = 0;
        }
    }
    /* The specified device could not be found in the list */
    if (dev_found == 0) {
        LOG_ERR("Cannot find the device with minor no. %d", iminor(inode));
        return NULL;
    }
    return NULL;
}

/*
 * This operation is always the first operation performed on the device file.
 * when the user call open fd, this is where it lands.
 */
int dnvme_device_open(struct inode *inode, struct file *filp)
{
    struct  metrics_device_list *pmetrics_device_element; /* Metrics device  */

    LOG_DBG("Call to open the device...");
    pmetrics_device_element = find_device(inode);
    if (pmetrics_device_element == NULL) {
        LOG_ERR("Cannot find the device with minor no. %d", iminor(inode));
        return -ENODEV;
    }
    if (pmetrics_device_element->metrics_device->open_flag == 0) {
        pmetrics_device_element->metrics_device->open_flag = 1;
        deallocate_all_queues(pmetrics_device_element, ST_DISABLE_COMPLETELY);
    } else {
        LOG_ERR("Attempt to open device multiple times not allowed!!");
        return -EPERM; /* Operation not permitted */
    }
    return SUCCESS;
}

/*
 * This operation is invoked when the file structure is being released. When
 * the user app close a device then this is where the entry point is. The
 * driver cleans up any memory it has reference to. This ensures a clean state
 * of the device.
 */
int dnvme_device_release(struct inode *inode, struct file *filp)
{
    struct  metrics_device_list *pmetrics_device_element; /* Metrics device  */

    LOG_DBG("Call to Release the device...");

    pmetrics_device_element = find_device(inode);

    if (pmetrics_device_element == NULL) {
        LOG_ERR("Cannot find the device with minor no. %d", iminor(inode));
        return -ENODEV;
    }
    pmetrics_device_element->metrics_device->open_flag = 0;
    mutex_destroy(pmetrics_device_element->metrics_device->metrics_mtx);
    if (deallocate_all_queues(pmetrics_device_element, ST_DISABLE_COMPLETELY)
            != SUCCESS) {
        LOG_ERR("Deallocation failed!!");
        return -EINVAL;
    }
    return SUCCESS;
}

/*
 * dnvme_device_mmap - This function maps the contiguous device mapped area
 * to user space. This is specfic to device which is called though fd.
 */
int dnvme_device_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct  metrics_device_list *pmetrics_device_element; /* Metrics device */
    struct  metrics_sq  *pmetrics_sq_list;  /* SQ linked list               */
    struct  metrics_cq  *pmetrics_cq_list;  /* CQ linked list               */
    unsigned long pfn;
    struct inode *inode = filp->f_dentry->d_inode;
    u32 qtype;
    u16 qid;

    vma->vm_flags |= VM_IO;
    LOG_DBG("Device Calling mmap function...");

    pmetrics_device_element = find_device(inode);
    if (pmetrics_device_element == NULL) {
        return -ENODEV;
    }

    /* Calculate the q id and q type from offset */
    qtype = (vma->vm_pgoff >> 0x10) & 0x1;
    qid = vma->vm_pgoff & 0xFFFF;

    LOG_DBG("Q Type = %d", qtype);
    LOG_DBG("Q ID = 0x%x", qid);

    /* If Q type is 1 implies SQ */
    if (qtype != 0) {
        pmetrics_sq_list = find_sq(pmetrics_device_element, qid);
        if (pmetrics_sq_list == NULL) {
            return -EBADSLT;
        }
        pfn = virt_to_phys(pmetrics_sq_list->private_sq.vir_kern_addr) >>
                PAGE_SHIFT;
    } else {
        pmetrics_cq_list = find_cq(pmetrics_device_element, qid);
        if (pmetrics_cq_list == NULL) {
            return -EBADSLT;
        }
        pfn = virt_to_phys(pmetrics_cq_list->private_cq.vir_kern_addr) >>
                PAGE_SHIFT;
    }

    LOG_DBG("PFN = 0x%lx", pfn);

    return remap_pfn_range(vma, vma->vm_start, pfn,
                    vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

/*
 * This function is called whenever a process tries to do an ioctl on our
 * device file. We get two extra parameters (additional to the inode and file
 * structures, which all device functions get): the number of the ioctl called
 * and the parameter given to the ioctl function.
 *
 * If the ioctl is write or read/write (meaning output is returned to the
 * calling process), the ioctl call returns the output of this function.
 *
*/
int dnvme_ioctl_device(struct inode *inode, struct file *file,
        unsigned int ioctl_num, unsigned long ioctl_param)
{
    struct  metrics_device_list *pmetrics_device_element; /* Metrics device  */
    int ret_val = -EINVAL;        /* set ret val to invalid, chk for success */
    struct rw_generic *nvme_data; /* Local struct var for nvme rw dat        */
    int *nvme_dev_err_sts;        /* nvme device error status                */
    struct nvme_ctrl_state *ctrl_new_state; /* controller new state          */
    struct nvme_get_q_metrics *get_q_metrics; /* metrics q params            */
    struct nvme_create_admn_q *create_admn_q; /* create admn q params        */
    struct nvme_prep_sq *prep_sq;   /* SQ params for preparing IO SQ         */
    struct nvme_prep_cq *prep_cq;   /* CQ params for preparing IO CQ         */
    struct nvme_ring_sqxtdbl *ring_sqx;  /* Ring SQx door-bell params        */
    struct nvme_64b_send *nvme_64b_send; /* 64 byte cmd params               */
    struct nvme_file    *n_file;         /* dump metrics params              */
    struct nvme_reap_inquiry *reap_inq;  /* reap inquiry params              */
    u16 __user test_number;
    unsigned char __user *datap = (unsigned char __user *)ioctl_param;

    LOG_DBG("Minor No = %d", iminor(inode));
    pmetrics_device_element = find_device(inode);
    if (pmetrics_device_element == NULL) {
        LOG_ERR("Cannot find the device with minor no. %d", iminor(inode));
        return -ENODEV;
    }

    /* Grab the Mutex for this device in the linked list */
    mutex_lock(&pmetrics_device_element->metrics_mtx);

    /*
     * Given a ioctl_num invoke corresponding function
     */
    switch (ioctl_num) {
    case NVME_IOCTL_ERR_CHK:
        /* check if the device has any errors set in its status
           register. And report errors. */
        nvme_dev_err_sts = (int *)ioctl_param;
        LOG_DBG("Checking device Status");
        ret_val = device_status_chk(pmetrics_device_element, nvme_dev_err_sts);
        break;

    case NVME_IOCTL_READ_GENERIC:
        LOG_DBG("Invoking User App request to read  the PCI Header Space");
        nvme_data = (struct rw_generic *)ioctl_param;
        ret_val = driver_generic_read(nvme_data, pmetrics_device_element);
        break;

    case NVME_IOCTL_WRITE_GENERIC:
        LOG_DBG("Invoke IOCTL Generic Write Function");
        nvme_data = (struct rw_generic *)ioctl_param;
        ret_val = driver_generic_write(nvme_data, pmetrics_device_element);
        break;

    case NVME_IOCTL_CREATE_ADMN_Q:
        LOG_DBG("IOCTL for Create Admin Q's...");
        create_admn_q = (struct nvme_create_admn_q *)ioctl_param;
        /* Check the type of Admin Q and call corresponding functions */
        if (create_admn_q->type == ADMIN_SQ) {
            LOG_DBG("Create Admin SQ");
            /* call driver routine to create admin sq from ll */
            ret_val = driver_create_asq(create_admn_q, pmetrics_device_element);
        } else if (create_admn_q->type == ADMIN_CQ) {
            LOG_DBG("Create Admin CQ");
            /* call driver routine to create admin cq from ll */
            ret_val = driver_create_acq(create_admn_q, pmetrics_device_element);
        } else {
            LOG_ERR("Unknown Q type specified..");
            return -EINVAL;
        }
        break;

    case NVME_IOCTL_DEVICE_STATE:
        LOG_DBG("IOCTL for nvme controller set/reset Command");
        LOG_NRM("Invoke IOCTL for controller Status Setting");
        /* Assign user passed parameters to local struct */
        ctrl_new_state = (struct nvme_ctrl_state *)ioctl_param;
        if (ctrl_new_state->new_state == ST_ENABLE) {
            LOG_NRM("Ctrlr is getting ENABLED...");
            ret_val = nvme_ctrl_enable(pmetrics_device_element);
        } else if ((ctrl_new_state->new_state == ST_DISABLE) ||
                (ctrl_new_state->new_state == ST_DISABLE_COMPLETELY)) {
            LOG_NRM("Controller is going to DISABLE...");
            /* Waiting for the controller to go idle. */
            ret_val = nvme_ctrl_disable(pmetrics_device_element);
            if (ret_val == SUCCESS) {
                /* Clean Up the Data Structures. */
                deallocate_all_queues(pmetrics_device_element,
                        ctrl_new_state->new_state);
            }
         } else {
            LOG_ERR("Device State not correctly specified.");
            return -EINVAL;
        }
        break;

    case NVME_IOCTL_GET_Q_METRICS:
        LOG_DBG("User App Requested Q Metrics...");
        /* Assign user passed parameters to q metrics structure. */
        get_q_metrics = (struct nvme_get_q_metrics *)ioctl_param;
        /* Call the Q metrics function and return the data to user. */
        ret_val = nvme_get_q_metrics(pmetrics_device_element, get_q_metrics);
        break;

    case NVME_IOCTL_PREPARE_SQ_CREATION:
        LOG_DBG("Driver Preparation for IO SQ");
        /* Assign user passed parameters to q metrics structure. */
        prep_sq = (struct nvme_prep_sq *)ioctl_param;
        /* Call alloc_sq function to add a node in liked list */
        ret_val = driver_nvme_prep_sq(prep_sq, pmetrics_device_element);
        break;

    case NVME_IOCTL_PREPARE_CQ_CREATION:
        LOG_DBG("Driver Preparation for IO CQ");
        /* Assign user passed parameters to q metrics structure. */
        prep_cq = (struct nvme_prep_cq *)ioctl_param;
        /* Call alloc_sq function to add a node in liked list */
        ret_val = driver_nvme_prep_cq(prep_cq, pmetrics_device_element);
        break;

    case NVME_IOCTL_RING_SQ_DOORBELL:
        LOG_DBG("Driver Call to Ring SQx Doorbell");
        /* Assign user passed parameters to q metrics structure. */
        ring_sqx = (struct nvme_ring_sqxtdbl *)ioctl_param;
        /* Call the ring doorbell driver function */
        ret_val = nvme_ring_sqx_dbl(ring_sqx, pmetrics_device_element);
        break;

    case NVME_IOCTL_SEND_64B_CMD:
        LOG_DBG("IOCTL NVME_IOCTL_SEND_64B_CMD Command");
        /* Assign user passed parameters to local struct pointrs */
        nvme_64b_send = (struct nvme_64b_send *)ioctl_param;
        ret_val =  driver_send_64b(pmetrics_device_element, nvme_64b_send);
        /* Display success or fail */
        if (ret_val >= 0) {
            LOG_NRM("PRP Creation Success");
        } else {
            LOG_NRM("PRP Creation Failed");
        }
        break;

    case NVME_IOCTL_DUMP_METRICS:
        LOG_DBG("Dump Data Structure Metrics:");
        /* Assign user passed parameters to local struct pointers */
        n_file = (struct nvme_file *)ioctl_param;
        /* call logging routine */
        ret_val = driver_log(n_file);
        break;

    case NVME_IOCTL_REAP_INQUIRY:
        LOG_DBG("Reap Inquiry ioctl:");
        /* Assign user passed parameters to local reap structure */
        reap_inq = (struct nvme_reap_inquiry *)ioctl_param;
        /* call reap inquiry driver routine */
        ret_val = driver_reap_inquiry(pmetrics_device_element, reap_inq);
        break;

    case IOCTL_UNIT_TESTS:
        /* Get the test_number that user passed */
        copy_from_user(&test_number, datap, sizeof(u16));

        LOG_DBG("Test Number = %d", test_number);
        /* Call the Test setup based on user request */
        switch (test_number) {
        case 0: /* Unit Test for IOCTL REAP INQUIRY */
            LOG_DBG("UT Reap Inquiry ioctl:");
            unit_test_reap_inq(pmetrics_device_element);
            ret_val = SUCCESS;
            break;
        case 1:
            LOG_DBG("UT Mmap ioctl:");
            unit_test_mmap(pmetrics_device_element);
            ret_val = SUCCESS;
            break;
        default:
            LOG_DBG("Invalid Test Setup called....%d", test_number);
            ret_val = -EINVAL;
        }

        break;

    default:
        LOG_DBG("Cannot find IOCTL going to default case");
        break;
    }

    /* Release the Mutex */
    mutex_unlock(&pmetrics_device_element->metrics_mtx);

    return ret_val;
}

/*
*  Module Exit code.
*  dnvme_exit - Perform clean exit
*/
static void __exit dnvme_exit(void)
{
    struct pci_dev *pdev;
    struct  metrics_device_list *pmetrics_device_element;  /* Metrics device */

    device_del(device);
    class_destroy(class_nvme);
    cdev_del(char_dev);
    unregister_chrdev(NVME_MAJOR, NVME_DEVICE_NAME);
    pci_unregister_driver(&dnvme_pci_driver);

    /* Loop through the devices available in the metrics list */
    list_for_each_entry(pmetrics_device_element, &metrics_dev_ll,
            metrics_device_hd) {
        /* Free up the DMA pool */
        destroy_dma_pool(pmetrics_device_element->metrics_device);
        /* Clean Up the Data Structures. */
        deallocate_all_queues(pmetrics_device_element, ST_DISABLE_COMPLETELY);
        mutex_destroy(pmetrics_device_element->metrics_device->metrics_mtx);
        pdev = pmetrics_device_element->metrics_device->pdev;
        pci_release_regions(pdev);
        /* free up the cq linked list */
        list_del(&pmetrics_device_element->metrics_cq_list);
        /* free up the cq linked list */
        list_del(&pmetrics_device_element->metrics_sq_list);
    }

    /* free up the device linked list */
    list_del(&metrics_dev_ll);

    LOG_DBG("dnvme driver Exited...Bye!!");
}

/*
*  Driver Module Calls.
*/
MODULE_DESCRIPTION("Kernel Device Driver for NVME PCI Express card");
MODULE_AUTHOR("T Sravan Kumar <sravan.kumar.thokala@intel.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:"DRV_NAME);

module_init(dnvme_init);
module_exit(dnvme_exit);
