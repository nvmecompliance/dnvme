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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <linux/mman.h>
#include <linux/dma-mapping.h>

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
#include "dnvme_irq.h"

#define    DRV_NAME             "dnvme"
#define    NVME_DEVICE_NAME     "nvme"


static DEFINE_PCI_DEVICE_TABLE(dnvme_pci_tbl) = {
    { PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xffffff) },
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
    .unlocked_ioctl = dnvme_ioctl_device,
    .open  = dnvme_device_open,
    .release = dnvme_device_release,
    .mmap = dnvme_device_mmap,
};

/* local functions static declarations */
static struct  metrics_device_list *lock_device(struct inode *inode);
static void unlock_device(struct  metrics_device_list *pmetrics_device_element);
static struct  metrics_device_list *find_device(struct inode *inode);

/* metrics driver */
struct metrics_driver g_metrics_drv;

/* char device specific parameters */
static int NVME_MAJOR;
static struct class *class_nvme;
struct cdev *char_dev;
struct device *device;
LIST_HEAD(metrics_dev_ll);
int nvme_minor_x;
module_param(NVME_MAJOR, int, 0);

/*
 * First initialization for Driver code.
 * dnvme_init - Perform early initialization of the host
 * host: dnvme host to initialize
 * @return returns 0 if initialization was successful.
*/
static int __init dnvme_init(void)
{
    int err;

    LOG_NRM("dnvme loading; dnvme version: %d.%d", VER_MAJOR, VER_MINOR);
    g_metrics_drv.driver_version = DRIVER_VERSION;
    g_metrics_drv.api_version = API_VERSION;

    NVME_MAJOR = register_chrdev(NVME_MAJOR, NVME_DEVICE_NAME, &dnvme_fops_f);
    if (NVME_MAJOR < 0) {
        LOG_ERR("NVME Char Registration failed");
        return -ENODEV;
    }

    /* Check if class_nvme creation has any issues */
    class_nvme = class_create(THIS_MODULE, NVME_DEVICE_NAME);
    if (IS_ERR(class_nvme)) {
        err = PTR_ERR(class_nvme);
        LOG_ERR("Nvme class creation failed and stopping");
        return err;
    }

    /* Register this driver */
    err = pci_register_driver(&dnvme_pci_driver);
    if (err < 0) {
        /*Unable to register the PCI device */
        LOG_ERR("PCI Driver Registration unsuccessful");
        return err;
    }

    LOG_DBG("PCI Registration complete");
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
    int err;
    int bars = 0;
    dev_t devno = 0;
    void __iomem *bar0;
    struct metrics_device_list *pmetrics_device_element;


    /* Allocate kernel memory for each of the device(s) */
    char_dev = kzalloc(sizeof(struct cdev), GFP_KERNEL);
    if (char_dev == NULL) {
        LOG_ERR("Alloc memory for kernel rep failed: 0x%p", char_dev);
        return -ENOMEM;
    }

    err = pci_enable_device(pdev);
    if (err < 0) {
        LOG_ERR("Enabling the PCI device has failed: 0x%04X", err);
        return err;
    }
    err = pci_enable_device_mem(pdev);
    if (err < 0) {
        LOG_ERR("Enabling the PCI device memory has failed: 0x%04X", err);
        return err;
    }

    /* Reserve all the mem map'd config registers for this device */
    bars = pci_select_bars(pdev, IORESOURCE_MEM);
    err = pci_request_selected_regions(pdev, bars, DRV_NAME);
    LOG_DBG("pci mapped bars = 0x%08x", bars);
    if (err < 0) {
        LOG_ERR("Can't preserved memory regions");
        return err;
    }

    LOG_DBG("NVMe dev: 0x%x, vendor: 0x%x", pdev->device, pdev->vendor);
    LOG_DBG("NVMe bus #%d, dev slot: %d", pdev->bus->number,
        PCI_SLOT(pdev->devfn));
    LOG_DBG("NVMe func: 0x%x, class: 0x%x", PCI_FUNC(pdev->devfn),
        pdev->class);

    /* Make device with nvme major and minor number */
    devno = MKDEV(NVME_MAJOR, nvme_minor_x);
    cdev_init(char_dev, &dnvme_fops_f);
    char_dev->owner = THIS_MODULE;
    char_dev->ops = &dnvme_fops_f;
    err = cdev_add(char_dev, devno, 1);
    if (err) {
        LOG_ERR("Adding NVMe device into the kernel failed: %d", err);
        return err;
    }

    /* Create an NVMe special device */
    device = device_create(class_nvme, NULL, devno, NULL, NVME_DEVICE_NAME"%d",
        nvme_minor_x);
    if (IS_ERR(device)) {
        err = PTR_ERR(device);
        LOG_ERR("Creation of special device file failed: %d", err);
        return err;
    }

    pci_set_master(pdev);
    dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
    dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64));

    /* Needed to enable IRQ's upon subsequent insmod dnvme.ko */
    err = pci_reenable_device(pdev);
    if (err < 0) {
        LOG_ERR("Can't reenable PCI device");
        return err;
    }

    bar0 = ioremap(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
    if (bar0 == NULL) {
        LOG_ERR("Mapping BAR0 mem map'd registers failed");
        return -EINVAL;
    }

    pmetrics_device_element = kmalloc(
        sizeof(struct metrics_device_list), (GFP_KERNEL | __GFP_ZERO));
    if (pmetrics_device_element == NULL) {
        LOG_ERR("Failed alloc mem for internal device metric storage");
        return -ENOMEM;
    }

    err = driver_ioctl_init(pdev, bar0, pmetrics_device_element);
    if (err < 0) {
        LOG_ERR("Failed to init dnvme's internal state metrics");
        return err;
    }

    mutex_init(&pmetrics_device_element->metrics_mtx);
    pmetrics_device_element->metrics_device->private_dev.open_flag = 0;
    pmetrics_device_element->metrics_device->private_dev.minor_no =
        nvme_minor_x++;
    list_add_tail(&pmetrics_device_element->metrics_device_hd, &metrics_dev_ll);
    return 0;
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
        if (iminor(inode) == pmetrics_device_element->metrics_device->
                private_dev.minor_no) {
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
 * lock_device function will call find_device and if found device locks my
 * taking the mutex. This function returns a pointer to successfully found
 * device.
 */
static struct  metrics_device_list *lock_device(struct inode *inode)
{
    struct  metrics_device_list *pmetrics_device_element;
    pmetrics_device_element = find_device(inode);
    if (pmetrics_device_element == NULL) {
        LOG_ERR("Cannot find the device with minor no. %d", iminor(inode));
        return NULL;
    }

    /* Grab the Mutex for this device in the linked list */
    mutex_lock(&pmetrics_device_element->metrics_mtx);
    return pmetrics_device_element;
}

/*
 * Release the mutex for this device.
 */
static void unlock_device(struct  metrics_device_list *pmetrics_device_element)
{
    if (mutex_is_locked(&pmetrics_device_element->metrics_mtx)) {
        mutex_unlock(&pmetrics_device_element->metrics_mtx);
    }
}

/*
 * This operation is always the first operation performed on the device file.
 * when the user call open fd, this is where it lands.
 */
int dnvme_device_open(struct inode *inode, struct file *filp)
{
    struct  metrics_device_list *pmetrics_device_element; /* Metrics device  */
    int ret_val = SUCCESS;

    LOG_DBG("Call to open the device...");
    pmetrics_device_element = lock_device(inode);
    if (pmetrics_device_element == NULL) {
        LOG_ERR("Cannot lock on this device with minor no. %d", iminor(inode));
        ret_val = -ENODEV;
        goto op_exit;
    }

    if (pmetrics_device_element->metrics_device->private_dev.open_flag == 0) {
        pmetrics_device_element->metrics_device->private_dev.open_flag = 1;
        deallocate_all_queues(pmetrics_device_element, ST_DISABLE_COMPLETELY);
        deallocate_mb(pmetrics_device_element);
    } else {
        LOG_ERR("Attempt to open device multiple times not allowed!!");
        ret_val =  -EPERM; /* Operation not permitted */
    }

op_exit:
    unlock_device(pmetrics_device_element);
    return ret_val;
}

/*
 * This operation is invoked when the file structure is being released. When
 * the user app close a device then this is where the entry point is. The
 * driver cleans up any memory it has reference to. This ensures a clean state
 * of the device.
 */
int dnvme_device_release(struct inode *inode, struct file *filp)
{
    /* Metrics device */
    struct  metrics_device_list *pmetrics_device_element;
    int ret_val = SUCCESS;

    LOG_DBG("Call to Release the device...");
    pmetrics_device_element = lock_device(inode);
    if (pmetrics_device_element == NULL) {
        LOG_ERR("Cannot lock on this device with minor no. %d", iminor(inode));
        ret_val = -ENODEV;
        goto rel_exit;
    }

    /* Set the device open flag to false */
    pmetrics_device_element->metrics_device->private_dev.open_flag = 0;
    /* Disable the NVME controller */
    nvme_disable(pmetrics_device_element, ST_DISABLE_COMPLETELY);

rel_exit:
    LOG_DBG("Device Closed. Releasing Mutex...");
    unlock_device(pmetrics_device_element);
    return ret_val;
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
    struct  metrics_meta *pmeta_data;       /* pointer to meta node         */
    u8 *vir_kern_addr;
    unsigned long pfn = 0;
    struct inode *inode = filp->f_dentry->d_inode;
    u32 type;
    u32 id;
    u32 mmap_range;
    int npages;
    int ret_val = SUCCESS;

    LOG_DBG("Device Calling mmap function...");

    pmetrics_device_element = lock_device(inode);
    if (pmetrics_device_element == NULL) {
        LOG_ERR("Cannot lock on this device with minor no. %d", iminor(inode));
        ret_val = -ENODEV;
        goto mmap_exit;
    }

    vma->vm_flags |= (VM_IO | VM_RESERVED);

    /* Calculate the id and type from offset */
    type = (vma->vm_pgoff >> 0x12) & 0x3;
    id = vma->vm_pgoff & 0x3FFFF;

    LOG_DBG("Type = %d", type);
    LOG_DBG("ID = 0x%x", id);

    /* If type is 1 implies SQ, 0 implies CQ and 2 implies meta data */
    if (type == 0x1) {
        /* Process for SQ */
        if (id > USHRT_MAX) { /* 16 bits */
            LOG_ERR("SQ Id is greater than 16 bits..");
            ret_val = -EINVAL;
            goto mmap_exit;
        }
        /* Find SQ node in the list with id */
        pmetrics_sq_list = find_sq(pmetrics_device_element, id);
        if (pmetrics_sq_list == NULL) {
            ret_val = -EBADSLT;
            goto mmap_exit;
        }
        if (pmetrics_sq_list->private_sq.contig == 0) {
            LOG_ERR("MMAP does not work on non contig SQ's");
            #ifndef ENOTSUP
                ret_val = -EOPNOTSUPP; /* aka ENOTSUP in userland for POSIX */
            #else                      /*  parisc does define it separately.*/
                ret_val = -ENOTSUP;
            #endif
            goto mmap_exit;
        }
        vir_kern_addr = pmetrics_sq_list->private_sq.vir_kern_addr;
        mmap_range = pmetrics_sq_list->private_sq.size;
    } else if (type == 0x0) {
        /* Process for CQ */
        if (id > USHRT_MAX) { /* 16 bits */
            LOG_ERR("CQ Id is greater than 16 bits..");
            ret_val = -EINVAL;
            goto mmap_exit;
        }
        /* Find CQ node in the list with id */
        pmetrics_cq_list = find_cq(pmetrics_device_element, id);
        if (pmetrics_cq_list == NULL) {
            ret_val = -EBADSLT;
            goto mmap_exit;
        }
        if (pmetrics_cq_list->private_cq.contig == 0) {
            LOG_ERR("MMAP does not work on non contig CQ's");
            #ifndef ENOTSUP
                ret_val = -EOPNOTSUPP; /* aka ENOTSUP in userland for POSIX */
            #else                      /*  parisc does define it separately.*/
                ret_val = -ENOTSUP;
            #endif
            goto mmap_exit;
        }
        vir_kern_addr = pmetrics_cq_list->private_cq.vir_kern_addr;
        mmap_range = pmetrics_cq_list->private_cq.size;
    } else if (type == 0x2) {
        /* Process for Meta data */
        if (id > (2^18)) { /* 18 bits */
            LOG_ERR("Meta Id is greater than 18 bits..");
            ret_val = -EINVAL;
            goto mmap_exit;
        }
        /* find Meta Node data */
        pmeta_data = find_meta_node(pmetrics_device_element, id);
        if (pmeta_data == NULL) {
            ret_val = -EBADSLT;
            goto mmap_exit;
        }
        vir_kern_addr = pmeta_data->vir_kern_addr;
        mmap_range = pmetrics_device_element->metrics_meta.meta_buf_size;
    } else {
        ret_val = -EINVAL;
        goto mmap_exit;
    }

    npages = (mmap_range/PAGE_SIZE) + 1;
    if ((npages * PAGE_SIZE) < (vma->vm_end - vma->vm_start)) {
        LOG_ERR("Request to Map more than allocated pages...");
        ret_val = -EINVAL;
        goto mmap_exit;
    }
    LOG_DBG("Virt Address = 0x%llx", (u64)vir_kern_addr);
    LOG_DBG("Npages = %d", npages);

    /* Associated struct page ptr for kernel logical address */
    pfn = virt_to_phys(vir_kern_addr) >> PAGE_SHIFT;
    if (!pfn) {
        LOG_ERR("pfn is NULL");
        ret_val = -EINVAL;
        goto mmap_exit;
    }
    LOG_DBG("PFN = 0x%lx", pfn);

    /* remap kernel memory to userspace */
    ret_val = remap_pfn_range(vma, vma->vm_start, pfn,
                    vma->vm_end - vma->vm_start, vma->vm_page_prot);

mmap_exit:
    unlock_device(pmetrics_device_element);
    return ret_val;
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
long dnvme_ioctl_device(struct file *filp, unsigned int ioctl_num,
        unsigned long ioctl_param)
{
    struct  metrics_device_list *pmetrics_device_element; /* Metrics device  */
    int ret_val = -EINVAL;        /* set ret val to invalid, chk for success */
    struct rw_generic *nvme_data; /* Local struct var for nvme rw data       */
    int *nvme_dev_err_sts;        /* nvme device error status                */
    enum nvme_state *ctrl_new_state;          /* controller new state        */
    struct nvme_get_q_metrics *get_q_metrics; /* metrics q params            */
    struct nvme_create_admn_q *create_admn_q; /* create admn q params        */
    struct nvme_prep_sq *prep_sq;   /* SQ params for preparing IO SQ         */
    struct nvme_prep_cq *prep_cq;   /* CQ params for preparing IO CQ         */
    u16   ring_sqx;                 /* SQ ID to ring the door-bell           */
    struct nvme_64b_send *nvme_64b_send; /* 64 byte cmd params               */
    struct nvme_file    *n_file;         /* dump metrics params              */
    struct nvme_reap_inquiry *reap_inq;  /* reap inquiry params              */
    struct nvme_reap *reap_data;         /* Actual Reap params               */
    struct interrupts *irq_data;         /* IRQ type and IRQ vectors         */
    struct metrics_driver *dnvme_metrics;/* Dnvme Metrics params             */
    struct public_metrics_dev *dev_metrics;  /* public nvme dev metrics      */
    struct inode *inode = filp->f_dentry->d_inode;

    pmetrics_device_element = lock_device(inode);
    if (pmetrics_device_element == NULL) {
        LOG_ERR("Cannot lock device with minor no. %d", iminor(inode));
        ret_val = -ENODEV;
        goto ictl_exit;
    }

    /* Given a ioctl_num invoke corresponding function */
    switch (ioctl_num) {
    case NVME_IOCTL_ERR_CHK:
        LOG_DBG("NVME_IOCTL_ERR_CHK");
        /* check if the device has any errors set in its status
           register. And report errors. */
        nvme_dev_err_sts = (int *)ioctl_param;
        ret_val = device_status_chk(pmetrics_device_element, nvme_dev_err_sts);
        break;

    case NVME_IOCTL_READ_GENERIC:
        LOG_DBG("NVME_IOCTL_READ_GENERIC");
        nvme_data = (struct rw_generic *)ioctl_param;
        ret_val = driver_generic_read(nvme_data, pmetrics_device_element);
        break;

    case NVME_IOCTL_WRITE_GENERIC:
        LOG_DBG("NVME_IOCTL_WRITE_GENERIC");
        nvme_data = (struct rw_generic *)ioctl_param;
        ret_val = driver_generic_write(nvme_data, pmetrics_device_element);
        break;

    case NVME_IOCTL_CREATE_ADMN_Q:
        LOG_DBG("NVME_IOCTL_CREATE_ADMN_Q");
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
        }
        break;

    case NVME_IOCTL_DEVICE_STATE:
        LOG_DBG("NVME_IOCTL_DEVICE_STATE");

        /* Assign user passed parameters to local struct */
        ctrl_new_state = (enum nvme_state *)ioctl_param;
        if (*ctrl_new_state == ST_ENABLE) {
            LOG_DBG("Invoke ST_ENABLE");
            ret_val = nvme_ctrl_enable(pmetrics_device_element);
        } else if ((*ctrl_new_state == ST_DISABLE) ||
                   (*ctrl_new_state == ST_DISABLE_COMPLETELY)) {
            LOG_DBG("Invoke ST_DISABLE");
            ret_val = nvme_ctrl_disable(pmetrics_device_element);
            if (ret_val == SUCCESS) {
                nvme_disable(pmetrics_device_element, *ctrl_new_state);
            }
         } else {
            LOG_ERR("Unknown IOCTL parameter");
        }
        break;

    case NVME_IOCTL_GET_Q_METRICS:
        LOG_DBG("NVME_IOCTL_GET_Q_METRICS");
        /* Assign user passed parameters to q metrics structure. */
        get_q_metrics = (struct nvme_get_q_metrics *)ioctl_param;
        /* Call the Q metrics function and return the data to user. */
        ret_val = get_public_qmetrics(pmetrics_device_element, get_q_metrics);
        break;

    case NVME_IOCTL_PREPARE_SQ_CREATION:
        LOG_DBG("NVME_IOCTL_PREPARE_SQ_CREATION");
        /* Assign user passed parameters to prep_sq structure. */
        prep_sq = (struct nvme_prep_sq *)ioctl_param;
        /* Call alloc_sq function to add a node in liked list */
        ret_val = driver_nvme_prep_sq(prep_sq, pmetrics_device_element);
        break;

    case NVME_IOCTL_PREPARE_CQ_CREATION:
        LOG_DBG("NVME_IOCTL_PREPARE_CQ_CREATION");
        /* Assign user passed parameters to prep_cq structure. */
        prep_cq = (struct nvme_prep_cq *)ioctl_param;
        /* Call alloc_sq function to add a node in liked list */
        ret_val = driver_nvme_prep_cq(prep_cq, pmetrics_device_element);
        break;

    case NVME_IOCTL_RING_SQ_DOORBELL:
        LOG_DBG("NVME_IOCTL_RING_SQ_DOORBELL");
        /* Assign user passed parameters to sqx to be rung */
        ring_sqx = (u16)ioctl_param;
        /* Call the ring doorbell driver function */
        ret_val = nvme_ring_sqx_dbl(ring_sqx, pmetrics_device_element);
        break;

    case NVME_IOCTL_SEND_64B_CMD:
        LOG_DBG("NVME_IOCTL_SEND_64B_CMD");
        /* Assign user passed parameters to local struct pointers */
        nvme_64b_send = (struct nvme_64b_send *)ioctl_param;
        ret_val =  driver_send_64b(pmetrics_device_element, nvme_64b_send);
        /* Display success or fail */
        if (ret_val >= 0) {
            LOG_DBG("Command sent succesfully");
        } else {
            LOG_DBG("Sending of Command Failed");
        }
        break;

    case NVME_IOCTL_DUMP_METRICS:
        LOG_DBG("NVME_IOCTL_DUMP_METRICS");
        /* Assign user passed parameters to local struct pointers */
        n_file = (struct nvme_file *)ioctl_param;
        /* call logging routine */
        ret_val = driver_log(n_file);
        break;

    case NVME_IOCTL_REAP_INQUIRY:
        LOG_DBG("NVME_IOCTL_REAP_INQUIRY");
        /* Assign user passed parameters to local reap structure */
        reap_inq = (struct nvme_reap_inquiry *)ioctl_param;
        /* call reap inquiry driver routine */
        ret_val = driver_reap_inquiry(pmetrics_device_element, reap_inq);
        break;

    case NVME_IOCTL_REAP:
        LOG_DBG("NVME_IOCTL_REAP");
        /* Assign user passed parameters to local reap structure */
        reap_data = (struct nvme_reap *)ioctl_param;
        /* call reap inquiry driver routine */
        ret_val = driver_reap_cq(pmetrics_device_element, reap_data);
        break;

    case NVME_IOCTL_GET_DRIVER_METRICS:
        LOG_DBG("NVME_IOCTL_GET_DRIVER_METRICS");
        dnvme_metrics = (struct metrics_driver *)ioctl_param;
        ret_val = copy_to_user(dnvme_metrics, &g_metrics_drv, sizeof(struct
                metrics_driver));
        break;

    case NVME_IOCTL_METABUF_CREATE:
        LOG_DBG("NVME_IOCTL_METABUF_CREATE");
        /* Assign user passed parameters to alloc_size */
        if (ioctl_param > MAX_METABUFF_SIZE) {
            LOG_ERR("Size Exceeds Max(16KB) = 0x%x", (u16)ioctl_param);
            ret_val = -EINVAL;
            break;
        }
        /* Call meta buff create routine */
        ret_val = metabuff_create(pmetrics_device_element, (u16)ioctl_param);
        break;

    case NVME_IOCTL_METABUF_ALLOC:
        LOG_DBG("NVME_IOCTL_METABUF_ALLOC");
        /* Call meta buff allocation routine */
        ret_val = metabuff_alloc(pmetrics_device_element, (u32)ioctl_param);
        break;

    case NVME_IOCTL_METABUF_DELETE:
        LOG_DBG("NVME_IOCTL_METABUF_DELETE");
        /* Call meta buff delete routine */
        ret_val = metabuff_del(pmetrics_device_element, (u32)ioctl_param);
        break;

    case NVME_IOCTL_SET_IRQ:
        LOG_DBG("NVME_IOCTL_SET_IRQ");
        irq_data = (struct interrupts *)ioctl_param;
        LOG_DBG("IRQ Scheme = %d", irq_data->irq_type);
        /* Call set irq routine to set new interrupt scheme */
        ret_val = nvme_set_irq(pmetrics_device_element, irq_data);

        break;

    case NVME_IOCTL_GET_DEVICE_METRICS:
        LOG_DBG("NVME_IOCTL_GET_DEVICE_METRICS");
        dev_metrics = (struct public_metrics_dev *)ioctl_param;
        ret_val = copy_to_user(dev_metrics, &pmetrics_device_element->
            metrics_device->public_dev, sizeof(struct public_metrics_dev));
        break;

    default:
        LOG_DBG("Unknown IOCTL");
        break;
    }

ictl_exit:
    /* Unlock the device */
    unlock_device(pmetrics_device_element);
    return ret_val;
}

/*
 *  Module Exit code.
 *  dnvme_exit - Perform clean exit
 */
static void __exit dnvme_exit(void)
{
    struct pci_dev *pdev;
    int bars;
    struct  metrics_device_list *pmetrics_device_element;  /* Metrics device */

    /* Loop through the devices available in the metrics list */
    list_for_each_entry(pmetrics_device_element, &metrics_dev_ll,
            metrics_device_hd) {
        pdev = pmetrics_device_element->metrics_device->private_dev.pdev;
        /* Grab the Mutex for this device in the linked list */
        mutex_lock(&pmetrics_device_element->metrics_mtx);
        /* Free up the DMA pool */
        destroy_dma_pool(pmetrics_device_element->metrics_device);
        /* Disable the NVME controller */
        nvme_disable(pmetrics_device_element, ST_DISABLE_COMPLETELY);
        /* unlock the device mutex before destroying */
        mutex_unlock(&pmetrics_device_element->metrics_mtx);
        /* destroy all mutexes */
        mutex_destroy(pmetrics_device_element->metrics_mtx);
        mutex_destroy(pmetrics_device_element->irq_process->irq_track_mtx);
        /* Release the seleted PCI regions that were reserved */
        bars = pci_select_bars(pdev, IORESOURCE_MEM);
        pci_release_selected_regions(pdev, bars);
        /* free up the cq linked list */
        list_del(&pmetrics_device_element->metrics_cq_list);
        /* free up the sq linked list */
        list_del(&pmetrics_device_element->metrics_sq_list);
    }

    /* free up the device linked list */
    list_del(&metrics_dev_ll);

    /* Driver clean up */
    pci_unregister_driver(&dnvme_pci_driver);
    unregister_chrdev(NVME_MAJOR, NVME_DEVICE_NAME);
    device_del(device);
    class_destroy(class_nvme);
    cdev_del(char_dev);

    LOG_DBG("dnvme driver Exited...Bye!!");
}

/*
 *  Driver Module Calls.
 */
MODULE_DESCRIPTION("Kernel Device Driver for NVME PCI Express card");
MODULE_AUTHOR("T Sravan Kumar <sravan.kumar.thokala@intel.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION_STR(DRIVER_VERSION));
MODULE_ALIAS("platform:"DRV_NAME);

module_init(dnvme_init);
module_exit(dnvme_exit);
