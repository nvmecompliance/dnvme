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
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "dnvme_ioctls.h"
#include "dnvme_interface.h"
#include "definitions.h"
#include "dnvme_reg.h"
#include "sysfuncproto.h"
#include "sysdnvme.h"
#include "dnvme_sts_chk.h"
#include "dnvme_queue.h"
#include "dnvme_cmds.h"
#include "dnvme_ds.h"
#include "dnvme_irq.h"

/*
 *  device_status_chk  - Generic error checking function
 *  which checks error registers and set kernel
 *  alert if a error is detected.
 */
int device_status_chk(struct  metrics_device_list *pmetrics_device_element,
        int *status)
{
    /* Local variable declaration. */
    u16 data; /* Unsigned 16 bit data. */
    int ret_code;
    int ker_status = -1;

    /*
    * Read a word (16bit value) from the configuration register
    * and pass it to user.
    */
    ret_code = pci_read_config_word(pmetrics_device_element->
        metrics_device->private_dev.pdev, PCI_DEVICE_STATUS, &data);
    /*
    * Check the return code to know if pci read is success.
    */
    if (ret_code < 0) {
        LOG_ERR("pci_read_config failed in driver error check");
        return -EINVAL;
    }

    /*
    * Get the Device Status from the PCI Header.
    */
    ker_status = device_status_pci(data);
    if (ker_status == SUCCESS) {
        LOG_DBG("PCI Device Status SUCCESS (STS)");
    } else {
        LOG_ERR("PCI Device Status FAIL (STS)");
    }

    ker_status = (ker_status == SUCCESS) ? device_status_next
        (pmetrics_device_element->metrics_device->private_dev.pdev) : FAIL;
    if (ker_status == SUCCESS) {
        LOG_DBG("NEXT Capability Status SUCCESS.");
    } else {
        LOG_ERR("NEXT Capabilty status FAIL");
    }

    ker_status = (ker_status == SUCCESS) ? nvme_controller_status
        (pmetrics_device_element->metrics_device->private_dev.bar0) : FAIL;
    if (ker_status == SUCCESS) {
        LOG_DBG("NVME Controller Status SUCCESS (CSTS)");
    } else {
        LOG_ERR("iNVME Controller Status FAIL (CSTS)");
    }

    if (copy_to_user((void __user *) status, &ker_status, sizeof(int))) {
        LOG_ERR("Invalid copy to user space");
        return -EFAULT;
    }
    return 0;
}

/*
 *   driver_genric_read - Generic Read functionality for reading
 *   NVME PCIe registers and memory mapped address
 */
int driver_generic_read(struct rw_generic *nvme_data,
    struct  metrics_device_list *pmetrics_device_element)
{
    u16 index;
    int ret_code = -EINVAL; /* to verify if return code is success. */
    struct pci_dev *pdev;
    struct nvme_device *nvme_dev;
    void *datap;


    /* Allocating memory for the read in kernel space */
    datap = kmalloc(nvme_data->nBytes, GFP_KERNEL | __GFP_ZERO);
    if (!datap) {
        LOG_ERR("Unable to allocate kernel memory");
        return -ENOMEM;
    }

    /* get the device from the list */
    pdev = pmetrics_device_element->metrics_device->private_dev.pdev;
    nvme_dev = pmetrics_device_element->metrics_device;

    switch (nvme_data->type) {

    case NVMEIO_PCI_HDR:
        LOG_DBG("User App request to read  the PCI Header Space");
        LOG_DBG("Read request for bytes = 0x%x", nvme_data->nBytes);
        LOG_DBG("off:acc= 0x%x:0x%2x", nvme_data->offset, nvme_data->acc_type);

        for (index = 0; index < nvme_data->nBytes; ) {
            LOG_DBG("Reading for index = 0x%x", index);
            LOG_DBG("PCI Offset = 0x%x", nvme_data->offset + index);

            if ((nvme_data->offset + index) > MAX_PCI_EXPRESS_CFG) {
                LOG_ERR("Offset is more than the PCI Express");
                LOG_ERR("Extended config space...");
                ret_code = -EINVAL;
                goto err;
            }

            /*
            * Check the access width and access the PCI space as per
            * the requested width.
            */
            if ((nvme_data->acc_type == DWORD_LEN)
                && (nvme_data->nBytes % 4 == 0)) {

                ret_code = pci_read_config_dword(pdev,
                    (nvme_data->offset + index), (datap + index));
                LOG_DBG("Reading PCI offset, data = 0x%x, 0x%x",
                    (nvme_data->offset + index), *(u32 *)(datap + index));
                index += 4;

            } else if ((nvme_data->acc_type == WORD_LEN)
                && (nvme_data->nBytes % 2 == 0)) {

                ret_code = pci_read_config_word(pdev,
                    (nvme_data->offset + index), (datap + index));
                LOG_DBG("Reading PCI offset, data = 0x%x, 0x%x",
                    (nvme_data->offset + index), *(u16 *)(datap + index));
                index += 2;

            } else if (nvme_data->acc_type == BYTE_LEN) {

                ret_code = pci_read_config_byte(pdev,
                    (nvme_data->offset + index), (datap + index));
                LOG_DBG("Reading PCI offset, data = 0x%x, 0x%x",
                    (nvme_data->offset + index), *(u8 *)(datap + index));
                index++;

            } else {
                LOG_ERR("PCI space accessed by DWORD, WORD or BYTE");
                ret_code = -EINVAL;
                goto err;
            }

            if (ret_code < 0) {
                LOG_ERR("pci_read_config failed");
                goto err;
            }
        }
        break;

    case NVMEIO_BAR01:
        if ((nvme_data->acc_type == DWORD_LEN) &&
            (((nvme_data->nBytes % 4) != 0) ||
            ((nvme_data->offset % 4) != 0))) {

            LOG_ERR("Offset must be DWORD align, nBytes must be DWORD align");
            ret_code = -EINVAL;
            goto err;
        } else if ((nvme_data->acc_type == QUAD_LEN) &&
            (((nvme_data->nBytes % 8) != 0) ||
            ((nvme_data->offset % 4) != 0))) {

            LOG_ERR("Offset must be DWORD align, nBytes must be QWORD align");
            ret_code = -EINVAL;
            goto err;
        } else if ((nvme_data->acc_type == WORD_LEN) &&
            (((nvme_data->nBytes % 2) != 0) ||
            ((nvme_data->offset % 2) != 0))) {

            LOG_ERR("Offset must be WORD align, nBytes must be WORD align");
            ret_code = -EINVAL;
            goto err;
        }

        ret_code = read_nvme_reg_generic(nvme_dev->private_dev.bar0,
            datap, nvme_data->nBytes, nvme_data->offset, nvme_data->acc_type);
        if (ret_code < 0) {
            LOG_ERR("Read NVME Space failed");
            goto err;
        }
        break;

    default:
        LOG_DBG("Could not find switch case using default");
        ret_code = -EINVAL;
        goto err;
    }

    if (copy_to_user((void __user *) nvme_data->buffer, datap,
        nvme_data->nBytes)) {

        LOG_ERR("Invalid copy to user space");
        ret_code = -EFAULT;
        goto err;
    }

err:
    kfree(datap);
    return ret_code;
}

/*
 *   driver_generic_write - Generic write function for
 *   NVME PCIe registers and memory mapped address
 */
int driver_generic_write(struct rw_generic *nvme_data,
        struct  metrics_device_list *pmetrics_device_element)
{
    u16 index = 0; /* Index to loop */
    int ret_code = -EINVAL; /* return code to verify if written success. */
    struct pci_dev *pdev;
    struct nvme_device *nvme_dev;
    void *datap;

    LOG_DBG("Inside Generic write Funtion of the IOCTLs");

    /* get the device from the list */
    pdev = pmetrics_device_element->metrics_device->private_dev.pdev;
    nvme_dev = pmetrics_device_element->metrics_device;

    /* Allocating memory for the data in kernel space */
    datap = kmalloc(nvme_data->nBytes, GFP_KERNEL | __GFP_ZERO);
    if (!datap) {
        LOG_ERR("Unable to allocate kernel memory");
        return -ENOMEM;
    }

    /* Copying userspace buffer to kernel memory */
    if (copy_from_user(datap, (void __user *) nvme_data->buffer,
        nvme_data->nBytes)) {

        LOG_ERR("Invalid copy from user space");
        ret_code = -EFAULT;
        goto err;
    }

    switch (nvme_data->type) {

    case NVMEIO_PCI_HDR:
        for (index = 0; index < nvme_data->nBytes; index++) {

            /*
            * write user data to pci config space at location
            * indicated by (offset + index) as per access width.
            */
            if ((nvme_data->acc_type == DWORD_LEN)
                && (nvme_data->nBytes % 4 == 0)) {

                ret_code = pci_write_config_dword(pdev,
                    (nvme_data->offset + index), *(u32 *)(datap + index));
                LOG_DBG("Writing to PCI offset, data = 0x%x, 0x%x",
                    (nvme_data->offset + index), *(u32 *)(datap + index));
                index += 4;

            } else if ((nvme_data->acc_type == WORD_LEN)
                && (nvme_data->nBytes % 2 == 0)) {
                ret_code = pci_write_config_word(pdev,
                    (nvme_data->offset + index), *(u16 *)(datap + index));
                LOG_DBG("Writing to PCI offset, data = 0x%x, 0x%x",
                    (nvme_data->offset + index), *(u16 *)(datap + index));
                index += 2;

            } else if (nvme_data->acc_type == BYTE_LEN) {

                ret_code = pci_write_config_byte(pdev,
                    (nvme_data->offset + index), *(u8 *)(datap + index));
                LOG_DBG("Writing to PCI offset, data = 0x%x, 0x%x",
                    (nvme_data->offset + index), *(u8 *)(datap + index));
                index++;

            } else {
                LOG_ERR("PCI space accessed by DWORD, WORD or BYTE");
                ret_code = -EINVAL;
                goto err;
            }

            if (ret_code < 0) {
                LOG_ERR("pci_write_config failed");
                goto err;
            }
        }
        break;

    case NVMEIO_BAR01:
        if ((nvme_data->acc_type == DWORD_LEN) &&
            (((nvme_data->nBytes % 4) != 0) ||
            ((nvme_data->offset % 4) != 0))) {

            LOG_ERR("Offset must be DWORD align, nBytes must be DWORD align");
            ret_code = -EINVAL;
            goto err;
        } else if ((nvme_data->acc_type == QUAD_LEN) &&
            (((nvme_data->nBytes % 8) != 0) ||
            ((nvme_data->offset % 4) != 0))) {

            LOG_ERR("Offset must be DWORD align, nBytes must be QWORD align");
            ret_code = -EINVAL;
            goto err;
        } else if ((nvme_data->acc_type == WORD_LEN) &&
            (((nvme_data->nBytes % 2) != 0) ||
            ((nvme_data->offset % 2) != 0))) {

            LOG_ERR("Offset must be WORD align, nBytes must be WORD align");
            ret_code = -EINVAL;
            goto err;
        }

        ret_code = write_nvme_reg_generic(nvme_dev->private_dev.bar0,
            datap, nvme_data->nBytes, nvme_data->offset, nvme_data->acc_type);
        if (ret_code < 0) {
            LOG_ERR("Write NVME Space failed");
            goto err;
        }
        break;

    default:
        LOG_DBG("Could not find switch case using default");
        ret_code = -EINVAL;
        break;
    }

err:
    kfree(datap);
    return ret_code;
}

/*
 *   driver_create_asq - Driver Admin Submission Queue creation routine
 *   from Q create ioctl.
 */
int driver_create_asq(struct nvme_create_admn_q *create_admn_q,
    struct  metrics_device_list *pmetrics_device_element)
{
    int ret_code = -EINVAL; /* Set return value to error                */
    u8 admn_id = 0;         /* Always admin ID is 0                     */
    struct  metrics_sq  *pmetrics_sq_list = NULL; /* SQ linked list     */
    struct nvme_device *pnvme_dev;

    /* get the device from the list */
    pnvme_dev = pmetrics_device_element->metrics_device;

    if (readl(&pnvme_dev->private_dev.ctrlr_regs->cc) & NVME_CC_ENABLE) {
        LOG_ERR("Device enable bit is set already!!");
        goto asq_exit;
    }

    /* Search if admin sq already exists. */
    LOG_DBG("Searching for Node in the sq_list_hd...");
    ret_code = identify_unique(admn_id, METRICS_SQ, pmetrics_device_element);
    if (ret_code != SUCCESS) {
        LOG_ERR("ASQ already exists..");
        goto asq_exit; /* use invalid return code */
    }

    LOG_DBG("Alloc mem for a node...");
    pmetrics_sq_list = kmalloc(sizeof(struct metrics_sq), GFP_KERNEL);
    if (pmetrics_sq_list == NULL) {
        LOG_ERR("failed mem alloc in ASQ creation.");
        ret_code = -ENOMEM;
        goto asq_exit;
    }
    /* Reset the data for this node */
    memset(pmetrics_sq_list, 0, sizeof(struct metrics_sq));

    /* Set Admin Q Id. */
    pmetrics_sq_list->public_sq.sq_id = admn_id;
    pmetrics_sq_list->public_sq.elements = create_admn_q->elements;
    /* Admin SQ is always associated with Admin CQ. */
    pmetrics_sq_list->public_sq.cq_id = admn_id;

    /* Adding Command Tracking list */
    INIT_LIST_HEAD(&(pmetrics_sq_list->private_sq.cmd_track_list));

    /* Call dma allocation, creation of contiguous memory for ASQ */
    ret_code = create_admn_sq(pnvme_dev, pmetrics_sq_list->public_sq.elements,
        pmetrics_sq_list);
    if (ret_code != SUCCESS) {
        LOG_ERR("Failed Admin Q creation!!");
        goto asq_exit;
    }

    LOG_DBG("Adding node for Admin SQ to the list.");
    /* Add an element to the end of the list */
    list_add_tail(&pmetrics_sq_list->sq_list_hd,
            &pmetrics_device_element->metrics_sq_list);
    return ret_code;

asq_exit:
    if (pmetrics_sq_list != NULL) {
        kfree(pmetrics_sq_list);
    }
    return ret_code;
}

/*
 *  driver_create_acq - Driver Admin Completion Queue creation routine
 *  from Q create ioctl.
 */
int driver_create_acq(struct nvme_create_admn_q *create_admn_q,
        struct  metrics_device_list *pmetrics_device_element)
{
    int ret_code = -EINVAL; /* return value is invalid              */
    u8 admn_id = 0;         /* Always Admin ID is zero.             */
    struct  metrics_cq  *pmetrics_cq_list = NULL; /* CQ linked list */
    struct nvme_device *pnvme_dev;

    /* get the device from the list */
    pnvme_dev = pmetrics_device_element->metrics_device;

    if (readl(&pnvme_dev->private_dev.ctrlr_regs->cc) & NVME_CC_ENABLE) {
        LOG_ERR("Device enable bit is set already!!");
        goto acq_exit;
    }

    /* Search if admin sq already exists. */
    LOG_DBG("Searching for Node in the cq_list_hd");
    ret_code = identify_unique(admn_id, METRICS_CQ, pmetrics_device_element);
    if (ret_code != SUCCESS) {
        LOG_ERR("ACQ already exists");
        goto acq_exit;
    }

    LOG_DBG("Alloc mem for a Admin CQ node.");
    pmetrics_cq_list = kmalloc(sizeof(struct metrics_cq), GFP_KERNEL);
    if (pmetrics_cq_list == NULL) {
        LOG_ERR("failed mem alloc in ACQ creation.");
        ret_code = -ENOMEM;
        goto acq_exit;
    }
    /* Reset the data for this node */
    memset(pmetrics_cq_list, 0, sizeof(struct metrics_cq));

    /* Set Admin CQ Id. */
    pmetrics_cq_list->public_cq.q_id = admn_id;
    pmetrics_cq_list->public_cq.elements = create_admn_q->elements;
    pmetrics_cq_list->public_cq.irq_no = 0;
    pmetrics_cq_list->public_cq.irq_enabled = 1;

    LOG_DBG("Adding node for Admin CQ to the list.");

    /* Call dma allocation, creation of contiguous memory for ACQ */
    ret_code = create_admn_cq(pnvme_dev, pmetrics_cq_list->public_cq.elements,
        pmetrics_cq_list);
    if (ret_code != SUCCESS) {
        LOG_ERR("Admin CQ creation failed!!");
        goto acq_exit;
    }

    /* Set the pbit_new_entry value */
    pmetrics_cq_list->public_cq.pbit_new_entry = 1;

    /* Add an element to the end of the list */
    list_add_tail(&pmetrics_cq_list->cq_list_hd,
            &pmetrics_device_element->metrics_cq_list);

    LOG_DBG("Admin CQ successfully created and added to linked list...");
    return ret_code;

acq_exit:
    if (pmetrics_cq_list != NULL) {
        kfree(pmetrics_cq_list);
    }
    return ret_code;
}

/*
 *  Driver Initialization routine before starting to issue ioctls.
 */
int driver_ioctl_init(struct pci_dev *pdev, void __iomem *ctrlrRegs,
    struct metrics_device_list *pmetrics_device_list)
{
    int err;

    pmetrics_device_list->metrics_device =
        kmalloc(sizeof(struct nvme_device), GFP_KERNEL);
    if (pmetrics_device_list->metrics_device == NULL) {
        LOG_ERR("Failed alloc of devel level metric storage area");
        err = -ENOMEM;
        goto driver_ioctl_init_err;
    }

    /* Init linked lists for this device. */
    INIT_LIST_HEAD(&(pmetrics_device_list->metrics_sq_list));
    INIT_LIST_HEAD(&(pmetrics_device_list->metrics_cq_list));
    INIT_LIST_HEAD(&(pmetrics_device_list->metrics_meta.meta_trk_list));
    INIT_LIST_HEAD(&(pmetrics_device_list->irq_process.irq_track_list));
    INIT_LIST_HEAD(&(pmetrics_device_list->irq_process.wrk_item_list));

    mutex_init(&pmetrics_device_list->irq_process.irq_track_mtx);
    pmetrics_device_list->metrics_device->private_dev.pdev = pdev;
    pmetrics_device_list->metrics_device->private_dev.bar0 = ctrlrRegs;
    pmetrics_device_list->metrics_device->private_dev.ctrlr_regs = ctrlrRegs;
    pmetrics_device_list->metrics_device->private_dev.dmadev =
        &pmetrics_device_list->metrics_device->private_dev.pdev->dev;

    /* Used to create Coherent DMA mapping for PRP List */
    pmetrics_device_list->metrics_meta.meta_dmapool_ptr = NULL;
    pmetrics_device_list->metrics_device->private_dev.prp_page_pool =
        dma_pool_create("prp page",
        &pmetrics_device_list->metrics_device->private_dev.pdev->dev,
        PAGE_SIZE, PAGE_SIZE, 0);
    if (pmetrics_device_list->metrics_device->private_dev.prp_page_pool ==
        NULL) {

        LOG_ERR("Creating DMA Pool failed");
        err = -ENOMEM;
        goto driver_ioctl_init_err;
     }

    /* Spinlock to protect from kernel preemption in ISR handler */
    spin_lock_init(&pmetrics_device_list->irq_process.isr_spin_lock);

    /* Initialize irq scheme to INT_NONE and perform cleanup of all lists */
    err = init_irq_lists(pmetrics_device_list, INT_NONE);
    if (err < 0) {
        LOG_ERR("IRQ track initialization failed...");
        goto driver_ioctl_init_err;
    }
    return SUCCESS;


driver_ioctl_init_err:
    if (pmetrics_device_list->metrics_device != NULL) {
        kfree(pmetrics_device_list->metrics_device);
    }
    if (pmetrics_device_list->metrics_device->private_dev.prp_page_pool !=
        NULL) {

        dma_pool_destroy(
            pmetrics_device_list->metrics_device->private_dev.prp_page_pool);
    }
    return err;
}

/*
 * Allocate a dma pool for the requested size. Initialize the DMA pool pointer
 * with DWORD alignment and associate it with the active device.
 */
int metabuff_create(struct metrics_device_list *pmetrics_device_elem,
    u16 alloc_size)
{
    /* First Check if the meta pool already exists */
    if (pmetrics_device_elem->metrics_meta.meta_dmapool_ptr != NULL) {
        if (alloc_size == pmetrics_device_elem->metrics_meta.meta_buf_size) {
            return SUCCESS;
        }
        LOG_ERR("Meta Pool already exists, of a different size");
        return -EINVAL;
    }

    /* Create coherent DMA mapping for meta data buffer creation */
    pmetrics_device_elem->metrics_meta.meta_dmapool_ptr = dma_pool_create
        ("meta_buff", &pmetrics_device_elem->metrics_device->
        private_dev.pdev->dev, sizeof(u32), alloc_size, 0);
    if (pmetrics_device_elem->metrics_meta.meta_dmapool_ptr == NULL) {
        LOG_ERR("Creation of DMA Pool failed");
        return -ENOMEM;
    }
    pmetrics_device_elem->metrics_meta.meta_buf_size = alloc_size;
    return SUCCESS;
}

/*
 * alloc a meta buffer node when user request and allocate a consistent
 * dma memory from the meta dma pool. Add this node into the meta data
 * linked list.
 */
int metabuff_alloc(struct metrics_device_list *pmetrics_device_elem,
        u32 meta_id)
{
    struct metrics_meta *pmeta_data = NULL;
    int ret_val = SUCCESS;

    /* Check if parameters passed to this function are valid */
    if (pmetrics_device_elem->metrics_meta.meta_dmapool_ptr == NULL) {
        LOG_ERR("Meta data pool is not created...");
        LOG_ERR("Call to Create the meta data pool first...");
        return -EINVAL;
    }
    /* Check if this id is already created. */
    pmeta_data = find_meta_node(pmetrics_device_elem, meta_id);
    if (pmeta_data != NULL) {
        LOG_ERR("Meta ID = %d already exists!!", pmeta_data->meta_id);
        return -EINVAL;
    }

    /* Allocate memory to metrics_meta for each node */
    pmeta_data = kmalloc(sizeof(struct metrics_meta), GFP_KERNEL);
    if (pmeta_data == NULL) {
        LOG_ERR("Allocation of Meta data mem failed");
        return -ENOMEM;
    }

    /* Assign the user passed id for tracking this meta id */
    pmeta_data->meta_id = meta_id;
    /* Allocated the dma memory and assign to vir_kern_addr */
    pmeta_data->vir_kern_addr = dma_pool_alloc(pmetrics_device_elem->
        metrics_meta.meta_dmapool_ptr, GFP_ATOMIC, &pmeta_data->meta_dma_addr);
    if (pmeta_data->vir_kern_addr == NULL) {
        LOG_ERR("DMA Allocation failed for meta data buffer.");
        ret_val = -ENOMEM;
        goto meta_alloc_out;
    }
    /* Add the meta data node into the linked list */
    list_add_tail(&pmeta_data->meta_list_hd, &pmetrics_device_elem->
        metrics_meta.meta_trk_list);
    return ret_val;

meta_alloc_out:
    kfree(pmeta_data);
    return ret_val;
}

/*
 * Delete the meta buffer node for given meta id from the linked list.
 * First Free the dma pool allocated memory then delete the entry from the
 * linked list and finally free the node memory from the kernel.
 */
int metabuff_del(struct metrics_device_list *pmetrics_device_element,
    u32 meta_id)
{
    struct metrics_meta *pmeta_data = NULL;

    /* Check if invalid parameters are passed */
    if (pmetrics_device_element->metrics_meta.meta_dmapool_ptr == NULL) {
        LOG_ERR("Meta data pool is not created.");
        LOG_ERR("Call to Create the meta data pool first");
        return -EINVAL;
    }
    /* check if meta node id exists */
    pmeta_data = find_meta_node(pmetrics_device_element, meta_id);
    if (pmeta_data == NULL) {
        LOG_ERR("Meta ID does not exists!!");
        return SUCCESS;
    }
    /* free the dma memory if exists */
    if (pmeta_data->vir_kern_addr != NULL) {
        dma_pool_free(pmetrics_device_element->metrics_meta.meta_dmapool_ptr,
            pmeta_data->vir_kern_addr, pmeta_data->meta_dma_addr);
    }
    /* Remove from the linked list and free the node */
    list_del(&pmeta_data->meta_list_hd);
    kfree(pmeta_data);
    return SUCCESS;
}

/*
 * deallocate_mb - This function will start freeing up the memory and
 * nodes for the meta buffers allocated during the alloc and create meta.
 */
void deallocate_mb(struct  metrics_device_list *pmetrics_device)
{
    struct metrics_meta *pmeta_data = NULL;
    struct metrics_meta *pmeta_data_next = NULL;

    /* do not assume the node exists always */
    if (pmetrics_device->metrics_meta.meta_dmapool_ptr == NULL) {
        LOG_DBG("Meta node is not allocated..");
        return;
    }
    /* Loop for each meta data node */
    list_for_each_entry_safe(pmeta_data, pmeta_data_next,
        &(pmetrics_device->metrics_meta.meta_trk_list), meta_list_hd) {

        /* free the dma memory if exists */
        if (pmeta_data->vir_kern_addr != NULL) {
            dma_pool_free(pmetrics_device->metrics_meta.meta_dmapool_ptr,
                pmeta_data->vir_kern_addr, pmeta_data->meta_dma_addr);
        }
        /* Remove from the linked list and free the node */
        list_del(&pmeta_data->meta_list_hd);
        kfree(pmeta_data);
    }

    /* check if it has dma pool created then destroy */
    if (pmetrics_device->metrics_meta.meta_dmapool_ptr != NULL) {
        dma_pool_destroy(pmetrics_device->metrics_meta.meta_dmapool_ptr);
        pmetrics_device->metrics_meta.meta_dmapool_ptr = NULL;
    }
    pmetrics_device->metrics_meta.meta_buf_size = 0;

    /* Prepare a clean list, empty, ready for next use */
    INIT_LIST_HEAD(&pmetrics_device->metrics_meta.meta_trk_list);
}

/*
 *  driver_send_64b - Routine for sending 64 bytes command into
 *  admin/IO SQ/CQ's
 */
int driver_send_64b(struct  metrics_device_list *pmetrics_device,
    struct nvme_64b_send *nvme_64b_send)
{
    /* ret code to verify status of sending 64 bytes command */
    int ret_code = -EINVAL;
    u32 cmd_buf_size = 0; /* Size of command buffer */
    /* Particular SQ from linked list of SQ's for device */
    struct  metrics_sq  *pmetrics_sq;
    /* SQ represented by the CMD.QID */
    struct  metrics_sq  *p_cmd_sq;
    /* Particular CQ (within CMD) from linked list of Q's for device */
    struct  metrics_cq  *p_cmd_cq;
    /* Strucutre describing the meta buf */
    struct metrics_meta *meta_buf;
    /* Kernel space memory for passed in command */
    void *nvme_cmd_ker;
    /* Pointer to passed in command DW0-DW9 */
    struct nvme_gen_cmd *nvme_gen_cmd;
    /* Pointer to Gen IOSQ command */
    struct nvme_create_sq *nvme_create_sq;
    /* Pointer to Gen IOCQ command */
    struct nvme_create_cq *nvme_create_cq;
    /* Pointer to Delete IO Q command */
    struct nvme_del_q *nvme_del_q;
    /* Void * pointer to check validity of Queues */
    void *q_ptr = NULL;
    struct nvme_prps prps; /* Pointer to PRP List */

    /* Initial invalid arguments checking */
    if (NULL == nvme_64b_send->cmd_buf_ptr) {
        LOG_ERR("Command Buffer does not exist");
        goto ret;
    } else if ((nvme_64b_send->data_buf_size != 0 &&
        NULL == nvme_64b_send->data_buf_ptr) ||
        (nvme_64b_send->data_buf_size == 0 &&
        NULL != nvme_64b_send->data_buf_ptr)) {

        LOG_ERR("Data buffer size and data buffer inconsistent");
        goto ret;
    }

    /* Get the required SQ through which command should be sent */
    list_for_each_entry(pmetrics_sq, &pmetrics_device->metrics_sq_list,
        sq_list_hd) {

        if (nvme_64b_send->q_id == pmetrics_sq->public_sq.sq_id) {
            /* Learn the command size */
            cmd_buf_size = (pmetrics_sq->private_sq.size /
                pmetrics_sq->public_sq.elements);
            break;
        }
    }

    if (!cmd_buf_size) {
        ret_code = -EPERM;
        LOG_ERR("Required SQ node for sending the command not found");
        goto ret;
    }

    /* Check for SQ is full */
    if ((((u32)pmetrics_sq->public_sq.tail_ptr_virt + 1UL) %
        pmetrics_sq->public_sq.elements) ==
        (u32)pmetrics_sq->public_sq.head_ptr) {

        LOG_ERR("SQ is full");
        ret_code = -EPERM;
        goto ret;
    }

    /* Allocating memory for the command in kernel space */
    nvme_cmd_ker = kmalloc(cmd_buf_size, GFP_ATOMIC | __GFP_ZERO);
    if (!nvme_cmd_ker) {
        LOG_ERR("Unable to allocate kernel memory");
        ret_code = -ENOMEM;
        goto ret;
    }

    /* Copying userspace buffer to kernel memory */
    if (copy_from_user(nvme_cmd_ker,
        (void __user *)nvme_64b_send->cmd_buf_ptr, cmd_buf_size)) {

        LOG_ERR("Invalid copy from user space");
        ret_code = -EFAULT;
        goto free;
    }

    nvme_gen_cmd = (struct nvme_gen_cmd *)nvme_cmd_ker;
    memset(&prps, 0, sizeof(prps));

    /* Copy and Increment the CMD ID, copy back to user spavce so can see ID */
    nvme_gen_cmd->command_id = pmetrics_sq->private_sq.unique_cmd_id++;
    if (copy_to_user((void __user *) (nvme_64b_send->cmd_buf_ptr + 0x02),
        &nvme_gen_cmd->command_id, UNIQUE_ID)) {

        LOG_ERR("Invalid copy to user space");
        ret_code = -EFAULT;
        goto err;
    }

    /* Handling metabuffer */
    if (nvme_64b_send->bit_mask & MASK_MPTR) {
        meta_buf = find_meta_node(pmetrics_device, nvme_64b_send->meta_buf_id);
        if (NULL == meta_buf) {
            LOG_ERR("Meta Buff ID not found");
            ret_code = -EINVAL;
            goto err;
        }
        /* Add the required information to the command */
        nvme_gen_cmd->metadata = cpu_to_le64(meta_buf->meta_dma_addr);
        LOG_DBG("Metadata address: 0x%llx", nvme_gen_cmd->metadata);
    }

    /* Special handling for opcodes 0x00,0x01,0x04 and 0x05 of Admin cmd set */
    if (nvme_64b_send->cmd_set == CMD_ADMIN && nvme_gen_cmd->opcode == 0x01) {
        /* Create IOSQ command */
        nvme_create_sq = (struct nvme_create_sq *) nvme_cmd_ker;

        /* Get the required SQ from the global linked list from CMD.DW10.QID */
        list_for_each_entry(p_cmd_sq, &pmetrics_device->metrics_sq_list,
            sq_list_hd) {
            if (nvme_create_sq->sqid == p_cmd_sq->public_sq.sq_id) {
                q_ptr = (struct  metrics_sq  *)p_cmd_sq;
                break;
            }
        }
        if (q_ptr == NULL) {
            ret_code = -EPERM;
            LOG_ERR("SQID node present in create SQ, but lookup found nothing");
            goto err;
        }

        /* Sanity Checks */
        if (((nvme_create_sq->sq_flags & CDW11_PC) &&
            (p_cmd_sq->private_sq.contig == 0)) ||
            (!(nvme_create_sq->sq_flags & CDW11_PC) &&
            (p_cmd_sq->private_sq.contig != 0))) {

            LOG_DBG("Contig flag out of sync with what cmd says");
            goto data_err;
        } else if ((p_cmd_sq->private_sq.contig == 0 &&
            nvme_64b_send->data_buf_ptr == NULL) ||
            (p_cmd_sq->private_sq.contig != 0 &&
            p_cmd_sq->private_sq.vir_kern_addr == NULL)) {

            LOG_DBG("Contig flag out of sync with what cmd says");
            goto data_err;
        } else if ((p_cmd_sq->private_sq.bit_mask & UNIQUE_QID_FLAG) == 0) {
            /* Avoid duplicate Queue creation */
            LOG_ERR("Required Queue already created!");
            ret_code = -EINVAL;
            goto err;
        }

        if (p_cmd_sq->private_sq.contig == 0) {
            /* Creation of Discontiguous IO SQ */
            if (p_cmd_sq->private_sq.size != nvme_64b_send->data_buf_size) {
                LOG_DBG("Contig flag out of sync with what cmd says");
                goto data_err;
            }
            ret_code = prep_send64b_cmd(pmetrics_device->metrics_device,
                pmetrics_sq, nvme_64b_send, &prps, nvme_gen_cmd,
                nvme_create_sq->sqid, DISCONTG_IO_Q, PRP_PRESENT);
            if (ret_code < 0) {
                LOG_ERR("Failure to prepare 64 byte command");
                goto err;
            }
        } else {
            /* Contig IOSQ creation */
            ret_code = prep_send64b_cmd(pmetrics_device->metrics_device,
                pmetrics_sq, nvme_64b_send, &prps, nvme_gen_cmd,
                nvme_create_sq->sqid, CONTG_IO_Q, PRP_ABSENT);
            if (ret_code < 0) {
                LOG_ERR("Failure to prepare 64 byte command");
                goto err;
            }
            nvme_gen_cmd->prp1 = cpu_to_le64(p_cmd_sq->private_sq.sq_dma_addr);
        }
        /* Fill the persistent entry structure */
        memcpy(&p_cmd_sq->private_sq.prp_persist, &prps, sizeof(prps));

        /* Resetting the unique QID bitmask flag */
        p_cmd_sq->private_sq.bit_mask =
            (p_cmd_sq->private_sq.bit_mask & ~UNIQUE_QID_FLAG);
    } else if (nvme_64b_send->cmd_set == CMD_ADMIN &&
        nvme_gen_cmd->opcode == 0x05) {

        /* Create IOCQ command */
        nvme_create_cq = (struct nvme_create_cq *) nvme_cmd_ker;

        /* Get the required CQ from the global linked list
         * represented by CMD.DW10.QID */
        list_for_each_entry(p_cmd_cq, &pmetrics_device->metrics_cq_list,
            cq_list_hd) {

            if (nvme_create_cq->cqid == p_cmd_cq->public_cq.q_id) {
                q_ptr = (struct  metrics_cq  *)p_cmd_cq;
                break;
            }
        }
        if (q_ptr == NULL) {
            ret_code = -EPERM;
            LOG_ERR("CQID node present in create CQ, but lookup found nothing");
            goto err;
        }

        /* Sanity Checks */
        if (((nvme_create_cq->cq_flags & CDW11_PC) &&
            (p_cmd_cq->private_cq.contig == 0))
            || (!(nvme_create_cq->cq_flags & CDW11_PC) &&
            (p_cmd_cq->private_cq.contig != 0))) {

            LOG_DBG("Contig flag out of sync with what cmd says");
            goto data_err;
        } else if ((p_cmd_cq->private_cq.contig == 0 &&
            nvme_64b_send->data_buf_ptr == NULL) ||
            (p_cmd_cq->private_cq.contig != 0 &&
            p_cmd_cq->private_cq.vir_kern_addr == NULL)) {

            LOG_DBG("Contig flag out of sync with what cmd says");
            goto data_err;
        } else if ((p_cmd_cq->private_cq.bit_mask & UNIQUE_QID_FLAG) == 0) {
            /* Avoid duplicate Queue creation */
            LOG_ERR("Required Queue already created!");
            ret_code = -EINVAL;
            goto err;
        }

        /* Check if interrupts should be enabled for IO CQ */
        if (nvme_create_cq->cq_flags & CDW11_IEN) {
            /* Check the Interrupt scheme set up */
            if (pmetrics_device->metrics_device->public_dev.irq_active.irq_type
                == INT_NONE) {
                LOG_ERR("Interrupt scheme and Create IOCQ cmd out of sync");
                ret_code = -EINVAL;
                goto err;
            }

            /* Setting the IO CQ for the irq_no */
            ret_code = update_cq_irqtrack(pmetrics_device,
                p_cmd_cq->public_cq.q_id, nvme_create_cq->irq_no,
                &p_cmd_cq->public_cq.irq_enabled);
            if (ret_code < 0) {
                LOG_ERR("Setting Irq No = %d failed for IO CQ = %d",
                    nvme_create_cq->irq_no, p_cmd_cq->public_cq.q_id);
                goto err;
            }
            /* Assign irq_no for IO_CQ in public CQ metrics node */
            p_cmd_cq->public_cq.irq_no = nvme_create_cq->irq_no;
        }

        if (p_cmd_cq->private_cq.contig == 0) {
            /* Discontig IOCQ creation */
            if (p_cmd_cq->private_cq.size != nvme_64b_send->data_buf_size) {
                LOG_DBG("Contig flag out of sync with what cmd says");
                goto data_err;
            }
            ret_code = prep_send64b_cmd(pmetrics_device->metrics_device,
                pmetrics_sq, nvme_64b_send, &prps, nvme_gen_cmd,
                nvme_create_cq->cqid, DISCONTG_IO_Q, PRP_PRESENT);
            if (ret_code < 0) {
                LOG_ERR("Failure to prepare 64 byte command");
                goto err;
            }
        } else {
            /* Contig IOCQ creation */
            ret_code = prep_send64b_cmd(pmetrics_device->metrics_device,
                pmetrics_sq, nvme_64b_send, &prps, nvme_gen_cmd,
                nvme_create_cq->cqid, CONTG_IO_Q, PRP_ABSENT);
            if (ret_code < 0) {
                LOG_ERR("Failure to prepare 64 byte command");
                goto err;
            }
            nvme_gen_cmd->prp1 = cpu_to_le64(p_cmd_cq->private_cq.cq_dma_addr);
        }

        /* Fill the persistent entry structure */
        memcpy(&p_cmd_cq->private_cq.prp_persist, &prps, sizeof(prps));

        /* Resetting the unique QID bitmask flag */
        p_cmd_cq->private_cq.bit_mask =
            (p_cmd_cq->private_cq.bit_mask & ~UNIQUE_QID_FLAG);
    } else if (nvme_64b_send->cmd_set == CMD_ADMIN &&
        nvme_gen_cmd->opcode == 0x00) {

        /* Delete IOSQ case */
        nvme_del_q = (struct nvme_del_q *) nvme_cmd_ker;

        if (nvme_64b_send->data_buf_ptr != NULL) {
            LOG_ERR("Invalid argument for opcode 0x00");
            goto err;
        }

        ret_code = prep_send64b_cmd(pmetrics_device->metrics_device,
            pmetrics_sq, nvme_64b_send, &prps, nvme_gen_cmd, nvme_del_q->qid,
             0, PRP_ABSENT);

        if (ret_code < 0) {
            LOG_ERR("Failure to prepare 64 byte command");
            goto err;
        }

    } else if (nvme_64b_send->cmd_set == CMD_ADMIN &&
        nvme_gen_cmd->opcode == 0x04) {

        /* Delete IOCQ case */
        nvme_del_q = (struct nvme_del_q *) nvme_cmd_ker;

        if (nvme_64b_send->data_buf_ptr != NULL) {
            LOG_ERR("Invalid argument for opcode 0x00");
            goto err;
        }

        ret_code = prep_send64b_cmd(pmetrics_device->metrics_device,
            pmetrics_sq, nvme_64b_send, &prps, nvme_gen_cmd, nvme_del_q->qid,
            0, PRP_ABSENT);

        if (ret_code < 0) {
            LOG_ERR("Failure to prepare 64 byte command");
            goto err;
        }

    } else {
        /* For rest of the commands */
        if (nvme_64b_send->data_buf_ptr != NULL) {
            ret_code = prep_send64b_cmd(pmetrics_device->metrics_device,
                pmetrics_sq, nvme_64b_send, &prps, nvme_gen_cmd,
                PERSIST_QID_0, DATA_BUF, PRP_PRESENT);
            if (ret_code < 0) {
                LOG_ERR("Failure to prepare 64 byte command");
                goto err;
            }
        }
    }

    /* Copying the command in to appropriate SQ and handling sync issues */
    if (pmetrics_sq->private_sq.contig) {
        memcpy((pmetrics_sq->private_sq.vir_kern_addr +
            ((u32)pmetrics_sq->public_sq.tail_ptr_virt * cmd_buf_size)),
            nvme_cmd_ker, cmd_buf_size);
    } else {
        memcpy((pmetrics_sq->private_sq.prp_persist.vir_kern_addr +
            ((u32)pmetrics_sq->public_sq.tail_ptr_virt * cmd_buf_size)),
            nvme_cmd_ker, cmd_buf_size);

        dma_sync_sg_for_device(pmetrics_device->metrics_device->
            private_dev.dmadev, pmetrics_sq->private_sq.prp_persist.sg,
            pmetrics_sq->private_sq.prp_persist.num_map_pgs,
            pmetrics_sq->private_sq.prp_persist.data_dir);

    }

    /* Increment the Tail pointer and handle roll over conditions */
    pmetrics_sq->public_sq.tail_ptr_virt =
        (u16)(((u32)pmetrics_sq->public_sq.tail_ptr_virt + 1UL) %
        pmetrics_sq->public_sq.elements);
    kfree(nvme_cmd_ker);
    return 0;

data_err:
    LOG_ERR("Global Metrics inconsistent");
err:
    pmetrics_sq->private_sq.unique_cmd_id--;
free:
    kfree(nvme_cmd_ker);
ret:
    return ret_code;
}

/*
 * get_public_qmetrics will return the q metrics from the global data
 * structures if the q_id send down matches any q_id for this device.
 * If the Q id does not exist in the list then it returns error.
 * This function also returns error when kernel cannot allocate for
 * at-least one element memory of public_sq or public_cq.
 */
int get_public_qmetrics(struct  metrics_device_list *pmetrics_device,
        struct nvme_get_q_metrics *get_q_metrics)
{
    struct  metrics_sq  *pmetrics_sq_node;
    struct  metrics_cq  *pmetrics_cq_node;

    /* Determine the type of Q for which the metrics was needed */
    if (get_q_metrics->type == METRICS_SQ) {
        if (get_q_metrics->nBytes < sizeof(struct nvme_gen_sq)) {
            LOG_ERR("Not sufficient buffer size to copy SQ metrics");
            return -EINVAL;
        }
        pmetrics_sq_node = find_sq(pmetrics_device, get_q_metrics->q_id);
        if (pmetrics_sq_node == NULL) {
            LOG_ERR("SQ ID = %d does not exist", get_q_metrics->q_id);
            return -EBADSLT; /* Invalid slot */
        }
        /* Copy sq public metrics to user space */
        if (copy_to_user((void __user *) get_q_metrics->buffer,
                &pmetrics_sq_node->public_sq, get_q_metrics->nBytes)) {
            LOG_ERR("Invalid copy to user space");
            return -EFAULT;
        }
    } else if (get_q_metrics->type == METRICS_CQ) {
        if (get_q_metrics->nBytes < sizeof(struct nvme_gen_cq)) {
            LOG_ERR("Not sufficient buffer size to copy CQ metrics");
            return -EINVAL;
        }
        /* Find given CQ in list */
        pmetrics_cq_node = find_cq(pmetrics_device, get_q_metrics->q_id);
        if (pmetrics_cq_node == NULL) {
            /* if the control comes here it implies q id not in list */
            LOG_ERR("CQ ID = %d is not in list", get_q_metrics->q_id);
            return -ENODEV;
        }
        /* Copy public cq metrics to user space */
        if (copy_to_user((void __user *) get_q_metrics->buffer,
                &pmetrics_cq_node->public_cq, get_q_metrics->nBytes)) {
            LOG_ERR("Invalid copy to user space");
            return -EFAULT;
        }
    } else {
        /* The Q type is not SQ or CQ, so error out */
        LOG_ERR("Metrics Type: METRICS_SQ/METRICS_CQ only");
        return -EINVAL;
    }

    return SUCCESS;
}

/*
 * identify_unique - This routine checkes if the q_id requested is already
 * in the linked list. If present then it will return error. If its not in
 * the list this returns success.
 */
int identify_unique(u16 q_id, enum metrics_type type,
    struct  metrics_device_list *pmetrics_device_element)
{
    struct  metrics_sq  *pmetrics_sq_list;
    struct  metrics_cq  *pmetrics_cq_list;

    /* Determine the type of Q for which the metrics was needed */
    if (type == METRICS_SQ) {
        LOG_DBG("Searching for Node in the sq_list_hd...");
        pmetrics_sq_list = find_sq(pmetrics_device_element, q_id);
        if (pmetrics_sq_list != NULL) {
            LOG_ERR("SQ ID %d isn't unique", pmetrics_sq_list->public_sq.sq_id);
            return -EINVAL;
        }
    } else if (type == METRICS_CQ) {
        pmetrics_cq_list = find_cq(pmetrics_device_element, q_id);
        if (pmetrics_cq_list != NULL) {
            LOG_ERR("CQ ID %d isn't unique", pmetrics_cq_list->public_cq.q_id);
            return -EINVAL;
        }
    }
    return SUCCESS;
}

/*
 * driver_nvme_prep_sq - This function will try to allocate kernel
 * space for the corresponding unique SQ. If the sq_id is duplicated
 * this will return error to the caller. If the kernel memory is not
 * available then fail and return NOMEM error code.
 */
int driver_nvme_prep_sq(struct nvme_prep_sq *prep_sq,
    struct  metrics_device_list *pmetrics_device_element)
{
    int ret_code;
    struct nvme_device *pnvme_dev;
    struct metrics_sq *pmetrics_sq_node = NULL;

    /* get the device from the list */
    pnvme_dev = pmetrics_device_element->metrics_device;

    ret_code = identify_unique(prep_sq->sq_id, METRICS_SQ,
        pmetrics_device_element);
    if (ret_code != SUCCESS) {
        LOG_ERR("SQ ID is not unique.");
        goto exit_prep_sq;
    }

    if (READQ(&pnvme_dev->private_dev.ctrlr_regs->cap) & REGMASK_CAP_CQR) {
        if (prep_sq->contig == 0) {
            LOG_DBG("Device doesn't support discontig Q memory");
            ret_code = -ENOMEM;
            goto exit_prep_sq;
        }
    }

    LOG_DBG("Allocating SQ node in linked list.");
    pmetrics_sq_node = kmalloc(sizeof(struct metrics_sq), GFP_KERNEL);
    if (pmetrics_sq_node == NULL) {
        LOG_ERR("Failed kernel alloc for SQ metrics node");
        ret_code = -ENOMEM;
        goto exit_prep_sq;
    }

    /* Filling the data elements of sq metrics. */
    memset(pmetrics_sq_node, 0, sizeof(struct metrics_sq));
    pmetrics_sq_node->public_sq.sq_id = prep_sq->sq_id;
    pmetrics_sq_node->public_sq.cq_id = prep_sq->cq_id;
    pmetrics_sq_node->public_sq.elements = prep_sq->elements;
    pmetrics_sq_node->private_sq.contig = prep_sq->contig;

    ret_code = nvme_prepare_sq(pmetrics_sq_node, pnvme_dev);
    if (ret_code < 0) {
        goto exit_prep_sq;
    }

    pmetrics_sq_node->private_sq.bit_mask =
        (pmetrics_sq_node->private_sq.bit_mask | UNIQUE_QID_FLAG);

    /* Adding Command Tracking list */
    INIT_LIST_HEAD(&(pmetrics_sq_node->private_sq.cmd_track_list));

    /* Add this element to the end of the list */
    list_add_tail(&pmetrics_sq_node->sq_list_hd,
        &pmetrics_device_element->metrics_sq_list);
    return SUCCESS;

exit_prep_sq:
    if (pmetrics_sq_node != NULL) {
        kfree(pmetrics_sq_node);
    }
    return ret_code;
}

/*
 * driver_nvme_prep_cq - This function will try to allocate kernel
 * space for the corresponding unique CQ. If the cq_id is duplicated
 * this will return error to the caller. If the kernel memory is not
 * available then fail and return NOMEM error code.
 */
int driver_nvme_prep_cq(struct nvme_prep_cq *prep_cq,
        struct  metrics_device_list *pmetrics_device_element)
{
    int ret_code;
    struct  metrics_cq  *pmetrics_cq_node = NULL;
    struct nvme_device *pnvme_dev;

    /* get the device from the list */
    pnvme_dev = pmetrics_device_element->metrics_device;

    ret_code = identify_unique(prep_cq->cq_id, METRICS_CQ,
        pmetrics_device_element);
    if (ret_code != SUCCESS) {
        LOG_ERR("CQ ID is not unique.");
        goto exit_prep_cq;
    }

    if (READQ(&pnvme_dev->private_dev.ctrlr_regs->cap) & REGMASK_CAP_CQR) {
        if (prep_cq->contig == 0) {
            LOG_DBG("Device doesn't support discontig Q memory");
            ret_code = -ENOMEM;
            goto exit_prep_cq;
        }
    }

    LOG_DBG("Allocating CQ node in linked list.");
    pmetrics_cq_node = kmalloc(sizeof(struct metrics_cq), GFP_KERNEL);
    if (pmetrics_cq_node == NULL) {
        LOG_ERR("Failed kernel alloc for CQ metrics node");
        ret_code = -ENOMEM;
        goto exit_prep_cq;
    }

    /* Filling the data elements of sq metrics. */
    memset(pmetrics_cq_node, 0, sizeof(struct metrics_cq));
    pmetrics_cq_node->public_cq.q_id = prep_cq->cq_id;
    pmetrics_cq_node->public_cq.elements = prep_cq->elements;
    pmetrics_cq_node->public_cq.irq_enabled = 0;
    pmetrics_cq_node->private_cq.contig = prep_cq->contig;

    ret_code = nvme_prepare_cq(pmetrics_cq_node, pnvme_dev);
    if (ret_code < 0) {
        goto exit_prep_cq;
    }

    pmetrics_cq_node->public_cq.pbit_new_entry = 1;
    pmetrics_cq_node->private_cq.bit_mask =
        (pmetrics_cq_node->private_cq.bit_mask | UNIQUE_QID_FLAG);

    /* Add this element to the end of the list */
    list_add_tail(&pmetrics_cq_node->cq_list_hd,
        &pmetrics_device_element->metrics_cq_list);
    return SUCCESS;

exit_prep_cq:
    if (pmetrics_cq_node != NULL) {
        kfree(pmetrics_cq_node);
    }
    return ret_code;
}
