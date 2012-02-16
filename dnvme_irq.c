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
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#include <dnvme_irq.h>

/* Static function declarations used for setting interrupt schemes. */
static int validate_irq_inputs(struct metrics_device_list
    *pmetrics_device_elem, struct interrupts *irq_new,
        struct msix_info *pmsix_tbl_info);
static int set_msix(struct metrics_device_list *pmetrics_device_elem,
    u16 num_irqs, struct msix_info *pmsix_tbl_info);
static int set_msi_single(struct metrics_device_list *pmetrics_device_elem);
static int set_msi_multi(struct metrics_device_list *pmetrics_device_elem,
    u16 num_irqs);
static int add_irq_node(struct  metrics_device_list *pmetrics_device_elem,
    u32 int_vec, u16 irq_no);
static void bh_callback(struct work_struct *work);
static void dealloc_all_icqs(struct  irq_track *pirq_trk_list);
static int disable_active_irq(struct metrics_device_list
    *pmetrics_device_elem, enum nvme_irq_type  irq_active);
static int check_cntlr_cap(struct pci_dev *pdev, enum nvme_irq_type cap_type,
    u16 *offset);
static void inc_isr_count(struct irq_processing *pirq_process,
    u16 irq_no);
static struct irq_track *find_irq_node(
    struct  metrics_device_list *pmetrics_device_elem, u16 irq_no);
static struct irq_cq_track *find_icq_node(struct  irq_track *pirq_node,
    u16 cq_id);
static struct work_container *get_work_item(struct  irq_processing
    *pirq_process, u32 int_vec);
static void nvme_disable_pin(struct pci_dev *dev);
static int update_msixptr(struct  metrics_device_list
    *pmetrics_device_elem, u16 offset, struct msix_info *pmsix_tbl_info);
static void set_msix_mask_bit(u8 *irq_msixptr, u16 irq_no, u32 flag);
static int work_queue_init(struct irq_processing *pirq_process);
static int add_wk_item(struct irq_processing *pirq_process,
    u32 int_vec, u16 irq_no);

/*
 * nvme_set_irq will set the new interrupt scheme for this device regardless
 * of the current irq scheme that is active for this device. It also validates
 * if the inputs given for setting up new scheme are within bounds.
 * NOTE: The controller should be disabled before setting up new scheme.
 */
int nvme_set_irq(struct metrics_device_list *pmetrics_device_elem,
    struct interrupts *irq_new)
{
    int ret_val = SUCCESS;
    /* pointer to the metrics device */
    struct nvme_device *pnvme_dev = pmetrics_device_elem->metrics_device;
    struct msix_info msix_tbl_info; /* Info for MSI-X tables */

    /* First validate if the inputs given are correct */
    ret_val = validate_irq_inputs(pmetrics_device_elem, irq_new,
        &msix_tbl_info);
    if (ret_val < 0) {
        LOG_ERR("Invalid inputs set or device is not disabled..");
        return ret_val;
    }

    /* lock onto IRQ linked list mutex as we would access the IRQ list */
    mutex_lock(&pmetrics_device_elem->irq_process.irq_track_mtx);

    /* disable the current IRQ scheme */
    ret_val = disable_active_irq(pmetrics_device_elem, pnvme_dev->
            public_dev.irq_active.irq_type);
    if (ret_val < 0) {
        LOG_ERR("Reset of IRQ to INT_NONE failed...");
        goto mutex_unlck;
    }
    /* initialize work queue */
    ret_val = work_queue_init(&pmetrics_device_elem->irq_process);
    if (ret_val < 0) {
        LOG_ERR("Failed to initialize resources for work queue/items");
        goto mutex_unlck;
    }

    /* Switch based on new irq type desired */
    switch (irq_new->irq_type) {
    case INT_MSI_SINGLE: /* MSI Single interrupt settings */
        ret_val = set_msi_single(pmetrics_device_elem);
        break;
    case INT_MSI_MULTI: /* MSI Multi interrupt settings */
        ret_val = set_msi_multi(pmetrics_device_elem, irq_new->num_irqs);
        break;
    case INT_MSIX: /* MSI-X interrupt settings */
        ret_val = set_msix(pmetrics_device_elem, irq_new->num_irqs,
            &msix_tbl_info);
        break;
    case INT_NONE: /* Set IRQ type to NONE */
        /* if here then already the IRQ scheme is none */
        ret_val = SUCCESS;
        break;
    default:
        LOG_ERR("Invalid Interrupt Type specified.");
        ret_val = -EBADRQC;
        break;
    }

    /* Return value can be +ve,0 or -ve. 0:SUCCESS */
    if (ret_val == SUCCESS) {
        /* set to the new irq scheme only if success */
        pnvme_dev->public_dev.irq_active.irq_type = irq_new->irq_type;
        pnvme_dev->public_dev.irq_active.num_irqs = irq_new->num_irqs;
        /* Will only be read by ISR */
        pmetrics_device_elem->irq_process.irq_type = irq_new->irq_type;
    }

mutex_unlck:
    /* mutex is locked so release it here.. */
    mutex_unlock(&pmetrics_device_elem->irq_process.irq_track_mtx);
    return ret_val;
}

/*
 * Used for Initializing the IRQ lists before any scheme is run
 * Lock on to the mutex and remove all the irq and cq track nodes.
 * Also removes all the enqueued wk items
 * set the current active scheme to INT_NONE.
 * NOTE: This will grab the irq mutex and releases.
 */
int init_irq_lists(struct metrics_device_list
        *pmetrics_device_elem, enum nvme_irq_type  irq_active)
{
    int ret_val;
    /* locking on IRQ MUTEX here for irq track ll access */
    mutex_lock(&pmetrics_device_elem->irq_process.irq_track_mtx);
    /* Initialize active irq to INT_NONE */
    ret_val = disable_active_irq(pmetrics_device_elem, pmetrics_device_elem->
            metrics_device->public_dev.irq_active.irq_type);
    /* Unlock IRQ MUTEX as we are done with updated irq track list */
    mutex_unlock(&pmetrics_device_elem->irq_process.irq_track_mtx);

    return ret_val;
}

/*
 * Used for releasing the IRQ lists after any scheme is run
 * Also removes all the enqueued wk items
 * set the current active scheme to INT_NONE.
 */
void release_irq(struct metrics_device_list *pmetrics_device_elem)
{
    /* Disable the IRQ */
    irq_disable(pmetrics_device_elem);

    if (pmetrics_device_elem->irq_process.wq) {
        LOG_DBG("Wait for the WQ to get flushed");
        /* Flush the WQ and wait till all BH's are executed */
        flush_workqueue(pmetrics_device_elem->irq_process.wq);
        LOG_DBG("Destroy the recently flushed WQ");
        /* Destroy the WQ */
        destroy_workqueue(pmetrics_device_elem->irq_process.wq);
        pmetrics_device_elem->irq_process.wq = NULL;
    }
    /* Note Mutex lock and unlock not required
     * even though we are editing the IRQ track list
     * since no more ISR's and BH's are pending
     */
    /* clean up and free all IRQ linked list nodes */
    deallocate_irq_trk(pmetrics_device_elem);
    /*Dealloc the work list if it exists */
    dealloc_wk_list(&pmetrics_device_elem->irq_process);

    /* Now we can Set IRQ type to INT_NONE */
    pmetrics_device_elem->metrics_device->public_dev.irq_active.
      irq_type = INT_NONE;
    pmetrics_device_elem->metrics_device->public_dev.irq_active.
      num_irqs = 0;
    /* Will only be read by ISR */
    pmetrics_device_elem->irq_process.irq_type = INT_NONE;
}
/*
 * The function first deallocates the IRQ linked list, then disables IRQ
 * scheme sent in irq_active, finally resets active irq scheme to INT_NONE.
 * Also re-initializes the irq track linked list.
 * NOTE: Always call this function with IRQ MUTEX locked, otherwise it fails.
 */
static int disable_active_irq(struct metrics_device_list
        *pmetrics_device_elem, enum nvme_irq_type  irq_active)
{

#ifdef DEBUG
    /* If mutex is not locked then exit here */
    if (!mutex_is_locked(&pmetrics_device_elem->irq_process.irq_track_mtx)) {
        LOG_ERR("Mutex should have been locked before this...");
        /* Mutex is not locked so exiting */
        return -EINVAL;
    }
#endif

    /* Disable the IRQ */
    irq_disable(pmetrics_device_elem);

    /* clean up and free all IRQ linked list nodes */
    deallocate_irq_trk(pmetrics_device_elem);

    /*Dealloc the work list if it exists */
    dealloc_wk_list(&pmetrics_device_elem->irq_process);

    /* Now we can Set IRQ type to INT_NONE */
    pmetrics_device_elem->metrics_device->public_dev.irq_active.
        irq_type = INT_NONE;
    pmetrics_device_elem->metrics_device->public_dev.irq_active.
        num_irqs = 0;
    /* Will only be read by ISR */
    pmetrics_device_elem->irq_process.irq_type = INT_NONE;
    return SUCCESS;
}

/*
 * nvme_disable_pin will read the CMD register in PCI space and sets
 * 10th bit in the CMD register to 1. This will disable the controller
 * from generating PIN# based interrupts.
 * NOTE: MSI operation is not affected.
 */
static void nvme_disable_pin(struct pci_dev *dev)
{
    u16 val;    /* temp variable to read cmd value */
    /* disable pin based INT by writing 0 in bit position 10 of CMD_OFFSET  */
    pci_read_config_word(dev, CMD_OFFSET, &val);    /* Read value           */
    val |= PIN_INT_BIT_MASK;                        /* Modify in place      */
    pci_write_config_word(dev, CMD_OFFSET, val);    /* write value          */
}

/*
 * We need to enable the HW pin based interrupts when the driver unloads.
 * This will allow the card to generate PIN# IRQ.
 */
/* static void nvme_enable_pin(struct pci_dev *dev)
{
    u16 val;    // temp variable to read cmd value
    // Enable pin based INT by writing 1 in bit position 10 of CMD_OFFSET
    pci_read_config_word(dev, CMD_OFFSET, &val);    // Read value
    val &= ~PIN_INT_BIT_MASK;                       // Modify in place
    pci_write_config_word(dev, CMD_OFFSET, val);    // write value
}
*/

/*
 * Check if the controller supports the interrupt type requested. If it
 * supports returns the offset, otherwise it will return invalid for the
 * caller to indicate that the controller does not support the capability
 * type.
 */
static int check_cntlr_cap(struct pci_dev *pdev, enum nvme_irq_type cap_type,
        u16 *offset)
{
    u16 val = 0;
    u16 pci_offset = 0;
    int ret_val = -EINVAL;

    if (pci_read_config_word(pdev, PCI_DEVICE_STATUS, &val) < 0) {
        LOG_ERR("pci_read_config failed...");
        return -EINVAL;
    }
    LOG_DBG("PCI_DEVICE_STATUS = 0x%X", val);
    if (!(val & CL_MASK)) {
        LOG_ERR("Controller does not support Capability list...");
        return -EINVAL;
    } else {
        if (pci_read_config_word(pdev, CAP_REG, &pci_offset) < 0) {
            LOG_ERR("pci_read_config failed...");
            return -EINVAL;
        }
    }
    /* Interrupt Type MSI-X*/
    if (cap_type == INT_MSIX) {
        /* Loop through Capability list */
        while (pci_offset) {
            if (pci_read_config_word(pdev, pci_offset, &val) < 0) {
                LOG_ERR("pci_read_config failed...");
                return -EINVAL;
            }
            /* exit when we find MSIX_capbility offset */
            if ((val & ~NEXT_MASK) == MSIXCAP_ID) {
                /* write msix cap offset */
                *offset = pci_offset;
                ret_val = SUCCESS;
                /* break from while loop */
                break;
            }
            /* Next Capability offset. */
            pci_offset = (val & NEXT_MASK) >> 8;
        } /* end of while loop */
    } else if (cap_type == INT_MSI_SINGLE || cap_type == INT_MSI_MULTI) {
        /* Loop through Capability list */
        while (pci_offset) {
            if (pci_read_config_word(pdev, pci_offset, &val) < 0) {
                LOG_ERR("pci_read_config failed...");
                return -EINVAL;
            }
            /* exit when we find MSIX_capbility offset */
            if ((val & ~NEXT_MASK) == MSICAP_ID) {
                /* write the msi offset */
                *offset = pci_offset;
                ret_val = SUCCESS;
                /* break from while loop */
                break;
            }
            /* Next Capability offset. */
            pci_offset = (val & NEXT_MASK) >> 8;
        } /* end of while loop */
    } else {
        LOG_ERR("Invalid capability type specified...");
        ret_val = -EINVAL;
    }

    return ret_val;
}

/*
 * Validates the IRQ inputs for MSI-X, MSI-Single and MSI-Mutli.
 * If the CC.EN bit is set or the number of irqs are invalid then
 * return failure otherwise success.
 */
static int validate_irq_inputs(struct metrics_device_list
    *pmetrics_device_elem, struct interrupts *irq_new,
        struct msix_info *pmsix_tbl_info)
{
    int ret_val = SUCCESS;
    struct nvme_device *pnvme_dev = pmetrics_device_elem->metrics_device;
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->
        private_dev.pdev;
    u16 msi_offset;
    u16 mc_val;

    /* Check if the EN bit is set and return failure if set */
    if (readl(&pnvme_dev->private_dev.nvme_ctrl_space->cc) & NVME_CC_ENABLE) {
        LOG_ERR("IRQ Scheme cannot change when CC.EN bit is set!!");
        LOG_ERR("Call Disable or Disable completely first...");
        return -EINVAL;
    }

    /* Switch based on new irq type desired */
    switch (irq_new->irq_type) {
    case INT_MSI_SINGLE: /* MSI Single interrupt settings */
        if (irq_new->num_irqs != MAX_IRQ_VEC_MSI_SIN) {
            LOG_ERR("IRQ vectors cannot be greater/equal %d in MSI Single IRQ",
                MAX_IRQ_VEC_MSI_SIN);
            return -EINVAL;
        }
        /* Check if the card Supports MSI capability */
        if (check_cntlr_cap(pdev, INT_MSI_SINGLE, &msi_offset) < 0) {
            LOG_ERR("Controller does not support for MSI capability!!");
            return -EINVAL;
        }
        /* Update interrupt vector Mask Set and Mask Clear offsets */
        pmetrics_device_elem->irq_process.mask_ptr = pmetrics_device_elem->
            metrics_device->private_dev.bar_0_mapped + INTMS_OFFSET;
        break;
    case INT_MSI_MULTI: /* MSI Multi interrupt settings */
        if (irq_new->num_irqs > MAX_IRQ_VEC_MSI_MUL ||
            irq_new->num_irqs == 0) {
            LOG_ERR("IRQ vectors cannot be greater/equal %d in MSI Multi IRQ",
                MAX_IRQ_VEC_MSI_MUL);
            return -EINVAL;
        }
        /* Check if the card Supports MSI capability */
        if (check_cntlr_cap(pdev, INT_MSI_MULTI, &msi_offset) < 0) {
            LOG_ERR("Controller does not support for MSI capability!!");
            return -EINVAL;
        }
        /* compute MSI MC offset if MSI is supported */
        msi_offset += 2;
        /* Read MSI-MC value */
        pci_read_config_word(pdev, msi_offset, &mc_val);
        if (irq_new->num_irqs > ((mc_val & MSI_MME) >> 4)) {
            LOG_ERR("IRQs = %d exceed MSI MME = %d", irq_new->num_irqs,
                ((mc_val & MSI_MME) >> 4));
            /* does not support the requested irq's*/
            return -EINVAL;
        }
        /* Update interrupt vector Mask Set and Mask Clear offsets */
        pmetrics_device_elem->irq_process.mask_ptr = pmetrics_device_elem->
            metrics_device->private_dev.bar_0_mapped + INTMS_OFFSET;
        break;
    case INT_MSIX: /* MSI-X interrupt settings */
        /* First check if num irqs req are greater than MAX MSIX SUPPORTED */
        if (irq_new->num_irqs > MAX_IRQ_VEC_MSI_X ||
            irq_new->num_irqs == 0) {
            LOG_ERR("IRQ vectors cannot be greater/equal %d in MSI-X IRQ",
                MAX_IRQ_VEC_MSI_X);
            return -EINVAL;
        }
        /* Check if the card Supports MSIX capability */
        if (check_cntlr_cap(pdev, INT_MSIX, &msi_offset) < 0) {
            LOG_ERR("Controller does not support for MSI-X capability!!");
            return -EINVAL;
        }
        /* if msix exists then update the msix pointer for this device */
        if (update_msixptr(pmetrics_device_elem, msi_offset, pmsix_tbl_info)
            < 0) {
            return -EINVAL;
        }
        /* compute MSI-X MXC offset if MSI-X is supported */
        msi_offset += 2;
        /* Read MSIX-MXC value */
        pci_read_config_word(pdev, msi_offset, &mc_val);
        pmsix_tbl_info->ts = (mc_val & MSIX_TS);
        /* check if Table size of MSIXCAP supports requested irqs.
         * as TS is 0 based and num_irq is 1 based, so we add 1 */
        if (irq_new->num_irqs > (pmsix_tbl_info->ts + 1)) {
            LOG_ERR("IRQs = %d exceed MSI-X table size = %d", irq_new->
                num_irqs, pmsix_tbl_info->ts);
            /* does not support the requested irq's*/
            return -EINVAL;
        } /* if table size */
        break;
    case INT_NONE: /* INT_NONE validation always returns success */
        /* no validation for INT_NONE schemes return success. */
        break;
    default:
        /* invalidate other type of IRQ schemes */
        LOG_ERR("No validation for default case..");
        ret_val = -EINVAL;
        break;
    }
    return ret_val;
}

/*
 * Sets up the active IRQ scheme to MSI-X. It gets the number of irqs
 * requested and loops from 0 to n -1 irqs, enables the active irq
 * scheme to MSI-X. Calls request_irq for each irq no and gets the OS
 * allocated interrupt vector. This function add each of this irq node
 * in the irq track linked list with int_vec and irq no. At any point
 * if the adding of node fails it cleans up and exits with invalid return
 * code.
 * Return 0 on sucess and -ve or +ve values on error
 */
static int set_msix(struct metrics_device_list *pmetrics_device_elem,
    u16 num_irqs, struct msix_info *pmsix_tbl_info)
{
    int ret_val, i;
    struct msix_entry msix_entries[num_irqs];
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->
        private_dev.pdev;
    struct irq_track *pirq_node;

    /* Unmask MSI-x and initialize PBA table */

    for (i = 0; i <= pmsix_tbl_info->ts; i++) {
        if (!(i % 32)) {
            writel(0, (pmsix_tbl_info->pba_tbl + (i / 32)));
        }
        set_msix_mask_bit(pmsix_tbl_info->msix_tbl, i, 0x0);
    }
    /* Assign irq entries from 0 to n-1 */
    for (i = 0; i < num_irqs; i++) {
        msix_entries[i].entry = i;
    }



    /* Allocate msix interrupts to this device */
    ret_val = pci_enable_msix(pdev, msix_entries, num_irqs);
    if (ret_val) {
        LOG_ERR("Can't enable MSI-X");
        return ret_val;
    }

    /* Request irq on each interrupt vector */
    for (i = 0; i < num_irqs; i++) {
        /* If request fails on any interrupt vector then fail here */
        ret_val = request_irq(msix_entries[i].vector, tophalf_isr, IRQF_DISABLED
            | IRQF_SHARED, "msi-x", &pmetrics_device_elem->irq_process);
        if (ret_val < 0) {
            LOG_ERR("MSI-X-Err: request irq failed for ivec= %u",
                msix_entries[i].vector);
            /* As we are allocating memory for one node at a time
             * failing here needs freeing up memory previously allocated */
            goto free_msix;
        }
        /* Add node after determining interrupt vector req is successful */
        LOG_DBG("Add Node for Vector = %d", msix_entries[i].vector);
        ret_val = add_irq_node(pmetrics_device_elem, msix_entries[i].vector,
            msix_entries[i].entry);
        if (ret_val < 0) {
            LOG_ERR("MSI-X-Err: can't add irq node");
            goto free_msix;
        } /* end of if add_irq_node */

        /* Add node after determining interrupt vector req is successful */
        LOG_DBG("Add Wk item node for Vector = %d", msix_entries[i].vector);
        ret_val = add_wk_item(&pmetrics_device_elem->irq_process,
            msix_entries[i].vector, msix_entries[i].entry);
        if (ret_val < 0) {
            LOG_ERR("MSI-X-Err: can't add work item node");
            goto free_msix;
        } /* end of if add_wk_item */
    } /* end of for num_irqs */

    /* fetch the Irq node 0 */
    pirq_node = find_irq_node(pmetrics_device_elem, 0);
    if (pirq_node == NULL) {
        LOG_ERR("Can't find CQ 0 node inside irq_track list");
        ret_val = -EINVAL;
        goto free_msix;
    }
    /* Add the default ACQ node into the default irq node */
    ret_val = add_icq_node(pirq_node, 0);
    if (ret_val < 0) {
        LOG_ERR("Can't add CQ 0 node inside irq_track list");
        goto free_msix;
    }

    return ret_val;

free_msix:
    disable_active_irq(pmetrics_device_elem, pmetrics_device_elem->
        metrics_device->public_dev.irq_active.irq_type);
    return ret_val;
}

/*
 * Set up the active irq scheme to MSI Single interrupt scheme. The interrupt
 * vector is assigned by the pci_enable_msi call and pdev->irq is updated with
 * new interrupt vector for this device and same is added in the irq linked
 * list for 0 irq number. If irq request and memory for the irq node is
 * successful, only then the irq track list is updated otherwise fails and
 * returns invalid return code. Finally clear all the interrupts using INTMC
 * register by writing all 1's to this register.
 * Return 0 on sucess and -ve or +ve values on error
 */
static int set_msi_single(struct metrics_device_list *pmetrics_device_elem)
{
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->
            private_dev.pdev;
    int ret_val;
    struct irq_track *pirq_node;

    /* Clear all the interrupts for MSI Single by writing to INTMC */
     writel(UINT_MAX, (pmetrics_device_elem->irq_process.mask_ptr + 0x04));

    /*
     * Allocate one interrupt to this device. A successful call will switch the
     * device to msi single mode and pdev->irq is changed to a new number which
     * represents msi interrupt vector consequently.
     */
    ret_val = pci_enable_msi(pdev);
    if (ret_val) {
        LOG_ERR("Can't enable msi ");
        return ret_val; /* exit from here */
    }
    /* request irq with top half handler and int vec stored in pdev->irq. */
    ret_val = request_irq(pdev->irq, tophalf_isr, IRQF_DISABLED
            | IRQF_SHARED, "msi-single", &pmetrics_device_elem->irq_process);
    if (ret_val < 0) {
        LOG_ERR("Request irq failed for ivec= %i", pdev->irq);
        return ret_val; /* exit from here */
    }
    LOG_DBG("MSI-Single Interrupt Vector = %d", pdev->irq);
    /* Add node after determining interrupt vector is successful */
    ret_val = add_irq_node(pmetrics_device_elem, pdev->irq, 0);
    if (ret_val < 0) {
        LOG_ERR("Can't add irq node");
        goto free_msis;
    }

    ret_val = add_wk_item(&pmetrics_device_elem->irq_process, pdev->irq, 0);
    if (ret_val < 0) {
        LOG_ERR("Can't add work item node");
        goto free_msis;
    }

    /* fetch the Irq node 0 */
    pirq_node = find_irq_node(pmetrics_device_elem, 0);
    if (pirq_node == NULL) {
        LOG_ERR("Can't find CQ 0 node inside irq_track list");
        ret_val = -EINVAL;
        goto free_msis;
    }
    /* Add the default ACQ node into the default irq node */
    ret_val = add_icq_node(pirq_node, 0);
    if (ret_val < 0) {
        LOG_ERR("Can't add CQ 0 node inside irq_track list");
        goto free_msis;
    }

    return ret_val;
free_msis:
    disable_active_irq(pmetrics_device_elem, pmetrics_device_elem->
        metrics_device->public_dev.irq_active.irq_type);
    return ret_val;
}

/*
 * Sets up the active IRQ scheme to MSI-Multi. Number of irqs to be allocated
 * is passed as a parameter and After successfully enabling the msi block, this
 * loops from pdev->irq to (pdev->irq + num_irqs -1) and calls request_irq for
 * each irq no. This function appends each of this irq node in the irq track
 * linked list with int_vec and irq no. At any point if the adding of node
 * fails it cleans up and exits with invalid return code. Finally clear all
 * the interrupts using INTMC register by writing all 1's to this register.
 * Return 0 on sucess and -ve or +ve values on error
 */
static int set_msi_multi(struct metrics_device_list *pmetrics_device_elem,
        u16 num_irqs)
{
    int ret_val, i;
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->
        private_dev.pdev;
    struct irq_track *pirq_node;

    /* Clear all the interrupts for MSI Single by writing to INTMC */
    writel(UINT_MAX, (pmetrics_device_elem->irq_process.mask_ptr + 0x04));

    /* Enable msi-block interrupts for this device. The pdev->irq will
     * be the lowest of the new interrupts assigned to this device. */
    ret_val = pci_enable_msi_block(pdev, num_irqs);
    if (ret_val) {
        LOG_ERR("Can't enable MSI-Multi");
        return ret_val;
    }
    /* Request irq on each interrupt vector */
    for (i = 0; i < num_irqs; i++) {
        /* If request fails on any interrupt vector then fail here */
        ret_val = request_irq((pdev->irq + i), tophalf_isr, IRQF_DISABLED
            | IRQF_SHARED, "msi-multi", &pmetrics_device_elem->irq_process);
        if (ret_val < 0) {
            LOG_ERR("Request IRQ failed ivec = %d", pdev->irq + i);
            /* As we are allocating memory for one node at a time
             * failing here needs freeing up memory previously allocated */
            goto free_msim;
        }

        /* Add node after determining interrupt vector req is successful */
        LOG_DBG("Add Node for Vector = %d", pdev->irq + i);
        ret_val = add_irq_node(pmetrics_device_elem, (pdev->irq + i), i);
        if (ret_val < 0) {
            LOG_ERR("Can't add irq node");
            goto free_msim;
        } /* end of if add_irq_node */

        /* Add node after determining interrupt vector req is successful */
        LOG_DBG("Add Wk item node for Vector = %d", pdev->irq + i);
        ret_val = add_wk_item(&pmetrics_device_elem->irq_process,
            (pdev->irq + i), i);
        if (ret_val < 0) {
            LOG_ERR("Can't add work item node");
            goto free_msim;
        } /* end of if add_wk_item */
    } /* end of for num_irqs */

    /* fetch the Irq node 0 */
    pirq_node = find_irq_node(pmetrics_device_elem, 0);
    if (pirq_node == NULL) {
        LOG_ERR("Can't find CQ 0 node inside irq_track list");
        ret_val = -EINVAL;
        goto free_msim;
    }
    /* Add the default ACQ node into the default irq node */
    ret_val = add_icq_node(pirq_node, 0);
    if (ret_val < 0) {
        LOG_ERR("Can't add CQ 0 node inside irq_track list");
        goto free_msim;
    }

    return ret_val;
free_msim:
    disable_active_irq(pmetrics_device_elem, pmetrics_device_elem->
        metrics_device->public_dev.irq_active.irq_type);
    return ret_val;
}

/*
 * Update MSIX pointer in the irq process structure.
 */
static int update_msixptr(struct  metrics_device_list *pmetrics_device_elem,
    u16 offset, struct msix_info *pmsix_tbl_info)
{
    u8 *msixptr;    /* msix pointer for this device   */
    u32 msi_x_to;   /* msix table offset in pci space */
    u32 msix_tbir;  /* msix table BIR BAR indicator   */
    u32 msix_mpba;  /* msix pba offset and pba BIR */
    u32 msix_pbir;  /* msix table PBIR BAR indicator   */
    struct nvme_device *metrics_device = pmetrics_device_elem->metrics_device;
    struct pci_dev *pdev = metrics_device->private_dev.pdev;

    /* compute MSI-X Table offset using given MSI-X offset */
    offset += 0x4;
    /* Read the MTAB Table offset and TBIR from pci space */
    pci_read_config_dword(pdev, offset, &msi_x_to);
    /* Compute TBIR using data located in NMVE space at the offset */
    msix_tbir = msi_x_to & MSIX_TBIR_MASK;
    /* compute MSI-X PBA offset using given MSI-X offset */
    offset += 0x4;
    /* Read the MPBA Table offset and PBIR from pci space */
    pci_read_config_dword(pdev, offset, &msix_mpba);
    /* Compute PBIR using data located in NMVE space at the offset */
    msix_pbir = msix_mpba & MSIX_TBIR_MASK;

    /* Check which BAR locations are used for MSI-X Table offset */
    /* TODO: Support for all BAR's */
    if (msix_tbir == 0x0) {
        /* if BIR is 0 it indicates BAR 0 offset is used. */
        msixptr = metrics_device->private_dev.bar_0_mapped +
            (msi_x_to & ~MSIX_TBIR_MASK);
    } else if (msix_tbir == 0x4) {
        LOG_DBG("BAR 4 Location not supported yet...");
        return -EINVAL;
    } else if (msix_tbir == 0x5) {
        LOG_DBG("BAR 5 locations not supported yet...");
        return -EINVAL;
    } else {
        LOG_ERR("Trying to access TBIR reserved bits");
        return -EINVAL;
    }

    /* Check which BAR locations are used for PBA Table offset */
    /* TODO: Support for all BAR's */
    if (msix_pbir == 0x0) {
        pmsix_tbl_info->pba_tbl = metrics_device->private_dev.bar_0_mapped +
            (msix_mpba & ~MSIX_TBIR_MASK);
    } else if (msix_pbir == 0x4) {
        LOG_DBG("BAR 4 Location not supported yet...");
        return -EINVAL;
    } else if (msix_pbir == 0x5) {
        LOG_DBG("BAR 5 locations not supported yet...");
        return -EINVAL;
    } else {
        LOG_ERR("Trying to access PBIR reserved bits");
        return -EINVAL;
    }
    /* Update the msix pointer in the device metrics */
    pmetrics_device_elem->irq_process.mask_ptr = msixptr;
    pmsix_tbl_info->msix_tbl = msixptr;
    return SUCCESS;
}

/*
 * set_msix_mask_bit - This function will mask or unmask the vector
 * control bit inside the MSIX table for the corresponding irq_no.
 * When the mask is set, the controller will not generate the interrupts
 * for this interrupt vector. To unmask, flag value is passed with value
 * 0x0 and the corresponding interrupt vector is unmasked and controller
 * will start generating the interrupts.
 */
static void set_msix_mask_bit(u8 *irq_msixptr, u16 irq_no, u32 flag)
{
    u8 *msixptr;
    /* Get the MSI-X pointer offset */
    msixptr = irq_msixptr;
    /* Compute vector control offset for this irq no. */
    msixptr += MSIX_VEC_CTRL + irq_no * MSIX_ENTRY_SIZE;
    /* Mask or unmask the the MSIX vector by writing flag bit */
    writel(flag, (u32 *) msixptr);
    /* Flush the write */
    readl((u32 *) msixptr);
}

/*
 * mask_interrupts - Determine the type of masking required based
 * on the interrupt scheme active. Mask interrupts for MSI-Single,
 * MSI- Multi and MSIX
 */
void mask_interrupts(u16 irq_no, struct irq_processing
    *pirq_process)
{
    /* handle all masking here */
    switch (pirq_process->irq_type) {
    case INT_MSI_SINGLE: /* Same as MSI MULTI */
    case INT_MSI_MULTI: /* Masking MSI interrupt in INTMS register */
        /* Mask INTMS register for the int generated */
        writel((0x1 << irq_no), (u32 *) pirq_process->mask_ptr);
        /* Flush the wrte */
        readl((u32 *) pirq_process->mask_ptr);
        break;
    case INT_MSIX: /* Masking for MSI-X using vector control */
        set_msix_mask_bit(pirq_process->mask_ptr,
            irq_no, 0x1);
        break;
    case INT_NONE:
        break;
    default:
        LOG_ERR("Unknown interrupt type to Mask...");
        break;
    }
}

/*
 * unmask_interrupts - Determine the type of interrupt scheme and
 * unmask interrupts for MSI-Single, MSI- Multi and MSIX.
 */
void unmask_interrupts(u16 irq_no, struct irq_processing
    *pirq_process)
{
    /* handle all unmasking here */
    switch (pirq_process->irq_type) {
    case INT_MSI_SINGLE:
    case INT_MSI_MULTI:
        /* unMask INTMC register for the int generated */
        writel((0x1 << irq_no), (u32 *) (pirq_process->mask_ptr + 0x04));
        /* Flush the wrte */
        readl((u32 *) pirq_process->mask_ptr + 0x04);
        break;
    case INT_MSIX: /* ummask for MSI-X */
        set_msix_mask_bit(pirq_process->mask_ptr,
                irq_no, 0x0);
        break;
    case INT_NONE:
        break;
    default:
        LOG_ERR("Unknown interrupt type to Mask...");
        break;
    }
}

/*
 * Top half isr responds to the interrupt by masking the corresponding irq
 * vector if required and scheduling the bottom half processing.
 */
irqreturn_t tophalf_isr(int int_vec, void *dev_id)
{
    /* Point to the right hardware item using this dev_id */
    struct  irq_processing *pirq_process =
        (struct  irq_processing *) dev_id;
    struct work_container *pwk;
    irqreturn_t ret_val = IRQ_HANDLED;

    /* Look for the required work_struct */
    pwk = get_work_item(pirq_process, int_vec);
    if (pwk == NULL) {
        LOG_ERR("spurious irq with int_vec = %d", int_vec);
        return IRQ_NONE;
    }
    /* To resolve contention between ISR's getting fired on different cores */
    spin_lock(&pirq_process->isr_spin_lock);
    LOG_DBG("TH:IRQNO = %d is serviced", pwk->irq_no);
    /* Mask the interrupts which was fired till BH */
    mask_interrupts(pwk->irq_no, pirq_process);

    if (queue_work(pirq_process->wq, &pwk->sched_wq)
        == 0) {
        LOG_ERR("Work item already in Queue");
    }
    /* unlock as we are done with critical section */
    spin_unlock(&pirq_process->isr_spin_lock);
    return ret_val;
}

/*
 * inc_isr_count:
 * Search the CQ node for which the IRQ is fired and set flag. Increment
 * count by one.
 */
static void inc_isr_count(struct irq_processing *pirq_process,
    u16 irq_no)
{
    struct  irq_track     *pirq_node;  /* Pointer to irq node     */

    /* Loop for each irq node */
    list_for_each_entry(pirq_node, &pirq_process->irq_track_list,
        irq_list_hd) {
        if (irq_no == pirq_node->irq_no) {
            pirq_node->isr_fired = 1;
            pirq_node->isr_count++;
            LOG_DBG("BH:isr count = %d for irq no = %d",
                pirq_node->isr_count, pirq_node->irq_no);
        } /* end of if ivec */
    } /* end of list for irq */
}

/*
 * This is the work item that gets scheduled when the ISR gets fired.
 * The bottom halfs are queued up and we should not worry about multiple
 * interrupts getting fired up as we store them in the array as a lookup
 * table.
 */
static void bh_callback(struct work_struct *work_element)
{
    /* Getting the work item from work_struct */
    struct work_container *pwork =
        container_of(work_element, struct work_container, sched_wq);

    /* lock irq mutex as we will access the irq nodes */
    mutex_lock(&pwork->pirq_process->irq_track_mtx);

    /* Set the values in the node */
    inc_isr_count(pwork->pirq_process, pwork->irq_no);

    /* unlock as we are done with updating the irq nodes */
    mutex_unlock(&pwork->pirq_process->irq_track_mtx);

    LOG_DBG("BH:IRQNO = %d is serviced in Bottom Half", pwork->irq_no);
}

/*
 * work_queue_init:
 * Is used for setting up memory for work queue
 */
int work_queue_init(struct  irq_processing *pirq_process)
{
    if (pirq_process->wq == NULL) {
        /* If work Queue does not exist from previous run of the IOCTL
         * then create a new work queue.
         */
        pirq_process->wq = create_workqueue("work_queue");
        if (pirq_process->wq == NULL) {
            LOG_ERR("Failed work queue creation");
            return -ENOMEM;
        }
    }

    return SUCCESS;
}

/*
 * Add a irq track node in irq_track linked list. Allocates memory for one
 * irq_track node, sets up the values for this node. Initialize the CQ_track
 * node for this irq_node then add the irq_node to the itq_track linked list.
 */
static int add_irq_node(struct  metrics_device_list *pmetrics_device_elem,
        u32 int_vec, u16 irq_no)
{
    struct irq_track *irq_trk_node;

#ifdef DEBUG
    /* If mutex is not locked then exit here */
    if (!mutex_is_locked(&pmetrics_device_elem->irq_process.irq_track_mtx)) {
        LOG_ERR("Mutex should have been locked before this...");
        /* Mutex is not locked so exiting */
        return -EINVAL;
    }
#endif

    LOG_DBG("Adding Irq Node...%d", irq_no);
    /* Allocate memory for the cq node and check if success */
    irq_trk_node = kmalloc(sizeof(struct irq_track), GFP_KERNEL);
    if (irq_trk_node == NULL) {
            LOG_ERR("IRQ track Node allocation failed");
            return -ENOMEM;
    }
    /* Fill the irq track node */
    irq_trk_node->int_vec = int_vec; /* int vector number   */
    irq_trk_node->irq_no = irq_no;   /* irq number assigned */
    irq_trk_node->isr_fired = 0;
    irq_trk_node->isr_count = 0;

    /* Init irq cq linked list */
    INIT_LIST_HEAD(&irq_trk_node->irq_cq_track);

    /* Add this irq node element to the end of the list */
    list_add_tail(&irq_trk_node->irq_list_hd, &pmetrics_device_elem->
            irq_process.irq_track_list);

    return SUCCESS;
}

/*
 * Add a wk item node in work item's linked list. Allocates memory for one
 * wk item  node, sets up the values for this node.
 */
static int add_wk_item(struct irq_processing *pirq_process,
        u32 int_vec, u16 irq_no)
{
    struct work_container *pwork;

    pwork = kmalloc(sizeof(struct work_container), GFP_KERNEL);
    if (pwork == NULL) {
        LOG_ERR("Failed mem alloc in work_container creation");
        return -ENOMEM;
    }
    /* Initialize the bottom half call back with the work struct */
    INIT_WORK(&pwork->sched_wq, bh_callback);
    /* Update the values */
    pwork->pirq_process = pirq_process;
    pwork->int_vec = int_vec;
    pwork->irq_no = irq_no;
    /* Add work item to work items list */
    list_add_tail(&pwork->wrk_list_hd,
        &pirq_process->wrk_item_list);

    return SUCCESS;
}

/*
 * Add a cq node into cq track linked list. Allocate memory for one CQ node
 * in the irq_track node. Assign the CQ id passed and reset other parameters.
 * Finally add this cq node in the cq track linked list for this irq node.
 */
int add_icq_node(struct irq_track *pirq_trk_node, u16 cq_id)
{
    struct irq_cq_track *irq_cq_node;

    /* Allocate memory for the cq node and check if success */
    irq_cq_node = kmalloc(sizeof(struct irq_cq_track), GFP_KERNEL);
    if (irq_cq_node == NULL) {
        LOG_ERR("IRQ CQ Node allocation failed");
        return -ENOMEM;
    }
    /* Fill the node with init data */
    irq_cq_node->cq_id = cq_id;

    /* Add this cq node to the end of the list */
    list_add_tail(&irq_cq_node->irq_cq_head,
            &pirq_trk_node->irq_cq_track);
    return SUCCESS;
}

/*
 * Additions to IOCTL_SEND_64B_CMD for IOCQs.
 * Updates the CQ list inside the specific IRQ node in irq_track list
 * NOTE:
 * This function will set irq_enabled flag if SUCCESS.
 */
int update_cq_irqtrack(struct  metrics_device_list *pmetrics_device_elem,
    u16 cq_id, u16 irq_no, u8 *irq_enabled)
{
    int ret_val;
    struct irq_track *pirq_node;

    /* lock onto IRQ linked list mutex as we would access the IRQ list */
    mutex_lock(&pmetrics_device_elem->irq_process.irq_track_mtx);

    /* fetch the Irq node for given irq no */
    pirq_node = find_irq_node(pmetrics_device_elem, irq_no);
    if (pirq_node == NULL) {
        ret_val = -EINVAL;
        goto exit;
    }

    /* Add the CQ node into this irq node */
    ret_val = add_icq_node(pirq_node, cq_id);
    if (ret_val == SUCCESS) {
        /* After successfully adding the cq node then enable the irq in
         * metrics cq linked list */
        *irq_enabled = 1;
    }

exit:
    /* unlock IRQ linked list mutex */
    mutex_unlock(&pmetrics_device_elem->irq_process.irq_track_mtx);
    return ret_val;
}

/*
 * reap_inquiry_isr - will process reap inquiry for the given cq using irq_vec
 * and isr_fired flags from two nodes, public cq node and irq_track list node.
 * NOTE: This function should be called with irq mutex locked otherwise it
 * will error out.
 */
int reap_inquiry_isr(struct metrics_cq  *pmetrics_cq_node,
    struct  metrics_device_list *pmetrics_device_elem,
        u16 *num_remaining, u32 *isr_count)
{
    u16 irq_no = pmetrics_cq_node->public_cq.irq_no; /* irq_no for CQ   */
    struct irq_track *pirq_node;

#ifdef DEBUG
    /* If mutex is not locked then exit here */
    if (!mutex_is_locked(&pmetrics_device_elem->irq_process.irq_track_mtx)) {
        LOG_ERR("Mutex should have been locked before this...");
        /* Mutex is not locked so exiting */
        return -EINVAL;
    }
#endif

    /* Get the Irq node for given irq vector */
    pirq_node = find_irq_node(pmetrics_device_elem, irq_no);
    if (pirq_node == NULL) {
        LOG_ERR("Node for IRQ No = %d does not exist in IRQ list!", irq_no);
        return -EINVAL;
    }

    /* Check if ISR is really fired for this CQ */
    if (pirq_node->isr_fired != 0) {
        /* process reap inquiry for isr fired case */
        *num_remaining = reap_inquiry(pmetrics_cq_node, &pmetrics_device_elem
            ->metrics_device->private_dev.pdev->dev);
    } else {
        /* To deal with ISR's aggregation, not supposed to notify CE's yet */
        *num_remaining = 0;
    }
    /* return the isr_count flag */
    *isr_count = pirq_node->isr_count;
    return SUCCESS;
}

/*
 * Gets the required work_container from the list of work items based
 * upon int_vec number else NULL
 */
static struct work_container *get_work_item(struct  irq_processing
    *pirq_process, u32 int_vec)
{
    struct work_container *pwk_item;  /* wk item in the list */
    /* Loop for each wk item */
    list_for_each_entry(pwk_item,
        &pirq_process->wrk_item_list, wrk_list_hd) {
        if (int_vec == pwk_item->int_vec) {
            return pwk_item;
        }
    }
    return NULL;
}

/*
 * Find_irq_node - return the pointer to the irq node in the irq track list
 * for the irq_no if found otherwise return NULL.
 */
static struct irq_track *find_irq_node(
        struct  metrics_device_list *pmetrics_device_elem, u16 irq_no)
{
    struct  irq_track     *pirq_node;     /* Pointer to irq node */

    /* Loop for the irq node in irq track list */
    list_for_each_entry(pirq_node, &pmetrics_device_elem->irq_process.
            irq_track_list, irq_list_hd) {
            /* if the irq no matches then return this irq node */
            if (irq_no == pirq_node->irq_no) {
                return pirq_node;
            }
    }

    return NULL;
}

/*
 * find CQ node within the given irq track list node. Loop through the
 * irq_track linked list and match the CQ ID and if found return a ptr
 * to the cq node in the irq track linked list.
 */
static struct irq_cq_track *find_icq_node(struct  irq_track *pirq_node,
        u16 cq_id)
{
    struct irq_cq_track *picq_node;

    list_for_each_entry(picq_node, &pirq_node->irq_cq_track,
            irq_cq_head) {
        if (cq_id == picq_node->cq_id) {
            return picq_node;
        }
    }
    return NULL;
}

/*
 * Remove the given CQ node from the IRQ track linked list. From the
 * irq_track linked list, get the pointer to the irq_node for the
 * given irq_no and inside this irq_node, find the corresponding
 * CQ node. using this pointer to CQ node, delete this CQ node and
 * free up the memory.
 */
int remove_icq_node(struct  metrics_device_list
        *pmetrics_device, u16 cq_id, u16 irq_no)
{
    struct irq_track *pirq_node;
    struct irq_cq_track *picq_node;

    LOG_DBG("Call to remove the ICQ = %d for irq_no = %d",
            cq_id, irq_no);
    /* Get the Irq node for given irq vector */
    pirq_node = find_irq_node(pmetrics_device, irq_no);
    if (pirq_node == NULL) {
        return -EINVAL;
    }
    /* Get the CQ node in the irq node found */
    picq_node = find_icq_node(pirq_node, cq_id);
    if (picq_node == NULL) {
        LOG_ERR("CQ node does not exist in the IRQ Tracked node!");
        return -EINVAL;
    }
    /* remove the cq node from the linked list and initialize it */
    list_del_init(&picq_node->irq_cq_head);
    /* free up the memory allocated for cq node in irq list */
    kfree(picq_node);

    return SUCCESS;
}

/*
 * Deallocate all the work item nodes within the work items list
 */
void dealloc_wk_list(struct  irq_processing *pirq_process)
{
    struct work_container *pwk_item_next;  /* Next wk item in the list */
    struct work_container *pwk_item_curr;  /* Current wk item in the list */

    LOG_DBG("Deallocate wk Nodes...");
    /* Loop for each cq node within this irq node */
    list_for_each_entry_safe(pwk_item_curr, pwk_item_next,
        &pirq_process->wrk_item_list, wrk_list_hd) {
        /* remove the wk node from the linked list */
        list_del(&pwk_item_curr->wrk_list_hd);
        /* free up the memory allocated for wk node in irq list */
        kfree(pwk_item_curr);
    }
}

/*
 * deallocate all the cq nodes within this irq node linked list. Remove
 * CQ nodes from the linked list and free up the memory allocated for CQ
 * inside irq_track node.
 */
static void dealloc_all_icqs(struct  irq_track *pirq_trk_list)
{
    struct  irq_cq_track    *pirq_cq_list;  /* Type to use as loop cursor  */
    struct  irq_cq_track    *pirq_cq_next;  /* type to use as temp storage */

    LOG_DBG("Deallocate IRQ CQ Nodes...");
    /* Loop for each cq node within this irq node */
    list_for_each_entry_safe(pirq_cq_list, pirq_cq_next,
            &pirq_trk_list->irq_cq_track, irq_cq_head) {
        /* remove the cq node from the linked list */
        list_del(&pirq_cq_list->irq_cq_head);
        /* free up the memory allocated for cq node in irq list */
        kfree(pirq_cq_list);
    }
}

/*
 * deallocate all IRQ track nodes from the linked list. Calls free_irq on
 * the interrupt vector that was reserved using request_irq.
 * Also reinitializes the irq_track list
 */
void deallocate_irq_trk(struct  metrics_device_list
    *pmetrics_device_elem)
{
    struct  irq_track   *pirq_trk_list; /* Type to use as loop cursor       */
    struct  irq_track   *pirq_trk_next; /* Same type to use as temp storage */

    LOG_DBG("Deallocate IRQ Track...");
    /* Loop for each irq node */
    list_for_each_entry_safe(pirq_trk_list, pirq_trk_next,
            &pmetrics_device_elem->irq_process.irq_track_list, irq_list_hd) {
        LOG_DBG("Int Vec:Irq No = %d:%d", pirq_trk_list->int_vec,
                pirq_trk_list->irq_no);
        /* remove all the cq nodes for this irq node */
        dealloc_all_icqs(pirq_trk_list);
        list_del(&pirq_trk_list->irq_list_hd);
        kfree(pirq_trk_list);
    }
}

/*
 * Disable and free IRQ's which were requested earlier
 */
void irq_disable(struct  metrics_device_list
    *pmetrics_device_elem)
{
    struct  irq_track   *pirq_trk_node; /* Node iside IRQ track list */
    /* pointer to the pci device */
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->
            private_dev.pdev;
    enum nvme_irq_type  irq_active = pmetrics_device_elem->
        metrics_device->public_dev.irq_active.irq_type;

    /* disable the PIN interrupts*/
    nvme_disable_pin(pdev);

    list_for_each_entry(pirq_trk_node,
        &pmetrics_device_elem->irq_process.irq_track_list,
            irq_list_hd) {
        free_irq(pirq_trk_node->int_vec, &pmetrics_device_elem->irq_process);
    }

    /* Perform setting of IRQ to none based on active scheme of IRQ */
    if (irq_active == INT_MSI_SINGLE) {
        /* Only one interrupt so call disable once */
        pci_disable_msi(pdev);
        LOG_DBG("INT_MSI SINGLE Disabled");
    } else if (irq_active == INT_MSI_MULTI) {
        /* Only main interrupt should be disabled so call disable on device */
        pci_disable_msi(pdev);
        LOG_DBG("INT_MSI-Multiple Disabled");
    } else if (irq_active == INT_MSIX) {
        /* disable OS to generate MISX */
        pci_disable_msix(pdev);
        LOG_DBG("INT_MSIX Disabled");
    } else {
        /* Calling the interrupt disable functions */
        LOG_DBG("No Active IRQ scheme to disable...");
    }
}

/* Loop through all CQ's associated with irq_no and check whehter
 * they are empty and if empty reset the isr_flag for that particular
 * irq_no
 */
int reset_isr_flag(struct metrics_device_list *pmetrics_device,
    u16 irq_no)
{
    struct irq_track *pirq_node; /* IRQ node inside irq track list */
    struct irq_cq_track *picq_node; /* CQ node inside irq node */
    struct metrics_cq  *pmetrics_cq_node; /* CQ node in metrics_cq_list */
    u16 num_rem = 0;

    /* Get the Irq node for given irq vector */
    pirq_node = find_irq_node(pmetrics_device, irq_no);
    if (pirq_node == NULL) {
        LOG_ERR("Irq node not found for irq_no: %d", irq_no);
        return -EINVAL;
    }

    /* Loop through each CQ for the given irq_no */
    list_for_each_entry(picq_node, &pirq_node->irq_cq_track,
        irq_cq_head) {
        /* Find CQ metrics */
        pmetrics_cq_node = find_cq(pmetrics_device, picq_node->cq_id);
        if (pmetrics_cq_node == NULL) {
            LOG_ERR("CQ ID = %d not found", picq_node->cq_id);
            return -EBADSLT;
        }
        /* Reap on all CQ's */
        num_rem = reap_inquiry(pmetrics_cq_node, &pmetrics_device
            ->metrics_device->private_dev.pdev->dev);
        if (num_rem != 0) {
            break;
        }
    }

    /* reset the isr flag */
    if (num_rem == 0) {
        pirq_node->isr_fired = 0;
    }
    return 0;
}

