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
        *pmetrics_device_elem, struct interrupts *irq_new);
static int set_msix(struct metrics_device_list *pmetrics_device_elem,
        u16 num_irqs);
static int set_msi_single(struct metrics_device_list *pmetrics_device_elem);
static int add_irq_node(struct  metrics_device_list *pmetrics_device_elem,
        u16 int_vec, u16 irq_no);
static void bh_callback(struct work_struct *work);
static void dealloc_all_icqs(struct  irq_track *pirq_trk_list);
static int disable_active_irq(struct metrics_device_list
        *pmetrics_device_elem, enum nvme_irq_type  irq_active);
static int check_cntlr_cap(struct pci_dev *pdev, enum nvme_irq_type cap_type,
        u16 *offset);
static void set_irq_cq_nodes(struct metrics_device_list *pmetrics_device_elem,
        u16 int_vec);
static u16 get_irq_enabled(u16 int_vec[]);
static struct irq_track *find_irq_node(
        struct  metrics_device_list *pmetrics_device_elem, u16 irq_vector);
static struct irq_cq_track *find_icq_node(struct  irq_track *pirq_node,
        u16 cq_id);
static u16 get_ivec_index(struct  metrics_device_list *pmetrics_device_elem,
        u16 int_vec);
static void mask_interrupts(int irq_no, struct nvme_device *metrics_device);
static void ummask_interrupts(int irq_no, struct nvme_device *metrics_device);
static void nvme_disable_pin(struct pci_dev *dev);

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

    /* First validate if the inputs given are correct */
    ret_val = validate_irq_inputs(pmetrics_device_elem, irq_new);
    if (ret_val < 0) {
        LOG_ERR("Invalid inputs set or device is not disabled..");
        return ret_val;
    }

    /* lock onto IRQ linked list mutex as we would access the IRQ list */
    mutex_lock(&pmetrics_device_elem->irq_process.irq_track_mtx);

    /* disable the current IRQ scheme */
    ret_val = disable_active_irq(pmetrics_device_elem, pnvme_dev->irq_active.
            irq_type);
    if (ret_val < 0) {
        LOG_ERR("Reset of IRQ to INT_NONE failed...");
        ret_val = -EINVAL;
        goto set_irq_out;
    }

    /* initialize parameters required for setting callbacks */
    isr_init(pmetrics_device_elem);

    /* Switch based on new irq type desired */
    switch (irq_new->irq_type) {
    case INT_MSI_SINGLE: /* MSI Single interrupt settings */
        /* Call to set msi single interrupt scheme to get int vec */
        ret_val = set_msi_single(pmetrics_device_elem);
        break;
    case INT_MSI_MULTI: /* MSI Multi interrupt settings */
        /*TODO: Add multi MSI interrupt functions */
        break;
    case INT_MSIX: /* MSI-X interrupt settings */
        ret_val = set_msix(pmetrics_device_elem, irq_new->num_irqs);
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

    /* set to the new irq scheme only if success */
    if (ret_val == SUCCESS) {
        pnvme_dev->irq_active.irq_type = irq_new->irq_type;
        pnvme_dev->irq_active.num_irqs = irq_new->num_irqs;
    }

set_irq_out:
    /* mutex is locked so release it here.. */
    mutex_unlock(&pmetrics_device_elem->irq_process.irq_track_mtx);

    return ret_val;
}

/*
 * Lock on to the mutex and remove all the irq and cq track nodes.
 * set the current active scheme to INT_NONE.
 * NOTE: This will grab the irq mutex and releases.
 */
int init_irq_track(struct metrics_device_list
        *pmetrics_device_elem, enum nvme_irq_type  irq_active)
{
    int ret_val;
    /* locking on IRQ MUTEX here for irq track ll access */
    mutex_lock(&pmetrics_device_elem->irq_process.irq_track_mtx);
    /* Initialize active irq to INT_NONE */
    ret_val = disable_active_irq(pmetrics_device_elem,
            pmetrics_device_elem->metrics_device->irq_active.irq_type);
    /* Unlock IRQ MUTEX as we are done with updated irq track list */
    mutex_unlock(&pmetrics_device_elem->irq_process.irq_track_mtx);

    return ret_val;
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
    /* pointer to the pci device */
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->pdev;

#ifdef DEBUG
    /* If mutex is not locked then exit here */
    if (!mutex_is_locked(&pmetrics_device_elem->irq_process.irq_track_mtx)) {
        LOG_ERR("Mutex should have been locked before this...");
        /* Mutex is not locked so exiting */
        return -EINVAL;
    }
#endif

    /* clean up and free all IRQ linked list nodes */
    deallocate_irq_trk(pmetrics_device_elem);

    /* disable the PIN interrupts*/
    nvme_disable_pin(pdev);

    /* Perform setting of IRQ to none based on active scheme of IRQ */
    if (irq_active == INT_MSI_SINGLE) {
        /* Only one interrupt so call disable once */
        pci_disable_msi(pdev);
        LOG_DBG("INT_MSI SINGLE Disabled");
    } else if (irq_active == INT_MSI_MULTI) {
        /* TODO loop and disable multiple times..*/
        LOG_DBG("INT_MSI-Multiple Disabled (not done yet!!)");
    } else if (irq_active == INT_MSIX) {
        /* disable OS to generate MISX */
        pci_disable_msix(pdev);
        LOG_DBG("INT_MSIX Disabled");
    } else {
        /* Calling the interrupt disable functions */
        LOG_DBG("No Active IRQ scheme to disable...");
    }
    /* Now we can Set IRQ type to INT_NONE */
    pmetrics_device_elem->metrics_device->irq_active.irq_type = INT_NONE;
    pmetrics_device_elem->metrics_device->irq_active.num_irqs = 0;

    /* Reinitialize irq linked list for this device. */
    INIT_LIST_HEAD(&(pmetrics_device_elem->irq_process.irq_track_list));

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
        *pmetrics_device_elem, struct interrupts *irq_new)
{
    int ret_val = SUCCESS;
    struct nvme_device *pnvme_dev = pmetrics_device_elem->metrics_device;
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->pdev;
    u16 msi_offset;
    u16 mc_val;

    /* Check if the EN bit is set and return failure if set */
    if (readl(&pnvme_dev->nvme_ctrl_space->cc) & NVME_CC_ENABLE) {
        LOG_ERR("IRQ Scheme cannot change when CC.EN bit is set!!");
        LOG_ERR("Call Disable or Disable completely first...");
        return -EINVAL;
    }

    /* Switch based on new irq type desired */
    switch (irq_new->irq_type) {
    case INT_MSI_SINGLE: /* MSI Single interrupt settings */
        if (irq_new->num_irqs > MAX_IRQ_VEC_MSI_SIN) {
            LOG_ERR("IRQ vectors cannot be > %d in MSI Single IRQ!!",
                      MAX_IRQ_VEC_MSI_SIN);
            return -EINVAL;
        }
        /* Check if the card Supports MSI capability */
        if (check_cntlr_cap(pdev, INT_MSI_SINGLE, &msi_offset) < 0) {
            LOG_ERR("Controller does not support for MSI capability!!");
            return -EINVAL;
        }
        break;
    case INT_MSI_MULTI: /* MSI Multi interrupt settings */
        if (irq_new->num_irqs > MAX_IRQ_VEC_MSI_MUL) {
            LOG_ERR("IRQ vectors cannot be > %d in MSI Multi IRQ!!",
                    MAX_IRQ_VEC_MSI_MUL);
            return -EINVAL;
        }
        /* Check if the card Supports MSI capability */
        if (check_cntlr_cap(pdev, INT_MSI_MULTI, &msi_offset) < 0) {
            LOG_ERR("Controller does not support for MSI capability!!");
            return -EINVAL;
        } else {
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
        }
        break;
    case INT_MSIX: /* MSI-X interrupt settings */
        /* First check if num irqs req are greater than MAX MSIX SUPPORTED */
        if (irq_new->num_irqs > MAX_IRQ_VEC_MSI_X) {
            LOG_ERR("IRQ vectors cannot be > %d in MSI-X IRQ!!",
                    MAX_IRQ_VEC_MSI_X);
            return -EINVAL;
        }
        /* Check if the card Supports MSIX capability */
        if (check_cntlr_cap(pdev, INT_MSIX, &msi_offset) < 0) {
            LOG_ERR("Controller does not support for MSI-X capability!!");
            return -EINVAL;
        } else {
            /* compute MSI-X MXC offset if MSI-X is supported */
            msi_offset += 2;
            /* Read MSIX-MXC value */
            pci_read_config_word(pdev, msi_offset, &mc_val);
            /* check if Table size of MSIXCAP supports requested irqs.
             * as TS is 0 based and num_irq is 1 based, so we add 1 */
            if (irq_new->num_irqs > ((mc_val & MSIX_TS) + 1)) {
                LOG_ERR("IRQs = %d exceed MSI-X table size = %d", irq_new->
                        num_irqs, mc_val & MSIX_TS);
                /* does not support the requested irq's*/
                return -EINVAL;
            } /* if table size */
        } /* if msix cap */
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
 */
static int set_msix(struct metrics_device_list *pmetrics_device_elem,
        u16 num_irqs)
{
    int ret_val = SUCCESS;
    int i;
    struct msix_entry msix_entries[num_irqs];
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->pdev;

    /* Assign irq entries from 0 to n-1 */
    for (i = 0; i < num_irqs; i++) {
        msix_entries[i].entry = i;
    }

    /* Allocate msix interrupts to this device */
    ret_val = pci_enable_msix(pdev, msix_entries, num_irqs);
    if (ret_val) {
        LOG_ERR("MSI-X: Error = %d can't enable MSI-X!!", ret_val);
        return -EINVAL;
    }

    /* Request irq on each interrupt vector */
    for (i = 0; i < num_irqs; i++) {
        /* If request fails on any interrupt vector then fail here */
        if (request_irq(msix_entries[i].vector, tophalf_isr, IRQF_DISABLED
                | IRQF_SHARED, "msi-x", pmetrics_device_elem) < 0) {
            LOG_ERR("Request IRQ failed...");
            ret_val = -EINVAL;
            /* As we are allocating memory for one node at a time
             * failing here needs freeing up memory previously allocated */
            goto free_msix;
        } else {
            /* Add node after determining interrupt vector req is successful */
            LOG_DBG("Add Node for Vector = %d", msix_entries[i].vector);
            if (add_irq_node(pmetrics_device_elem, msix_entries[i].vector,
                    msix_entries[i].entry) < 0) {
                LOG_ERR("MSI-X: can't add irq node!!");
                ret_val = -ENOMEM;
                goto free_msix;
            } /* end of if add_irq_node */
        } /* end of if request_irq */
    } /* end of for num_irqs */

    return ret_val;

free_msix:
    /* clean up all nodes even if one irq request fails */
    deallocate_irq_trk(pmetrics_device_elem);
    return ret_val;
}

/*
 * set the irq to MSI Single interrupt scheme.
 */
static int set_msi_single(struct metrics_device_list *pmetrics_device_elem)
{
    struct pci_dev *pdev = pmetrics_device_elem->metrics_device->pdev;
    int ret_val = SUCCESS;

    /* Allocate one interrupt to this device */
    ret_val = pci_enable_msi(pdev);
    if (ret_val) {
        LOG_ERR("MSI-SINGLE : Error = %d can't enable msi!!", ret_val);
        return ret_val; /* exit from here */
    }

    /* request irq with top half handler and int vec. */
    ret_val = request_irq(pdev->irq, tophalf_isr, 0,
            "dnvme-msi-s", pmetrics_device_elem);
    if (ret_val < 0) {
        LOG_ERR("MSI-SINGLE : Error can't get assigned irq = %i", pdev->irq);
        return ret_val; /* exit from here */
    }

    LOG_DBG("Req Irq = %d", pdev->irq);

    /* Add node after determining interrupt vector is successful */
    ret_val = add_irq_node(pmetrics_device_elem, pdev->irq, 0);
    if (ret_val != SUCCESS) {
        LOG_ERR("MSI-SINGLE: can't add irq node!!");
        return ret_val; /* exit from here */
    }

    return ret_val;
}

/*
 * mask_interrupts - Determine the type of masking required based
 * on the interrupt scheme active. Mask interrupts for MSI-Single,
 * MSI- Multi.
 */
static void mask_interrupts(int irq_no, struct  nvme_device *metrics_device)
{
    /* handle all masking here */
    switch (metrics_device->irq_active.irq_type) {
    case INT_MSI_SINGLE:
    case INT_MSI_MULTI:
        /* Mask INTMS register for the int generated */
        writel((0x1 << irq_no), &metrics_device->nvme_ctrl_space->intms);
        break;
    case INT_MSIX:
        LOG_DBG("TODO: Mask for MSI-X to do in future.");
        break;
    case INT_NONE:
        LOG_DBG("INT_NONE should not be fired...");
        break;
    default:
        LOG_ERR("Unknown interrupt type to Mask...");
        break;
    }
}

/*
 * ummask_interrupts - Determine the type of interrupt scheme and
 * unmask interrupts for MSI-Single, MSI- Multi.
 */
static void ummask_interrupts(int irq_no, struct  nvme_device *metrics_device)
{
    /* handle all unmasking here */
    switch (metrics_device->irq_active.irq_type) {
    case INT_MSI_SINGLE:
    case INT_MSI_MULTI:
        /* unMask INTMC register for the int generated */
        writel((0x1 << irq_no), &metrics_device->nvme_ctrl_space->intmc);
        break;
    case INT_MSIX:
        LOG_DBG("TODO: ummask for MSI-X to do in future.");
        break;
    case INT_NONE:
        LOG_DBG("INT_NONE should not be fired...");
        break;
    default:
        LOG_ERR("Unknown interrupt type to Mask...");
        break;
    }
}

/*
 * Top half isr responds to the interrupt by masking the corresponding irq
 * vector if required and scheduling the bottom half processing. Initilally
 * it grabs the spin_lock to avoid multi-core systems from kernel preemption.
 */
irqreturn_t tophalf_isr(int int_vec, void *dev_id)
{
    /* Point to the right hardware item using this dev_id */
    struct  metrics_device_list *pmetrics_device_elem = dev_id;
    int irq_no;

    /* Acquire spin lock and enter into critical section */
    spin_lock(&pmetrics_device_elem->irq_process.isr_spin_lock);
    /* Compute the index for the int vector fired. */
    irq_no = get_ivec_index(pmetrics_device_elem, int_vec);

    /* TODO: Remove this debug, before releasing */
    LOG_DBG("Int_vec:Irq_no = %d:%d", int_vec, irq_no);
    /* Mask the interrupts if required */
    mask_interrupts(irq_no, pmetrics_device_elem->metrics_device);
    /* Checking if the irq fired is not a spurious interrupt */
    if (irq_no < pmetrics_device_elem->metrics_device->irq_active.
            num_irqs) {
        /* Set the irq that was fired in its place in the array */
        pmetrics_device_elem->irq_process.wrk_sched.int_vec_ctx[irq_no] = 1;
    }
    /* Queue the bh. Don't worry about multiple en-queuing as bottom half
     * will handle for every int fired. */
    if (schedule_work(&pmetrics_device_elem->irq_process.wrk_sched.sched_wq)
            == 0) {
        LOG_DBG("Work item already scheduled...");
    }
    /* unlock as we are done with critical section */
    spin_unlock(&pmetrics_device_elem->irq_process.isr_spin_lock);

    return IRQ_HANDLED;
}

/*
 * Seek the slot for which the top half has triggered the interrupt.
 * Returns the interrupt vector which got set in the Top Half when
 * IRQ was fired. Before returning, it resets this position in the array.
 * If no slot is set in the array, returns MAX + 1 to indicate invalid
 * index.
 */
static u16 get_irq_enabled(u16 int_vec[])
{
    int i;
    for (i = 0; i < (sizeof(int_vec)/sizeof(int_vec[0])); i++) {
        if (int_vec[i] != 0) {
            int_vec[i] = 0;
            return i;
        }
    }
    /* If nothing is set then return MAX for identification */
    return MAX_IRQ_VEC_MSI_X;
}

/*
 * set_irq_node is the bottom half processing for the interrupt that got fired.
 * Search the CQ node for which the IRQ is fired and set this flag. Increment
 * count by one.
 */
static void set_irq_cq_nodes(struct metrics_device_list *pmetrics_device_elem,
        u16 int_vec)
{
    struct  irq_track     *pirq_node;     /* Pointer to irq node */
    struct  irq_cq_track  *picq_node;  /* Pointer to cq node  */

    /* Loop for each irq node */
    list_for_each_entry(pirq_node, &pmetrics_device_elem->irq_process.
            irq_track_list, irq_list_hd) {
        if (int_vec == pirq_node->int_vec) {
            list_for_each_entry(picq_node, &pirq_node->irq_cq_track,
                    irq_cq_head) {
                picq_node->isr_fired = 1;
                picq_node->isr_count++;
                LOG_DBG("ISR Count = %d for CQ ID = %d",
                        picq_node->isr_count, picq_node->cq_id);
            }
            return;
        }
    }
    LOG_DBG("IRQ node does not exist");
}

/*
 * This is the work item that gets scheduled when the ISR gets fired.
 * The bottom halfs are queued up and we should not worry about multiple
 * interrupts getting fired up as we store them in the array as a lookup
 * table.
 */
static void bh_callback(struct work_struct *work)
{
    /* Point to the metrics device using container_of mutiple times */
    struct work_container *pwork_cnt =
            container_of(work, struct work_container, sched_wq);
    struct irq_processing *pirq_process =
            container_of(pwork_cnt, struct irq_processing, wrk_sched);
    struct metrics_device_list *pmetrics_device_elem =
            container_of(pirq_process, struct metrics_device_list, irq_process);
    u16 irq_no = 0;
    u16 int_vec = 0;

    /* Check if irq is enabled */
    irq_no = get_irq_enabled(pmetrics_device_elem->irq_process.
            wrk_sched.int_vec_ctx);
    /* Process for any irq that would have been set in the Array by Top Half */
    while (irq_no < MAX_IRQ_VEC_MSI_X) {
        /* lookup for int_vec using irq_no */
        int_vec = get_int_vec(pmetrics_device_elem, irq_no);
        /* lock irq mutex as we will access the irq nodes */
        mutex_lock(&pmetrics_device_elem->irq_process.irq_track_mtx);
        /* Set the values in the node */
        set_irq_cq_nodes(pmetrics_device_elem, int_vec);
        /* unlock as we are done wiht updating the irq nodes */
        mutex_unlock(&pmetrics_device_elem->irq_process.irq_track_mtx);
        /* unmask the irq for which it was masked in Top Half */
        ummask_interrupts(irq_no, pmetrics_device_elem->metrics_device);
        LOG_DBG("ISR = %d is serviced in Bottom Half", int_vec);
        /* Get the irq_no that was set in sync/asynchronously */
        irq_no = get_irq_enabled(pmetrics_device_elem->irq_process.
                wrk_sched.int_vec_ctx);
    }
}

/*
 * ISR Initialization routine for resetting IRQ parameters.
 * Initialize spin_lock, bottom half and irq array.
 */
void isr_init(struct  metrics_device_list *pmetrics_device_elem)
{
    int i;

#ifdef DEBUG
    /* If mutex is not locked then exit here */
    if (!mutex_is_locked(&pmetrics_device_elem->irq_process.irq_track_mtx)) {
        /* Mutex is not locked so exiting */
        LOG_ERR("Mutex should have been locked before this...");
        return;
    }
#endif

    /* initialize the bottom half call back with the work struct */
    INIT_WORK(&pmetrics_device_elem->irq_process.wrk_sched.sched_wq,
            bh_callback);
    /* Initialize the array of all the irq set to 0 */
    for (i = 0; i < (sizeof(pmetrics_device_elem->irq_process.wrk_sched.
            int_vec_ctx)/sizeof(pmetrics_device_elem->irq_process.wrk_sched.
                int_vec_ctx[0])); i++) {
        pmetrics_device_elem->irq_process.wrk_sched.int_vec_ctx[i] = 0;
    }
}

/*
 * Add a irq track node in irq_track linked list. Allocates memory for one
 * irq_track node, sets up the values for this node. Initialize the CQ_track
 * node for this irq_node then add the irq_node to the itq_track linked list.
 */
static int add_irq_node(struct  metrics_device_list *pmetrics_device_elem,
        u16 int_vec, u16 irq_no)
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

    /* Init irq cq linked list */
    INIT_LIST_HEAD(&irq_trk_node->irq_cq_track);

    /* Add this irq node element to the end of the list */
    list_add_tail(&irq_trk_node->irq_list_hd, &pmetrics_device_elem->
            irq_process.irq_track_list);

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
    irq_cq_node->isr_count = 0;
    irq_cq_node->isr_fired = 0;

    /* Add this cq node to the end of the list */
    list_add_tail(&irq_cq_node->irq_cq_head,
            &pirq_trk_node->irq_cq_track);
    return SUCCESS;
}

/*
 * Get the index into the list for the given int vector in the irq linked list.
 * Search the irq_track linked list, if the int_vec matches then return the
 * irq_no for this int_vector. If search fails then return MAX_MSIX + 1 to
 * indicate called that this search failed.
 */
static u16 get_ivec_index(struct  metrics_device_list *pmetrics_device_elem,
        u16 int_vec)
{
    struct  irq_track     *pirq_node;     /* Pointer to irq node */
    /* Loop for the irq node in irq track list */
    list_for_each_entry(pirq_node, &pmetrics_device_elem->irq_process.
            irq_track_list, irq_list_hd) {
            /* if the interrupt vector matches return the irq no */
            if (int_vec == pirq_node->int_vec) {
                return pirq_node->irq_no;
            }
    }
    return MAX_IRQ_VEC_MSI_X;
}

/*
 * Get the corresponding interrupt vector for the given irq_no in the
 * irq track list. Search the irq_track nodes and match the irq no,
 * if found return the interrupt vector. If search does not find the
 * corresponding irq_no node, then return MAX_IRQ_VEC_MSI_X + 1 to
 * indicate invalid value to the caller
 */
u16 get_int_vec(struct  metrics_device_list *pmetrics_device_elem,
                    u16 irq_no)
{
    struct  irq_track     *pirq_node;     /* Pointer to irq node */
    /* Loop for the irq node in irq track list */
    list_for_each_entry(pirq_node, &pmetrics_device_elem->irq_process.
            irq_track_list, irq_list_hd) {
            /* if the irq no matches then return this irq vector */
            if (irq_no == pirq_node->irq_no) {
                return pirq_node->int_vec;
            }
    }
    return MAX_IRQ_VEC_MSI_X;
}

/*
 * Find_irq_node - return the pointer to the irq node in the irq track list
 * for the irq_vector if found otherwise return NULL.
 */
static struct irq_track *find_irq_node(
        struct  metrics_device_list *pmetrics_device_elem, u16 irq_vector)
{
    struct  irq_track     *pirq_node;     /* Pointer to irq node */

    /* Loop for the irq node in irq track list */
    list_for_each_entry(pirq_node, &pmetrics_device_elem->irq_process.
            irq_track_list, irq_list_hd) {
            /* if the irq vector matches then return this irq node */
            if (irq_vector == pirq_node->int_vec) {
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
 * given irq_vector and inside this irq_node, find the corresponding
 * CQ node. using this pointer to CQ node, delete this CQ node and
 * free up the memory.
 */
int remove_icq_node(struct  metrics_device_list
        *pmetrics_device, u16 cq_id, u16 irq_vector)
{
    struct irq_track *pirq_node = NULL;
    struct irq_cq_track *picq_node = NULL;

    LOG_DBG("Call to remove the ICQ = %d for ivec = %d",
            cq_id, irq_vector);
    /* Get the Irq node for given irq vector */
    pirq_node = find_irq_node(pmetrics_device, irq_vector);
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
 */
void deallocate_irq_trk(struct  metrics_device_list *pmetrics_device_elem)
{
    struct  irq_track   *pirq_trk_list; /* Type to use as loop cursor       */
    struct  irq_track   *pirq_trk_next; /* Same type to use as temp storage */

    LOG_ERR("Deallocate IRQ Track...");
    /* Loop for each irq node */
    list_for_each_entry_safe(pirq_trk_list, pirq_trk_next,
            &pmetrics_device_elem->irq_process.irq_track_list, irq_list_hd) {
        LOG_DBG("Int Vec:Irq No = %d:%d", pirq_trk_list->int_vec,
                pirq_trk_list->irq_no);
        /* remove all the cq nodes for this irq node */
        dealloc_all_icqs(pirq_trk_list);
        list_del(&pirq_trk_list->irq_cq_track);
        free_irq(pirq_trk_list->int_vec, pmetrics_device_elem);
        kfree(pirq_trk_list);
    }
}
