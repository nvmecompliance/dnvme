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

#ifndef _DNVME_IRQ_H_
#define _DNVME_IRQ_H_

#include "dnvme_ds.h"
#include "sysdnvme.h"
#include "dnvme_queue.h"
#include "dnvme_interface.h"
#include "dnvme_sts_chk.h"
#include "definitions.h"

/* Max IRQ vectors for MSI SINGLE IRQ scheme */
#define     MAX_IRQ_VEC_MSI_SIN     1

/* Max IRQ vectors for MSI MULTI IRQ scheme */
#define     MAX_IRQ_VEC_MSI_MUL     32

/* Max IRQ vectors for MSI X IRQ scheme */
#define     MAX_IRQ_VEC_MSI_X       2048

/* Command offset in PCI space */
#define     CMD_OFFSET              0x4

/* Bit mask for pin# interrupts */
#define     PIN_INT_BIT_MASK        (1 << 10)

/* Table size mask bits for MSIX */
#define     MSIX_TS                 0x7FF

/* Mask for MSIX enable */
#define     MSIX_ENABLE             0x8000

/* Mask for MSI enable bit */
#define     MSI_ENABLE             0x1

/* Mask for MSI Multi message enable bits */
#define     MSI_MME                 0x70

/* Mask for MSIX table BIR identification */
#define     MSIX_TBIR_MASK           0x7

/* Entry size in bytes for each entry in MSIX table */
#define     MSIX_ENTRY_SIZE         16

/* MSIX Table vector control offset in the entry */
#define     MSIX_VEC_CTRL           12

/* Interrupt vector mask set(IVMS) register offset */
#define     INTMS_OFFSET            0x0C

/* Interrupt vector mask clear(IVMC) register offset */
#define     INTMC_OFFSET            0x10
/*
 * nvme_set_irq will set the new interrupt scheme for this device regardless
 * of the current irq scheme that is present.
 */
int nvme_set_irq(struct metrics_device_list *pmetrics_device_elem,
        struct interrupts *irq_new);

/*
 * Lock on to the mutex and remove all the irq and cq track nodes.
 * set the current active scheme to INT_NONE.
 */
int init_irq_track(struct metrics_device_list
        *pmetrics_device_elem, enum nvme_irq_type  irq_active);

/*
 * deallocate irq trak, will delete the cq nodes of each irq node, deallocates
 * memory allocated to each of this node. Reinitalize the linked list to
 * contain no elements.
 */
void deallocate_irq_trk(struct  metrics_device_list *pmetrics_device_elem);

/*
 * add_icq_node : This function will add a Completion Q node into the
 * interrupt linked list with the given cq id.
 */
int add_icq_node(struct irq_track *pirq_trk_node, u16 cq_id);

/*
 * Initialization work for isr is done in this function.
 */
void isr_init(struct  metrics_device_list *pmetrics_device_elem);

/*
 * ISR callback routine - When any irq is fired the driver invokes the
 * top half isr to process the irq request.
 */
irqreturn_t tophalf_isr(int int_vec, void *dev_id);

/*
 * Deletes the given cq node for the corresponding irq_no. If either the
 * irq no is not found or the cq id is not in the list it returns invalid.
 */
int remove_icq_node(struct  metrics_device_list
        *pmetrics_device, u16 cq_id, u16 irq_no);

/*
 * Get the corresponding interrupt vector for the given irq_no in the
 * irq track list.
 */
u16 get_int_vec(struct  metrics_device_list *pmetrics_device_elem,
                    u16 irq_no);

/*
 * Set the IO CQ interrupt vector for the given cq_id. Add a node in the
 * IRQ tracklist with this CQ entry.
 */
int set_ivec_cq(struct  metrics_device_list *pmetrics_device_elem, u16 cq_id,
        u16 irq_vector, u8 *irq_enabled, u16 *int_vec);

/*
 * reap_inquiry_isr will process reap inquiry for the given cq using irq_vec
 * and isr_fired flags from two nodes, public cq node and irq_track list node.
 * It fills the num_remaining with number of elements remaining or 0 based on
 * CE entries. If the IRQ aggregation is enabled it returns 0 if aggregation
 * limit is not reached.
 */
int reap_inquiry_isr(struct metrics_cq  *pmetrics_cq_node,
        struct  metrics_device_list *pmetrics_device_elem, u16 *num_remaining);

/*
 * Reset ISR will reset the ISR fired and ISR count to 0 when called for
 * the given cq node.
 */
int reset_isr_reap(struct metrics_cq  *pmetrics_cq_node,
        struct  metrics_device_list *pmetrics_device_elem);

#endif
