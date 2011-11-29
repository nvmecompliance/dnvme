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
int isr_init(struct  metrics_device_list *pmetrics_device_elem);

/*
 * ISR callback routine - When any irq is fired the driver invokes the
 * top half isr to process the irq request.
 */
irqreturn_t tophalf_isr(int int_vec, void *dev_id);

/*
 * Sets the PIN# based interrupt to enable state. This is done to while
 * cleaning up and returning from the driver.
 */
void nvme_enable_pin(struct pci_dev *dev);

/*
 * Deletes the given cq node for the corresponding irq_vector. If either the
 * irq vector is not found or the cq id is not in the list it returns invalid.
 */
int remove_icq_node(struct  metrics_device_list
        *pmetrics_device, u16 cq_id, u16 irq_vector);

/*
 * Get the corresponding interrupt vector for the given irq_no in the
 * irq track list.
 */
u16 get_int_vec(struct  metrics_device_list *pmetrics_device_elem,
                    u16 irq_no);

#endif
