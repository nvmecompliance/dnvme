#include <linux/kernel.h>
#include <linux/module.h>

#include "definitions.h"
#include "sysdnvme.h"
#include "dnvme_ds.h"
#include "dnvme_queue.h"
#include "ut_reap_inq.h"

void unit_test_reap_inq(struct  metrics_device_list *pmetrics_device)
{
    struct metrics_cq  *pmetrics_cq_node;   /* ptr to cq node       */
    u8 *q_head_ptr;                         /* head ptr in cq       */
    struct cq_completion *cq_entry;         /* cq entry format      */
    u16 comp_entry_size;
    u16 num = 0;
    u16 tmpbit = 1;

    /* Lookup the CQ for which the reap inquiry is requested */
    list_for_each_entry(pmetrics_cq_node, &pmetrics_device->metrics_cq_list,
            cq_list_hd) {
        /* Test: CQ Empty, CQ ID = 1 */
        /* Empty already . */

        /* Test: CQ Full, CQ ID = 2 */
        if (pmetrics_cq_node->public_cq.q_id == 2) {
            comp_entry_size = (pmetrics_cq_node->private_cq.size) /
                                    (pmetrics_cq_node->public_cq.elements);
            /* point the head ptr to corresponding head ptr */
            q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr +
                    (comp_entry_size * pmetrics_cq_node->public_cq.head_ptr);

            while (1) {
                cq_entry = (struct cq_completion *)q_head_ptr;
                cq_entry->phase_bit = 1;
                q_head_ptr += comp_entry_size;
                if (q_head_ptr == pmetrics_cq_node->private_cq.vir_kern_addr +
                                    pmetrics_cq_node->private_cq.size) {
                    break;
                }
            }
        }
        /* Test: CQ 10 Elements CQ ID = 3 */
        if (pmetrics_cq_node->public_cq.q_id == 3) {
            comp_entry_size = (pmetrics_cq_node->private_cq.size) /
                                    (pmetrics_cq_node->public_cq.elements);
            /* point the head ptr to corresponding head ptr */
            q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr +
                    (comp_entry_size * pmetrics_cq_node->public_cq.head_ptr);
            num = 0;
            while (1) {
                cq_entry = (struct cq_completion *)q_head_ptr;
                cq_entry->phase_bit = 1;
                q_head_ptr += comp_entry_size;
                num++;
                if ((q_head_ptr == pmetrics_cq_node->private_cq.vir_kern_addr +
                                    pmetrics_cq_node->private_cq.size) ||
                                    (num == 10)) {
                    LOG_NRM("Num = %d", num);
                    break;
                }
            }
        }
        /* Test: CQ Wrapped around CQ ID = 4 */
        if (pmetrics_cq_node->public_cq.q_id == 4) {
            comp_entry_size = (pmetrics_cq_node->private_cq.size) /
                                    (pmetrics_cq_node->public_cq.elements);

            /* point the head ptr to corresponding head ptr */
            q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr +
                    (comp_entry_size * pmetrics_cq_node->public_cq.head_ptr);
            LOG_DBG("CQ 4 hd ptr = %llx", (u64)q_head_ptr);
            num = 0;
            tmpbit = 1;
            while (1) {
                cq_entry = (struct cq_completion *)q_head_ptr;
                cq_entry->phase_bit = tmpbit;
                q_head_ptr += comp_entry_size;
                num++;
                if (q_head_ptr >= pmetrics_cq_node->private_cq.vir_kern_addr +
                                    pmetrics_cq_node->private_cq.size) {
                    /* Roll over */
                    q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr;
                    tmpbit = 0;
                    LOG_DBG("Roll CQ 4 hd ptr = %llx", (u64)q_head_ptr);
                }

                if (num == 128) {
                    LOG_NRM("Entries in Q = %d", num);
                    break;
                }
            }
            /* Set Head Pointer in between the Q */
            pmetrics_cq_node->public_cq.head_ptr = 80;
        }
        /* Test: CQ tail_ptr is last element CQ ID = 5 */
        if (pmetrics_cq_node->public_cq.q_id == 5) {
            comp_entry_size = (pmetrics_cq_node->private_cq.size) /
                                    (pmetrics_cq_node->public_cq.elements);

            /* point the head ptr to corresponding head ptr */
            q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr +
                    (comp_entry_size * pmetrics_cq_node->public_cq.head_ptr);
            LOG_DBG("CQ 5 hd ptr = %llx", (u64)q_head_ptr);
            num = 0;
            tmpbit = 1;
            while (1) {
                cq_entry = (struct cq_completion *)q_head_ptr;
                cq_entry->phase_bit = tmpbit;
                q_head_ptr += comp_entry_size;
                num++;
                if (q_head_ptr == pmetrics_cq_node->private_cq.vir_kern_addr +
                                    pmetrics_cq_node->private_cq.size) {
                    /* Roll over */
                    q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr;
                    tmpbit = 0;
                }

                if (num == 99) {
                    LOG_NRM("Entries in Q = %d", num);
                    break;
                }
            }
        }
        /* Test: CQ tail ptr is first elemnet CQ ID = 6 */
        if (pmetrics_cq_node->public_cq.q_id == 6) {
            comp_entry_size = (pmetrics_cq_node->private_cq.size) /
                                    (pmetrics_cq_node->public_cq.elements);

            /* point the head ptr to corresponding head ptr */
            q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr +
                    (comp_entry_size * pmetrics_cq_node->public_cq.head_ptr);
            LOG_DBG("CQ 6 hd ptr = %llx", (u64)q_head_ptr);
            num = 0;
            tmpbit = 1;
            while (1) {
                cq_entry = (struct cq_completion *)q_head_ptr;
                cq_entry->phase_bit = tmpbit;
                q_head_ptr += comp_entry_size;
                num++;
                if (q_head_ptr == pmetrics_cq_node->private_cq.vir_kern_addr +
                                    pmetrics_cq_node->private_cq.size) {
                    /* Roll over */
                    q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr;
                    tmpbit = 0;
                }

                if (num == 1) {
                    LOG_NRM("Entries in Q = %d", num);
                    break;
                }
            }
        }
    }
}

