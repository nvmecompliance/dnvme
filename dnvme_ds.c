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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <asm/segment.h>

#include "definitions.h"
#include "sysdnvme.h"
#include "dnvme_interface.h"
#include "sysfuncproto.h"
#include "dnvme_queue.h"

#define IDNT_L1             "\n\t"
#define IDNT_L2             "\n\t\t"
#define IDNT_L3             "\n\t\t\t"
#define IDNT_L4             "\n\t\t\t\t"
#define IDNT_L5             "\n\t\t\t\t\t"
#define IDNT_L6             "\n\t\t\t\t\t\t"

#define SIZE_OF_WORK        256

/* local static functions */
static loff_t meta_nodes_log(struct file *file, loff_t pos,
    struct  metrics_device_list *pmetrics_device);
static loff_t irq_nodes_log(struct file *file, loff_t pos,
    struct  metrics_device_list *pmetrics_device_elem);


int driver_log(struct nvme_file *n_file)
{
    struct file *file;  /* File pointer where the data is written */
    loff_t pos = 0;     /* offset into the file */
    int dev = 0;        /* local var for tracking no. of devices */
    int i = 0;          /* local var to track no. of SQ's and CQ's */
    int cmd = 0;        /* Local variable to track no. of cmds */
    mm_segment_t oldfs; /* Old file segment to map between Kernel and usp */
    struct  metrics_sq  *pmetrics_sq_list;        /* SQ linked list */
    u8 work[SIZE_OF_WORK];
    struct  metrics_cq  *pmetrics_cq_list;        /* CQ linked list */
    struct  metrics_device_list *pmetrics_device; /* Metrics device list */
    struct  cmd_track  *pcmd_track_list;          /* cmd track linked list */
    u8 *filename = NULL;
    int err = SUCCESS;
    struct nvme_file *user_data = NULL;


    /* Allocating memory for user struct in kernel space */
    user_data = kmalloc(sizeof(struct nvme_file), GFP_KERNEL);
    if (user_data == NULL) {
        LOG_ERR("Unable to alloc kernel memory to copy user data");
        err = -ENOMEM;
        goto fail_out;
    }
    if (copy_from_user(user_data, n_file, sizeof(struct nvme_file))) {
        LOG_ERR("Unable to copy from user space");
        err = -EFAULT;
        goto fail_out;
    }

    /* Allocating memory for the data in kernel space, add 1 for a NULL term */
    filename = kmalloc(user_data->flen+1, (GFP_KERNEL | __GFP_ZERO));
    if (NULL == filename) {
        LOG_ERR("Unable to allocate kernel memory");
        err = -ENOMEM;
        goto fail_out;
    }

    /* Copy userspace buffer to kernel memory */
    if (copy_from_user(filename, user_data->file_name, user_data->flen)) {
        LOG_ERR("Unable to copy from user space");
        err = -EFAULT;
        goto fail_out;
    }

    /* If the user didn't provide a NULL term, we will to avoid problems */
    filename[user_data->flen] = '\0';

    LOG_DBG("Dumping dnvme metrics to output file: %s", filename);
    oldfs = get_fs();
    set_fs(KERNEL_DS);

    file = filp_open(filename, O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (file) {

        /* Loop through the devices */
        list_for_each_entry(pmetrics_device, &metrics_dev_ll,
            metrics_device_hd) {

            /* Get the variable from metrics structure and write to file */
            snprintf(work, SIZE_OF_WORK, "metrics_device_list[%d]\n", dev++);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "Minor Number = %d\n",
                pmetrics_device->metrics_device->private_dev.minor_no);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "open_flag = %d\n",
                pmetrics_device->metrics_device->private_dev.open_flag);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "pdev = 0X%llX\n",
                (u64)pmetrics_device->metrics_device->private_dev.pdev);
            vfs_write(file, work, strlen(work), &pos);

            snprintf(work, SIZE_OF_WORK, "bar0 = 0X%llX\n",
                (u64)pmetrics_device->metrics_device->private_dev.bar0);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "bar1 = 0X%llX\n",
                (u64)pmetrics_device->metrics_device->private_dev.bar1);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "bar2 = 0X%llX\n",
                (u64)pmetrics_device->metrics_device->private_dev.bar2);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "ctrlr_regs = 0X%llX\n",
                (u64)pmetrics_device->metrics_device->private_dev.ctrlr_regs);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "dmadev = 0X%llX\n",
                (u64)pmetrics_device->metrics_device->private_dev.dmadev);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "prp_page_pool = 0X%llX\n", (u64)
                pmetrics_device->metrics_device->private_dev.prp_page_pool);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "spcl_dev = 0X%llX\n",
                (u64)pmetrics_device->metrics_device->private_dev.spcl_dev);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK,
                "Interrupts:Active Scheme (S=0/M=1/X=2/N=3) = %d\n",
                pmetrics_device->metrics_device->public_dev.irq_active.
                irq_type);
            vfs_write(file, work, strlen(work), &pos);
            snprintf(work, SIZE_OF_WORK, "Interrupts:num_irqs = %d\n",
                pmetrics_device->metrics_device->public_dev.irq_active.
                num_irqs);
            vfs_write(file, work, strlen(work), &pos);
            /* Looping through the available CQ list */
            list_for_each_entry(pmetrics_cq_list, &pmetrics_device->
                metrics_cq_list, cq_list_hd) {

                /* Get the variable from CQ strucute and write to file */
                snprintf(work, SIZE_OF_WORK,
                    IDNT_L1"pmetrics_cq_list->public_cq[%d]", i);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"q_id = %d",
                    pmetrics_cq_list->public_cq.q_id);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"tail_ptr = %d",
                    pmetrics_cq_list->public_cq.tail_ptr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"head pointer = %d",
                    pmetrics_cq_list->public_cq.head_ptr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"elements = %d",
                    pmetrics_cq_list->public_cq.elements);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"irq enabled = %d",
                    pmetrics_cq_list->public_cq.irq_enabled);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"irq_no = %d",
                    pmetrics_cq_list->public_cq.irq_no);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"pbit_new_entry = %d",
                    pmetrics_cq_list->public_cq.pbit_new_entry);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK,
                    IDNT_L1"pmetrics_cq_list->private_cq[%d]", i++);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"vir_kern_addr = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.vir_kern_addr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"dma_addr_t = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.cq_dma_addr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK,
                    IDNT_L2"contig (1 = Y/(0 = N) = %d",
                    pmetrics_cq_list->private_cq.contig);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"size = %d",
                    pmetrics_cq_list->private_cq.size);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"dbs = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.dbs);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L3"prp_persist:");
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"npages = %d",
                    pmetrics_cq_list->private_cq.prp_persist.npages);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"type = %d",
                    pmetrics_cq_list->private_cq.prp_persist.type);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"vir_kern_addr = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.prp_persist.
                    vir_kern_addr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"vir_prp_list = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.prp_persist.vir_prp_list);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"prp1 = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.prp_persist.prp1);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"prp2 = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.prp_persist.prp2);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"first dma = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.prp_persist.first_dma);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"data_dir = %d",
                    pmetrics_cq_list->private_cq.prp_persist.data_dir);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"data_buf_addr = 0X%llX",
                    (u64)pmetrics_cq_list->
                    private_cq.prp_persist.data_buf_addr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"data_buf_size = %d",
                    pmetrics_cq_list->private_cq.prp_persist.data_buf_size);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"sq = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.prp_persist.sg);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"num_map_pgs = 0X%llX",
                    (u64)pmetrics_cq_list->private_cq.prp_persist.num_map_pgs);
                vfs_write(file, work, strlen(work), &pos);
            } /* End of CQ list */

            i = 0;  /* Reset Q cnt */

            /* looping through available sq list */
            list_for_each_entry(pmetrics_sq_list, &pmetrics_device->
                metrics_sq_list, sq_list_hd) {

                /* Get each member of SQ structure and write to file */
                snprintf(work, SIZE_OF_WORK,
                    IDNT_L1"pmetrics_sq_list->public_sq[%d]", i);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"sq_id = %d",
                    pmetrics_sq_list->public_sq.sq_id);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"Assoc cq_id = %d",
                    pmetrics_sq_list->public_sq.cq_id);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"elements = %d",
                    pmetrics_sq_list->public_sq.elements);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"head_ptr = %d",
                    pmetrics_sq_list->public_sq.head_ptr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"tail_ptr_virt = %d",
                    pmetrics_sq_list->public_sq.tail_ptr_virt);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"tail_ptr = %d",
                    pmetrics_sq_list->public_sq.tail_ptr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK,
                    IDNT_L1"pmetrics_sq_list->private_sq[%d]", i++);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"vir_kern_addr = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.vir_kern_addr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK,
                    IDNT_L2"contig (1 = Y/ 0 = N) = %d",
                    pmetrics_sq_list->private_sq.contig);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"size = %d",
                    pmetrics_sq_list->private_sq.size);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"unique_cmd_id(Cnt) = %d",
                    pmetrics_sq_list->private_sq.unique_cmd_id);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L2"dbs = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.dbs);
                vfs_write(file, work, strlen(work), &pos);

                snprintf(work, SIZE_OF_WORK, IDNT_L3"prp_persist:");
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"npages = %d",
                    pmetrics_sq_list->private_sq.prp_persist.npages);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"vir_kern_addr = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.prp_persist.
                    vir_kern_addr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"type = %d",
                    pmetrics_sq_list->private_sq.prp_persist.type);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"vir_prp_list = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.prp_persist.vir_prp_list);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"prp1 = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.prp_persist.prp1);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"prp2 = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.prp_persist.prp2);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"first dma = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.prp_persist.first_dma);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"data_dir = %d",
                    pmetrics_sq_list->private_sq.prp_persist.data_dir);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"data_buf_addr = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.prp_persist.
                    data_buf_addr);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"data_buf_size = %d",
                    pmetrics_sq_list->private_sq.prp_persist.data_buf_size);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"sg = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.prp_persist.sg);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L4"num_map_pgs = 0X%llX",
                    (u64)pmetrics_sq_list->private_sq.prp_persist.num_map_pgs);
                vfs_write(file, work, strlen(work), &pos);
                snprintf(work, SIZE_OF_WORK, IDNT_L3"cmd track list = ");
                vfs_write(file, work, strlen(work), &pos);
                /* Looping through the cmds if any */
                list_for_each_entry(pcmd_track_list,
                    &(pmetrics_sq_list->private_sq.cmd_track_list),
                    cmd_list_hd) {

                    /* write to file if any cmds exist */
                    snprintf(work, SIZE_OF_WORK, IDNT_L4"cmd track no = %d",
                        cmd++);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L4"unique_id = %d",
                        pcmd_track_list->unique_id);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L4"persist_q_id = %d",
                        pcmd_track_list->persist_q_id);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L4"opcode = %d",
                        pcmd_track_list->opcode);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L4"cmd_set = %d",
                        pcmd_track_list->cmd_set);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L5"prp_nonpersist:");
                    vfs_write(file, work, strlen(work), &pos);
                    /* Printing prp_nonpersist memeber variables */
                    snprintf(work, SIZE_OF_WORK,
                        IDNT_L6"vir_kern_addr = 0X%llX", (u64)
                        pcmd_track_list->prp_nonpersist.vir_kern_addr);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"npages = %d",
                        pcmd_track_list->prp_nonpersist.npages);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"type = %d",
                        pcmd_track_list->prp_nonpersist.type);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"vir_prp_list = 0X%llX",
                        (u64)pcmd_track_list->prp_nonpersist.vir_prp_list);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"prp1 = 0X%llX",
                        (u64)pcmd_track_list->prp_nonpersist.prp1);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"prp2 = 0X%llX",
                        (u64)pcmd_track_list->prp_nonpersist.prp2);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"first dma = 0X%llX",
                         (u64)pcmd_track_list->prp_nonpersist.first_dma);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"data_dir = %d",
                         pcmd_track_list->prp_nonpersist.data_dir);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK,
                        IDNT_L6"data_buf_addr = 0X%llX",
                         (u64)pcmd_track_list->prp_nonpersist.data_buf_addr);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"data_buf_size = %d",
                         pcmd_track_list->prp_nonpersist.data_buf_size);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"sg = 0X%llX",
                         (u64)pcmd_track_list->prp_nonpersist.sg);
                    vfs_write(file, work, strlen(work), &pos);
                    snprintf(work, SIZE_OF_WORK, IDNT_L6"num_map_pgs = 0X%llX",
                         (u64)pcmd_track_list->prp_nonpersist.num_map_pgs);
                    vfs_write(file, work, strlen(work), &pos);
                } /* End of cmd track list */
            } /* End of SQ metrics list */
            pos = meta_nodes_log(file, pos, pmetrics_device);
            pos = irq_nodes_log(file, pos, pmetrics_device);
        } /* End of file writing */

        fput(file);
        filp_close(file, NULL);
    }
    set_fs(oldfs);
    /* Fall through to label in intended */

fail_out:
    if (filename == NULL) {
        kfree(filename);
    }
    if (user_data != NULL) {
        kfree(user_data);
    }
    return err;
}


/*
 * Logging Meta data nodes into user space file.
 */
static loff_t meta_nodes_log(struct file *file, loff_t pos,
     struct metrics_device_list *pmetrics_device)
{
    struct metrics_meta *pmetrics_meta;
    u8 work[SIZE_OF_WORK];
    int i = 0;


    if (pmetrics_device->metrics_meta.meta_dmapool_ptr == NULL) {
        return pos;
    }
    snprintf(work, SIZE_OF_WORK,
        IDNT_L1"pmetrics_device->metrics_meta.meta_dmapool_ptr = 0x%llx",
        (u64)pmetrics_device->metrics_meta.meta_dmapool_ptr);
    vfs_write(file, work, strlen(work), &pos);

    list_for_each_entry(pmetrics_meta, &pmetrics_device->metrics_meta.
            meta_trk_list, meta_list_hd) {
        /* Get each Meta buffer node and write to file */
        snprintf(work, SIZE_OF_WORK,
            IDNT_L2"pmetrics_device->pmetrics_meta[%d]", i++);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK, IDNT_L3"pmetrics_meta->meta_id = %d",
            pmetrics_meta->meta_id);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK,
            IDNT_L3"pmetrics_meta->meta_dma_addr = 0x%llx",
            (u64)pmetrics_meta->meta_dma_addr);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK,
            IDNT_L3"pmetrics_meta->vir_kern_addr = 0x%llx",
            (u64)pmetrics_meta->vir_kern_addr);
        vfs_write(file, work, strlen(work), &pos);
    }
    return pos;
}


/*
 * logging irq nodes into user space file.
 */
static loff_t irq_nodes_log(struct file *file, loff_t pos,
    struct metrics_device_list *pmetrics_device_elem)
{
    u8 work[SIZE_OF_WORK];
    int i = 0;
    struct irq_track *pirq_node;
    struct irq_cq_track *pirq_cq_node;
    struct work_container *pwk_item_curr;  /* Current wk item in the list */


    /* locking on IRQ MUTEX here for irq track ll access */
    mutex_lock(&pmetrics_device_elem->irq_process.irq_track_mtx);

    snprintf(work, SIZE_OF_WORK, "\nirq_process.mask_ptr = 0x%llX",
        (u64)pmetrics_device_elem->irq_process.mask_ptr);
    vfs_write(file, work, strlen(work), &pos);
    snprintf(work, SIZE_OF_WORK, "\nirq_process.irq_type = %d",
        pmetrics_device_elem->irq_process.irq_type);
    vfs_write(file, work, strlen(work), &pos);

    /* Loop for the first irq node in irq track list */
    list_for_each_entry(pirq_node, &pmetrics_device_elem->
        irq_process.irq_track_list, irq_list_hd) {

        snprintf(work, SIZE_OF_WORK, IDNT_L1"pirq_node[%d]", i++);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK, IDNT_L2"pirq_node->irq_no = %d",
            pirq_node->irq_no);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK, IDNT_L2"pirq_node->int_vec = %d",
            pirq_node->int_vec);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK, IDNT_L2"pirq_node->isr_fired = %d",
            pirq_node->isr_fired);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK, IDNT_L2"pirq_node->isr_count = %d",
            pirq_node->isr_count);
        vfs_write(file, work, strlen(work), &pos);

        /* Loop for each cq node within this irq node */
        list_for_each_entry(pirq_cq_node, &pirq_node->irq_cq_track,
            irq_cq_head) {

            snprintf(work, SIZE_OF_WORK, IDNT_L3"pirq_cq_node->cq_id = %d",
                pirq_cq_node->cq_id);
            vfs_write(file, work, strlen(work), &pos);
        }

    }

    i = 0;
    /* Loop for the work nodes */
    list_for_each_entry(pwk_item_curr, &pmetrics_device_elem->
        irq_process.wrk_item_list, wrk_list_hd) {

        snprintf(work, SIZE_OF_WORK, IDNT_L1"wk_node[%d]", i++);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK, IDNT_L2"wk_node->irq_no = %d",
            pwk_item_curr->irq_no);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK, IDNT_L2"wk_node->int_vec = %d",
            pwk_item_curr->int_vec);
        vfs_write(file, work, strlen(work), &pos);
        snprintf(work, SIZE_OF_WORK, IDNT_L2"wk_node->pirq_process = 0x%llX",
            (u64)pwk_item_curr->pirq_process);
        vfs_write(file, work, strlen(work), &pos);
    }

    /* unlock IRQ MUTEX here */
    mutex_unlock(&pmetrics_device_elem->irq_process.irq_track_mtx);
    return pos;
}
