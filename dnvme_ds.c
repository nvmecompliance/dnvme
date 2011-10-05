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

#define IDNT_L1         "\n\t"
#define IDNT_L2         "\n\t\t"
#define IDNT_L3         "\n\t\t\t"
#define IDNT_L4         "\n\t\t\t\t"
#define IDNT_L5         "\n\t\t\t\t\t"
#define IDNT_L6         "\n\t\t\t\t\t\t"

/*
*   driver_log - Generic Log functionality for logging metrics data
*   into file name specified from user.
*/
int driver_log(struct nvme_file *n_file)
{
    struct file *file;  /* File pointer where the data is written           */
    loff_t pos = 0;     /* offset into the file                             */
    int dev = 0;        /* local var for tracking no. of devices            */
    int i = 0;          /* local var to track no. of SQ's and CQ's          */
    int cmd = 0;        /* Local variable to track no. of cmds              */
    mm_segment_t oldfs; /* Old file segment to map between Kernel and usp   */
    struct  metrics_sq  *pmetrics_sq_list;        /* SQ linked list         */
    u8 data1[100];       /* Tmp buffer to spit out data to file             */
    struct  metrics_cq  *pmetrics_cq_list;        /* CQ linked list         */
    struct  metrics_device_list *pmetrics_device; /* Metrics device list    */
    struct  cmd_track  *pcmd_track_list;          /* cmd track linked list  */
    unsigned char __user *filename =
            (unsigned char __user *)n_file->file_name; /* user file name    */

    /* copy the file name sent from user */
    copy_from_user((u8 *)filename, (u8 *)n_file->file_name,
            n_file->flen * sizeof(u8));
    LOG_DBG("File Name in Driver = %s", filename);
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    file = filp_open(filename, O_WRONLY|O_CREAT, 0644);
    if (file) {
        /* Loop through the devices */
        list_for_each_entry(pmetrics_device, &metrics_dev_ll,
                metrics_device_hd) {
            /* Get the variable from metrics structure and write to file */
            sprintf(data1, "metrics_device_list[%d]\n", dev++);
            vfs_write(file, data1, strlen(data1), &pos);
            sprintf(data1, "Minor Number = %d", pmetrics_device->pnvme_device->
                    minor_no);
            vfs_write(file, data1, strlen(data1), &pos);
            /* Looping through the available CQ list */
            list_for_each_entry(pmetrics_cq_list, &metrics_cq_ll, cq_list_hd) {
                /* Get the variable from CQ strucute and write to file */
                sprintf(data1, IDNT_L1"pmetrics_cq_list->public_cq[%d]", i);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"q_id = %d", pmetrics_cq_list->
                        public_cq.q_id);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"tail_ptr = %d", pmetrics_cq_list->
                        public_cq.tail_ptr);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"head pointer = %d", pmetrics_cq_list->
                        public_cq.head_ptr);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"elements = %d", pmetrics_cq_list->
                        public_cq.elements);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"pbit_new_entry = %d", pmetrics_cq_list->
                        public_cq.pbit_new_entry);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L1"pmetrics_cq_list->private_cq[%d]", i++);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"vir_kern_addr = 0x%llX",
                        (u64)pmetrics_cq_list->private_cq.vir_kern_addr);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"contig (0=Y/(!=0)=N) = %d",
                        pmetrics_cq_list->private_cq.contig);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"size = %d", pmetrics_cq_list->
                        private_cq.size);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"dbs = 0X%llX", (u64)pmetrics_cq_list->
                        private_cq.dbs);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L3"prp.sq_id = %d", pmetrics_cq_list->
                        private_cq.prp.sq_id);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L3"prp.unique_id = %d", pmetrics_cq_list->
                        private_cq.prp.unique_id);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L4"prp.nvme_prps:");
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"npages = %d", pmetrics_cq_list->
                        private_cq.prp.prps.npages);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"type = %d", pmetrics_cq_list->
                        private_cq.prp.prps.type);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"vir_prp_list = 0x%llX",
                        (u64)pmetrics_cq_list->private_cq.prp.prps.
                        vir_prp_list);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"prp1 = 0x%llX", (u64)pmetrics_cq_list->
                        private_cq.prp.prps.prp1);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"prp2 = 0x%llX", (u64)pmetrics_cq_list->
                        private_cq.prp.prps.prp2);
                vfs_write(file, data1, strlen(data1), &pos);
             } /* End of CQ list */
            /* Reset Q cnt */
            i = 0;
            /* looping through available sq list */
            list_for_each_entry(pmetrics_sq_list, &metrics_sq_ll, sq_list_hd) {
                /* Get each member of SQ structure and write to file */
                sprintf(data1, IDNT_L1"pmetrics_sq_list->public_sq[%d]", i);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"sq_id = %d", pmetrics_sq_list->
                        public_sq.sq_id);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"Assoc cq_id = %d", pmetrics_sq_list->
                        public_sq.cq_id);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"elements = %d", pmetrics_sq_list->
                        public_sq.elements);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"head_ptr = %d", pmetrics_sq_list->
                        public_sq.head_ptr);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"tail_ptr_virt = %d", pmetrics_sq_list->
                        public_sq.tail_ptr_virt);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"tail_ptr = %d", pmetrics_sq_list->
                        public_sq.tail_ptr);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L1"pmetrics_sq_list->private_sq[%d]", i++);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"vir_kern_addr = 0X%llX",
                        (u64)pmetrics_sq_list->private_sq.vir_kern_addr);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"contig (0=Y/(!=0)=N) = %d",
                        pmetrics_sq_list->private_sq.contig);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"size = %d", pmetrics_sq_list->
                        private_sq.size);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"unique_cmd_id(Cnt) = %d",
                        pmetrics_sq_list->private_sq.unique_cmd_id);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L2"dbs = 0X%llX",
                        (u64)pmetrics_sq_list->private_sq.dbs);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L3"prp.sq_id = %d", pmetrics_sq_list->
                        private_sq.prp.sq_id);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L3"prp.unique_id = %d", pmetrics_sq_list->
                        private_sq.prp.unique_id);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L4"prp.nvme_prps:");
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"npages = %d", pmetrics_sq_list->
                        private_sq.prp.prps.npages);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"type = %d", pmetrics_sq_list->
                        private_sq.prp.prps.type);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"vir_prp_list = 0x%llX",
                        (u64)pmetrics_sq_list->private_sq.prp.prps.
                        vir_prp_list);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"prp1 = 0x%llX", (u64)pmetrics_sq_list->
                        private_sq.prp.prps.prp1);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L5"prp2 = 0x%llX", (u64)pmetrics_sq_list->
                        private_sq.prp.prps.prp2);
                vfs_write(file, data1, strlen(data1), &pos);
                sprintf(data1, IDNT_L4"cmd track list = ");
                vfs_write(file, data1, strlen(data1), &pos);

                /* Looping through the cmds if any */
                list_for_each_entry(pcmd_track_list, &sq_cmd_ll,
                        cmd_list_hd) {
                    /* write to file if any cmds exist */
                    sprintf(data1, IDNT_L5"cmd track no = %d", cmd++);
                    vfs_write(file, data1, strlen(data1), &pos);
                    sprintf(data1, IDNT_L5"unique_id = %d", pmetrics_sq_list->
                            private_sq.cmd_track_list->unique_id);
                    vfs_write(file, data1, strlen(data1), &pos);
                    sprintf(data1, IDNT_L5"sq_id = %d", pmetrics_sq_list->
                            private_sq.cmd_track_list->sq_id);
                    vfs_write(file, data1, strlen(data1), &pos);
                    sprintf(data1, IDNT_L5"opcode = %d", pmetrics_sq_list->
                            private_sq.cmd_track_list->opcode);
                    vfs_write(file, data1, strlen(data1), &pos);
                    sprintf(data1, IDNT_L5"cmd_set = %d", pmetrics_sq_list->
                            private_sq.cmd_track_list->cmd_set);
                    vfs_write(file, data1, strlen(data1), &pos);
                } /* End of cmd track list */
            } /* End of SQ metrics list */
        } /* End of file writing */
        fput(file);
    }
    set_fs(oldfs);
    filp_close(file, NULL); /* Close the file */
    return SUCCESS;
}
