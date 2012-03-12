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
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include "definitions.h"
#include "sysdnvme.h"
#include "dnvme_reg.h"
#include "dnvme_queue.h"
#include "dnvme_ds.h"
#include "dnvme_cmds.h"
#include "dnvme_irq.h"

/* Static functions used in this file  */
static void reinit_admn_sq(struct  metrics_sq  *pmetrics_sq_list,
    struct  metrics_device_list *pmetrics_device);
static void reinit_admn_cq(struct  metrics_cq  *pmetrics_cq_list);
static void deallocate_metrics_cq(struct device *dev,
    struct  metrics_cq  *pmetrics_cq_list,
    struct  metrics_device_list *pmetrics_device);
static void deallocate_metrics_sq(struct device *dev,
    struct  metrics_sq  *pmetrics_sq_list,
    struct  metrics_device_list *pmetrics_device);
static void pos_cq_head_ptr(struct metrics_cq  *pmetrics_cq_node,
    u16 num_reaped);
static int process_reap_algos(struct cq_completion *cq_entry,
    struct  metrics_device_list *pmetrics_device);
static int process_algo_q(struct metrics_sq *pmetrics_sq_node,
    struct cmd_track *pcmd_node, u8 status,
    struct  metrics_device_list *pmetrics_device,
    enum metrics_type type);
static int process_algo_gen(struct metrics_sq *pmetrics_sq_node,
    u16 cmd_id, struct  metrics_device_list *pmetrics_device);
static int copy_cq_data(struct metrics_cq  *pmetrics_cq_node, u8 *cq_head_ptr,
    u16 comp_entry_size, u16 *num_reaped, u8 *buffer,
    struct  metrics_device_list *pmetrics_device);

/*
 * nvme_ctrlrdy_capto - This function is used for checking if the controller
 * is ready to process commands after CC.EN is set to 1. This will wait a
 * min of CAP.TO seconds before failing.
 */
int nvme_ctrlrdy_capto(struct nvme_device *pnvme_dev)
{
    u64 timer_delay;    /* Timer delay read from CAP.TO register          */
    u64 ini, end;

    /* Read in the value of CAP.TO */

    timer_delay = (u64) (readl(&pnvme_dev->private_dev.nvme_ctrl_space->cap)
        >> NVME_TO_SHIFT_MASK);
    timer_delay = timer_delay * (CAP_TO_UNIT);
    LOG_DBG("Checking NVME Device Status (CSTS.RDY = 1)...");
    LOG_DBG("Timer Expires in %lld ms", timer_delay);
    ini = get_jiffies_64(); /* Read Jiffies for timer */
    /* Check if the device status set to ready */
    while (!(readl(&pnvme_dev->private_dev.nvme_ctrl_space->csts)
            & NVME_CSTS_RDY)) {
        LOG_DBG("Waiting...");
        msleep(250);
        end = get_jiffies_64();
        if (end < ini) {
            /* Roll over */
            ini = ULLONG_MAX - ini;
            end = ini + end;
        }
        /* Check if the time out occured */
        if (jiffies_to_msecs(end - ini) > timer_delay) {
            LOG_ERR("ASQ Setup Failed before Timeout");
            LOG_ERR("Check if Admin Completion Queue is Created First");
            /* set return invalid/failed */
            return -EINVAL;
        }
    }
    LOG_DBG("NVME Controller is Ready to process commands");
    return SUCCESS;
}

/*
 * nvme_ctrl_enable - NVME controller enable function.This will set the CAP.EN
 * flag and this function which call the timer handler and check for the timer
 * expiration. It returns success if the ctrl in rdy before timeout.
 */
int nvme_ctrl_enable(struct  metrics_device_list *pmetrics_device_element)
{
    struct nvme_device *pnvme_dev;
    u32 ctrl_config;

    /* get the device from the list */
    pnvme_dev = pmetrics_device_element->metrics_device;

    /* Read Controller Configuration as we can only write 32 bits */
    ctrl_config = readl(&pnvme_dev->private_dev.nvme_ctrl_space->cc);

    /* BIT 0 is set to 1 i.e., CC.EN = 1 */
    ctrl_config |= 0x1;

    /* Write the enable bit into CC register */
    writel(ctrl_config, &pnvme_dev->private_dev.nvme_ctrl_space->cc);

   /* Check the Timeout flag */
    if (nvme_ctrlrdy_capto(pnvme_dev) != SUCCESS) {
        LOG_ERR("Controller is not ready before time out");
        return -EINVAL;
    }

    return SUCCESS;
}

/*
 * nvme_ctrl_disable - NVME controller disable function.This will reset the
 * CAP.EN flag and this function which call the timer handler and check for
 * the timer expiration. It returns success if the ctrl in rdy before timeout.
 */
int nvme_ctrl_disable(struct  metrics_device_list *pmetrics_device_element)
{
    struct nvme_device *pnvme_dev;
    u32 ctrl_config;
    u8 rdy_sts = 0xFF;
    u8 i;
    /* get the device from the list */
    pnvme_dev = pmetrics_device_element->metrics_device;

    /* Read Controller Configuration as we can only write 32 bits */
    ctrl_config = readl(&pnvme_dev->private_dev.nvme_ctrl_space->cc);
    /* BIT 0 is set to 0 i.e., CC.EN = 0 */
    ctrl_config &= ~0x1;
    /* Write the enable bit into CC register */
    writel(ctrl_config, &pnvme_dev->private_dev.nvme_ctrl_space->cc);
    /* poll until a constant T/O to see when device really becomes disabled */
    for (i = 0; i < 20; i++) {
        msleep(100);
        rdy_sts = readl(&pnvme_dev->private_dev.nvme_ctrl_space->csts);
        if (! (rdy_sts & NVME_CSTS_RDY)) {
            return SUCCESS;
        }
    }
    LOG_ERR("Disabling ctrlr failed. CSTS.RDY=1 after T/O");
    return -EINVAL;
}

/*
 * Called for Controller Disable to clean up the driver data structures
 */
void nvme_disable(struct  metrics_device_list *pmetrics_device,
    enum nvme_state new_state)
{
    /* Clean the IRQ data structures */
    release_irq(pmetrics_device);
    /* Clean Up the data structures */
    deallocate_all_queues(pmetrics_device, new_state);
    /* Clean up meta buffers in all disable cases */
    deallocate_mb(pmetrics_device);
}

/*
 * create_admn_sq - This routine is called when the driver invokes the ioctl for
 * admn sq creation. It returns success if the submission q creation is success
 * after dma_coherent_alloc else returns failure at any step which fails.
 */
int create_admn_sq(struct nvme_device *pnvme_dev, u16 qsize,
        struct  metrics_sq  *pmetrics_sq_list)
{
    u16 asq_id;     /* Admin Submission Q Id                          */
    u32 aqa;        /* Admin Q attributes in 32 bits size             */
    u32 tmp_aqa;    /* Temp var to hold admin q attributes            */
    u32 asq_depth = 0;  /* the size of bytes to allocate              */
    int ret_code = SUCCESS;

    LOG_DBG("Creating Admin Submission Queue...");

    /* As the Admin Q ID is always 0*/
    asq_id = 0;

    /* Checking for overflow or underflow. */
    if (qsize > MAX_AQ_ENTRIES || qsize == 0) {
        LOG_ERR("ASQ entries is more than MAX Q size or specified NULL");
        ret_code = -EINVAL;
        goto asq_out;
    }

    /*
    * As the qsize send is in number of entries this computes the no. of bytes
    * computed.
    */
    asq_depth = qsize * 64;

    LOG_DBG("ASQ Depth: 0x%x", asq_depth);

    /*
     * The function dma_alloc_coherent  maps the dma address for ASQ which gets
     * the DMA mapped address from the kernel virtual address.
     */
    pmetrics_sq_list->private_sq.vir_kern_addr =
            dma_alloc_coherent(&pnvme_dev->private_dev.pdev->dev, asq_depth,
                    &pmetrics_sq_list->private_sq.sq_dma_addr, GFP_KERNEL);
    if (!pmetrics_sq_list->private_sq.vir_kern_addr) {
        LOG_ERR("Unable to allocate DMA Address for ASQ!!");
        ret_code = -ENOMEM;
        goto asq_out;
    }

    /* Zero out all ASQ memory before processing */
    memset(pmetrics_sq_list->private_sq.vir_kern_addr, 0, asq_depth);

    LOG_DBG("Virtual ASQ DMA Address: 0x%llx",
            (u64)pmetrics_sq_list->private_sq.vir_kern_addr);

    /* Read, Modify, Write  the aqa as per the q size requested */
    aqa = (qsize - 1) & ASQS_MASK; /* asqs is zero based value */
    tmp_aqa = readl(&pnvme_dev->private_dev.nvme_ctrl_space->aqa);
    tmp_aqa &= ~ASQS_MASK;
    aqa |= tmp_aqa;

    LOG_DBG("Mod Attributes from AQA Reg = 0x%x", tmp_aqa);
    LOG_DBG("AQA Attributes in ASQ:0x%x", aqa);

    /* Write new ASQ size using AQA */
    writel(aqa, &pnvme_dev->private_dev.nvme_ctrl_space->aqa);

    /* Write the DMA address into ASQ base address */
    WRITEQ(pmetrics_sq_list->private_sq.sq_dma_addr,
            &pnvme_dev->private_dev.nvme_ctrl_space->asq);
#ifdef DEBUG
    /* Debug statements */
    LOG_DBG("Admin CQ Base Address = 0x%x",
        (u32)readl(&pnvme_dev->private_dev.nvme_ctrl_space->acq));
    /* Read the AQA attributes after writing and check */
    tmp_aqa = readl(&pnvme_dev->private_dev.nvme_ctrl_space->aqa);

    LOG_DBG("Reading AQA after writing = 0x%x", tmp_aqa);

    /* Read the status register and printout to log */
    tmp_aqa = readl(&pnvme_dev->private_dev.nvme_ctrl_space->csts);

    LOG_DBG("Reading status reg = 0x%x", tmp_aqa);
#endif

    /* Set the door bell of ASQ to 0x1000 as per spec 1.0b */
    pmetrics_sq_list->private_sq.dbs =
        ((void __iomem *)pnvme_dev->private_dev.nvme_ctrl_space)
            + NVME_SQ0TBDL;
    /* set private members in sq metrics */
    pmetrics_sq_list->private_sq.size = asq_depth;
    pmetrics_sq_list->private_sq.unique_cmd_id = 0;
    pmetrics_sq_list->private_sq.contig = 1;

    /* returns success */
    return ret_code;

asq_out:
    if (pmetrics_sq_list->private_sq.vir_kern_addr != NULL) {
        /* Admin SQ dma mem allocated, so free the DMA memory */
        dma_free_coherent(&pnvme_dev->private_dev.pdev->dev, asq_depth,
            (void *)pmetrics_sq_list->private_sq.vir_kern_addr,
        pmetrics_sq_list->private_sq.sq_dma_addr);
    }
    /* returns failure*/
    return ret_code;
}

/*
 * create_admn_cq - This routine is called when the driver invokes the ioctl for
 * admn cq creation. It returns success if the completion q creation is success
 * after dma_coherent_alloc else returns failure at any step which fails.
 */
int create_admn_cq(struct nvme_device *pnvme_dev, u16 qsize,
        struct  metrics_cq  *pmetrics_cq_list)
{

    int ret_code = SUCCESS; /* Ret code set to SUCCESS check for otherwise */
    u16 acq_id;             /* Admin Submission Q Id                       */
    u32 aqa;                /* Admin Q attributes in 32 bits size          */
    u32 tmp_aqa;            /* local var to hold admin q attributes        */
    u32 acq_depth = 0;      /* local var to cal nbytes based on elements   */
    u8  cap_dstrd;          /* local var to cal the doorbell stride.       */

    LOG_DBG("Creating Admin Completion Queue...");

    /* As the Admin Q ID is always 0*/
    acq_id = 0;

    /* Checking for overflow or underflow. */
    if (qsize > MAX_AQ_ENTRIES || qsize == 0) {
        LOG_ERR("ASQ size is more than MAX Q size or specified NULL");
        ret_code = -EINVAL;
        goto acq_out;
    }
    /*
    * As the qsize send is in number of entries this computes the no. of bytes
    * computed.
    */
    acq_depth = qsize * 16;
    LOG_DBG("ACQ Depth: 0x%x", acq_depth);
    /*
     * The function dma_alloc_coherent  maps the dma address for ACQ which gets
     * the DMA mapped address from the kernel virtual address.
     */
    pmetrics_cq_list->private_cq.vir_kern_addr =
            dma_alloc_coherent(&pnvme_dev->private_dev.pdev->dev, acq_depth,
                    &pmetrics_cq_list->private_cq.cq_dma_addr, GFP_KERNEL);
    if (!pmetrics_cq_list->private_cq.vir_kern_addr) {
        LOG_ERR("Unable to allocate DMA Address for ACQ!!");
        ret_code = -ENOMEM;
        goto acq_out;
    }

    /* Zero out all ACQ memory before processing */
    memset(pmetrics_cq_list->private_cq.vir_kern_addr, 0, acq_depth);

    LOG_DBG("Virtual ACQ DMA Address: 0x%llx",
            (u64)pmetrics_cq_list->private_cq.vir_kern_addr);
    LOG_DBG("ACQ DMA Address: 0x%llx",
            (u64)pmetrics_cq_list->private_cq.cq_dma_addr);

    /* Read, Modify and write the Admin Q attributes */
    aqa = (qsize - 1) << 16; /* acqs is zero based value */
    aqa &= ACQS_MASK;
    tmp_aqa = readl(&pnvme_dev->private_dev.nvme_ctrl_space->aqa);
    tmp_aqa &= ~ACQS_MASK;

    /* Final value to write to AQA Register */
    aqa |= tmp_aqa;

    LOG_DBG("Modified Attributes (AQA) = 0x%x", tmp_aqa);
    LOG_DBG("AQA Attributes in ACQ:0x%x", aqa);

    /* Write new ASQ size using AQA */
    writel(aqa, &pnvme_dev->private_dev.nvme_ctrl_space->aqa);
    /* Write the DMA address into ACQ base address */
    WRITEQ(pmetrics_cq_list->private_cq.cq_dma_addr,
            &pnvme_dev->private_dev.nvme_ctrl_space->acq);
#ifdef DEBUG
    /* Read the AQA attributes after writing and check */
    tmp_aqa = readl(&pnvme_dev->private_dev.nvme_ctrl_space->aqa);

    LOG_DBG("Reading AQA after writing in ACQ = 0x%x\n", tmp_aqa);

#endif

    pmetrics_cq_list->private_cq.size = acq_depth;
    pmetrics_cq_list->private_cq.contig = 1;

    /* Get the door bell stride from CAP register */
    cap_dstrd = (READQ(&pnvme_dev->private_dev.nvme_ctrl_space->cap) >> 32)
        & 0xF;

    /* CQ 0 Head DoorBell admin computed used doorbell stride. */
    pmetrics_cq_list->private_cq.dbs =
        ((void __iomem *)pnvme_dev->private_dev.nvme_ctrl_space)
            + NVME_SQ0TBDL + (4 << cap_dstrd);

    /* returns success */
    return ret_code;

acq_out:
    if (pmetrics_cq_list->private_cq.vir_kern_addr != NULL) {
        /* Admin CQ dma mem allocated, so free the DMA memory */
        dma_free_coherent(&pnvme_dev->private_dev.pdev->dev, acq_depth,
                (void *)pmetrics_cq_list->private_cq.vir_kern_addr,
                pmetrics_cq_list->private_cq.cq_dma_addr);
    }
    /* returns success */
    return ret_code;

}

/*
 * nvme_prepare_sq - This routine is called when the driver invokes the ioctl
 * for IO SQ Creation. It will retrieve the q size from IOSQES from CC.
 */
int nvme_prepare_sq(struct  metrics_sq  *pmetrics_sq_list,
            struct nvme_device *pnvme_dev)
{
    int ret_code = SUCCESS;
    u32 ctrl_config = 0;
    u8  cap_dstrd;
#ifdef DEBUG
    u16 u16cap_mqes = 0;
#endif

    /*Read Controller Configuration CC register at offset 0x14h. */
    ctrl_config = readl(&pnvme_dev->private_dev.nvme_ctrl_space->cc);
    /* Extract the IOSQES from CC */
    ctrl_config = (ctrl_config >> 16) & 0xF;

    LOG_DBG("CC.IOSQES = 0x%x, 2^x = %d", ctrl_config, (1 << ctrl_config));
    pmetrics_sq_list->private_sq.size = pmetrics_sq_list->public_sq.elements *
            (1 << ctrl_config);

#ifdef DEBUG
    /* Check to see if the entries exceed the Max Q entries supported */
    u16cap_mqes = readl(&pnvme_dev->private_dev.nvme_ctrl_space->cap) & 0xFFFF;
    LOG_DBG("Max Q:Actual Q elements = 0x%x:0x%x", u16cap_mqes,
            pmetrics_sq_list->public_sq.elements);
    /* I should not return from here if exceeds */
    if (pmetrics_sq_list->public_sq.elements > u16cap_mqes) {
        LOG_ERR("The IO SQ id = %d exceeds maximum elements allowed!",
                pmetrics_sq_list->public_sq.sq_id);
    }
#endif
    /*
     * call dma_alloc_coherent or SQ which gets DMA mapped address from
     * the kernel virtual address.
     * != 0 is contiguous SQ as per design.
     */
    if (pmetrics_sq_list->private_sq.contig != 0) {
        /* Assume that CMD.DW11.PC bit will be set to one. */
        pmetrics_sq_list->private_sq.vir_kern_addr = dma_alloc_coherent(
            &pnvme_dev->private_dev.pdev->dev, pmetrics_sq_list->private_sq.
                size, &pmetrics_sq_list->private_sq.sq_dma_addr, GFP_KERNEL);
        /* Check if the dma alloc was successful */
        if (!pmetrics_sq_list->private_sq.vir_kern_addr) {
            LOG_ERR("Unable to allocate DMA Address for IO SQ!!");
            ret_code = -ENOMEM;
            goto psq_out;
        }
        /* Zero out all IO SQ memory before processing */
        memset(pmetrics_sq_list->private_sq.vir_kern_addr, 0,
                pmetrics_sq_list->private_sq.size);
    }
    /* Set Unique Command value to zero for starters. */
    pmetrics_sq_list->private_sq.unique_cmd_id = 0;
    cap_dstrd = (READQ(&pnvme_dev->private_dev.nvme_ctrl_space->cap) >> 32)
        & 0xF;
    LOG_DBG("CAP DSTRD Value = 0x%x", cap_dstrd);
    pmetrics_sq_list->private_sq.dbs = ((void __iomem *)pnvme_dev->private_dev.
        nvme_ctrl_space) + NVME_SQ0TBDL +
            ((2 * pmetrics_sq_list->public_sq.sq_id) * (4 << cap_dstrd));

    return ret_code;

psq_out:
    if (pmetrics_sq_list->private_sq.vir_kern_addr != NULL) {
        /* Admin SQ dma mem allocated, so free the DMA memory */
        dma_free_coherent(&pnvme_dev->private_dev.pdev->dev, pmetrics_sq_list->
            private_sq.size, (void *)pmetrics_sq_list->private_sq.
            vir_kern_addr, pmetrics_sq_list->private_sq.sq_dma_addr);
    }
    /* returns failure*/
    return ret_code;
}

/*
 * nvme_prepare_cq - This routine is called when the driver invokes the ioctl
 * for IO CQ Preparation. It will retrieve the q size from IOSQES from CC.
 */
int nvme_prepare_cq(struct  metrics_cq  *pmetrics_cq_list,
            struct nvme_device *pnvme_dev)
{
    int ret_code = SUCCESS;
    u32 ctrl_config = 0;
    u8  cap_dstrd;
#ifdef DEBUG
    u16 u16cap_mqes = 0;
#endif

    /* Read Controller Configuration CC register at offset 0x14h. */
    ctrl_config = readl(&pnvme_dev->private_dev.nvme_ctrl_space->cc);
    /* Extract the IOCQES from CC */
    ctrl_config = (ctrl_config >> 20) & 0xF;

    LOG_DBG("CC.IOCQES = 0x%x, 2^x = %d", ctrl_config, (1 << ctrl_config));
    pmetrics_cq_list->private_cq.size = pmetrics_cq_list->public_cq.elements *
            (1 << ctrl_config);
#ifdef DEBUG
    /* Check to see if the entries exceed the Max Q entries supported */
    u16cap_mqes = readl(&pnvme_dev->private_dev.nvme_ctrl_space->cap) & 0xFFFF;
    LOG_DBG("Max CQ:Actual CQ elements = 0x%x:0x%x", u16cap_mqes,
            pmetrics_cq_list->public_cq.elements);
    /* I should not return from here if exceeds */
    if (pmetrics_cq_list->public_cq.elements > u16cap_mqes) {
        LOG_ERR("The IO CQ id = %d exceeds maximum elements allowed!",
                pmetrics_cq_list->public_cq.q_id);
    }
#endif
    /*
     * call dma_alloc_coherent for CQ which gets DMA mapped address from
     * the kernel virtual address only for contiguous CQ case.
     * != 0 is contiguous CQ as per design.
     */
    if (pmetrics_cq_list->private_cq.contig != 0) {
        /* Assume that CMD.DW11.PC bit will be set to one. */
        pmetrics_cq_list->private_cq.vir_kern_addr = dma_alloc_coherent(
            &pnvme_dev->private_dev.pdev->dev, pmetrics_cq_list->private_cq.
                size, &pmetrics_cq_list->private_cq.cq_dma_addr, GFP_KERNEL);
        /* Check if the dma alloc was successful */
        if (!pmetrics_cq_list->private_cq.vir_kern_addr) {
            LOG_ERR("Unable to allocate DMA Address for IO CQ!!");
            ret_code = -ENOMEM;
            goto pcq_out;
        }
        /* Zero out all IO CQ memory before processing */
        memset(pmetrics_cq_list->private_cq.vir_kern_addr, 0,
                pmetrics_cq_list->private_cq.size);
    }

    cap_dstrd = (READQ(&pnvme_dev->private_dev.nvme_ctrl_space->cap) >> 32)
        & 0xF;
    LOG_DBG("CAP DSTRD Value = 0x%x", cap_dstrd);
    /* Here CQ also used SQ0TDBL offset i.e., 0x1000h. */
    pmetrics_cq_list->private_cq.dbs =
        ((void __iomem *)pnvme_dev->private_dev.nvme_ctrl_space) +
            NVME_SQ0TBDL + ((2 * pmetrics_cq_list->public_cq.q_id + 1)
                * (4 << cap_dstrd));

    /* returns success */
    return ret_code;

 pcq_out:
    if (pmetrics_cq_list->private_cq.vir_kern_addr != NULL) {
        /* Admin CQ dma mem allocated, so free the DMA memory */
        dma_free_coherent(&pnvme_dev->private_dev.pdev->dev, pmetrics_cq_list->
            private_cq.size, (void *)pmetrics_cq_list->private_cq.
            vir_kern_addr, pmetrics_cq_list->private_cq.cq_dma_addr);
    }
    /* returns failure */
    return ret_code;
}

/*
 * nvme_ring_sqx_dbl - This routine is called when the driver invokes the ioctl
 * for Ring SQ doorbell. It will retrieve the q from the linked list, copy the
 * tail_ptr with virtual pointer, and write the tail pointer value to SqxTDBL
 * already in dbs.
 */
int nvme_ring_sqx_dbl(u16 ring_sqx, struct  metrics_device_list
        *pmetrics_device_element)
{
    struct  metrics_sq  *pmetrics_sq_list;  /* SQ linked list */
    struct nvme_device *pnvme_dev;

    /* get the device from the list */
    pnvme_dev = pmetrics_device_element->metrics_device;

    /* Seek the SQ within metrics device SQ list */
    list_for_each_entry(pmetrics_sq_list, &pmetrics_device_element->
            metrics_sq_list, sq_list_hd) {
        /* Check if the Q Id matches */
        if (ring_sqx == pmetrics_sq_list->public_sq.sq_id) {
            LOG_DBG("SQ_ID= %d found in the linked list.",
                    pmetrics_sq_list->public_sq.sq_id);
            LOG_DBG("\tVirt Tail Ptr = 0x%x",
                     pmetrics_sq_list->public_sq.tail_ptr_virt);
            LOG_DBG("\tTail Ptr = 0x%x",
                     pmetrics_sq_list->public_sq.tail_ptr);
            /* Copy tail_prt_virt to tail_prt */
            pmetrics_sq_list->public_sq.tail_ptr = pmetrics_sq_list->
                    public_sq.tail_ptr_virt;
            /* Ring the doorbell with tail_prt */
            writel(pmetrics_sq_list->public_sq.tail_ptr, pmetrics_sq_list->
                     private_sq.dbs);
            LOG_DBG("After Writing Doorbell...");
            LOG_DBG("\tVirt Tail Ptr = 0x%x",
                     pmetrics_sq_list->public_sq.tail_ptr_virt);
            LOG_DBG("\tTail Ptr = 0x%x",
                     pmetrics_sq_list->public_sq.tail_ptr);
            /* Done with this function return success */
            return SUCCESS;
        }
    }
    LOG_ERR("SQ ID = %d not found to ring its doorbell", ring_sqx);
    /* If it falls here no SQ ID is found */
    return -EINVAL;
}

/*
 * Deallocate function is called when we want to free up the kernel or prp
 * memory based on the contig flag. The kernel memory is given back, nodes
 * from the cq list are deleted.
 */
static void deallocate_metrics_cq(struct device *dev,
        struct  metrics_cq  *pmetrics_cq_list,
        struct  metrics_device_list *pmetrics_device)
{
    /* Delete memory for all metrics_cq for current id here */
    if (pmetrics_cq_list->private_cq.contig == 0) {
        /* Deletes the PRP persist entry */
        del_prps(pmetrics_device->metrics_device,
            &pmetrics_cq_list->private_cq.prp_persist);

    } else {
        /* Contiguous CQ, so free the DMA memory */
        dma_free_coherent(dev, pmetrics_cq_list->private_cq.size,
                 (void *)pmetrics_cq_list->private_cq.vir_kern_addr,
                 pmetrics_cq_list->private_cq.cq_dma_addr);

    }
    /* Delete the current cq entry from the list, and free it */
    list_del(&pmetrics_cq_list->cq_list_hd);
    kfree(pmetrics_cq_list);

}

/*
 * Deallocate function is called when we want to free up the kernel or prp
 * memory based on the contig flag. The kernel memory is given back, nodes
 * from the sq list are deleted. The cmds tracked are dropped and nodes in
 * command list are deleted.
 */
static void deallocate_metrics_sq(struct device *dev,
        struct  metrics_sq  *pmetrics_sq_list,
        struct  metrics_device_list *pmetrics_device)
{
    /* Clean the Cmd track list */
    empty_cmd_track_list(pmetrics_device->metrics_device, pmetrics_sq_list);

    if (pmetrics_sq_list->private_sq.contig == 0) {
        /* Deletes the PRP persist entry */
        del_prps(pmetrics_device->metrics_device,
            &pmetrics_sq_list->private_sq.prp_persist);
    } else {
        LOG_DBG("DMA Free for contig sq id = %d", pmetrics_sq_list->
            public_sq.sq_id);
        /* Contiguous SQ, so free the DMA memory */
        dma_free_coherent(dev, pmetrics_sq_list->private_sq.size,
            (void *)pmetrics_sq_list->private_sq.vir_kern_addr,
            pmetrics_sq_list->private_sq.sq_dma_addr);
    }

    /* Delete the current sq entry from the list */
    list_del(&pmetrics_sq_list->sq_list_hd);
    kfree(pmetrics_sq_list);
}

/*
 * Reinitialize the admin completion queue's public parameters, when
 * a controller is not completely disabled
 */
static void reinit_admn_cq(struct  metrics_cq  *pmetrics_cq_list)
{
    /* reinit required params in admin node */
    pmetrics_cq_list->public_cq.head_ptr = 0;
    pmetrics_cq_list->public_cq.tail_ptr = 0;
    pmetrics_cq_list->public_cq.pbit_new_entry = 1;
    memset(pmetrics_cq_list->private_cq.vir_kern_addr, 0,
        pmetrics_cq_list->private_cq.size);
    pmetrics_cq_list->public_cq.irq_enabled = 1;
}

/*
 * Reinitialize the admin Submission queue's public parameters, when
 * a controller is not completely disabled
 */
static void reinit_admn_sq(struct  metrics_sq  *pmetrics_sq_list,
    struct  metrics_device_list *pmetrics_device)
{
    /* Free command track list for admin */
    empty_cmd_track_list(pmetrics_device->metrics_device, pmetrics_sq_list);

    /* reinit required params in admin node */
    pmetrics_sq_list->public_sq.head_ptr = 0;
    pmetrics_sq_list->public_sq.tail_ptr = 0;
    pmetrics_sq_list->public_sq.tail_ptr_virt = 0;
    pmetrics_sq_list->private_sq.unique_cmd_id = 0;
}

/*
 * deallocate_all_queues - This function will start freeing up the memory for
 * the queues (SQ and CQ) allocated during the prepare queues. The parameter
 * 'new_state', ST_DISABLE or ST_DISABLE_COMPLETELY, identifies if you need to
 * clear Admin Q along with other Q's.
 */
void deallocate_all_queues(struct  metrics_device_list *pmetrics_device,
    enum nvme_state new_state)
{
    u8 exclude_admin = (new_state == ST_DISABLE) ? ~0 : 0;
    struct  metrics_sq  *pmetrics_sq_list;
    struct  metrics_sq  *pmetrics_sq_next;
    struct  metrics_cq  *pmetrics_cq_list;
    struct  metrics_cq  *pmetrics_cq_next;
    struct device *dev;


    dev = &pmetrics_device->metrics_device->private_dev.pdev->dev;

    /* Loop for each sq node */
    list_for_each_entry_safe(pmetrics_sq_list, pmetrics_sq_next,
        &pmetrics_device->metrics_sq_list, sq_list_hd) {

        /* Check if Admin Q is excluded or not */
        if (exclude_admin && (pmetrics_sq_list->public_sq.sq_id == 0)) {
            LOG_DBG("Retaining ASQ from deallocation");
            /* drop sq cmds and set to zero the public metrics of asq */
            reinit_admn_sq(pmetrics_sq_list, pmetrics_device);
        } else {
            /* Call the generic deallocate sq function */
            deallocate_metrics_sq(dev, pmetrics_sq_list, pmetrics_device);
        }
    }

    /* Loop for each cq node */
    list_for_each_entry_safe(pmetrics_cq_list, pmetrics_cq_next,
        &pmetrics_device->metrics_cq_list, cq_list_hd) {

        /* Check if Admin Q is excluded or not */
        if (exclude_admin && pmetrics_cq_list->public_cq.q_id == 0) {
            LOG_DBG("Retaining ACQ from deallocation");
            /* set to zero the public metrics of acq */
            reinit_admn_cq(pmetrics_cq_list);
        } else {
            /* Call the generic deallocate cq function */
            deallocate_metrics_cq(dev, pmetrics_cq_list, pmetrics_device);
        }
    }

    /* if complete disable then reset the controller admin registers. */
    if (new_state == ST_DISABLE_COMPLETELY) {
        /* Set the Registers to default values. */
        /* Write 0 to AQA */
        writel(0x0, &pmetrics_device->metrics_device->private_dev.
            nvme_ctrl_space->aqa);
        /* Write 0 to the DMA address into ASQ base address */
        WRITEQ(0x0, &pmetrics_device->metrics_device->private_dev.
            nvme_ctrl_space->asq);
        /* Write 0 to the DMA address into ACQ base address */
        WRITEQ(0x0, &pmetrics_device->metrics_device->private_dev.
            nvme_ctrl_space->acq);
    }
}

/*
 *  reap_inquiry - This generic function will try to inquire the number of
 *  commands in the Completion Queue that are waiting to be reaped for any
 *  given q_id.
 */
u16 reap_inquiry(struct metrics_cq  *pmetrics_cq_node,
        struct device *dev)
{
    u8 tmp_pbit;                    /* Local phase bit      */
    /* mem head ptr in cq, base address for queue */
    u8 *q_head_ptr, *queue_base_addr;
    struct cq_completion *cq_entry; /* cq entry format      */
    u16 comp_entry_size = 16;       /* acq entry size       */
    u16 num_remaining = 0;          /* reap elem remaining  */

    /* If IO CQ set the completion Q entry size */
    if (pmetrics_cq_node->public_cq.q_id != 0) {
        comp_entry_size = (pmetrics_cq_node->private_cq.size) /
                        (pmetrics_cq_node->public_cq.elements);
    }

    /* local tmp phase bit */
    tmp_pbit = pmetrics_cq_node->public_cq.pbit_new_entry;
    if (pmetrics_cq_node->private_cq.contig != 0) {
        /* point the address to corresponding head ptr */
        q_head_ptr = pmetrics_cq_node->private_cq.vir_kern_addr +
              (comp_entry_size * pmetrics_cq_node->public_cq.head_ptr);
        queue_base_addr = pmetrics_cq_node->private_cq.vir_kern_addr;
    } else {
        /* do sync and update when pointer to discontig Q is reaped inq */
        dma_sync_sg_for_cpu(dev, pmetrics_cq_node->private_cq.prp_persist.sg,
                pmetrics_cq_node->private_cq.prp_persist.dma_mapped_pgs,
                pmetrics_cq_node->private_cq.prp_persist.data_dir);
        /* Point to discontig Q memory here */
        queue_base_addr =
            pmetrics_cq_node->private_cq.prp_persist.vir_kern_addr;
        q_head_ptr = queue_base_addr +
            (comp_entry_size * pmetrics_cq_node->public_cq.head_ptr);

    }

    LOG_DBG("Reap Inquiry on CQ_ID:PBit:EntrySize = %d:%d:%d",
            pmetrics_cq_node->public_cq.q_id, tmp_pbit, comp_entry_size);
    LOG_DBG("CQ Hd Ptr = %d", pmetrics_cq_node->public_cq.head_ptr);
    LOG_DBG("Rp Inq. Tail Ptr before = %d", pmetrics_cq_node->public_cq.
            tail_ptr);

    /* Start from head ptr and update till the remaining cnt */
    pmetrics_cq_node->public_cq.tail_ptr = pmetrics_cq_node->public_cq.
            head_ptr;

    /* loop through the entries in the cq */
    while (1) {
        cq_entry = (struct cq_completion *)q_head_ptr;
        if (cq_entry->phase_bit == tmp_pbit) {
            pmetrics_cq_node->public_cq.tail_ptr += 1;
            q_head_ptr += comp_entry_size;
            num_remaining += 1;
            /* Q wrapped around */
            if (q_head_ptr >= (queue_base_addr +
                pmetrics_cq_node->private_cq.size)) {
                tmp_pbit = !tmp_pbit;
                q_head_ptr = queue_base_addr;
                pmetrics_cq_node->public_cq.tail_ptr = 0;
            }
        } else {
            /* we reached stale element */
            break;
        }
    } /* end of while loop */

    LOG_DBG("Rp Inq. Tail Ptr After = %d", pmetrics_cq_node->public_cq.
            tail_ptr);
    LOG_DBG("cq.elements = %d", pmetrics_cq_node->public_cq.elements);
    LOG_DBG("Number of elements remaining = %d", num_remaining);

    return num_remaining;
}
/*
 *  driver_reap_inquiry - This function will try to inquire the number of
 *  commands in the Completion Queue that are waiting to be reaped.
 */
int driver_reap_inquiry(struct  metrics_device_list *pmetrics_device,
    struct nvme_reap_inquiry *usr_reap_inq)
{
    struct metrics_cq  *pmetrics_cq_node;   /* ptr to cq node       */
    int ret_val = SUCCESS;
    struct nvme_reap_inquiry *kern_reap_inq;

    /* Allocating memory for the command in kernel space */
    kern_reap_inq = kmalloc(sizeof(struct nvme_reap_inquiry), GFP_KERNEL);
    if (!kern_reap_inq) {
        LOG_ERR("Unable to allocate kernel memory");
        return -ENOMEM;
    }

    /* Copying userspace nvme_reap to kernel memory */
    if (copy_from_user(kern_reap_inq,
        (void __user *) usr_reap_inq, sizeof(struct nvme_reap_inquiry))) {
        LOG_ERR("Invalid copy from user space");
        ret_val = -EFAULT;
        goto exit;
    }

    /* Find given CQ in list */
    pmetrics_cq_node = find_cq(pmetrics_device, kern_reap_inq->q_id);
    if (pmetrics_cq_node == NULL) {
        /* if the control comes here it implies q id not in list */
        LOG_ERR("CQ ID = %d is not in list", kern_reap_inq->q_id);
        ret_val = -ENODEV;
        goto exit;
    }
    /* Initializing ISR count for all the possible cases */
    kern_reap_inq->isr_count = 0;
    /* Note: If ISR's are enabled then ACQ will always be attached to INT 0 */
    if (pmetrics_device->metrics_device->public_dev.irq_active.irq_type
        == INT_NONE) {
        /* Process reap inquiry for non-isr case */
        LOG_DBG("Non-ISR Reap Inq on CQ = %d",
            pmetrics_cq_node->public_cq.q_id);
        kern_reap_inq->num_remaining = reap_inquiry(pmetrics_cq_node,
            &pmetrics_device->metrics_device->private_dev.pdev->dev);
    } else {
        /* When INT scheme is other than INT_NONE */
        /* If the irq is enabled, process reap_inq isr else
         * do polling based inq
         */
        if (pmetrics_cq_node->public_cq.irq_enabled == 0) {
            /* Process reap inquiry for non-isr case */
            LOG_DBG("Non-ISR Reap Inq on CQ = %d",
                pmetrics_cq_node->public_cq.q_id);
            kern_reap_inq->num_remaining = reap_inquiry(pmetrics_cq_node,
                &pmetrics_device->metrics_device->private_dev.pdev->dev);
        } else {
            LOG_DBG("ISR Reap Inq on CQ = %d",
                pmetrics_cq_node->public_cq.q_id);
            /* Lock onto irq mutex for reap inquiry. */
            mutex_lock(&pmetrics_device->irq_process.irq_track_mtx);
            /* Process ISR based reap inquiry as isr is enabled */
            ret_val = reap_inquiry_isr(pmetrics_cq_node, pmetrics_device,
                &kern_reap_inq->num_remaining, &kern_reap_inq->isr_count);
            /* unlock irq track mutex here */
            mutex_unlock(&pmetrics_device->irq_process.irq_track_mtx);
            /* delay ret_val checking to return after mutex unlock */
            if (ret_val < 0) {
                LOG_ERR("ISR Reap Inquiry failed...");
                ret_val = -EINVAL;
                goto exit;
             }
         }
    }

    /* Copy to user the remaining elements in this q */
    if (copy_to_user(usr_reap_inq, kern_reap_inq,
        sizeof(struct nvme_reap_inquiry))) {
        LOG_ERR("Error copying to user buffer returning");
        ret_val = -EFAULT;
        goto exit;
    }

    /* Check for hw violation of full Q definition */
    if (kern_reap_inq->num_remaining >= pmetrics_cq_node->public_cq.elements) {
        LOG_ERR("HW violating full Q definition");
        ret_val = -EINVAL;
    }

exit:
    kfree(kern_reap_inq);
    return ret_val;
}

/*
 * find sq node in the given device element node and given sq id. If found
 * returns the pointer to the sq node in the sq linked list. Otherwise returns
 * NULL.
 */
struct metrics_sq *find_sq(struct  metrics_device_list
        *pmetrics_device_element, u16 sq_id)
{
    struct  metrics_sq  *pmetrics_sq_list;

    list_for_each_entry(pmetrics_sq_list, &pmetrics_device_element->
            metrics_sq_list, sq_list_hd) {
        if (sq_id == pmetrics_sq_list->public_sq.sq_id) {
            return pmetrics_sq_list;
        }
    }
    return NULL;
}

/*
 * find cq node in the given device element node and given cq id. If found
 * returns the pointer to the cq node in the cq linked list otherwise returns
 * NULL.
 */
struct metrics_cq *find_cq(struct  metrics_device_list
        *pmetrics_device_element, u16 cq_id)
{
    struct  metrics_cq  *pmetrics_cq_list;

    list_for_each_entry(pmetrics_cq_list, &pmetrics_device_element->
            metrics_cq_list, cq_list_hd) {
        if (cq_id == pmetrics_cq_list->public_cq.q_id) {
            return pmetrics_cq_list;
        }
    }
    return NULL;
}

/*
 * Find the command node for the given sq node and cmd id. If the cmd node
 * is found in the list, returns pointer to the cmd node otherwise returns
 * NULL.
 */

struct cmd_track *find_cmd(struct metrics_sq *pmetrics_sq_node, u16 cmd_id)
{
    struct cmd_track *pcmd_track_list;
    list_for_each_entry(pcmd_track_list, &pmetrics_sq_node->private_sq.
            cmd_track_list, cmd_list_hd) {
        if (cmd_id == pcmd_track_list->unique_id) {
            return pcmd_track_list;
        }
    }
    return NULL;
}

/*
 * Finds the meta data node in the linked list and if found returns
 * the pointer to the node otherwise returns NULL.
 */
struct metrics_meta *find_meta_node(struct metrics_device_list
        *pmetrics_device_elem, u32 meta_id)
{
    struct  metrics_meta  *pmetrics_meta = NULL;

    list_for_each_entry(pmetrics_meta, &pmetrics_device_elem->metrics_meta.
            meta_trk_list, meta_list_hd) {
        if (meta_id == pmetrics_meta->meta_id) {
            return pmetrics_meta;
        }
    }
    /* couldn't find the node returning empty */
    return NULL;
}
/*
 * Free the given cmd id node from the command track list.
 */
static int remove_cmd_node(struct metrics_sq *pmetrics_sq_node, u16 cmd_id)
{
    struct cmd_track *pcmd_node;

    pcmd_node = find_cmd(pmetrics_sq_node, cmd_id);
    if (pcmd_node == NULL) {
        LOG_ERR("Cmd id = %d does not exist", cmd_id);
        return -EBADSLT; /* Invalid slot */
    }
    list_del(&pcmd_node->cmd_list_hd);
    kfree(pcmd_node);

    return SUCCESS;
}

/*
 * Remove the given sq node from the linked list.
 */
static int remove_sq_node(struct  metrics_device_list
        *pmetrics_device, u16 sq_id)
{
    struct  metrics_sq  *pmetrics_sq_node;

    pmetrics_sq_node = find_sq(pmetrics_device, sq_id);
    if (pmetrics_sq_node == NULL) {
        LOG_ERR("SQ ID = %d does not exist", sq_id);
        return -EBADSLT; /* Invalid slot */
    }

    deallocate_metrics_sq(&pmetrics_device->metrics_device->private_dev.
        pdev->dev, pmetrics_sq_node, pmetrics_device);

    return SUCCESS;
}

/*
 * remove the given cq node from the linked list.Also if
 * the CQ node is present inside the IRQ track list it deletes it
 */
static int remove_cq_node(struct  metrics_device_list
        *pmetrics_device, u16 cq_id)
{
    struct  metrics_cq  *pmetrics_cq_node;
    int ret_val = SUCCESS;

    pmetrics_cq_node = find_cq(pmetrics_device, cq_id);
    if (pmetrics_cq_node == NULL) {
        LOG_ERR("CQ ID = %d does not exist", cq_id);
        return -EBADSLT;
    }

    /* If irq is enabled then clean up the irq track list
     * NOTE:- only for IO queues
     */
    if (pmetrics_cq_node->public_cq.irq_enabled != 0) {
        if (remove_icq_node(pmetrics_device, cq_id, pmetrics_cq_node->
            public_cq.irq_no) < 0) {
            LOG_ERR("Removal of IRQ CQ node failed. ");
            ret_val = -EINVAL;
        }
    }

    deallocate_metrics_cq(&pmetrics_device->metrics_device->private_dev.
        pdev->dev, pmetrics_cq_node, pmetrics_device);

    return ret_val;
}

/*
 * Process Algorithm for IO Qs.
 */
static int process_algo_q(struct metrics_sq *pmetrics_sq_node,
        struct cmd_track *pcmd_node, u8 status,
        struct  metrics_device_list *pmetrics_device,
        enum metrics_type type)
{
    int ret_val = SUCCESS;

    LOG_DBG("Persist Q Id = %d", pcmd_node->persist_q_id);
    LOG_DBG("Unique Cmd Id = %d", pcmd_node->unique_id);
    LOG_DBG("Status = %d", status);

    if (status != SUCCESS) {
        if (pcmd_node->persist_q_id == 0) {
            LOG_ERR("Trying to delete ACQ is blunder");
            ret_val = -EINVAL;
            return ret_val;
        }
        if (type == METRICS_CQ) {
            ret_val = remove_cq_node(pmetrics_device, pcmd_node->persist_q_id);
            if (ret_val != SUCCESS) {
                LOG_ERR("CQ Removal failed...");
                return ret_val;
            }

        } else if (type == METRICS_SQ) {
            ret_val = remove_sq_node(pmetrics_device, pcmd_node->persist_q_id);
            if (ret_val != SUCCESS) {
                LOG_ERR("SQ Removal failed...");
                return ret_val;
            }
        }
    }
    ret_val = remove_cmd_node(pmetrics_sq_node, pcmd_node->unique_id);
    if (ret_val != SUCCESS) {
        LOG_ERR("Cmd Removal failed...");
        return ret_val;
    }

    return ret_val;
}
/*
 * Process General Algorithm.
 */
static int process_algo_gen(struct metrics_sq *pmetrics_sq_node,
        u16 cmd_id, struct  metrics_device_list *pmetrics_device)
{
    int ret_val = SUCCESS;
    struct cmd_track *pcmd_node;

    /* Find the commnand ndoe */
    pcmd_node = find_cmd(pmetrics_sq_node, cmd_id);
    if (pcmd_node == NULL) {
        LOG_ERR("Command id = %d does not exist", cmd_id);
        return -EBADSLT; /* Invalid slot */
    }

    del_prps(pmetrics_device->metrics_device, &pcmd_node->prp_nonpersist);

    ret_val = remove_cmd_node(pmetrics_sq_node, cmd_id);

    return ret_val;
}

/*
 * Process Admin Commands.
 */
static int process_admin_cmd(struct metrics_sq *pmetrics_sq_node,
        struct cmd_track *pcmd_node, u8 status,
        struct  metrics_device_list *pmetrics_device)
{
    int ret_val = SUCCESS;

    switch (pcmd_node->opcode) {
    case 0x00:
        /* Delete IO SQ */
        ret_val = process_algo_q(pmetrics_sq_node, pcmd_node, !status,
                pmetrics_device, METRICS_SQ);
        break;
    case 0x01:
        /* Create IO SQ */
        ret_val = process_algo_q(pmetrics_sq_node, pcmd_node, status,
                pmetrics_device, METRICS_SQ);
        break;
    case 0x04:
        /* Delete IO CQ */
        ret_val = process_algo_q(pmetrics_sq_node, pcmd_node, !status,
                pmetrics_device, METRICS_CQ);
        break;
    case 0x05:
        /* Create IO CQ */
        ret_val = process_algo_q(pmetrics_sq_node, pcmd_node, status,
                pmetrics_device, METRICS_CQ);
        break;
    default:
        /* General algo */
        ret_val = process_algo_gen(pmetrics_sq_node, pcmd_node->unique_id,
                pmetrics_device);
        break;
    }
    return ret_val;
}

/*
 * Process various algorithms depending on the Completion entry in a CQ
 * This works for both Admin and IO CQ entries.
 */
static int process_reap_algos(struct cq_completion *cq_entry,
        struct  metrics_device_list *pmetrics_device)
{
    int ret_val = SUCCESS;
    struct metrics_sq *pmetrics_sq_node = NULL;
    struct cmd_track *pcmd_node = NULL;

    /* find sq node for given sq id in CE */
    pmetrics_sq_node = find_sq(pmetrics_device, cq_entry->sq_identifier);
    if (pmetrics_sq_node == NULL) {
        LOG_ERR("SQ ID = %d does not exist", cq_entry->sq_identifier);
        /* do not return invalid here as user want to reap all ce entries. */
        return -EBADSLT; /* Invalid slot */
    }

    /* Update our understanding of the corresponding SQ hdw head ptr */
    pmetrics_sq_node->public_sq.head_ptr = cq_entry->sq_head_ptr;

    /* Find command in sq node */
    pcmd_node = find_cmd(pmetrics_sq_node, cq_entry->cmd_identifier);
    if (pcmd_node != NULL) {
        /* Command Node exists */
        if (pcmd_node->cmd_set == CMD_ADMIN) {
            LOG_DBG("Admin Command Set...");
            ret_val = process_admin_cmd(pmetrics_sq_node, pcmd_node, cq_entry->
                    status_field, pmetrics_device);
        } else {
            LOG_DBG("NVM or AON Cmd processing...");
            ret_val = process_algo_gen(pmetrics_sq_node, pcmd_node->unique_id,
                    pmetrics_device);
        }
    }
    return ret_val;
}

/*
 * Copy the cq data to user buffer for the elements reaped.
 */
static int copy_cq_data(struct metrics_cq  *pmetrics_cq_node, u8 *cq_head_ptr,
        u16 comp_entry_size, u16 *num_should_reap, u8 *buffer,
        struct  metrics_device_list *pmetrics_device)
{
    int latentErr = 0;
    u8 *queue_base_addr; /* Base address for Queue */

    if (pmetrics_cq_node->private_cq.contig != 0) {
        queue_base_addr = pmetrics_cq_node->private_cq.vir_kern_addr;
    } else {
        /* Point to discontig Q memory here */
        queue_base_addr =
            pmetrics_cq_node->private_cq.prp_persist.vir_kern_addr;
    }

    while (*num_should_reap) {
        LOG_DBG("Reaping CE's, %d left to reap", *num_should_reap);

        /* Call the process reap algos based on CE entry */
        latentErr = process_reap_algos((struct cq_completion *)cq_head_ptr,
            pmetrics_device);
        if (latentErr) {
            LOG_ERR("Unable to find CE.SQ_id in dnvme metrics");
        }

        /* Copy to user even on err; allows seeing latent err */
        if (copy_to_user(buffer, cq_head_ptr, comp_entry_size)) {
            LOG_ERR("Unable to copy request data to user space");
            return -EFAULT;
        }

        cq_head_ptr += comp_entry_size;     /* Point to next CE entry */
        buffer += comp_entry_size;          /* Prepare for next element */
        *num_should_reap -= 1;              /* decrease for the one reaped. */

        /* Q wrapped around */
        if (cq_head_ptr >= (queue_base_addr +
            pmetrics_cq_node->private_cq.size)) {
            /* Q wrapped so point to base again */
            cq_head_ptr = queue_base_addr;
        }

        if (latentErr) {
            /* Latent errors were introduced to allow reaping CE's to user
             * space and also counting them as reaped, because they were
             * successfully copied. However, there was something about the CE
             * that indicated an error, possibly malformed CE by hdw, thus the
             * entire IOCTL should error, but we successfully reaped some CE's
             * which allows tnvme to inspect and trust the copied CE's for debug
             */
            LOG_ERR("Detected a partial reap situation; some, not all reaped");
            return latentErr;
        }
    }

    return SUCCESS;
}

/*
 * move the cq head pointer to point to location of the elements that is
 * to be reaped.
 */
static void pos_cq_head_ptr(struct metrics_cq  *pmetrics_cq_node,
        u16 num_reaped)
{
    pmetrics_cq_node->public_cq.head_ptr += num_reaped;
    if (pmetrics_cq_node->public_cq.head_ptr >= (pmetrics_cq_node->
            public_cq.elements)) {
        pmetrics_cq_node->public_cq.pbit_new_entry =
                !pmetrics_cq_node->public_cq.pbit_new_entry;
        pmetrics_cq_node->public_cq.head_ptr =
                pmetrics_cq_node->public_cq.head_ptr %
                pmetrics_cq_node->public_cq.elements;
    }
    LOG_DBG("Head Ptr After = %d", pmetrics_cq_node->public_cq.head_ptr);
    LOG_DBG("Tail Ptr After = %d", pmetrics_cq_node->public_cq.tail_ptr);
}

/*
 * Reap the number of elements specified for the given CQ id and send
 * the reaped elements back. This is the main place and only place where
 * head_ptr is updated. The pbit_new_entry is inverted when Q wraps.
 */
int driver_reap_cq(struct  metrics_device_list *pmetrics_device,
    struct nvme_reap *usr_reap_data)
{
    int ret_val;
    u16 num_will_fit;
    u16 num_could_reap;
    u16 num_should_reap;
    struct metrics_cq  *pmetrics_cq_node;   /* ptr to CQ node in ll */
    u16 comp_entry_size = 16;               /* Assumption is for ACQ */
    /* base address for both contig and discontig queues */
    u8 *queue_base_addr;
    struct nvme_reap *kern_reap_data;


    /* Allocating memory for the command in kernel space */
    kern_reap_data = kmalloc(sizeof(struct nvme_reap), GFP_KERNEL);
    if (!kern_reap_data) {
        LOG_ERR("Unable to allocate kernel memory");
        return -ENOMEM;
    }

    /* Copying userspace nvme_reap to kernel memory */
    if (copy_from_user(kern_reap_data,
        (void __user *) usr_reap_data, sizeof(struct nvme_reap))) {
        LOG_ERR("Invalid copy from user space");
        ret_val = -EFAULT;
        goto exit;
    }

    /* Find CQ with given id from user */
    pmetrics_cq_node = find_cq(pmetrics_device, kern_reap_data->q_id);
    if (pmetrics_cq_node == NULL) {
        LOG_ERR("CQ ID = %d not found", kern_reap_data->q_id);
        ret_val = -EBADSLT;
        goto exit;
    }

    /* Initializing ISR count for all the possible cases */
    kern_reap_data->isr_count = 0;
    /* Call the reap inquiry on this CQ, see how many unreaped elements exist */
    /* Check if the IRQ is enabled and process accordingly */
    if (pmetrics_device->metrics_device->public_dev.irq_active.irq_type
        == INT_NONE) {
        /* Process reap inquiry for non-isr case */
        num_could_reap = reap_inquiry(pmetrics_cq_node, &pmetrics_device->
            metrics_device->private_dev.pdev->dev);
    } else {
        if (pmetrics_cq_node->public_cq.irq_enabled == 0) {
            /* Process reap inquiry for non-isr case */
            num_could_reap = reap_inquiry(pmetrics_cq_node, &pmetrics_device->
                metrics_device->private_dev.pdev->dev);
        } else { /* ISR Reap additions for IRQ support as irq_enabled is set */
            /* Lock the IRQ mutex to guarantee coherence with bottom half. */
            mutex_lock(&pmetrics_device->irq_process.irq_track_mtx);
            /* Process ISR based reap inquiry as isr is enabled */
            ret_val = reap_inquiry_isr(pmetrics_cq_node, pmetrics_device,
                &num_could_reap, &kern_reap_data->isr_count);
            if (ret_val < 0) {
                LOG_ERR("ISR Reap Inquiry failed...");
                goto mtx_unlk;
            }
        }
    }

    /* Check for hw violation of full Q definition */
    if (num_could_reap >= pmetrics_cq_node->public_cq.elements) {
        LOG_ERR("HW violating full Q definition");
        ret_val = -EINVAL;
        goto mtx_unlk;
    }

    /* If this CQ is an IOCQ, not ACQ, then lookup the CE size */
    if (pmetrics_cq_node->public_cq.q_id != 0) {
        comp_entry_size = (pmetrics_cq_node->private_cq.size) /
            (pmetrics_cq_node->public_cq.elements);
    }
    LOG_DBG("Tail ptr position before reaping = %d",
        pmetrics_cq_node->public_cq.tail_ptr);
    LOG_DBG("Detected CE size = 0x%04X", comp_entry_size);
    LOG_DBG("%d elements could be reaped", num_could_reap);
    if (num_could_reap == 0) {
        LOG_DBG("All elements reaped, CQ is empty");
        kern_reap_data->num_remaining = 0;
        kern_reap_data->num_reaped = 0;
    }

    /* Is this request asking for every CE element? */
    if (kern_reap_data->elements == 0) {
        kern_reap_data->elements = num_could_reap;
    }
    num_will_fit = (kern_reap_data->size / comp_entry_size);

    LOG_DBG("Requesting to reap %d elements", kern_reap_data->elements);
    LOG_DBG("User space reap buffer size = %d", kern_reap_data->size);
    LOG_DBG("Total buffer bytes needed to satisfy request = %d",
            num_could_reap * comp_entry_size);
    LOG_DBG("num elements which fit in buffer = %d", num_will_fit);

    /* Assume we can fit all which are requested, then adjust if necessary */
    num_should_reap = num_could_reap;
    kern_reap_data->num_remaining = 0;

    /* Adjust our assumption based on size and elements */
    if (kern_reap_data->elements <= num_could_reap) {
        if (kern_reap_data->size < (num_could_reap * comp_entry_size)) {
            /* Buffer not large enough to hold all requested */
            num_should_reap = num_will_fit;
            kern_reap_data->num_remaining = (num_could_reap - num_should_reap);
        }
    } else {    /* Asking for more elements than presently exist in CQ */
        if (kern_reap_data->size < (num_could_reap * comp_entry_size)) {
            if (num_could_reap > num_will_fit) {
                /* Buffer not large enough to hold all requested */
                num_should_reap = num_will_fit;
                kern_reap_data->num_remaining =
                    (num_could_reap - num_should_reap);
            }
        }
    }
    kern_reap_data->num_reaped = num_should_reap;    /* Expect success */

    LOG_DBG("num elements will attempt to reap = %d", num_should_reap);
    LOG_DBG("num elements expected to remain after reap = %d",
        kern_reap_data->num_remaining);
    LOG_DBG("Head ptr before reaping = %d",
        pmetrics_cq_node->public_cq.head_ptr);

    /* Get the required base address */
    if (pmetrics_cq_node->private_cq.contig != 0) {
        queue_base_addr = pmetrics_cq_node->private_cq.vir_kern_addr;
    } else {
        /* Point to discontig Q memory here */
        queue_base_addr =
            pmetrics_cq_node->private_cq.prp_persist.vir_kern_addr;
    }

    /* Copy the number of CE's we should be able to reap */
    ret_val = copy_cq_data(pmetrics_cq_node,
            (queue_base_addr +
            (comp_entry_size * pmetrics_cq_node->public_cq.head_ptr)),
            comp_entry_size, &num_should_reap, kern_reap_data->buffer,
            pmetrics_device);

    /* Reevaluate our success during reaping */
    kern_reap_data->num_reaped -= num_should_reap;
    kern_reap_data->num_remaining += num_should_reap;
    LOG_DBG("num CE's reaped = %d, num CE's remaining = %d",
        kern_reap_data->num_reaped, kern_reap_data->num_remaining);

    /* Updating the user strucutre */
    if (copy_to_user(usr_reap_data, kern_reap_data,
        sizeof(struct nvme_reap))) {
        LOG_ERR("Unable to copy request data to user space");
        ret_val = (ret_val == SUCCESS) ? -EFAULT : ret_val;
        goto mtx_unlk;
    }

    /* Update system with number actually reaped */
    pos_cq_head_ptr(pmetrics_cq_node, kern_reap_data->num_reaped);
    writel(pmetrics_cq_node->public_cq.head_ptr, pmetrics_cq_node->
        private_cq.dbs);

    /* if 0 CE in a given cq, then reset the isr flag. */
    if ((pmetrics_cq_node->public_cq.irq_enabled == 1) &&
        (kern_reap_data->num_remaining == 0) &&
            (pmetrics_device->metrics_device->public_dev.irq_active.irq_type
                != INT_NONE)) {
        /* reset isr fired flag for the particular irq_no */
        if (reset_isr_flag(pmetrics_device,
            pmetrics_cq_node->public_cq.irq_no) < 0) {
            ret_val = (ret_val == SUCCESS) ? -EINVAL : ret_val;
        }
    }

    /* unmask the irq for which it was masked in Top Half */
    unmask_interrupts(pmetrics_cq_node->public_cq.irq_no,
        &pmetrics_device->irq_process);

mtx_unlk:
    if ((pmetrics_cq_node->public_cq.irq_enabled == 1) &&
        (pmetrics_device->metrics_device->public_dev.irq_active.irq_type
            != INT_NONE)) {
        mutex_unlock(&pmetrics_device->irq_process.irq_track_mtx);
    }
exit:
    kfree(kern_reap_data);
    return ret_val;
}
