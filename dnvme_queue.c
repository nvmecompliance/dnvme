#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include "definitions.h"
#include "sysdnvme.h"
#include "dnvme_reg.h"
#include "dnvme_queue.h"
#include "dnvme_ds.h"

/* device metrics linked list */
struct metrics_device_list *pmetrics_device_list;
struct metrics_driver g_metrics_drv;

/* Conditional compilation for QEMU related modifications. */
#ifdef QEMU
/*
* if QEMU is defined then we do 64 bit write in two 32 bit writes using
* writel's otherwise directly call writeq.
*/
static inline void WRITEQ(__u64 val, volatile void __iomem *addr)
{
    writel(val, addr);
    writel(val >> 32, addr + 4);
}
static inline __u64 READQ(const volatile void __iomem *addr)
{
    const volatile u32 __iomem *p = addr;
    u32 low, high;
    u64 u64data;

    LOG_DBG("ADDR=0x%llx:p=0x%llx", (u64)addr, (u64)p);
    low = readl(p);
    high = readl(p + 1);

    u64data = high;
    u64data = (u64data << 32) + low;
    LOG_DBG("High:low:u64data = 0x%x:0x%x:0x%llx", high, low, u64data);
    return u64data;
}
#else
static inline void WRITEQ(__u64 val, volatile void __iomem *addr)
{
    writeq(val, addr);
}
static inline __u64 READQ(const volatile void __iomem *addr)
{
    return readq(addr);
}
#endif

/*
*  jit_timer_fn - Timer handler routine which gets invoked when the
*  timer expires for the set Time out value.
*/
void jit_timer_fn(unsigned long arg)
{
    unsigned long *data = (unsigned long *)arg;
    *data = 0;
    LOG_NRM("Inside Timer Handler...");
}

/*
* nvme_ctrlrdy_capto - This function is used for checking if the controller
* is ready to process commands after CC.EN is set to 1. This will wait a
* min of CAP.TO seconds before failing.
*/
int nvme_ctrlrdy_capto(struct nvme_device *pnvme_dev)
{
    u32 timer_delay;    /* Timer delay read from CAP.TO register          */
    unsigned long time_out_flag = 1;
    /* Time Out flag for timer handler                */
    struct timer_list asq_timer; /* asq timer declaration                  */

    /* As the TO is in lower 32 of 64 bit cap readl is good enough */
    timer_delay = readl(&pnvme_dev->nvme_ctrl_space->cap) & NVME_TO_MASK;

    /* Modify TO as it is specified in 500ms units, timer needs in jiffies */
    timer_delay >>= 24;
    timer_delay *= NVME_MSEC_2_JIFFIES;

    init_timer(&asq_timer);

    /* register the Timer function */
    asq_timer.data     = (unsigned long)&time_out_flag;
    asq_timer.function = jit_timer_fn;
    asq_timer.expires  = timer_delay;

    LOG_NRM("Checking NVME Device Status (CSTS.RDY = 1)...");
    LOG_NRM("Timer Expires in %ld ms", asq_timer.expires);

    /* Add timer just before the status check */
    add_timer(&asq_timer);

    /* Check if the device status set to ready */
    while (!(readl(&pnvme_dev->nvme_ctrl_space->csts) & NVME_CSTS_RDY)) {
        LOG_DBG("Waiting...");
        msleep(100);

        /* Check if the time out occured */
        if (time_out_flag == 0) {
            LOG_ERR("ASQ Setup Failed before Timeout");
            LOG_NRM("Check if Admin Completion Queue is Created First");

            /* Delete the timer once failed */
            del_timer(&asq_timer);

            /* set return invalid/failed */
            return -EINVAL;
        }
    }
    /* Timer function is done so delete before leaving*/
    del_timer(&asq_timer);

    LOG_NRM("NVME Controller is Ready to process commands");

    return SUCCESS;
}

/*
* nvme_ctrl_enable - NVME controller enable function.This will set the CAP.EN
* flag and this function which call the timer handler and check for the timer
* expiration. It returns success if the ctrl in rdy before timeout.
*/
int nvme_ctrl_enable(struct nvme_device *pnvme_dev)
{
    u32 ctrl_config;

    /* Read Controller Configuration as we can only write 32 bits */
    ctrl_config = readl(&pnvme_dev->nvme_ctrl_space->cc);

    /* BIT 0 is set to 1 i.e., CC.EN = 1 */
    ctrl_config |= 0x1;

    /* Write the enable bit into CC register */
    writel(ctrl_config, &pnvme_dev->nvme_ctrl_space->cc);

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
int nvme_ctrl_disable(struct nvme_device *pnvme_dev)
{
    u32 ctrl_config;
    u8 rdy_sts = 0xFF;

    /* Read Controller Configuration as we can only write 32 bits */
    ctrl_config = readl(&pnvme_dev->nvme_ctrl_space->cc);
    /* BIT 0 is set to 0 i.e., CC.EN = 0 */
    ctrl_config &= ~0x1;
    /* Write the enable bit into CC register */
    writel(ctrl_config, &pnvme_dev->nvme_ctrl_space->cc);
    /* Do clean up */
    /* Write the enable bit into CC register */
    writel(0, &pnvme_dev->nvme_ctrl_space->cc);
    /* introduces at-least one second of delay */
    mdelay(1000);
    rdy_sts = readl(&pnvme_dev->nvme_ctrl_space->csts) & NVME_CSTS_RDY;
    if (rdy_sts == NVME_CSTS_RDY) {
        LOG_ERR("Disable Controller failed. CSTS.RDY is still set");
        return -EINVAL;
    }
    return SUCCESS;
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
    u32 asq_depth;  /* Variable to hold the size of bytes allocated   */

    LOG_NRM("Creating Admin Submission Queue...");

    /* As the Admin Q ID is always 0*/
    asq_id = 0;

    /* Checking for overflow or underflow. */
    if (qsize > MAX_AQ_ENTRIES || qsize == 0) {
        LOG_ERR("ASQ entries is more than MAX Q size or specified NULL");
        return -EINVAL;
    }

    /*
    * As the qsize send is in number of entries this computes the no. of bytes
    * computed.
    */
    asq_depth = qsize*sizeof(u8)*64;

    LOG_DBG("ASQ Depth: 0x%x", asq_depth);

    /*
     * The function dma_alloc_coherent  maps the dma address for ASQ which gets
     * the DMA mapped address from the kernel virtual address.
     */
    pmetrics_sq_list->private_sq.vir_kern_addr =
            dma_alloc_coherent(&pnvme_dev->pdev->dev, asq_depth,
                    &pmetrics_sq_list->private_sq.asq_dma_addr, GFP_KERNEL);
    if (!pmetrics_sq_list->private_sq.vir_kern_addr) {
        LOG_ERR("Unable to allocate DMA Address for ASQ!!");
        return -ENOMEM;
    }
    LOG_NRM("Virtual ASQ DMA Address: 0x%llx",
            (u64)pmetrics_sq_list->private_sq.vir_kern_addr);

    /* Read, Modify, Write  the aqa as per the q size requested */
    aqa = (qsize - 1) & ASQS_MASK; /* asqs is zero based value */
    tmp_aqa = readl(&pnvme_dev->nvme_ctrl_space->aqa);
    tmp_aqa &= ~ASQS_MASK;
    aqa |= tmp_aqa;

    LOG_DBG("Mod Attributes from AQA Reg = 0x%x", tmp_aqa);
    LOG_NRM("AQA Attributes in ASQ:0x%x", aqa);

    /* Write new ASQ size using AQA */
    writel(aqa, &pnvme_dev->nvme_ctrl_space->aqa);

    /* Write the DMA address into ASQ base address */
    WRITEQ(pmetrics_sq_list->private_sq.asq_dma_addr,
            &pnvme_dev->nvme_ctrl_space->asq);
#ifdef DEBUG
    /* Debug statements */
    LOG_DBG("Admin CQ Base Address = 0x%x",
        (u32)readl(&pnvme_dev->nvme_ctrl_space->acq));
    /* Read the AQA attributes after writing and check */
    tmp_aqa = readl(&pnvme_dev->nvme_ctrl_space->aqa);

    LOG_NRM("Reading AQA after writing = 0x%x", tmp_aqa);

    /* Read the status register and printout to log */
    tmp_aqa = readl(&pnvme_dev->nvme_ctrl_space->csts);

    LOG_NRM("Reading status reg = 0x%x", tmp_aqa);
#endif

    /* Set the door bell of ASQ to 0x1000 as per spec 1.0b */
    pmetrics_sq_list->private_sq.dbs =
            ((void __iomem *)pnvme_dev->nvme_ctrl_space) + NVME_SQ0TBDL;
    /* set private members in sq metrics */
    pmetrics_sq_list->private_sq.size = asq_depth;
    pmetrics_sq_list->private_sq.unique_cmd_id = 0;
    /* returns success or failure*/
    return SUCCESS;
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
    u32 acq_depth;          /* local var to cal nbytes based on elements   */

    LOG_NRM("Creating Admin Completion Queue...");

    /* As the Admin Q ID is always 0*/
    acq_id = 0;

    /* Checking for overflow or underflow. */
    if (qsize > MAX_AQ_ENTRIES || qsize == 0) {
        LOG_ERR("ASQ size is more than MAX Q size or specified NULL");
        return -EINVAL;
    }
    /*
    * As the qsize send is in number of entries this computes the no. of bytes
    * computed.
    */
    acq_depth = qsize*sizeof(u8)*16;
    LOG_DBG("ACQ Depth: 0x%x", acq_depth);
    /*
     * The function dma_alloc_coherent  maps the dma address for ACQ which gets
     * the DMA mapped address from the kernel virtual address.
     */
    pmetrics_cq_list->private_cq.vir_kern_addr =
            dma_alloc_coherent(&pnvme_dev->pdev->dev, acq_depth,
                    &pmetrics_cq_list->private_cq.acq_dma_addr, GFP_KERNEL);
    if (!pmetrics_cq_list->private_cq.vir_kern_addr) {
        LOG_ERR("Unable to allocate DMA Address for ACQ!!");
        return -ENOMEM;
    }

    LOG_NRM("Virtual ACQ DMA Address: 0x%llx",
            (u64)pmetrics_cq_list->private_cq.vir_kern_addr);
    LOG_NRM("ACQ DMA Address: 0x%llx",
            (u64)pmetrics_cq_list->private_cq.acq_dma_addr);

    /* Read, Modify and write the Admin Q attributes */
    aqa = (qsize - 1) << 16; /* acqs is zero based value */
    aqa &= ACQS_MASK;
    tmp_aqa = readl(&pnvme_dev->nvme_ctrl_space->aqa);
    tmp_aqa &= ~ACQS_MASK;

    /* Final value to write to AQA Register */
    aqa |= tmp_aqa;

    LOG_DBG("Modified Attributes (AQA) = 0x%x", tmp_aqa);
    LOG_NRM("AQA Attributes in ACQ:0x%x", aqa);

    /* Write new ASQ size using AQA */
    writel(aqa, &pnvme_dev->nvme_ctrl_space->aqa);
    /* Write the DMA address into ACQ base address */
    WRITEQ(pmetrics_cq_list->private_cq.acq_dma_addr,
            &pnvme_dev->nvme_ctrl_space->acq);
#ifdef DEBUG
    /* Read the AQA attributes after writing and check */
    tmp_aqa = readl(&pnvme_dev->nvme_ctrl_space->aqa);

    LOG_NRM("Reading AQA after writing in ACQ = 0x%x\n", tmp_aqa);

#endif

    pmetrics_cq_list->private_cq.size = acq_depth;
    /* Set the door bell of ACQ to 0x1000 as per spec 1.0b */
    pmetrics_cq_list->private_cq.dbs =
            ((void __iomem *)pnvme_dev->nvme_ctrl_space) + NVME_CQ0TBDL;
    /* returns success or failure*/
    return ret_code;
}

/*
* nvme_prepare_sq - This routine is called when the driver invokes the ioctl for
* IO SQ Creation. It will retrieve the q size from IOSQES from CC.
*/
int nvme_prepare_sq(struct  metrics_sq  *pmetrics_sq_list,
            struct nvme_device *pnvme_dev)
{
    int ret_code = SUCCESS;
    u32 ctrl_config = 0;
    u16 u16cap_mqes = 0;
    u8  cap_dstrd;

    /*Read Controller Configuration CC register at offset 0x14h. */
    ctrl_config = readl(&pnvme_dev->nvme_ctrl_space->cc);
    /* Extract the IOSQES from CC */
    ctrl_config = (ctrl_config >> 16) & 0xF;
    LOG_NRM("CC.IOSQES = 0x%x, 2^x = %d", ctrl_config, (1 << ctrl_config));
    pmetrics_sq_list->private_sq.size = pmetrics_sq_list->public_sq.elements *
            (1 << ctrl_config);
    /* Check to see if the entries exceed the Max Q entries supported */
    u16cap_mqes = readl(&pnvme_dev->nvme_ctrl_space->cap) & 0xFFFF;
    LOG_NRM("Max Q:Actual Q elements = 0x%x:0x%x", u16cap_mqes,
            pmetrics_sq_list->public_sq.elements);
    /* I should not return from here if exceeds */
    if (pmetrics_sq_list->public_sq.elements > u16cap_mqes) {
        LOG_ERR("The IO SQ id = %d exceeds maximum elements allowed!",
                pmetrics_sq_list->public_sq.sq_id);
    }
    /*
     * call dma_alloc_coherent or SQ which gets DMA mapped address from
     * the kernel virtual address.
     * != 0 is contiguous SQ as per design.
     */
    if (pmetrics_sq_list->private_sq.contig != 0) {
        /* Assume that CMD.DW11.PC bit will be set to one. */
        pmetrics_sq_list->private_sq.vir_kern_addr = dma_alloc_coherent(
                &pnvme_dev->pdev->dev, pmetrics_sq_list->private_sq.size,
                &pmetrics_sq_list->private_sq.asq_dma_addr, GFP_KERNEL);
        /* Check if the dma alloc was successful */
        if (!pmetrics_sq_list->private_sq.vir_kern_addr) {
            LOG_ERR("Unable to allocate DMA Address for IO SQ!!");
            return -ENOMEM;
        }
    }
    /* Set Unique Command value to zero for starters. */
    pmetrics_sq_list->private_sq.unique_cmd_id = 0;
    cap_dstrd = (READQ(&pnvme_dev->nvme_ctrl_space->cap) >> 32) & 0xF;
    LOG_DBG("CAP DSTRD Value = 0x%x", cap_dstrd);
    pmetrics_sq_list->private_sq.dbs =
            ((void __iomem *)pnvme_dev->nvme_ctrl_space) + NVME_SQ0TBDL +
            ((2 * pmetrics_sq_list->public_sq.sq_id) * (4 << cap_dstrd));

    return ret_code;
}

/*
* nvme_prepare_cq - This routine is called when the driver invokes the ioctl for
* IO CQ Preparation. It will retrieve the q size from IOSQES from CC.
*/
int nvme_prepare_cq(struct  metrics_cq  *pmetrics_cq_list,
            struct nvme_device *pnvme_dev)
{
    int ret_code = SUCCESS;
    u32 ctrl_config = 0;
    u16 u16cap_mqes = 0;
    u8  cap_dstrd;

    /*Read Controller Configuration CC register at offset 0x14h. */
    ctrl_config = readl(&pnvme_dev->nvme_ctrl_space->cc);
    /* Extract the IOCQES from CC */
    ctrl_config = (ctrl_config >> 20) & 0xF;
    LOG_NRM("CC.IOCQES = 0x%x, 2^x = %d", ctrl_config, (1 << ctrl_config));
    pmetrics_cq_list->private_cq.size = pmetrics_cq_list->public_cq.elements *
            (1 << ctrl_config);
    /* Check to see if the entries exceed the Max Q entries supported */
    u16cap_mqes = readl(&pnvme_dev->nvme_ctrl_space->cap) & 0xFFFF;
    LOG_NRM("Max CQ:Actual CQ elements = 0x%x:0x%x", u16cap_mqes,
            pmetrics_cq_list->public_cq.elements);
    /* I should not return from here if exceeds */
    if (pmetrics_cq_list->public_cq.elements > u16cap_mqes) {
        LOG_ERR("The IO CQ id = %d exceeds maximum elements allowed!",
                pmetrics_cq_list->public_cq.q_id);
    }
    /*
     * call dma_alloc_coherent for CQ which gets DMA mapped address from
     * the kernel virtual address only for contiguous CQ case.
     * != 0 is contiguous CQ as per design.
     */
    if (pmetrics_cq_list->private_cq.contig != 0) {
        /* Assume that CMD.DW11.PC bit will be set to one. */
        pmetrics_cq_list->private_cq.vir_kern_addr = dma_alloc_coherent(
                &pnvme_dev->pdev->dev, pmetrics_cq_list->private_cq.size,
                &pmetrics_cq_list->private_cq.acq_dma_addr, GFP_KERNEL);
        /* Check if the dma alloc was successful */
        if (!pmetrics_cq_list->private_cq.vir_kern_addr) {
            LOG_ERR("Unable to allocate DMA Address for IO CQ!!");
            return -ENOMEM;
        }
    }

    cap_dstrd = (READQ(&pnvme_dev->nvme_ctrl_space->cap) >> 32) & 0xF;
    LOG_DBG("CAP DSTRD Value = 0x%x", cap_dstrd);
    /* Here CQ also used SQ0TDBL offset i.e., 0x1000h. */
    pmetrics_cq_list->private_cq.dbs =
            ((void __iomem *)pnvme_dev->nvme_ctrl_space) + NVME_SQ0TBDL +
            ((2 * pmetrics_cq_list->public_cq.q_id + 1) * (4 << cap_dstrd));

    return ret_code;
}

/*
* nvme_ring_sqx_dbl - This routine is called when the driver invokes the ioctl
* for Ring SQ doorbell. It will retrieve the q from the linked list, copy the
* tail_ptr with virtual pointer, and write the tail pointer value to SqxTDBL
* already in dbs.
*/
int nvme_ring_sqx_dbl(struct nvme_ring_sqxtdbl *ring_sqx,
        struct nvme_device *pnvme_dev)
{
    struct  metrics_sq  *pmetrics_sq_list;  /* SQ linked list */

    /* Seek the SQ within metrics device SQ list */
    list_for_each_entry(pmetrics_sq_list, &metrics_sq_ll, sq_list_hd) {
        /* Check if the Q Id matches */
        if (ring_sqx->sq_id == pmetrics_sq_list->public_sq.sq_id) {
            LOG_ERR("SQ_ID= %d found in the linked list.",
                    pmetrics_sq_list->public_sq.sq_id);
#ifdef DEBUG
             LOG_NRM("\tVirt Tail Ptr = 0x%x",
                     pmetrics_sq_list->public_sq.tail_ptr_virt);
             LOG_NRM("\tTail Ptr = 0x%x",
                     pmetrics_sq_list->public_sq.tail_ptr);
#endif
            /* Copy tail_prt_virt to tail_prt */
            pmetrics_sq_list->public_sq.tail_ptr = pmetrics_sq_list->
                    public_sq.tail_ptr_virt;
            /* Ring the doorbell with tail_prt */
             writel(pmetrics_sq_list->public_sq.tail_ptr, pmetrics_sq_list->
                     private_sq.dbs);
#ifdef DEBUG
             LOG_NRM("After Writing Doorbell...");
             LOG_NRM("\tVirt Tail Ptr = 0x%x",
                     pmetrics_sq_list->public_sq.tail_ptr_virt);
             LOG_NRM("\tTail Ptr = 0x%x",
                     pmetrics_sq_list->public_sq.tail_ptr);
#endif
             /* Done with this function return success */
             return SUCCESS;
        }
    }
    LOG_DBG("SQ ID = %d not found to ring its doorbell", ring_sqx->sq_id);
    /* If it falls here no SQ ID is found */
    return -EINVAL;
}

/*
 * Deallocate function is called when we want to free up the kernel or prp
 * memory based on the contig flag. The kernel memory is given back, nodes
 * from the cq list are deleted.
 */
int deallocate_metrics_cq(struct device *dev,
        struct  metrics_cq  *pmetrics_cq_list)
{
    /* Delete memory for all metrics_cq for current id here */
    if (pmetrics_cq_list->private_cq.contig == 0) {
        /* free prp list pointed by this non contig cq */
    } else {
        /* Contiguous CQ, so free the DMA memory */
        dma_free_coherent(dev, pmetrics_cq_list->private_cq.size,
                 (void *)pmetrics_cq_list->private_cq.vir_kern_addr,
                 pmetrics_cq_list->private_cq.acq_dma_addr);

    }
    /* Delete the current cq entry from the list */
    list_del_init(&pmetrics_cq_list->cq_list_hd);
    /* free the node from the ll */
    kfree(pmetrics_cq_list);

    return SUCCESS;
}

/*
 * Deallocate function is called when we want to free up the kernel or prp
 * memory based on the contig flag. The kernel memory is given back, nodes
 * from the sq list are deleted. The cmds tracked are dropped and nodes in
 * command list are deleted.
 */
int deallocate_metrics_sq(struct device *dev,
        struct  metrics_sq  *pmetrics_sq_list)
{
    struct cmd_track *pcmd_track_list;   /* to track a particular cmd     */
    struct cmd_track *pcmd_track_next;   /* to track a particular cmd     */

    /* drop all outstanding cmds for this SQ on the floor. */
    list_for_each_entry_safe(pcmd_track_list, pcmd_track_next, &sq_cmd_ll,
            cmd_list_hd) {
        /* Free prp list pointed by this tracked cmd. */
        LOG_DBG("Delete->cmd_id:sd_id = %d:%d", pcmd_track_list->unique_id,
                pmetrics_sq_list->public_sq.sq_id);
        /* Delete the current sq entry from the list */
        list_del_init(&pcmd_track_list->cmd_list_hd);
    }
    /* Clean up memory for all metrics_sq for current id here */
    if (pmetrics_sq_list->private_sq.contig == 0) {
        /* free the prp pool pointed by this non contig sq. */
        LOG_DBG("DMA Free for non-contig sq id = %d", pmetrics_sq_list->
                public_sq.sq_id);
    } else {
        LOG_DBG("DMA Free for contig sq id = %d", pmetrics_sq_list->
                public_sq.sq_id);
        /* Contiguous SQ, so free the DMA memory */
        dma_free_coherent(dev, pmetrics_sq_list->private_sq.size,
                (void *)pmetrics_sq_list->private_sq.vir_kern_addr,
                pmetrics_sq_list->private_sq.asq_dma_addr);
    }
    /* Delete the current sq entry from the list */
    list_del_init(&pmetrics_sq_list->sq_list_hd);
    kfree(pmetrics_sq_list);
    return SUCCESS;
}

/*
 * Reinitialize the admin completion queue's public parameters, when
 * a controller is not completely diabled
 */
int reinit_admn_cq(struct  metrics_cq  *pmetrics_cq_list)
{
    pmetrics_cq_list->public_cq.head_ptr = 0;
    pmetrics_cq_list->public_cq.tail_ptr = 0;
    return SUCCESS;
}

/*
 * Reinitialize the admin Submission queue's public parameters, when
 * a controller is not completely diabled
 */
int reinit_admn_sq(struct  metrics_sq  *pmetrics_sq_list)
{
    struct cmd_track *pcmd_track_list;   /* to track a particular cmd     */
    struct cmd_track *pcmd_track_next;   /* to track a particular cmd     */

    /* Wipe all outstanding cmds for this ASQ. */
    list_for_each_entry_safe(pcmd_track_list, pcmd_track_next, &sq_cmd_ll,
            cmd_list_hd) {
        LOG_DBG("Cmd Free for sq id = %d", pmetrics_sq_list->public_sq.sq_id);
        /* TODO Free the prp list described by cmd_track[j].prp_non_persist */
        list_del_init(&pcmd_track_list->cmd_list_hd);
        kfree(pcmd_track_list);
    }

    pmetrics_sq_list->public_sq.head_ptr = 0;
    pmetrics_sq_list->public_sq.tail_ptr = 0;
    pmetrics_sq_list->public_sq.tail_ptr_virt = 0;

    return SUCCESS;
}
/*
 *  deallocate_all_queues - This function will start freeing up the memory for
 * the queues (SQ and CQ) allocated during the prepare queues. The parameter
 * 'new_state', ST_DISABLE or ST_DISABLE_COMPLETELY, identifies if you need to
 * clear Admin Q as well along with other Q's.
 */
int deallocate_all_queues(struct  metrics_device_list *pmetrics_device,
        enum nvme_state new_state)
{
    s16 exclude_admin = -1;
    struct  metrics_sq  *pmetrics_sq_list;
    struct  metrics_sq  *pmetrics_sq_next;
    struct  metrics_cq  *pmetrics_cq_list;
    struct  metrics_cq  *pmetrics_cq_next;
    struct device *dev;

    /* Check the desired state of the controller */
    if (new_state == ST_DISABLE) {
        exclude_admin = 1;
    }
    /* Loop through the devices available in the metrics list */
    list_for_each_entry(pmetrics_device, &metrics_dev_ll, metrics_device_hd) {
        dev = &pmetrics_device->pnvme_device->pdev->dev;
        /* Loop for each sq node */
        list_for_each_entry_safe(pmetrics_sq_list, pmetrics_sq_next,
                &metrics_sq_ll, sq_list_hd) {
            /* Check if Admin Q is excluded or not */
            if ((exclude_admin == 1) &&
                    (pmetrics_sq_list->public_sq.sq_id == 0)) {
                LOG_DBG("Retaining Admin SQ from deallocation");
                /* drop sq cmds and set to zero the public metrics of asq */
                reinit_admn_sq(pmetrics_sq_list);
            } else {
                /* Call the generic deallocate sq function */
                deallocate_metrics_sq(dev, pmetrics_sq_list);
            }
        } /* list loop for sq list */
        /* Loop for each cq node */
        list_for_each_entry_safe(pmetrics_cq_list, pmetrics_cq_next,
                &metrics_cq_ll, cq_list_hd) {
            /* Check if Admin Q is excluded or not */
            if ((exclude_admin == 1) && pmetrics_cq_list->public_cq.q_id == 0) {
                LOG_DBG("Retaining Admin CQ from deallocation");
                /* set to zero the public metrics of acq */
                reinit_admn_cq(pmetrics_cq_list);
            } else {
                /* Call the generic deallocate cq function */
                deallocate_metrics_cq(dev, pmetrics_cq_list);
            }
        } /* list loop for cq list */

        /* if complete disable then reset the controller admin registers. */
        if (new_state == ST_DISABLE_COMPLETELY) {
            /* Set the Registers to default values. */
            /* Write 0 to AQA */
            writel(0x0, &pmetrics_device->pnvme_device->nvme_ctrl_space->aqa);
            /* Write 0 to the DMA address into ASQ base address */
            WRITEQ(0x0, &pmetrics_device->pnvme_device->nvme_ctrl_space->asq);
            /* Write 0 to the DMA address into ACQ base address */
            WRITEQ(0x0, &pmetrics_device->pnvme_device->nvme_ctrl_space->acq);
        }
    } /* list loop list of devices */

    return SUCCESS;
}
