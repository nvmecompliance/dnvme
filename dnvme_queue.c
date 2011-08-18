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

/* structure for nvme queue */
struct nvme_queue *nvme_q;

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
* nvme_queue_init - NVME Q initialization routine which initailized the
* queue parameters as per the user size.
*/
int nvme_queue_init(struct nvme_dev_entry *nvme_dev, u16 qsize)
{
   unsigned extra_qdepth;

   LOG_NRM("Performing Queue Initializations...");

   /*
   * As the qsize send is in number of entries this computes the no. of bytes
   * needed for the q size to allocate memeory.
   */
   extra_qdepth = qsize * sizeof(struct nvme_command) + 1024;

   /* Check if Q is allocated else do allocation */
   if (!nvme_q) {
	nvme_q = kzalloc(sizeof(struct nvme_queue) + extra_qdepth, GFP_KERNEL);
	if (nvme_q == NULL) {
		LOG_ERR("Unable to alloc kern mem in queue initialization!!");
		return -ENOMEM;
	}
   }
   /* Assign DMA device in Queue structure */
   nvme_q->dmadev = &nvme_dev->pdev->dev;

   /* Assign nvme device at one place in nvme_q */
   nvme_q->dev = nvme_dev;

   /* Initialize spin lock */
   spin_lock_init(&nvme_q->q_lock);

   /* Set init flag to done */
   nvme_q->q_init = 1;

   return SUCCESS;
}
/*
* create_admn_sq - This routine is called when the driver invokes the ioctl for
* admn sq creation. It will automatically reset the NVME controller as it has to
* toggle the CAP.EN flag to set the parameters. The timer call is implemented in
* this function which will call the timer handler when the timer expires and ASQ
* could not be set. It returns success if the submission q creation is success
* after dma_coherent_alloc else returns failure at any step which fails.
*/
int create_admn_sq(struct nvme_dev_entry *nvme_dev, u16 qsize)
{
   int ret_code = SUCCESS;
			/* Set to SUCCES and verify if fails in this func */
   u16 asq_id;		/* Admin Submisssion Q Id                         */
   u32 aqa;		/* Admin Q attributes in 32 bits size             */
   u32 tmp_aqa;		/* Temp var to hold admin q attributes            */
   u32 timer_delay;	/* Timer delay read from CAP.TO register          */
   unsigned long time_out_flag = 1;
			/* Time Out flag for timer handler                */
   struct timer_list asq_timer; /* asq timer declaration                  */

   LOG_NRM("Creating Admin Submission Queue...");

   /* Check if the q structure is initialized else init here */
   if (!nvme_q) {
	/* Call the initialization function */
	nvme_queue_init(nvme_dev, qsize);
	if (nvme_q->q_init != 1) {
		LOG_ERR("Q Init Failed");
		return -EINVAL;
	}
   }
   /* As the Admin Q ID is always 0*/
   asq_id = 0;

   /*
   * As the qsize send is in number of entries this computes the no. of bytes
   * computed.
   */
   nvme_q->asq_depth = qsize * sizeof(struct nvme_command);

   LOG_DBG("ASQ Depth: 0x%x", nvme_q->asq_depth);

   /*
   * The function dma_alloc_coherent  maps the dma address for ASQ which gets
   * the DMA mapped address from the kernel virtual address.
   */
   nvme_q->virt_asq_addr = dma_alloc_coherent(nvme_q->dmadev, nvme_q->asq_depth,
					&nvme_q->asq_dma_addr, GFP_KERNEL);
   if (!nvme_q->virt_asq_addr) {
	LOG_ERR("Unable to allocate DMA Address for ASQ!!");
	return -ENOMEM;
   }

   LOG_NRM("Virtual ASQ DMA Address: 0x%llx", (u64)nvme_q->virt_asq_addr);

   LOG_NRM("ASQ DMA Address: 0x%llx", (u64)nvme_q->asq_dma_addr);

   /* Set the door bell or ASQ to 0x1000 as per spec 1.0a */
   nvme_dev->dbs = ((void __iomem *)nvme_dev->nvme_ctrl_space) + NVME_SQ0TBDL;

   /* Read, Modify, Write  the aqa as per the q size requested */
   aqa = qsize & ASQS_MASK;
   tmp_aqa = readl(&nvme_dev->nvme_ctrl_space->aqa);
   tmp_aqa &= ~ASQS_MASK;
   aqa |= tmp_aqa;

   LOG_DBG("Mod Attributes from AQA Reg = 0x%x", tmp_aqa);
   LOG_NRM("AQA Attributes in ASQ:0x%x", aqa);

   /* Modify the Controller Configuration for the Normal settings */
   nvme_q->dev->ctrl_config = NVME_CC_ENABLE | NVME_CC_CSS_NVM;
   nvme_q->dev->ctrl_config |= (PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;
   nvme_q->dev->ctrl_config |= NVME_CC_ARB_RR | NVME_CC_SHN_NONE;

   /* The AQA can only be modified if EN bit is set to 0 */
   writel(0, &nvme_q->dev->nvme_ctrl_space->cc);

   /* Write new ASQ size using AQA */
   writel(aqa, &nvme_q->dev->nvme_ctrl_space->aqa);

   /* Write the DMA address into ASQ base address */
   writeq(nvme_q->asq_dma_addr, &nvme_q->dev->nvme_ctrl_space->asq);

   /* stmt be moved to create_acq function once values are retained in QEMU */
   writeq(nvme_q->acq_dma_addr, &nvme_q->dev->nvme_ctrl_space->acq);

   /* write back CC configuration into cc */
   writel(nvme_q->dev->ctrl_config, &nvme_q->dev->nvme_ctrl_space->cc);

   /* Read the AQA attributes after writing and check */
   tmp_aqa = readl(&nvme_dev->nvme_ctrl_space->aqa);

   LOG_NRM("Reading AQA after writing = 0x%x", tmp_aqa);

   /* As the TO is in lower 32 of 64 bit cap readl is good enough */
   timer_delay = readl(&nvme_dev->nvme_ctrl_space->cap) & NVME_TO_MASK;

   /* Modify TO as it is specified in 500ms units, timer needs in jiffies */
   timer_delay >>= 24;
   timer_delay *= NVME_MSEC_2_JIFFIES;
   init_timer(&asq_timer);

   /* register the Timer function */
   asq_timer.data     = (unsigned long)&time_out_flag;
   asq_timer.function = jit_timer_fn;
   asq_timer.expires  = timer_delay;

   LOG_NRM("Checking if the NVME Device Status(CSTS) is ready...");
   LOG_NRM("Timer Expires in =%ld ms", asq_timer.expires);

   /* Add timer just before the status check */
   add_timer(&asq_timer);

   /* Check if the device status set to ready */
   while (!(readl(&nvme_dev->nvme_ctrl_space->csts) & NVME_CSTS_RDY)) {
	LOG_NRM("Waiting...");
	msleep(100);

	/* Check if the time out occured */
	if (time_out_flag == 0) {
		LOG_ERR("ASQ Setup Failed before Timeout");
		LOG_NRM("Check if Admin Completion Queue is Created First");

		/* Delete the timer once failed */
		del_timer(&asq_timer);

		/* set return invalid/failed */
		ret_code = -EINVAL;

		/* break from while loop */
		break;
	}
   }

   /* Read the status register and printout to log */
   tmp_aqa = readl(&nvme_dev->nvme_ctrl_space->csts);

   LOG_NRM("Reading status reg = 0x%x", tmp_aqa);

   /* Timer function is done so delete before leaving*/
   del_timer(&asq_timer);

   /* returns success or failure*/
   return ret_code;
}

/*
* create_admn_cq - This routine is called when the driver invokes the ioctl for
* admn cq creation. It will automatically reset the NVME controller as it has to
* toggle the CAP.EN flag to set the parameters. The timer call is implemented in
* this function which will call the timer handler when the timer expires and ACQ
* could not be set. It returns success if the completion q creation is success
* after dma_coherent_alloc else returns failure at any step which fails.
*/
int create_admn_cq(struct nvme_dev_entry *nvme_dev, u16 qsize)
{

   int ret_code = SUCCESS; /* Ret code set to SUCCESS check for otherwise */
   u16 acq_id;          /* Admin Submisssion Q Id                         */
   u32 aqa;		/* Admin Q attributes in 32 bits size             */
   u32 tmp_aqa;		/* Temp var to hold admin q attributes            */

   LOG_NRM("Creating Admin Completion Queue...");

   /* Check if the q structure is initialized else init here */
   if (!nvme_q) {
	/* Call the initialization function */
	nvme_queue_init(nvme_dev, qsize);
	if (nvme_q->q_init != 1) {
		LOG_ERR("Q Init Failed");
		return -EINVAL;
	}
   }
   /* As the Admin Q ID is always 0*/
   acq_id = 0;

   /*
   * As the qsize send is in number of entries this computes the no. of bytes
   * computed.
   */
   nvme_q->acq_depth = qsize * sizeof(struct nvme_command);

   LOG_DBG("ACQ Depth: 0x%x", nvme_q->acq_depth);
   /*
   * The function dma_alloc_coherent  maps the dma address for ACQ which gets
   * the DMA mapped address from the kernel virtual address.
   */
   nvme_q->virt_acq_addr = dma_alloc_coherent(nvme_q->dmadev, nvme_q->acq_depth,
					&nvme_q->acq_dma_addr, GFP_KERNEL);
   if (!nvme_q->virt_acq_addr) {
	LOG_ERR("Unable to allocate DMA Address for ACQ!!");
	return -ENOMEM;
   }

   LOG_NRM("Virtual ACQ DMA Address: 0x%llx", (u64)nvme_q->virt_acq_addr);

   LOG_NRM("ACQ DMA Address: 0x%llx", (u64)nvme_q->acq_dma_addr);

   /* Read, Modify and write the Admin Q attributes */
   aqa = qsize << 16;
   aqa &= ACQS_MASK;
   tmp_aqa = readl(&nvme_dev->nvme_ctrl_space->aqa);
   tmp_aqa &= ~ACQS_MASK;

   /* Final value to write to AQA Register */
   aqa |= tmp_aqa;

   LOG_DBG("Modified Attributes (AQA) = 0x%x", tmp_aqa);
   LOG_NRM("AQA Attributes in ACQ:0x%x", aqa);

   /* Read Controller Configuration */
   nvme_q->dev->ctrl_config |= readl(&nvme_dev->nvme_ctrl_space->cc);

   /* The AQA can only be modified if EN bit is set to 0 */
   writel(0, &nvme_q->dev->nvme_ctrl_space->cc);

   /* Write new ASQ size using AQA */
   writel(aqa, &nvme_q->dev->nvme_ctrl_space->aqa);

   /* Write the DMA address into ACQ base address */
   writeq(nvme_q->acq_dma_addr, &nvme_q->dev->nvme_ctrl_space->acq);

   /* write back Controller configuration into cc register */
   writel(nvme_q->dev->ctrl_config, &nvme_q->dev->nvme_ctrl_space->cc);

   /* Read the AQA attributes after writing and check */
   tmp_aqa = readl(&nvme_dev->nvme_ctrl_space->aqa);

   LOG_NRM("Reading AQA after writing in ACQ = 0x%x\n", tmp_aqa);

   /* returns success or failure*/
   return ret_code;
}

