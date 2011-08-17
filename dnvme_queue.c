#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "definitions.h"
#include "sysdnvme.h"
#include "dnvme_queue.h"
#include "dnvme_reg.h"

struct nvme_queue *nvme_q;

int queue_ops_init(struct nvme_queue *nvme_q)
{
   nvme_q = kzalloc(sizeof(struct nvme_queue), GFP_KERNEL);
   if (nvme_q == NULL) {
        LOG_ERR("Unable to allocate kernel mem in queue initialization!!");
        return -ENOMEM;
   }
   nvme_q->q_init = 1;

   return SUCCESS;
}
int create_admn_sq(struct nvme_dev_entry *nvme_dev, u16 qsize)
{
   int ret_code = SUCCESS;
   u16 asq_id;
   unsigned extra_qdepth;
   u32 aqa;
   u32 tmp_aqa;

   LOG_NRM("Creating Admin Submission Queue...");

   extra_qdepth = qsize * sizeof(struct nvme_command);

   LOG_NRM("...1...");
   if (!nvme_q) {
	nvme_q = kzalloc(sizeof(struct nvme_queue) + extra_qdepth, GFP_KERNEL);
	if (nvme_q == NULL) {
		LOG_ERR("Unable to allocate kernel mem in queue initialization!!");
		return -ENOMEM;
	}
   }

   LOG_NRM("...2...");
   asq_id = 0;

   nvme_q->asq_depth = qsize * sizeof(struct nvme_command) - 1;

   nvme_q->dmadev = &nvme_dev->pdev->dev;

   LOG_NRM("...3...");
   nvme_q->virt_asq_addr = dma_alloc_coherent(nvme_q->dmadev, nvme_q->asq_depth,
					&nvme_q->asq_dma_addr, GFP_KERNEL);
   if (!nvme_q->virt_asq_addr)
   {
	LOG_ERR("Unable to allocate DMA Address for ASQ!!");
	return -ENOMEM;
   }

   LOG_NRM("...4...");
   nvme_dev->dbs = ((void __iomem *)nvme_dev->nvme_ctrl_space) + NVME_SQ0TBDL;

   LOG_NRM("...5...");
   nvme_q->dev = nvme_dev;
   
   spin_lock_init(&nvme_q->q_lock);

   aqa = nvme_q->asq_depth & ASQS_MASK;

   LOG_NRM("...6...");
   tmp_aqa = readl(&nvme_dev->nvme_ctrl_space->aqa);

   LOG_DBG("Reading the AQA from AQA Reg is ASQ = 0x%x\n", tmp_aqa);

   tmp_aqa &= ~ASQS_MASK;

   LOG_NRM("...7...");
   aqa |= tmp_aqa;
   
   nvme_q->dev->ctrl_config = NVME_CC_ENABLE | NVME_CC_CSS_NVM;

   nvme_q->dev->ctrl_config |= (PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

   nvme_q->dev->ctrl_config |= NVME_CC_ARB_RR | NVME_CC_SHN_NONE;

   LOG_NRM("...8...");
   writel(0, &nvme_q->dev->nvme_ctrl_space->cc);
   writel(aqa, &nvme_q->dev->nvme_ctrl_space->aqa);
   writeq(nvme_q->asq_dma_addr, &nvme_q->dev->nvme_ctrl_space->asq);
   //writeq(nvmeq->cq_dma_addr, &dev->nvme_ctrl_space->acq);
   writel(nvme_q->dev->ctrl_config, &nvme_q->dev->nvme_ctrl_space->cc);

   LOG_NRM("...9...");
   return ret_code;
}
