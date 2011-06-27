#include <linux/kthread.h>

#include "test_driver.h"
#include "definitions.h"

/*
#define PERSISTENT_QUEUE 1
*/
#ifdef PERSISTENT_QUEUE
struct nvme_queue *nvmeq = NULL;
#endif

static unsigned long free_cmdid(struct nvme_queue *nvmeq, int cmdix)
{
	unsigned long data;

	if (cmdix >= nvmeq->q_depth)
		return CMD_CTX_INVALID;

	data = nvmeq->cmdid_data[cmdix].ctx;
	nvmeq->cmdid_data[cmdix].ctx = CMD_CTX_COMPLETED;
	clear_bit(cmdix, (long unsigned int *)&nvmeq->cmdid_data[nvmeq->q_depth]);

	wake_up(&nvmeq->sq_not_full);

	return data;
}

static void sync_completion(struct nvme_queue *nvmeq, void *ctx,
	struct nvme_completion *cqe)
{
	struct sync_cmd_info *cmdinfo = ctx;

	if (unlikely((unsigned long)cmdinfo == CMD_CTX_CANCELLED))
		return;

	if ((unsigned long)cmdinfo == CMD_CTX_FLUSH)
		return;

	if (unlikely((unsigned long)cmdinfo == CMD_CTX_COMPLETED)) {
		LOG_NORMAL("[Nvme_Drv]completed id %d twice on queue %d\n",
			cqe->command_id, le16_to_cpup(&cqe->sq_id));
		return;
	}

	if (unlikely((unsigned long)cmdinfo == CMD_CTX_INVALID)) {
		LOG_NORMAL("[Nvme_Drv]invalid id %d completed on queue %d\n",
			cqe->command_id, le16_to_cpup(&cqe->sq_id));
		return;
	}

	cmdinfo->result = le32_to_cpup(&cqe->result);
	cmdinfo->status = le16_to_cpup(&cqe->status) >> 1;

	wake_up_process(cmdinfo->task);
}
static void async_completion(struct nvme_queue *nvmeq, void *ctx,
	struct nvme_completion *cqe)
{
	struct sync_cmd_info *cmdinfo = ctx;

	if (unlikely((unsigned long)cmdinfo == CMD_CTX_CANCELLED))
		return;

	if ((unsigned long)cmdinfo == CMD_CTX_FLUSH)
		return;

	if (unlikely((unsigned long)cmdinfo == CMD_CTX_COMPLETED)) {
		LOG_NORMAL("[Nvme_Drv]completed id %d twice on queue %d\n",
			cqe->command_id, le16_to_cpup(&cqe->sq_id));
		return;
	}

	if (unlikely((unsigned long)cmdinfo == CMD_CTX_INVALID)) {
		LOG_NORMAL("[Nvme_Drv]invalid id %d completed on queue %d\n",
			cqe->command_id, le16_to_cpup(&cqe->sq_id));
		return;
	}

	cmdinfo->result = le32_to_cpup(&cqe->result);
	cmdinfo->status = le16_to_cpup(&cqe->status) >> 1;

	kfree(cmdinfo);
}

typedef void (*completion_fn)(struct nvme_queue *, void *,
	struct nvme_completion *);

static irqreturn_t process_cq(struct nvme_queue *nvmeq)
{
	u16 head, phase;
	static const completion_fn completions[4] = {
		[sync_id] = sync_completion,
		[async_id] = async_completion,
	};
	struct nvme_completion cqe;

	head = nvmeq->cq_head;
	phase = nvmeq->cq_phase;
	
	cqe = nvmeq->cqes[head];

	while ((le16_to_cpu(cqe.status) & 1) == phase) {

		unsigned long data;
		void *ptr;
		unsigned char handler;

		LOG_DEBUG("[Nvme_Drv](le16_to_cpu(cqe.status) & 1) == phase), h-%x s-%x p-%x\n",
				head, (le16_to_cpu(cqe.status)), phase);

		nvmeq->sq_head = le16_to_cpu(cqe.sq_head);

		if (++head == nvmeq->q_depth) {
			head = 0;
			phase = !phase;
		}

		data = free_cmdid(nvmeq, cqe.command_id);
		handler = data & 3;
		ptr = (void *)(data & ~3UL);
		completions[handler](nvmeq, ptr, &cqe);
		cqe = nvmeq->cqes[head];
	}

	/* If the controller ignores the cq head doorbell and continuously
	 * writes to the queue, it is theoretically possible to wrap around
	 * the queue twice and mistakenly return IRQ_NONE.  Linux only
	 * requires that 0.1% of your interrupts are handled, so this isn't
	 * a big problem.
	 */
	if (head == nvmeq->cq_head && phase == nvmeq->cq_phase)
		return IRQ_NONE;

	writel(head, nvmeq->q_db + 1);
	nvmeq->cq_head = head;
	nvmeq->cq_phase = phase;

	return IRQ_HANDLED;
}

static irqreturn_t nvme_isr(int irq, void *data)
{
	irqreturn_t result;
	struct nvme_queue *nvmeq = data;

	spin_lock(&nvmeq->q_lock);
	result = process_cq(nvmeq);
	spin_unlock(&nvmeq->q_lock);

	return result;
}

int queue_request_irq(struct test_ctx *test_ctx, struct nvme_queue *nvmeq,
	const char *name)
{
	return request_irq(test_ctx->entry[nvmeq->cq_vector].vector, nvme_isr,
		IRQF_DISABLED | IRQF_SHARED, name, nvmeq);
}

struct nvme_queue *alloc_queue(struct test_ctx *test_ctx, int qid,
	int depth, int vector)
{

	int cmdid_data_size;
	struct device *dmadev = &test_ctx->test_dev->dev;
	struct nvme_queue *nvmeq;

	/* the cmd_data field of this nvmeq consist of depth nvme_cmd_info's plus a bitfield */
	cmdid_data_size = (depth * sizeof(struct nvme_cmd_info)) + sizeof(BITS_TO_LONGS(depth));

	nvmeq = kzalloc(sizeof(*nvmeq) + cmdid_data_size, GFP_KERNEL);
	if (!nvmeq)
		return NULL;

	nvmeq->cqes = dma_alloc_coherent(dmadev, CQ_SIZE(depth),
		&nvmeq->cq_dma_addr, GFP_KERNEL);
	if (!nvmeq->cqes)
		goto free_nvmeq;

	memset((void *)nvmeq->cqes, 0, CQ_SIZE(depth));

	nvmeq->sq_cmds = dma_alloc_coherent(dmadev, SQ_SIZE(depth),
		&nvmeq->sq_dma_addr, GFP_KERNEL);
	if (!nvmeq->sq_cmds)
		goto free_cqdma;

	nvmeq->q_dmadev = dmadev;
	nvmeq->test = test_ctx;
	spin_lock_init(&nvmeq->q_lock);
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	init_waitqueue_head(&nvmeq->sq_not_full);
	nvmeq->q_db = &test_ctx->dbs[qid * 2];
	nvmeq->q_depth = depth;
	nvmeq->cq_vector = vector;

	return nvmeq;

free_cqdma:
	dma_free_coherent(dmadev, CQ_SIZE(nvmeq->q_depth), (void *)nvmeq->cqes,
		nvmeq->cq_dma_addr);
free_nvmeq:
	kfree(nvmeq);
	return NULL;
}

int set_test_context(struct test_ctx *test_ctx)
{
	int result = 0;
	u32 aqa;
#ifndef PERSISTENT_QUEUE
	struct nvme_queue *nvmeq = NULL;
#endif
	test_ctx->entry = kcalloc(10, sizeof(*test_ctx->entry), GFP_KERNEL);
	test_ctx->queues = kcalloc(10 + 1, sizeof(void *), GFP_KERNEL);
	test_ctx->dbs = ((void __iomem *)test_ctx->bar_dev) + 4096;

	if (!nvmeq) {
		LOG_DEBUG("[Nvme_Drv]ALLOCATING ADMIN Q\n");
		nvmeq = alloc_queue(test_ctx, 0, 64, 0);
		if (!nvmeq)
			return -ENOMEM;

		aqa = nvmeq->q_depth - 1;
		aqa |= aqa << 16;

		test_ctx->ctrl_config = NVME_CC_ENABLE | NVME_CC_CSS_NVM;
		test_ctx->ctrl_config |= (PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;
		test_ctx->ctrl_config |= NVME_CC_ARB_RR | NVME_CC_SHN_NONE;
		test_ctx->ctrl_config |= NVME_CC_IOSQES | NVME_CC_IOCQES;

		writel(0, &test_ctx->bar_dev->cc);
		writel(aqa, &test_ctx->bar_dev->aqa);
		writeq(nvmeq->sq_dma_addr, &test_ctx->bar_dev->asq);
		writeq(nvmeq->cq_dma_addr, &test_ctx->bar_dev->acq);
		writel(test_ctx->ctrl_config, &test_ctx->bar_dev->cc);

		while (!(readl(&test_ctx->bar_dev->csts) & NVME_CSTS_RDY)) {
			msleep(100);
			if (fatal_signal_pending(current))
				return -EINTR;
		}
	}
	else {
		LOG_DEBUG("[Nvme_Drv]USING ALREADY ALLLOCATED ADMIN Q\n");
	}

	test_ctx->queues[0] = nvmeq;
	test_ctx->entry[0].vector = test_ctx->test_dev->irq;

	return result;
}

int clear_test_context(struct test_ctx *test_ctx)
{
	int result = 0;

	LOG_DEBUG("[Nvme_Drv]executing clear_test_context function\n");

#ifndef PERSISTENT_QUEUE
	dma_free_coherent(test_ctx->queues[0]->q_dmadev, 
		SQ_SIZE(test_ctx->queues[0]->q_depth),
		test_ctx->queues[0]->sq_cmds, test_ctx->queues[0]->sq_dma_addr);
	dma_free_coherent(test_ctx->queues[0]->q_dmadev, 
		CQ_SIZE(test_ctx->queues[0]->q_depth),
		(void *)test_ctx->queues[0]->cqes, test_ctx->queues[0]->cq_dma_addr);
#endif

	kfree(test_ctx->queues);
	kfree(test_ctx->entry);
	test_ctx->queues = 0;
	test_ctx->entry = 0;
	test_ctx->dbs = 0;
	/* reset bar_dev regs (cc,aqa,acq,asq)? NO */

	return result;
}

int delete_queue(struct test_ctx *test_ctx, u8 opcode, u16 id)
{
	int status;
	struct nvme_command c;

	memset(&c, 0, sizeof(c));
	c.delete_queue.opcode = opcode;
	c.delete_queue.qid = cpu_to_le16(id);

	status = submit_admin_cmd(test_ctx, &c, sync_id, NULL);

	return status;
}

static int alloc_cmdid(struct nvme_queue *nvmeq, void *ctx, int handler,
	unsigned timeout)
{
	int depth = nvmeq->q_depth;
	int cmdix;

	BUG_ON((unsigned long)ctx & 3);

	do {
		cmdix = find_first_zero_bit(
			(long unsigned int *)&nvmeq->cmdid_data[depth], 
			depth-1);
		if (cmdix >= depth)
			return -EBUSY;
	} while (test_and_set_bit(cmdix, 
			(long unsigned int *)&nvmeq->cmdid_data[depth]));

	nvmeq->cmdid_data[cmdix].ctx = (unsigned long)ctx | handler;
	nvmeq->cmdid_data[cmdix].timeout = jiffies + timeout;

	return cmdix;
}

static int alloc_cmdid_killable(struct nvme_queue *nvmeq, void *ctx,
	int handler, unsigned timeout)
{
	int cmdid;

	wait_event_killable(nvmeq->sq_not_full,
		(cmdid = alloc_cmdid(nvmeq, ctx, handler, timeout)) >= 0);

	return (cmdid < 0) ? -EINTR : cmdid;
}

static int submit_cmd(struct nvme_queue *nvmeq, struct nvme_command *cmd)
{
	unsigned long flags;
	u16 tail;

	spin_lock_irqsave(&nvmeq->q_lock, flags);
	tail = nvmeq->sq_tail;
	memcpy(&nvmeq->sq_cmds[tail], cmd, sizeof(*cmd));
	if (++tail == nvmeq->q_depth)
		tail = 0;

	writel(tail, nvmeq->q_db);
	nvmeq->sq_tail = tail;
	spin_unlock_irqrestore(&nvmeq->q_lock, flags);

	return 0;
}

static void cancel_cmdid_data(struct nvme_queue *nvmeq, int cmdix)
{
	if (cmdix < nvmeq->q_depth) {
		/* was this an async cmd? if so, need to free ctx. */
		if ((nvmeq->cmdid_data[cmdix].ctx & 3) == async_id)
			kfree((void *)(nvmeq->cmdid_data[cmdix].ctx & ~3));
		nvmeq->cmdid_data[cmdix].ctx = CMD_CTX_CANCELLED;
	}
}

static void abort_command(struct nvme_queue *nvmeq, int cmdid)
{
	spin_lock_irq(&nvmeq->q_lock);
	cancel_cmdid_data(nvmeq, cmdid);
	spin_unlock_irq(&nvmeq->q_lock);
}

static int submit_sync_cmd(struct nvme_queue *nvmeq,
	struct nvme_command *cmd, u32 *result, unsigned timeout)
{
	int cmdid;
	struct sync_cmd_info cmdinfo;

	cmdinfo.task = current;
	cmdinfo.status = -EINTR;

	cmdid = alloc_cmdid_killable(nvmeq, &cmdinfo, sync_id,
		timeout);
	if (cmdid < 0)
		return cmdid;

	cmd->common.command_id = cmdid;

	set_current_state(TASK_KILLABLE);
	submit_cmd(nvmeq, cmd);

	LOG_ERROR("[Nvme_Drv]SUBMIT_SYNC_CMD cmdid = %x wait for signal. . . .\n",cmdid);
	schedule();
/*	LOG_ERROR("[Nvme_Drv]SUBMIT_SYNC_CMD - continuing\n");  */

	if (cmdinfo.status == -EINTR) {
		abort_command(nvmeq, cmdid);
		return -EINTR;
	}

	if (result)
		*result = cmdinfo.result;

	return cmdinfo.status;
}
static int submit_async_cmd(struct nvme_queue *nvmeq,
	struct nvme_command *cmd)
{
	int cmdid;
	struct sync_cmd_info *cmdinfo;

	cmdinfo = kzalloc(sizeof(*cmdinfo), GFP_KERNEL);
	cmdinfo->task = current;
	cmdinfo->status = -EINTR;

	cmdid = alloc_cmdid_killable(nvmeq, cmdinfo, async_id, 0);
	if (cmdid < 0)
		return cmdid;
	LOG_ERROR("[Nvme_Drv]SUBMIT_*A*SYNC_CMD cmdid = %x wait for signal. . . .\n",cmdid);

	cmd->common.command_id = cmdid;

	submit_cmd(nvmeq, cmd);

	return 0;
}

int submit_admin_cmd(struct test_ctx *test_ctx, struct nvme_command *cmd,
	int id, u32 *result)
{
	if (id == async_id)
		return submit_async_cmd(test_ctx->queues[0], cmd);

	return submit_sync_cmd(test_ctx->queues[0], cmd, result, ADMIN_TIMEOUT);
}

int compl_kthread(void *data)
{
	int i;
	struct test_ctx *test_ctx;

	test_ctx = (struct test_ctx *)data;

	while (!kthread_should_stop()) {
		__set_current_state(TASK_RUNNING);
			/* for (i = 0; i < test_ctx->queue_count; i++) {
			 * FIXME - problem in kernel/lockdep.c:2621
			 * Check only ADMIN queue
			 */
			for (i = 0; i < 1; i++) {
				struct nvme_queue *nvmeq = test_ctx->queues[i];
				if (!nvmeq)
					continue;
				spin_lock_irq(&nvmeq->q_lock);
				process_cq(nvmeq);
				spin_unlock_irq(&nvmeq->q_lock);
			}
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);
	}
	return 0;
}

int create_cq(struct test_ctx *test_ctx, int qid, int vector, int flags,
	struct nvme_queue *nvmeq)
{
	struct nvme_command c;
	int result;

	memset(&c, 0, sizeof(c));
	c.create_cq.opcode = nvme_admin_create_cq;
	c.create_cq.prp1 = cpu_to_le64(nvmeq->cq_dma_addr);
	c.create_cq.cqid = cpu_to_le16(qid);
	c.create_cq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_cq.cq_flags = cpu_to_le16(flags);
	c.create_cq.irq_vector = cpu_to_le16(nvmeq->cq_vector);

	result = submit_admin_cmd(test_ctx, &c, sync_id, NULL);

	return result;
}

int create_sq(struct test_ctx *test_ctx, int qid, int flags,
		 struct nvme_queue *nvmeq)
{
	struct nvme_command c;

	memset(&c, 0, sizeof(c));
	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(nvmeq->sq_dma_addr);
	c.create_sq.sqid = cpu_to_le16(qid);
	c.create_sq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_sq.sq_flags = cpu_to_le16(flags);
	c.create_sq.cqid = cpu_to_le16(qid);

	return submit_admin_cmd(test_ctx, &c, sync_id, NULL);
}

int delete_cq(struct test_ctx *test_ctx, int cqid)
{
	return delete_queue(test_ctx, nvme_admin_delete_cq, cqid);
}

int delete_sq(struct test_ctx *test_ctx, int sqid)
{
	return delete_queue(test_ctx, nvme_admin_delete_sq, sqid);
}

int get_features(struct test_ctx *test_ctx, int feature_id, int *val)
{
	struct nvme_command c;
	u32 result;
	int status;

	memset(&c, 0, sizeof(c));
	c.features.opcode = nvme_admin_get_features;
	c.features.fid = cpu_to_le32(feature_id);

	status = submit_admin_cmd(test_ctx, &c, sync_id, &result);
	*val = result;

	return status;
}

int set_features(struct test_ctx *test_ctx, int feature_id, int val)
{
	struct nvme_command c;

	memset(&c, 0, sizeof(c));
	c.features.opcode = nvme_admin_set_features;
	c.features.fid = cpu_to_le32(feature_id);
	c.features.dword11 = cpu_to_le32(val);

#ifdef PERSISTENT_QUEUE
	/* probably not a good idea to send async cmd if nvmeq may be deleted */
	return submit_admin_cmd(test_ctx, &c, async_id, NULL);
#else
	return submit_admin_cmd(test_ctx, &c, sync_id, NULL);
#endif
}

int check_feature(struct test_ctx *test_ctx, int feature_id, int newval)
{
	int status;
	int result = STATUS_OK;
	int ret_val, orig_val;

	/* Get feature's original value */
	status = get_features(test_ctx, feature_id, &orig_val);
	if (!status) {
		LOG_DEBUG("[Nvme_Drv]FeaID: %d - original value %d\n", feature_id,
			orig_val);

		/* Set feature's new value */
		status = set_features(test_ctx, feature_id, newval);
		if (!status) {
			/* Check if value has been set correctly */
			status = get_features(test_ctx, feature_id, &ret_val);
			if (!status) {
				LOG_DEBUG("[Nvme_Drv]FeaID: %d - Value for Set %d, value "
					"from Get %d\n", feature_id, newval,
					ret_val);
				if (ret_val == newval)
					result = STATUS_OK;
				else {
					LOG_ERROR("[Nvme_Drv]FID: %d - New value hasn't "
						"been set\n", feature_id);
					result = STATUS_NOK;
				}
			} else
				result = STATUS_INIT_FAILED;

			/* Set feature's original value */
			status = set_features(test_ctx, feature_id, orig_val);
			if (status) {
				LOG_ERROR("[Nvme_Drv]FID: %d - Original value hasn't been"
					" restored\n", feature_id);
				result = STATUS_INIT_FAILED;
			}
		} else
			result = STATUS_INIT_FAILED;
	} else
		result = STATUS_INIT_FAILED;

	return result;
}
