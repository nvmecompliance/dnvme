#include <linux/kthread.h>

#include "test_driver.h"
#include "definitions.h"

static struct task_struct *compl_thread;

static int test_cmd_identify(struct test_ctx *test_ctx);
static int test_cmd_create_cq(struct test_ctx *test_ctx);
static int test_cmd_delete_cq(struct test_ctx *test_ctx);
static int test_cmd_get_non_ext_feat(struct test_ctx *test_ctx);
static int test_cmd_set_get_feats(struct test_ctx *test_ctx);
static int test_cmd_create_too_many_cqs(struct test_ctx *test_ctx);
static int test_cmd_abort(struct test_ctx *test_ctx);

static struct test_func test_functions[ALL_CMDS] = {
	{NULL, NULL},
	{test_cmd_identify, "Identify Command"},
	{test_cmd_create_cq, "Duplicate CQ Creation Commands"},
	{test_cmd_delete_cq, "Invalid Delete CQ Command"},
	{test_cmd_get_non_ext_feat, "Get Invalid Feature Command"},
	{test_cmd_set_get_feats, "Get and Set Features Command"},
	{test_cmd_create_too_many_cqs, "Create Too Many CQs Command"},
	{test_cmd_abort, "Abort Command"},
};

int test_cmd_identify(struct test_ctx *test_ctx)
{
	int result;
	int status;
	void *id;
	dma_addr_t dma_addr;
	struct nvme_command c;
	int val, min_val, max_val;
	int npss, i;
	struct nvme_id_ctrl *id_ctrl;
	struct pwr_state_desc *pwr_st_desc;

	id = dma_alloc_coherent(&test_ctx->test_dev->dev, PAGE_SIZE,
		&dma_addr, GFP_KERNEL);

	memset(&c, 0, sizeof(c));
	c.identify.opcode = nvme_admin_identify;
	c.identify.nsid = 0;
	c.identify.prp1 = cpu_to_le64(dma_addr);
	c.identify.cns = cpu_to_le32(1);

	status = submit_admin_cmd(test_ctx, &c, sync_id, NULL);
	if (!status) {
		id_ctrl = (struct nvme_id_ctrl *)id;

		LOG_NORMAL("[Nvme_Drv]PCI Vendor ID                               - %d\n",
			id_ctrl->vid);
		LOG_NORMAL("PCI Subsystem Vendor ID                     - %d\n",
			id_ctrl->ssvid);
		LOG_NORMAL("Model  Number                               - %s\n",
			id_ctrl->mn);
		LOG_NORMAL("Serial Number                               - %s\n",
			id_ctrl->sn);
		LOG_NORMAL("Firmware Number                             - %s\n",
			id_ctrl->fr);
		LOG_NORMAL("Recommended Arbitration Burst               - %d\n",
			id_ctrl->rab);
		LOG_NORMAL("Optional Admin Command Support              - %d\n",
			id_ctrl->oacs);

		val = id_ctrl->acl;
		if (val < ABORT_CMD_MIN_LIMIT)
			LOG_NORMAL("[Nvme_Drv]Abort Command Limit is %d. It is "
				"recommended to support a minimum of %d\n",
				val, ABORT_CMD_MIN_LIMIT);
		LOG_NORMAL("[Nvme_Drv]Abort Command Limit                         - %d\n",
			val);

		val = id_ctrl->aerl;
		if (val < ASYNCH_EVENT_REQ_LIMIT)
			LOG_NORMAL("[Nvme_Drv]Asynchronous Event Request Limit is %d. It "
				"is recommended to support a minimum of %d\n",
				val, ASYNCH_EVENT_REQ_LIMIT);
		LOG_NORMAL("Asynchronous Event Request Limit            - %d\n",
			val);

		val = ((id_ctrl->frmw >> 1) & 0x7);
		if ((val < FW_MIN_SLOT_SUPPORTED) ||
		    (val > FW_MAX_SLOT_SUPPORTED)) {
			LOG_ERROR("[Nvme_Drv]Firmware Update - Number of Supported Slots "
				"is %d (should be greater than %d and less "
				"than %d)\n", val, FW_MIN_SLOT_SUPPORTED - 1,
				FW_MAX_SLOT_SUPPORTED + 1);
			result = STATUS_NOK;
			goto clean_and_exit;
		}
		LOG_NORMAL("[Nvme_Drv]Firmware Update - Number of Supported Slots - %d\n",
			val);

		LOG_NORMAL("Log Page Attributes                         - %d\n",
			id_ctrl->lpa);
		LOG_NORMAL("Error Log Page Entries                      - %d\n",
			id_ctrl->elpe);

		npss = id_ctrl->npss + 1;
		if ((npss < NPSS_MIN_VAL) || (npss > NPSS_MAX_VAL)) {
			LOG_ERROR("[Nvme_Drv]Number of Power States Support is %d (should"
				" be greater or equal %d and less than %d)\n",
				npss, NPSS_MIN_VAL, NPSS_MAX_VAL + 1);
			result = STATUS_NOK;
			goto clean_and_exit;
		}
		LOG_NORMAL("[Nvme_Drv]Number of Power States Support              - %d\n",
			npss);

		min_val = (1 << (id_ctrl->cqes & 0xf));
		max_val = (1 << (id_ctrl->cqes >> 4));
		if (min_val != MIN_CQ_ENTRY_SIZE) {
			LOG_ERROR("[Nvme_Drv]Wrong Minimum CQ Entry Size is %d (should be"
			" %d)\n", min_val, MIN_CQ_ENTRY_SIZE);
			result = STATUS_NOK;
			goto clean_and_exit;
		}
		if (max_val < min_val) {
			LOG_ERROR("[Nvme_Drv]Wrong Maximum CQ Entry Size is %d (shouldn't"
			" be less then %d)\n", max_val, min_val);
			result = STATUS_NOK;
			goto clean_and_exit;
		}
		LOG_NORMAL("[Nvme_Drv]Minimum CQ Entry Size                       - %d\n",
			min_val);
		LOG_NORMAL("Maximum CQ Entry Size                       - %d\n",
			max_val);

		min_val = (1 << (id_ctrl->sqes & 0xf));
		max_val = (1 << (id_ctrl->sqes >> 4));
		if (min_val != MIN_SQ_ENTRY_SIZE) {
			LOG_ERROR("[Nvme_Drv]Wrong Minimum SQ Entry Size is %d (should be"
			" %d)\n", min_val, MIN_SQ_ENTRY_SIZE);
			result = STATUS_NOK;
			goto clean_and_exit;
		}
		if (max_val < min_val) {
			LOG_ERROR("[Nvme_Drv]Wrong Maximum SQ Entry Size is %d (shouldn't"
				" be less then %d)\n", max_val, min_val);
			result = STATUS_NOK;
			goto clean_and_exit;
		}
		LOG_DEBUG("[Nvme_Drv]Minimum SQ Entry Size                       - %d\n",
			min_val);
		LOG_DEBUG("Maximum SQ Entry Size                       - %d\n",
			max_val);

		LOG_NORMAL("Number of Namespaces                        - %d\n",
			id_ctrl->nn);
		LOG_NORMAL("Optional NVM Command Support                - %d\n",
			id_ctrl->oncs);
		LOG_NORMAL("Fused Operation Support                     - %d\n",
			id_ctrl->fuses);
		LOG_NORMAL("Format NVM Attributes                       - %d\n",
			id_ctrl->fna);
		LOG_NORMAL("Volatile Write Cache                        - %d\n",
			id_ctrl->vwc);
		LOG_NORMAL("Atomic Write Unit Normal                    - %d\n",
			id_ctrl->awun);

		LOG_NORMAL("Atomic Write Unit Power Fail                - %d\n",
			id_ctrl->awupf);

		for (i = NPSS_MIN_VAL; i < npss; i++) {
			pwr_st_desc = &(id_ctrl->psd[i]);

			val = pwr_st_desc->rrt;
			if ((val < NPSS_MIN_VAL) || (val > (npss - 1))) {
				LOG_ERROR("[Nvme_Drv]Power State %d Descriptor - Relative"
					" Read Throughput is %d (should be"
					" greater or equal %d and less than"
					" %d)\n", i, val, NPSS_MIN_VAL, npss);
				result = STATUS_NOK;
				goto clean_and_exit;
			}
			LOG_NORMAL("[Nvme_Drv]Power State %d Descriptor - Relative Read"
				" Throughput  - %d\n", i, val);

			val = pwr_st_desc->rrl;
			if ((val < NPSS_MIN_VAL) || (val > (npss - 1))) {
				LOG_ERROR("[Nvme_Drv]Power State %d Descriptor - Relative"
					" Read Latency is %d (should be"
					" greater or equal %d and less than"
					" %d)\n", i, val, NPSS_MIN_VAL, npss);
				result = STATUS_NOK;
				goto clean_and_exit;
			}
			LOG_NORMAL("[Nvme_Drv]Power State %d Descriptor - Relative Read"
				" Latency     - %d\n", i, val);

			val = pwr_st_desc->rwt;
			if ((val < NPSS_MIN_VAL) || (val > (npss - 1))) {
				LOG_ERROR("[Nvme_Drv]Power State %d Descriptor - Relative"
					" Write Throughput is %d (should be"
					" greater or equal %d and less than"
					" %d)\n", i, val, NPSS_MIN_VAL, npss);
				result = STATUS_NOK;
				goto clean_and_exit;
			}
			LOG_NORMAL("[Nvme_Drv]Power State %d Descriptor - Relative Write"
				" Throughput - %d\n", i, val);

			val = pwr_st_desc->rwl;
			if ((val < NPSS_MIN_VAL) || (val > (npss - 1))) {
				LOG_ERROR("[Nvme_Drv]Power State %d Descriptor - Relative"
					" Write Latency is %d (should be"
					" greater or equal %d and less than"
					" %d)\n", i, val, NPSS_MIN_VAL, npss);
				result = STATUS_NOK;
				goto clean_and_exit;
			}
			LOG_NORMAL("[Nvme_Drv]Power State %d Descriptor - Relative Write"
				" Latency    - %d\n", i, val);

			LOG_NORMAL("Power State %d Descriptor - Maximum Power "
				"            - %d\n", i, pwr_st_desc->mp);
			LOG_NORMAL("Power State %d Descriptor - Entry Latency "
				"            - %d\n", i, pwr_st_desc->enlat);
			LOG_NORMAL("Power State %d Descriptor - Exit Latency  "
				"            - %d\n", i, pwr_st_desc->exlat);
		}

		result = STATUS_OK;
	} else
		result = STATUS_INIT_FAILED;

clean_and_exit:
	dma_free_coherent(&test_ctx->test_dev->dev, PAGE_SIZE, id, dma_addr);

	return result;
}

int test_cmd_create_cq(struct test_ctx *test_ctx)
{
	struct nvme_queue *nvmeq = NULL;
	int status;
	int result = STATUS_OK;
	int qid = 1;
	int vector = 1;
	int flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;

	nvmeq = alloc_queue(test_ctx, qid, NVME_Q_DEPTH, vector);
	if (!nvmeq)
		return STATUS_INIT_FAILED;

	status = create_cq(test_ctx, qid, vector, flags, nvmeq);
	if (!status) {

		/*
		 * FIXME
		 *
		 * status = queue_request_irq(test_ctx, nvmeq,
		 * "test_driver queue");
		 */

		/* Create another queue with same/existing qid */
		status = create_cq(test_ctx, qid, vector, flags, nvmeq);
		if ((status & (NVME_SCT_MASK | NVME_SC_MASK)) != 
			(NVME_SCT_SPECIFIC_ERRS | NVME_SC_INVALID_QID))
			result = STATUS_NOK;
	} else {
		result = STATUS_INIT_FAILED;
		goto free_nvmeq;
	}

	/* Remove queue */
	delete_cq(test_ctx, qid);

free_nvmeq:
	dma_free_coherent(nvmeq->q_dmadev, CQ_SIZE(nvmeq->q_depth),
		(void *)nvmeq->cqes, nvmeq->cq_dma_addr);
	dma_free_coherent(nvmeq->q_dmadev, SQ_SIZE(nvmeq->q_depth),
		nvmeq->sq_cmds, nvmeq->sq_dma_addr);
	kfree(nvmeq);

	return result;
}

int test_cmd_delete_cq(struct test_ctx *test_ctx)
{
	int status;
	int result = STATUS_OK;
	int qid = 1;

	/* Remove queue */
	status = delete_cq(test_ctx, qid);
	if ((status & (NVME_SCT_MASK | NVME_SC_MASK)) != 
		(NVME_SCT_SPECIFIC_ERRS | NVME_SC_INVALID_QID))
		result = STATUS_NOK;
	return result;
}

int test_cmd_get_non_ext_feat(struct test_ctx *test_ctx)
{
	int status;
	int result = STATUS_OK;
	int feature_id;
	int val;

	/* Request undefined feature */
	feature_id = NVME_FEAT_SW_PROGRESS + 1;

	status = get_features(test_ctx, feature_id, &val);
	if ((status & (NVME_SCT_MASK | NVME_SC_MASK)) != 
		(NVME_SCT_GENERIC | NVME_SC_INVALID_FIELD))
		result = STATUS_NOK;

	return result;
}

int test_cmd_set_get_feats(struct test_ctx *test_ctx)
{
	int status;
	int result = STATUS_OK;
	int val;
	int failed = 0;

	/* NVME_FEAT_ARBITRATION == 0x01 */
	val = NVME_FEAT_ARBITRATION;
	status = check_feature(test_ctx, NVME_FEAT_ARBITRATION, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Arbitration\n");
		failed++;
	}

	/* NVME_FEAT_POWER_MGMT == 0x02 */
	val = NVME_FEAT_POWER_MGMT;
	status = check_feature(test_ctx, NVME_FEAT_POWER_MGMT, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Power Management\n");
		failed++;
	}

	/* NVME_FEAT_LBA_RANGE == 0x03 */
/*
	status = check_feature(test_ctx, NVME_FEAT_LBA_RANGE, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for LBA Range Type\n");
		failed++;
	}
*/

	/* NVME_FEAT_TEMP_THRESH == 0x04 */
	val = NVME_FEAT_TEMP_THRESH;
	status = check_feature(test_ctx, NVME_FEAT_TEMP_THRESH, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Temperature Threshold\n");
		failed++;
	}

	/* NVME_FEAT_ERR_RECOVERY == 0x05 */
	val = NVME_FEAT_ERR_RECOVERY;
	status = check_feature(test_ctx, NVME_FEAT_ERR_RECOVERY, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Error Recovery\n");
		failed++;
	}

	/* NVME_FEAT_NUM_QUEUES == 0x07 */
	val = NVME_FEAT_NUM_QUEUES;
	status = check_feature(test_ctx, NVME_FEAT_NUM_QUEUES, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Number of Queues\n");
		failed++;
	}

	/* NVME_FEAT_IRQ_COALESCE == 0x08 */
	val = NVME_FEAT_IRQ_COALESCE;
	status = check_feature(test_ctx, NVME_FEAT_IRQ_COALESCE, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Interrupt Coalescing\n");
		failed++;
	}

	/* NVME_FEAT_IRQ_CONFIG == 0x09 */
	val = NVME_FEAT_IRQ_CONFIG;
	status = check_feature(test_ctx, NVME_FEAT_IRQ_CONFIG, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Interrupt Vector "
			"Configuration\n");
		failed++;
	}

	/* NVME_FEAT_WRITE_ATOMIC == 0x0a */
	val = NVME_FEAT_WRITE_ATOMIC;
	status = check_feature(test_ctx, NVME_FEAT_WRITE_ATOMIC, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Write Atomicity\n");
		failed++;
	}

	/* NVME_FEAT_ASYNC_EVENT == 0x0b */
	val = NVME_FEAT_ASYNC_EVENT;
	status = check_feature(test_ctx, NVME_FEAT_ASYNC_EVENT, val);
	if (status != STATUS_OK) {
		LOG_ERROR("[Nvme_Drv]Set/Get Feature FAILED for Asynchronous Event "
			"Configuration\n");
		failed++;
	}

	if (failed)
		result = STATUS_NOK;

	return result;
}

int test_cmd_create_too_many_cqs(struct test_ctx *test_ctx)
{
	struct nvme_queue **nvmeq = NULL;
	struct nvme_queue *tmpq;
	int status;
	int result = STATUS_OK;
	int val;
	int supp_queue_nbr;
	int i;
	int creat_queue_nbr = 0;
	int alloc_queue_nbr = 0;
	int flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;

	/* NVME_FEAT_NUM_QUEUES == 0x07 */
	status = get_features(test_ctx, NVME_FEAT_NUM_QUEUES, &val);
	if (status)
		return STATUS_INIT_FAILED;
	
	supp_queue_nbr = (val >> 16) + 1;
	LOG_DEBUG("[Nvme_Drv]Number of supported CQ I/O is %d\n", supp_queue_nbr);

	/* Subtract ADMIN queue */
	supp_queue_nbr--;

	nvmeq = kzalloc((supp_queue_nbr + 1) * sizeof(*nvmeq), GFP_KERNEL);
	if (!nvmeq)
		return STATUS_INIT_FAILED;

	for (i = 0; i < supp_queue_nbr; i++) {
		nvmeq[i] = alloc_queue(test_ctx, i + 1, NVME_Q_DEPTH,
			i + 1);
		if (!nvmeq[i]) {
			result = STATUS_INIT_FAILED;
			goto free_nvmeq;
		}
		alloc_queue_nbr++;

		status = create_cq(test_ctx, i + 1, i + 1, flags,
			nvmeq[i]);
		if (!status) {
			/*
			 * FIXME
			 * status = queue_request_irq(test_ctx,
			 * nvmeq[i], "test_driver queue");
			 */

			/* Queue has been created */
			creat_queue_nbr++;
		}
		else {
			result = STATUS_INIT_FAILED;
			goto free_nvmeq;
		}
	}

	LOG_DEBUG("[Nvme_Drv]gonna attempt create  CQ I/O # %d id # %d\n", i, i + 1);

	/* Create queue above limit */
	nvmeq[i] = alloc_queue(test_ctx, i + 1, NVME_Q_DEPTH, i + 1);
	if (!nvmeq[i]) {
		result = STATUS_INIT_FAILED;
		goto free_nvmeq;
	}
	alloc_queue_nbr++;

	status = create_cq(test_ctx, i + 1, i + 1, flags, nvmeq[i]);

	if (!status) {
		/* Queue has been created . . . strange ????? */
		creat_queue_nbr++;
		LOG_DEBUG("[Nvme_Drv]More Queues created than should be supported\n");
		result = STATUS_NOK;
	}
	else if ((status & (NVME_SCT_MASK | NVME_SC_MASK)) !=
		(NVME_SCT_SPECIFIC_ERRS | NVME_SC_INVALID_QID)) {
		LOG_DEBUG("[Nvme_Drv]Queue Creation failure with unexpected status %x\n",
			status);
		result = STATUS_NOK;
	}

free_nvmeq:
	for (i = 0; i < alloc_queue_nbr; i++) {
		tmpq = nvmeq[i];
		if (i < creat_queue_nbr)
			delete_cq(test_ctx, i + 1);
		else {
			LOG_DEBUG("[Nvme_Drv]Queue # %d was allocated but not created\n", i);
		}
		dma_free_coherent(tmpq->q_dmadev, CQ_SIZE(tmpq->q_depth),
			(void *)nvmeq[i]->cqes, nvmeq[i]->cq_dma_addr);
		dma_free_coherent(tmpq->q_dmadev, SQ_SIZE(tmpq->q_depth),
			tmpq->sq_cmds, tmpq->sq_dma_addr);
		kfree(tmpq);
	}
	kfree(nvmeq);

	return result;
}

int test_cmd_set(struct test_ctx *test_ctx)
{
	int result = 0;
	int do_irq = 0;
	int cmd_entries;

	cmd_entries = sizeof(test_functions)/sizeof(struct test_func);

	/* TEST_ALL_CMDS must be handled in app */
	if ((test_ctx->test_num >= cmd_entries ) || 
		(test_ctx->test_num == TEST_ALL_CMDS))
		return STATUS_TEST_NOT_DEFINED;

	result = set_test_context(test_ctx);
	if (result)
		return STATUS_INIT_FAILED;

	if (do_irq) {
		if (queue_request_irq(test_ctx, test_ctx->queues[0], "test_driver admin"))
			return STATUS_INIT_FAILED;
		/* TO DO: enable interrupts */
	}
	else {
		compl_thread = kthread_run(compl_kthread, test_ctx, "compl_kthread");
		if (IS_ERR(compl_thread))
			return STATUS_INIT_FAILED;
	}

	LOG_DEBUG("[Nvme_Drv]Performing %s validation\n",
		test_functions[test_ctx->test_num].name);
	
	result = test_functions[test_ctx->test_num].funcptr(test_ctx);

	if (!do_irq)
		kthread_stop(compl_thread);
	else {
		/* TO DO: disable interrupts */
	}

	clear_test_context(test_ctx);

	return result;
}

int test_cmd_abort(struct test_ctx *test_ctx)
{
	int result;
	int status;
	void *id;
	dma_addr_t dma_addr;
	int cmdid;
	int qid = 1;
	int vector = 1;
	int cq_flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;
	int sq_flags = NVME_QUEUE_PHYS_CONTIG | NVME_SQ_PRIO_MEDIUM;
	struct nvme_command c;
	struct nvme_id_ctrl *id_ctrl;
	struct nvme_queue *nvmeq = NULL;

	id = dma_alloc_coherent(&test_ctx->test_dev->dev, PAGE_SIZE * 2,
		&dma_addr, GFP_KERNEL);

	memset(&c, 0, sizeof(c));
	c.identify.opcode = nvme_admin_identify;
	c.identify.nsid = 0;
	c.identify.prp1 = cpu_to_le64(dma_addr);
	c.identify.cns = cpu_to_le32(1);

	LOG_NORMAL("[Nvme_Drv]Send identify commands to get acl value\n");
	status = submit_admin_cmd(test_ctx, &c, sync_id, NULL);
	if (!status) {
		id_ctrl = (struct nvme_id_ctrl *)id;
		LOG_NORMAL("[Nvme_Drv]Abort Command Limit = %d\n", id_ctrl->acl);
		result = STATUS_OK;
	} else {
	       LOG_NORMAL("[Nvme_Drv]Identify command failed\n");
	       result = STATUS_INIT_FAILED;
	       goto clean_and_exit;
	       return result;
	}

	nvmeq = alloc_queue(test_ctx, qid, NVME_Q_DEPTH, vector);
	if (!nvmeq) {
		result = STATUS_INIT_FAILED;
		goto clean_queues_and_exit;
	}

	LOG_NORMAL("[Nvme_Drv]Send Completion queue command\n");
	status = create_cq(test_ctx, qid, vector, cq_flags, nvmeq);
	if (status == 0) {
		result = STATUS_OK;
		LOG_NORMAL("[Nvme_Drv]CQ creation SUCCESS\n");
	} else {
	        LOG_NORMAL("[Nvme_Drv]CQ creation FAILED\n");
		result = STATUS_INIT_FAILED;
		goto clean_cq_and_exit;
	}

	LOG_NORMAL("[Nvme_Drv]Send create SQ command\n");
	status = create_sq(test_ctx, qid, sq_flags, nvmeq);
	if (status == 0) {
		result = STATUS_OK;
		LOG_NORMAL("[Nvme_Drv]SQ creation SUCCESS\n");
	} else {
	        LOG_NORMAL("[Nvme_Drv]SQ creation FAILED\n");
		result = STATUS_INIT_FAILED;
		goto clean_sq_and_exit;
	}

	/*
	 * TODO: figure out the intention of this test . . . 
	 * the  spec leads me to believe I can submit upto 
	 * id_ctrl->acl abort commands but not id_ctrl->acl+1
	 * qemu returns id_ctrl->acl == 10
	 * then we submit with cmdid = 5 and expect a failure?????
	 * seems more reasonable to me that we should get back 
	 * NVME_SC_CMD_REQ_NOT_FOUND, unles by odd chance there is
	* an adm cmd with cmdid 5 outstanding
	*/
	qid = 1;
	cmdid = 5;
	LOG_NORMAL("[Nvme_Drv]Submit Abort command with wrong command id\n");
	memset(&c, 0, sizeof(c));
	c.abort.opcode = nvme_admin_abort_cmd;
	c.abort.sqid = cpu_to_le16(qid);
	c.abort.cmdid = cpu_to_le16(cmdid);

	status = submit_admin_cmd(test_ctx, &c, sync_id, NULL);
	if ((status & (NVME_SCT_MASK | NVME_SC_MASK)) ==
		(NVME_SCT_SPECIFIC_ERRS | NVME_SC_CMD_LIMIT_EXCEEDED)) {
		     LOG_NORMAL("[Nvme_Drv]Abort Command Failed with status:%x\n",
		     status);
		result = STATUS_OK;
	} else {
	       LOG_NORMAL("[Nvme_Drv]Abort command returned unexpected status %x\n",
			status);
	       result = STATUS_NOK;
	       goto clean_all_and_exit;
	       return result;
	}

	/*
	 * TODO: figure out the intention of this test . . . 
	 * if i submit an abort commmand with invalid sq id
	 * it is more reasonable we we should get back 
	 * NVME_SC_INVALID_FIELD rather than NVME_SC_CMD_REQ_NOT_FOUND
	 */
	cmdid = 1;
	qid = 10;
	LOG_NORMAL("[Nvme_Drv]Submit Abort command with wrong/invalid SQ ID id\n");
	memset(&c, 0, sizeof(c));
	c.abort.opcode = nvme_admin_abort_cmd;
	c.abort.sqid = cpu_to_le16(qid);
	c.abort.cmdid = cpu_to_le16(cmdid);

	status = submit_admin_cmd(test_ctx, &c, sync_id, NULL);
	if ((status & (NVME_SCT_MASK | NVME_SC_MASK)) ==
		(NVME_SCT_SPECIFIC_ERRS | NVME_SC_CMD_REQ_NOT_FOUND)) {
		     LOG_NORMAL("[Nvme_Drv]Abort Command failed with status:%x\n",
		     status);
		result = STATUS_OK;
	} else {
	       LOG_NORMAL("[Nvme_Drv]Abort command returned unexpected status %x\n",
			status);
	       result = STATUS_NOK;
	       goto clean_all_and_exit;
	       return result;
	}

clean_all_and_exit:
clean_sq_and_exit:
	delete_sq(test_ctx, 1);
clean_cq_and_exit:
	delete_cq(test_ctx, 1);
clean_queues_and_exit:
	dma_free_coherent(nvmeq->q_dmadev, SQ_SIZE(nvmeq->q_depth),
					nvmeq->sq_cmds, nvmeq->sq_dma_addr);
	dma_free_coherent(nvmeq->q_dmadev, CQ_SIZE(nvmeq->q_depth),
				(void *)nvmeq->cqes, nvmeq->cq_dma_addr);
clean_and_exit:
	dma_free_coherent(&test_ctx->test_dev->dev, PAGE_SIZE, id, dma_addr);
	kfree(nvmeq);

	return result;
}
