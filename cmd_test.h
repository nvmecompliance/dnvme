#ifndef _CMD_TEST_
#define _CMD_TEST_

#include <linux/interrupt.h>
#include "nvme.h"

struct nvme_cmd_info {
	unsigned long ctx;
	unsigned long timeout;
};

/*
 * An NVM Express queue.  Each device has at least two (one for admin
 * commands and one for I/O commands).
 *
 * the cmdid_data field below is composed of
 * q_depth num of nvme_cmd_info structs, followed by 
 * q_depth num of bits rounded up to some long size
 */
struct nvme_queue {
	struct device *q_dmadev;
	struct test_ctx *test;
	spinlock_t q_lock;
	struct nvme_command *sq_cmds;
	volatile struct nvme_completion *cqes;
	dma_addr_t sq_dma_addr;
	dma_addr_t cq_dma_addr;
	wait_queue_head_t sq_not_full;
	u32 __iomem *q_db;
	u16 q_depth;
	u16 cq_vector;
	u16 sq_head;
	u16 sq_tail;
	u16 cq_head;
	u16 cq_phase;
	struct nvme_cmd_info cmdid_data[];
};

struct sync_cmd_info {
	struct task_struct *task;
	u32 result;
	int status;
};

struct test_func {
	int (*funcptr) (struct test_ctx *test_ctx);
	char *name;
};

enum {
	sync_id = 0,
	async_id,
	bio_id,
};

#ifndef POISON_POINTER_DELTA
#define POISON_POINTER_DELTA	0
#endif

/* Special values must be a multiple of 4, and less than 0x1000 */
#define CMD_CTX_BASE		(POISON_POINTER_DELTA + sync_id)
#define CMD_CTX_CANCELLED	(0x30C + CMD_CTX_BASE)
#define CMD_CTX_COMPLETED	(0x310 + CMD_CTX_BASE)
#define CMD_CTX_INVALID		(0x314 + CMD_CTX_BASE)
#define CMD_CTX_FLUSH		(0x318 + CMD_CTX_BASE)

#define NVME_Q_DEPTH 1024
#define SQ_SIZE(depth)		(depth * sizeof(struct nvme_command))
#define CQ_SIZE(depth)		(depth * sizeof(struct nvme_completion))

#define ABORT_CMD_MIN_LIMIT	4
#define ASYNCH_EVENT_REQ_LIMIT	4
#define FW_MIN_SLOT_SUPPORTED	1
#define FW_MAX_SLOT_SUPPORTED	7
#define NPSS_MIN_VAL		0
#define NPSS_MAX_VAL		31
#define MIN_CQ_ENTRY_SIZE	16
#define MIN_SQ_ENTRY_SIZE	64

#define ADMIN_TIMEOUT	(60 * HZ)

extern int test_cmd_set(struct test_ctx *test_ctx);

extern struct nvme_queue *alloc_queue(struct test_ctx *test_ctx, int qid,
	int depth, int vector);

extern int submit_admin_cmd(struct test_ctx *test_ctx, struct nvme_command *cmd,
	int id, u32 *result);

extern int set_test_context(struct test_ctx *test_ctx);

extern int clear_test_context(struct test_ctx *test_ctx);

extern int compl_kthread(void *data);

extern void free_queues(struct test_ctx *test_ctx, int qid);

extern int queue_request_irq(struct test_ctx *test_ctx,
	struct nvme_queue *nvmeq, const char *name);

extern int create_cq(struct test_ctx *test_ctx, int qid, int vector, int flags,
	struct nvme_queue *nvmeq);

extern int create_sq(struct test_ctx *test_ctx, int qid, int flags,
	struct nvme_queue *nvmeq);

extern int delete_cq(struct test_ctx *test_ctx, int cqid);

extern int delete_sq(struct test_ctx *test_ctx, int sqid);

extern int get_features(struct test_ctx *test_ctx, int feature_id, int *val);

extern int set_features(struct test_ctx *test_ctx, int feature_id, int val);

extern int check_feature(struct test_ctx *test_ctx, int feature_id, int val);

extern int delete_queue(struct test_ctx *test_ctx, u8 opcode, u16 id);

#endif
