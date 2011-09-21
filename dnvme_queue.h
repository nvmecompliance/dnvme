#ifndef _DNVME_QUEUE_H_
#define _DNVME_QUEUE_H_

#include "dnvme_reg.h"
#include "dnvme_ds.h"

/* Admin SQ tail Door bell offset */
#define NVME_SQ0TBDL    0x1000

/* Admin SQ tail Door bell offset */
#define NVME_CQ0TBDL    0x1004

/* Admin SQ size Mask bits 0-11 in AQA */
#define ASQS_MASK       0xFFF

/* Admin Completion Q Mask Bits 16-21 in ADA */
#define ACQS_MASK       0x0FFF0000

/* As Time Out is in lower 32 bits of 64 bit CAP */
#define NVME_TO_MASK    0xFF000000

/*
* Each unit in TO is 500 ms, converting ms to jiffies
*/
#define NVME_MSEC_2_JIFFIES (500 * HZ / 1000)

/*
 * Maximum AQ entries allowed.
 */
#define MAX_AQ_ENTRIES   4096

/*
* This lines are commented to use inline functions.
* if required at multiple places uncomment this.
*/
#if 0
#ifdef QEMU
#define WRITEQ(a, b) { writel(a, b); writel(a >> 32, b + 4); }
#else
#define WRITEQ(a, b) { writeq(a, b); }
#endif
#endif
/*
* Enumerating the different NVME Controller Capabilities of the
* PCI Express device as per NVME Spec 1.0b.
*/
enum {
    NVME_CC_ENABLE        = 1 << 0,
    NVME_CC_CSS_NVM       = 0 << 4,
    NVME_CC_MPS_SHIFT     = 7,
    NVME_CC_ARB_RR        = 0 << 11,
    NVME_CC_ARB_WRRU      = 1 << 11,
    NVME_CC_ARB_VS        = 3 << 11,
    NVME_CC_SHN_NONE      = 0 << 13,
    NVME_CC_SHN_NORMAL    = 1 << 13,
    NVME_CC_SHN_ABRUPT    = 2 << 13,
    NVME_CSTS_RDY         = 1 << 0,
    NVME_CSTS_CFS         = 1 << 1,
    NVME_CSTS_SHST_NORMAL = 0 << 2,
    NVME_CSTS_SHST_OCCUR  = 1 << 2,
    NVME_CSTS_SHST_CMPLT  = 2 << 2,
};

/*
* Command structure with parameters for creating SQ.
*/
struct nvme_create_sq {
    __u16 comand_id;
};

/*
* Nvme Structure for different command sets.
*/
struct nvme_command {
    struct nvme_create_sq create_sq;
};

/*
* Structure definition for the NVME Express Queue. This is the main entry
* point for all the Q's that the NVME device functions with and manipulates.
*/
struct nvme_queue {
    struct    device   *dmadev;
    spinlock_t         q_lock;
    struct    nvme_command   *nvme_sq_cmds;
    void               *virt_asq_addr;
    void               *virt_acq_addr;
    dma_addr_t         asq_dma_addr;
    dma_addr_t         acq_dma_addr;
    u16                asq_depth;
    u16                acq_depth;
    u16                sq_head;
    u16                sq_tail;
    u8                 q_init;
};

/**
* The user selection of IOCTL for creating admin cq eventually calls
* this function if init is successful. This will create infrastructure
* for Admin Completion Q creation
* @param pnvme_dev
* @param qsize
* @param pmetrics_cq_list
* @return whether ACQ creation successful or not.
*/
int create_admn_cq(struct nvme_device *pnvme_dev, u16 qsize,
        struct  metrics_cq  *pmetrics_cq_list);

/**
* The user selection of IOCTL for creating admin sq eventually calls
* this function if init is successful. This will create infrastructure
* for Admin Submission Q creation
* @param pnvme_dev
* @param qsize
* @param pmetrics_sq_list
* @return whether ASQ creation successful or not.
*/
int create_admn_sq(struct nvme_device *pnvme_dev, u16 qsize,
        struct  metrics_sq  *pmetrics_sq_list);

/**
* This is the timer handler which will be invoked by the kernel when the timer
* expires in the timer.expires field. This function will set a flag which is
* used by the create admn sq routine to exit.
* @param arg
*/
void jit_timer_fn(unsigned long arg);

/**
* nvme_ctrl_enable - NVME controller enable function.This will set the CAP.EN
* flag and this function which call the timer handler and check for the timer
* expiration. It returns success if the ctrl in rdy before timeout.
* @param pnvme_dev
* @return SUCCESS or FAIL
*/
int nvme_ctrl_enable(struct nvme_device *pnvme_dev);

/**
* nvme_ctrl_disable - NVME controller disable function.This will reset the
* CAP.EN flag and this function which call the timer handler and check for
* the timer expiration. It returns success if the ctrl in rdy before timeout.
* @param pnvme_dev
* @return SUCCESS or FAIL
*/
int nvme_ctrl_disable(struct nvme_device *pnvme_dev);

/**
* identify_unique - verify if the q_id specified is unique. If not unique then
* return fail.
* @param q_id
* @param type
* @return SUCCESS or FAIL
*/
int identify_unique(u16 q_id, enum metrics_type type);

/**
* nvme_alloc_sq - NVME controller allocate sq function.This will check
* if q is allocated and then create a memory for the IO SQ.
* @param pmetrics_sq_list
* @param pnvme_dev
* @return SUCCESS or FAIL
*/
int nvme_alloc_sq(struct  metrics_sq  *pmetrics_sq_list,
            struct nvme_device *pnvme_dev);

#endif
