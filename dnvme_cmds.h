#ifndef _DNVME_CMDS_H_
#define _DNVME_CMDS_H_


/* Enum specifying Writes/Reads to mapped pages */
enum {
    READ_PG = 0,
    WRITE_PG = 1,
};

/* Enum specifying PRP1,PRP2 or List */
enum prp_type {
    NO_PRP = 0,
    PRP1 = 1,
    PRP2 = 2,
    PRP_List = 4,
    PRP_Size = 8,
};

/* Enum specifying bitmask passed on to IOCTL_SEND_64B */
enum bit_mask_enum {
    MASK_PRP1 = 1,
    MASK_PRP2 = 2,
    MASK_MPTR = 4,
};

/* Enum specifying type of data buffer */
enum data_buf_type {
    DATA_BUF = 0,
    DISCONTG_IO_Q = 1,
};

/**
* data_buf_to_prp :
* Creates persist or non persist PRP's from data_buf_ptr memory
* and addes a node inside cmd track list pointed by pmetrics_sq
* @param nvme_dev
* @param pmetrics_sq
* @param data_buf_type
* @param nvme_64b_send
* @param prps
* @param opcode
* @param persist_q_id
* @return Error codes
*/
int data_buf_to_prp(struct nvme_device *nvme_dev,
    struct metrics_sq *pmetrics_sq, enum data_buf_type data_buf_type,
        struct nvme_64b_send *nvme_64b_send, struct nvme_prps *prps,
            __u8 opcode, __u16 persist_q_id);

/**
 * destroy_dma_pool:
 * Destroy's the dma pool
 * @param nvme_dev
 * @return void
 */
void destroy_dma_pool(struct nvme_device *nvme_dev);

/**
 * add_cmd_track_node:
 * Add node inside the cmd track list
 * @param pmetrics_sq
 * @param persist_q_id
 * @param prps
 * @param cmd_type
 * @param opcode
 * @return Error codes
 */
int add_cmd_track_node(struct  metrics_sq  *pmetrics_sq,
    __u16 persist_q_id, struct nvme_prps *prps, enum nvme_cmds cmd_type,
        __u8 opcode);
/**
 * empty_cmd_track_list:
 * Delete command track list completley per SQ
 * @param pnvme_device
 * @param pmetrics_sq
 * @return void
 */
void empty_cmd_track_list(struct  nvme_device *pnvme_device,
    struct  metrics_sq  *pmetrics_sq);

#endif
