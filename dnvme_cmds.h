#ifndef _DNVME_CMDS_H_
#define _DNVME_CMDS_H_


/* Enum specifying Writes/Reads to device and pages */
enum {
    READ_DEV = 0,
    READ_PG = 0,
    WRITE_DEV = 1,
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

/* Enum specifying type of data buffer */
enum data_buf_type {
    DISCONTG_IO_Q = 0 ,
    DATA_BUF = 1,
};

/**
* submit_command :
* Entry point for Submitting 64Bytes Command which handles
* mapping user pages to memory, creating SG Lists and
* creating PRP Lists
* @param nvme_dev
* @param q_id
* @param buf_addr
* @param buf_len
* @return Error codes
* TODO:
* Add the implementation logic of complete ioctl and update the
* function arguments accordingly
*/
int submit_command(struct  metrics_device_list *pmetrics_device, __u16 q_id,
    __u8 *buf_addr, __u32 buf_len);

/**
 * destroy_dma_pool:
 * Destroy's the dma pool
 * @param nvme_dev
 * @return void
 */
void destroy_dma_pool(struct nvme_device *nvme_dev);

/**
 * empty_cmd_track_list:
 * Delete command track list completley per SQ
 * @param pmetrics_device
 * @param q_id
 * @return void
 */
void empty_cmd_track_list(struct  metrics_device_list *pmetrics_device,
    __u16 q_id);

/**
 * free_prp_pool:
 * Frees the PRP pool for a SQ or CQ node for this device.
 * @param dev
 * @param prps
 * @return npages
 */
void free_prp_pool(struct nvme_device *dev,
    struct nvme_prps *prps, __u32 npages);

/**
 * unmap_user_pg_to_dma:
 * Unmaps the dma memory for this device with given prps.
 * @param dev
 * @param prps
 * @return void
 */
void unmap_user_pg_to_dma(struct nvme_device *dev,
    struct nvme_prps *prps);

#endif
