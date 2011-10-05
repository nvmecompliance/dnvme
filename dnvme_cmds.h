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
int submit_command(struct nvme_device *nvme_dev, __u16 q_id,
    __u8 *buf_addr, __u32 buf_len);

/**
 * destroy_dma_pool:
 * Destroy's the dma pool
 * @param nvme_dev
 * @return void
 */
void destroy_dma_pool(struct nvme_device *nvme_dev);

#if 0
void free_nvme_prps(struct nvme_device *pnvme_device, __u8 write,
    unsigned long buf_addr, __u32 buf_len, struct scatterlist *sg,
    struct nvme_prps prps);
#endif
#endif
