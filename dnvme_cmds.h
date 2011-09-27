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
/* Strucutre for PRP */
/* TODO: Modify strucutre while implementing complete IOCTL */
struct nvme_prps {
    __u32 npages;
    __u32 type; /* refers to prp_type */
    __le64 **v_list; /* Virtual List of DMA Pools */
    __le64 prp1;
    __le64 prp2;
    dma_addr_t first_dma; /* First entry in PRP List */
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
int submit_command(struct nvme_dev_entry *nvme_dev, __u16 q_id,
    __u8 *buf_addr, __u32 buf_len);

/**
 * destroy_dma_pool:
 * Destroy's the dma pool
 * @param nvme_dev
 * @return void
 */
void destroy_dma_pool(struct nvme_dev_entry *nvme_dev);

#endif
