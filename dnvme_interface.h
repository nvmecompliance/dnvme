#ifndef _DNVME_INTERFACE_H_
#define _DNVME_INTERFACE_H_

/**
* These are the enum types used for branching to
* required offset as specified in the struct nvme_
* read_generic type parameter.
*/
enum nvme_io_space {
    NVMEIO_PCI_HDR,
    NVMEIO_BAR01,
    NVMEIO_FENCE    /* always must be the last element */
};

/**
* These are the enum types used for specifying the
* required access width of registers or memory space.
*/
enum nvme_acc_type {
    BYTE_LEN,
    WORD_LEN,
    DWORD_LEN,
    QUAD_LEN,
    ACC_FENCE
};

/**
* This struct is the basic structure which has important
* parameter for the generic read  and write function to seek the correct
* offset and length while reading or writing to nvme card.
*/
struct rw_generic {
    enum nvme_io_space type;
    unsigned int  offset;
    unsigned int  nBytes;
    enum nvme_acc_type acc_type;
    unsigned char *buffer;
};

/**
* These enums are used for creating Admin Completion Q types based on requested
* user type.
*/
enum nvme_qop_type {
    NVME_Q_POLLING, /* Polling based Q */
    NVME_Q_IRQ,   /* Interrupt Based Q */
};
/**
* This struct is the basic structure which has important parameter for
* creating admin submission queue and gets the size from user.
*/
struct nvme_asq_gen {
    unsigned int asq_size;
};

/**
* This struct is the basic structure which has important parameter for
* creating admin completion queue and gets the size from user including
* priority for Q creation.
*/
struct nvme_acq_gen {
    enum nvme_qop_type cq_type;
    unsigned int acq_size;
};

/**
* This enums are used while enabling or disabling the controller.
*/
enum nvme_en_dis {
    NVME_CTLR_ENABLE, /* It does controller reset functionality */
    NVME_CTLR_DISABLE, /* It shuts down the controller*/
};

/**
* This Struct is used for setting the controller either
*/
struct nvme_ctrl_enum {
    enum nvme_en_dis nvme_status;
};

/**
* Enumeration for the Command Sets which are supported
*/
enum nvme_cmd_set {
    CMD_ADMIN, /* Admin Command Set */
    CMD_NVME, /* NVME Command Set */
    CMD_AON, /* AON  Command Set */
};

/**
* This struct is the basic structure which has important parameter for
* sending 64 Bytes command to both admin  and IO SQ's and CQ's
*/
struct nvme_64b_send {
    u_int16_t queue_id; /* Queue ID where the cmd_buf command should go */
    u_int16_t bit_mask; /* BIT MASK for PRP1,PRP2 and Metadata pointer */
    u_int32_t data_buf_size; /* Size of Data Buffer */
    /* Data Buffer or Discontiguous CQ/SQ's user space address */
    __u8 *data_buf_ptr;
    __u8 *meta_buf_ptr; /* User space addr of Metabuffer else NULL */
    __u8 *cmd_buf_ptr; /* Virtual Address pointer to 64B command */
    enum nvme_cmd_set cmd_set; /* Command set for the cmd_buf command */
    u_int16_t meta_buf_size; /* Size of Meta Buffer */
};

#endif
