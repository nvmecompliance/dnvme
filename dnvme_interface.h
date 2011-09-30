#ifndef _DNVME_INTERFACE_H_
#define _DNVME_INTERFACE_H_

/**
* These are the enum types used for branching to
* required offset as specified by either PCI space
* or a NVME space enum value defined here.
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
 * These enums define the type of interrupt scheme that the overall
 * system uses.
 */
enum nvme_irq_type {
    INT_PIN,
    INT_MSI,
    INT_SINGLE,
    INT_MSI_MULTI,
    INT_MSIX,
    INT_NONE,
    INT_FENCE    /* Last item to guard from loop run-overs */
};

/**
 * enums to define the q types.
 */
enum nvme_q_type {
    ADMIN_SQ,
    ADMIN_CQ,
};
/**
* This struct is the basic structure which has important
* parameter for the generic read  and write function to seek the correct
* offset and length while reading or writing to nvme card.
*/
struct rw_generic {
    enum nvme_io_space type;
    uint32_t  offset;
    uint32_t  nBytes;
    enum nvme_acc_type acc_type;
    uint8_t *buffer;
};

/**
* These enums are used while enabling or disabling or completely disabling the
* controller.
*/
enum nvme_state {
    ST_ENABLE,              /* Set the NVME Controller to enable state      */
    ST_DISABLE,             /* Controller reset without affecting Admin Q   */
    ST_DISABLE_COMPLETELY,  /* Completely destroy even Admin Q's            */
};

/**
* The parametes in this structue is used for setting the controller a new
* state.
*/
struct nvme_ctrl_state {
    enum nvme_state new_state; /* New state of the controller requested. */
};

/**
 * enum providing the definitions of the NVME commands.
 */
enum nvme_cmds {
    CMD_ADMIN, /* Admin Command Set */
    CMD_NVME, /* NVME Command Set */
    CMD_AON, /* AON  Command Set */
    CMD_FENCE, /* last element for loop over-run */
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
    u_int8_t *data_buf_ptr;
    u_int8_t *meta_buf_ptr; /* User space addr of Metabuffer else NULL */
    u_int8_t *cmd_buf_ptr; /* Virtual Address pointer to 64B command */
    enum nvme_cmds cmd_set; /* Command set for the cmd_buf command */
    u_int16_t meta_buf_size; /* Size of Meta Buffer */
};

/**
 * This structure defines the overall interrupt scheme used and
 * defined parameters to specify the driver version and application
 * version. A verification is performed by driver and application to
 * check if these versions match.
 */
struct metrics_driver {
    enum        nvme_irq_type    irq;   /* Defines the IRQ type used         */
    uint16_t    driver_version;         /* dnvme driver version              */
    uint16_t    api_version;            /* tnvme test application version    */
};

/**
 * This structure defines the parameters required for creating any CQ.
 * It supports both Admin CQ and IO CQ.
 */
struct nvme_gen_cq {
    uint16_t    q_id;        /* even admin q's are supported here q_id = 0   */
    uint16_t    tail_ptr;    /* The value calculated for respective tail_ptr */
    uint16_t    head_ptr;    /* Actual value in CQxTDBL for this q_id        */
    uint16_t    elements;    /* pass the actual elements in this q           */
};

/**
 * This structure defines the parameters required for creating any SQ.
 * It supports both Admin SQ and IO SQ.
 */
struct nvme_gen_sq {
    uint16_t    sq_id;    /* Admin SQ are supported with q_id = 0            */
    uint16_t    cq_id;    /* The CQ ID to which this SQ is associated        */
    uint16_t    tail_ptr;    /* Acutal value in SQxTDBL for this SQ id       */
    uint16_t    tail_ptr_virt; /* future SQxTDBL write value based on no.
        of new cmds copied to SQ */
    uint16_t    head_ptr;    /* Calculate this value based on cmds reaped    */
    uint16_t    elements;    /* total number of elements in this Q           */
};

/**
 * enum for metrics type. These enums are used when returning the device
 * metrics.
 */
enum metrics_type {
    METRICS_CQ,     /* Completion Q Metrics     */
    METRICS_SQ,     /* Submission Q Metrics     */
    MTERICS_FENCE,  /* Always last item.        */
};

/**
  * Interface structure for returning the Q metrics. The buffer is where the
  * data is stored for the user to copy from. This assumes that the user will
  * provide correct buffer space to store the required metrics.
  * TODO: Add the buffer in the design doc.
  */
struct nvme_get_q_metrics {
    uint16_t    q_id;       /* Pass the Q id for which metrics is desired   */
    enum        metrics_type    type;   /* SQ or CQ metrics desired         */
    uint32_t    nBytes;     /* Number of bytes to copy into buffer          */
    uint8_t     *buffer;    /* to store the required data.                  */
};

/**
 * Interface structure for creating Admin Q's. The elements is a 1 based value.
 */
struct nvme_create_admn_q {
    enum        nvme_q_type     type;   /* Admin q type, ASQ or ACQ.    */
    uint16_t    elements;               /* No. of elements of size 64 B */
};

/*
 * Interface structure for allocating SQ memory. The elements are 1 based
 * values and the CC.IOSQES is 2^n based.
 */
struct nvme_prep_sq {
    uint16_t    elements;   /* Total number of entries that need kernal mem */
    uint16_t    sq_id;      /* The user specified unique SQ ID              */
    uint16_t    cq_id;      /* Existing or non-existing CQ ID.              */
    uint8_t     contig;     /* Indicates if SQ is contig or not, 1 = contig */
};

/*
 * Interface structure for allocating CQ memory. The elements are 1 based
 * values and the CC.IOSQES is 2^n based.
 */
struct nvme_prep_cq {
    uint16_t    elements;   /* Total number of entries that need kernal mem */
    uint16_t    cq_id;      /* Existing or non-existing CQ ID.              */
    uint8_t     contig;     /* Indicates if SQ is contig or not, 1 = contig */
};

/*
 * Interface structure for Ring Submission Q doorbell. The id passed is for SQ
 * to ring its doorbell.
 */
struct nvme_ring_sqxtdbl {
    uint16_t    sq_id;  /* The SQ ID of the SQ to ring doorbell */
};

struct nvme_file {
    uint16_t    flen; /* Length of file name, it is not the total bytes */
    uint8_t     *file_name; /* location and file name to copy metrics   */
};
#endif
