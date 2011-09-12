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
    uint32_t asq_size;
};

/**
* This struct is the basic structure which has important parameter for
* creating admin completion queue and gets the size from user including
* priority for Q creation.
*/
struct nvme_acq_gen {
    enum nvme_qop_type cq_type;
    uint32_t acq_size;
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
    uint8_t     irqEnabled;  /* pass if using IRQ's or not.                  */
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
    uint8_t     *buffer;    /* to store the required data.                  */
};

#endif
