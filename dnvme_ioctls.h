#ifndef _DNVME_IOCTLS_H_
#define _DNVME_IOCTLS_H_

/**
* Enumeration types which provide common interface between
* kernel driver and user app layer ioctl functions.
*/
enum {
    NVME_READ_GENERIC = 0, /** < enum to invoke read generic func call. */
    NVME_WRITE_GENERIC,    /** < enum to invoke write generic func call.*/
    NVME_ERR_CHK,          /** < enum Generic device status check func  */
    NVME_CREATE_ADMN_SQ,   /** < enum to invoke admin sq creation       */
    NVME_CREATE_ADMN_CQ,   /** < enum to invoke admin cq creation       */
    NVME_DEVICE_STATE,     /** < enum to enable and disable ctlr        */
    NVME_SEND_64B_CMD,     /** < enum Send 64B command.                 */
    NVME_GET_Q_METRICS,    /** < enum to get the q metrics              */
    NVME_CREATE_ADMN_Q,    /** < enum to invoke creation of admin q's   */
    NVME_PREPARE_SQ_CREATION, /** <enum Allocate SQ contig memory       */
    NVME_PREPARE_CQ_CREATION, /** <enum Allocate SQ contig memory       */
    NVME_RING_SQ_DOORBELL, /** <enum Ring SQ Tail doorbell              */
    NVME_DUMP_METRICS,     /** <enum Log data from Metrics structure    */
    NVME_REAP_INQUIRY,     /** <enum Invoke Reap inquiry                */
    UNIT_TESTS,            /* Generic IOCTL to set up UT scenarios      */
};

/**
* @def NVME_IOCTL_READ_GENERIC
* define a unique value using _IOWR call for Generic read capability
* the first parameter is the group to which this  IOCTL type belongs to,
* generally from (0-255) the second parameter is type within the group.
* the third parameter give the size of data and type of data that is passed
* to this ioctl from user level to kernel level.
*/
#define NVME_IOCTL_READ_GENERIC _IOWR('A', NVME_READ_GENERIC,\
        struct rw_generic)


/**
* @def NVME_IOCTL_WRITE_GENERIC
* define a unique value for Generic write capability the first parameter
* is the group to which this IOCTL type belongs to, generally from (0-255)
* the second parameter is type within the group.The third parameter give
* the size of data and type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_WRITE_GENERIC _IOWR('A', NVME_WRITE_GENERIC,\
        struct rw_generic)

/**
* @def NVME_IOCTL_ERR_CHK
* define unique ioctl for checking device error status. The first parameter
* is the group to which this IOCTL type belongs to, generally from (0-255)
* the second parameter is type within the group. The third parameter give the
* size of data and type of data that is passed to this ioctl from user
* level to kernel level.
*/
#define NVME_IOCTL_ERR_CHK _IOWR('A', NVME_ERR_CHK, int)

/**
* @def NVME_IOCTL_CREATE_ADMN_SQ
* define a unique value for creating admin submission queue. The 'A' value
* is the group to which this IOCTL type belongs to, generally from (0-255)
* the second parameter is type within the group defined in the enum. The
* third parameter give the size of data and type of data that is passed to
* this ioctl from user level to kernel level.
*/
#define NVME_IOCTL_CREATE_ADMN_SQ _IOWR('A', NVME_CREATE_ADMN_SQ,\
        struct nvme_asq_gen)

/**
* @def NVME_IOCTL_CREATE_ADMN_CQ
* define a unique value for creating admin submission queue. The 'A' value
* is the group to which this IOCTL type belongs to, generally from (0-255)
* the second parameter is type within the group defined in the enum. The
* third parameter give the size of data and type of data that is passed to
* this ioctl from user level to kernel level.
*/
#define NVME_IOCTL_CREATE_ADMN_CQ _IOWR('A', NVME_CREATE_ADMN_CQ,\
        struct nvme_acq_gen)

/**
* @def NVME_IOCTL_DEVICE_STATE
* define a unique value for resetting or enabling controller.  The 'A' value
* is the group to which this IOCTL type belongs to, generally from (0-255)
* the second parameter is type within the group defined in the enum. The
* third parameter give the size of data and type of data that is passed to
* this ioctl from user level to kernel level.
*/
#define NVME_IOCTL_DEVICE_STATE _IOWR('A', NVME_DEVICE_STATE,\
        struct nvme_ctrl_state)

/**
* @def NVME_IOCTL_GET_Q_METRICS
* define a unique value for getting the q metrics. The metrics is either for
* Submission queue including Admin SQ or Completion Queue including Admin CQ.
*/
#define NVME_IOCTL_GET_Q_METRICS _IOWR('A', NVME_GET_Q_METRICS,\
        struct nvme_get_q_metrics)

/**
* @def NVME_IOCTL_CREATE_ADMN_Q
* define a unique value for creating admin queues. The 'A' value
* is the group to which this IOCTL type belongs to, generally from (0-255)
* the second parameter is type within the group defined in the enum. The
* third parameter give the size of data and type of data that is passed to
* this ioctl from user level to kernel level.
*/
#define NVME_IOCTL_CREATE_ADMN_Q _IOWR('A', NVME_CREATE_ADMN_Q,\
        struct nvme_create_admn_q)

/**
* @def NVME_IOCTL_SEND_64B_CMD
* define a unique value for sending 64 Bytes command. The 'A' value
* is the group to which this IOCTL type belongs to, generally from (0-255)
* the second parameter is type within the group defined in the enum. The
* third parameter give the size of data and type of data that is passed to
* this ioctl from user level to kernel level.
*/
#define NVME_IOCTL_SEND_64B_CMD _IOWR('A', NVME_SEND_64B_CMD,\
        struct nvme_64b_send)

/**
* @def NVME_IOCTL_PREPARE_SQ_CREATION
* define a unique value for allocating contiguous memory for SQ. The 'A' value
* is the group to which this IOCTL type belongs to,generally from (0-255)the
* second parameter is type within the group defined in the enum. The third
* parameter give the size of data and type of data that is passed to this ioctl
* from user level to kernel level.
*/
#define NVME_IOCTL_PREPARE_SQ_CREATION _IOWR('A', NVME_PREPARE_SQ_CREATION,\
        struct nvme_prep_sq)

/**
* @def NVME_IOCTL_PREPARE_CQ_CREATION
* define a unique value for allocating contiguous memory for CQ. The 'A' value
* is the group to which this IOCTL type belongs to,generally from (0-255)the
* second parameter is type within the group defined in the enum. The third
* parameter give the size of data and type of data that is passed to this ioctl
* from user level to kernel level.
*/
#define NVME_IOCTL_PREPARE_CQ_CREATION _IOWR('A', NVME_PREPARE_CQ_CREATION,\
        struct nvme_prep_sq)


/**
* @def NVME_IOCTL_RING_SQ_DOORBELL
* define a unique value to ring SQ doorbell. The 'A' value
* is the group to which this IOCTL type belongs to,generally from (0-255)the
* second parameter is type within the group defined in the enum. The third
* parameter give the size of data and type of data that is passed to this ioctl
* from user level to kernel level.
*/
#define NVME_IOCTL_RING_SQ_DOORBELL _IOWR('A', NVME_RING_SQ_DOORBELL,\
        struct nvme_ring_sqxtdbl)

/**
* @def NVME_IOCTL_DUMP_METRICS
* define a unique value to Dump Q metrics. The 'A' value
* is the group to which this IOCTL type belongs to,generally from (0-255)the
* second parameter is type within the group defined in the enum. The third
* parameter give the size of data and type of data that is passed to this ioctl
* from user level to kernel level.
*/
#define NVME_IOCTL_DUMP_METRICS _IOWR('A', NVME_DUMP_METRICS, struct nvme_file)

/**
* @def NVME_IOCTL_REAP_INQUIRY
* define a unique value to reap inquiry ioctl. The 'A' value
* is the group to which this IOCTL type belongs to,generally from (0-255)the
* second parameter is type within the group defined in the enum. The third
* parameter give the size of data and type of data that is passed to this ioctl
* from user level to kernel level.
*/
#define NVME_IOCTL_REAP_INQUIRY _IOWR('A', NVME_REAP_INQUIRY,\
        struct nvme_reap_inquiry)

/**
 * @def IOCTL_UNIT_TESTS
 * To set up generic unit test scenarios.
 */
#define IOCTL_UNIT_TESTS _IOWR('A', UNIT_TESTS, uint16_t)

#endif
