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
   NVMEIO_FENCE	/* always must be the last element */
};

/**
* This struct is the basic structure which has important
* parameter for the generic read  and write funtion to seek the correct
* offset and length while reading or writing to nvme card.
*/
struct rw_generic {
   enum nvme_io_space type;
   unsigned int  offset;
   unsigned int  nBytes;
   unsigned char *buffer;
};

/**
* These enums are used for creating Admin Completion Q types based on reqeusted
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
* This Struct is used for setting the contrller either
*/
struct nvme_ctrl_enum {
   enum nvme_en_dis nvme_status;
};

#endif
