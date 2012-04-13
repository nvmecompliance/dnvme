/*
 * NVM Express Compliance Suite
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _DNVME_INTERFACE_H_
#define _DNVME_INTERFACE_H_

/**
 * API version coordinates the tnvme binary to the dnvme binary. If the dnvme
 * interface changes at all, then this file will be modified and thus this
 * revision will be bumped up. Only when this file changes does this version
 * change. The dnvme driver version is registered by the contents of file
 * version.h. Although version.h will change whenever API_VERSION changes, the
 * API_VERSION won't necessarily change each time version.h changes. Rather
 * version.h changes whenever a new release of the driver logic has changed.
 *
 * Thus when this API changes, then tnvme will have to be recompiled against
 * this file to adhere to the new modification and requirements of the API.
 * tnvme refuses to execute when it detects a API version mismatch to dnvme.
 */
#define    API_VERSION          0x00010300          /* 1.3.0 */


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
    INT_MSI_SINGLE,
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
    ST_ENABLE,              /* Set the NVME Controller to enable state */
    ST_DISABLE,             /* Controller reset without affecting Admin Q */
    ST_DISABLE_COMPLETELY   /* Completely destroy even Admin Q's */
};

/**
 * enum providing the definitions of the NVME commands.
 */
enum nvme_cmds {
    CMD_ADMIN,   /* Admin Command Set */
    CMD_NVM,     /* NVM Command Set */
    CMD_AON,     /* AON  Command Set */
    CMD_FENCE,   /* last element for loop over-run */
};

/* Enum specifying bitmask passed on to IOCTL_SEND_64B */
enum send_64b_bitmask {
    MASK_PRP1_PAGE = 1, /* PRP1 can point to a physical page */
    MASK_PRP1_LIST = 2, /* PRP1 can point to a PRP list */
    MASK_PRP2_PAGE = 4, /* PRP2 can point to a physical page */
    MASK_PRP2_LIST = 8, /* PRP2 can point to a PRP list */
    MASK_MPTR = 16,     /* MPTR may be modified */
};

/**
 * This struct is the basic structure which has important parameter for
 * sending 64 Bytes command to both admin  and IO SQ's and CQ's
 */
struct nvme_64b_send {
    /* BIT MASK for PRP1,PRP2 and metadata pointer */
    enum send_64b_bitmask bit_mask;
    /* Data buffer or discontiguous CQ/SQ's user space address */
    uint8_t const *data_buf_ptr;
    /* 0=none; 1=to_device, 2=from_device, 3=bidirectional, others illegal */
    uint8_t data_dir;

    uint8_t *      cmd_buf_ptr;   /* Virtual Address pointer to 64B command */
    enum nvme_cmds cmd_set;       /* Command set for the cmd_buf command */
    uint32_t       meta_buf_id;   /* Meta buffer ID when MASK_MPTR is set */
    uint32_t       data_buf_size; /* Size of Data Buffer */
    uint16_t q_id;      /* Queue ID where the cmd_buf command should go */
};

/**
 * This structure defines the overall interrupt scheme used and
 * defined parameters to specify the driver version and application
 * version. A verification is performed by driver and application to
 * check if these versions match.
 */
struct metrics_driver {
    uint32_t driver_version;  /* dnvme driver version */
    uint32_t api_version;     /* tnvme test application version */
};

/**
 * This structure defines the parameters required for creating any CQ.
 * It supports both Admin CQ and IO CQ.
 */
struct nvme_gen_cq {
    uint16_t q_id;            /* even admin q's are supported here q_id = 0 */
    uint16_t tail_ptr;        /* The value calculated for respective tail_ptr */
    uint16_t head_ptr;        /* Actual value in CQxTDBL for this q_id */
    uint32_t elements;        /* pass the actual elements in this q */
    uint8_t  irq_enabled;     /* sets when the irq scheme is active */
    uint16_t irq_no;          /* idx in list; always 0 based */
    uint8_t  pbit_new_entry;  /* Indicates if a new entry is in CQ */
};

/**
 * This structure defines the parameters required for creating any SQ.
 * It supports both Admin SQ and IO SQ.
 */
struct nvme_gen_sq {
    uint16_t sq_id;         /* Admin SQ are supported with q_id = 0 */
    uint16_t cq_id;         /* The CQ ID to which this SQ is associated */
    uint16_t tail_ptr;      /* Actual value in SQxTDBL for this SQ id */
    uint16_t tail_ptr_virt; /* future SQxTDBL write value based on no.
        of new cmds copied to SQ */
    uint16_t head_ptr;      /* Calculate this value based on cmds reaped */
    uint32_t elements;      /* total number of elements in this Q */
};

/**
 * enum for metrics type. These enums are used when returning the device
 * metrics.
 */
enum metrics_type {
    METRICS_CQ,     /* Completion Q Metrics */
    METRICS_SQ,     /* Submission Q Metrics */
    MTERICS_FENCE,  /* Always last item */
};

/**
 * Interface structure for returning the Q metrics. The buffer is where the
 * data is stored for the user to copy from. This assumes that the user will
 * provide correct buffer space to store the required metrics.
 */
struct nvme_get_q_metrics {
    uint16_t          q_id;     /* Pass the Q id for which metrics is desired */
    enum metrics_type type;     /* SQ or CQ metrics desired */
    uint32_t          nBytes;   /* Number of bytes to copy into buffer */
    uint8_t *         buffer;   /* to store the required data */
};

/**
 * Interface structure for creating Admin Q's. The elements is a 1 based value.
 */
struct nvme_create_admn_q {
    enum nvme_q_type type;      /* Admin q type, ASQ or ACQ */
    uint32_t         elements;  /* No. of elements of size 64 B */
};

/**
 * Interface structure for allocating SQ memory. The elements are 1 based
 * values and the CC.IOSQES is 2^n based.
 */
struct nvme_prep_sq {
    uint32_t elements;   /* Total number of entries that need kernel mem */
    uint16_t sq_id;      /* The user specified unique SQ ID  */
    uint16_t cq_id;      /* Existing or non-existing CQ ID */
    uint8_t  contig;     /* Indicates if SQ is contig or not, 1 = contig */
};

/**
 * Interface structure for allocating CQ memory. The elements are 1 based
 * values and the CC.IOSQES is 2^n based.
 */
struct nvme_prep_cq {
    uint32_t    elements;   /* Total number of entries that need kernal mem */
    uint16_t    cq_id;      /* Existing or non-existing CQ ID. */
    uint8_t     contig;     /* Indicates if SQ is contig or not, 1 = contig */
};

/**
 * Interface structure for getting the metrics structure into a user file.
 * The filename and location are specified thought file_name parameter.
 */
struct nvme_file {
    uint16_t    flen;       /* Length of file name, it is not the total bytes */
    const char *file_name;  /* location and file name to copy metrics */
};

/**
 * Interface structure for reap inquiry ioctl. It works well for both admin
 * and IO Q's.
 */
struct nvme_reap_inquiry {
    uint16_t q_id;           /* CQ ID to reap commands for */
    uint32_t num_remaining;  /* return no of cmds waiting to be reaped */

    /* no of times isr was fired which is associated with cq reaped on */
    uint32_t isr_count;
};

/**
 * Interface structure for reap ioctl. Admin Q and all IO Q's are supported.
 */
struct nvme_reap {
    uint16_t q_id;          /* CQ ID to reap commands for */
    uint32_t elements;      /* Get the no. of elements to be reaped */
    uint32_t num_remaining; /* return no. of cmds waiting for this cq */
    uint32_t num_reaped;    /* Return no. of elements reaped */
    uint8_t  *buffer;       /* Buffer to copy reaped data */
    /* no of times isr was fired which is associated with cq reaped on */
    uint32_t isr_count;
    uint32_t size;          /* Size of buffer to fill data to */
};

/**
 * Format of general purpose nvme command DW0-DW9
 */
struct nvme_gen_cmd {
    uint8_t  opcode;
    uint8_t  flags;
    uint16_t command_id;
    uint32_t nsid;
    uint64_t rsvd2;
    uint64_t metadata;
    uint64_t prp1;
    uint64_t prp2;
};

/**
 * Specific structure for Create CQ command
 */
struct nvme_create_cq {
    uint8_t  opcode;
    uint8_t  flags;
    uint16_t command_id;
    uint32_t rsvd1[5];
    uint64_t prp1;
    uint64_t rsvd8;
    uint16_t cqid;
    uint16_t qsize;
    uint16_t cq_flags;
    uint16_t irq_no;
    uint32_t rsvd12[4];
};

/**
 * Specific structure for Create SQ command
 */
struct nvme_create_sq {
    uint8_t  opcode;
    uint8_t  flags;
    uint16_t command_id;
    uint32_t rsvd1[5];
    uint64_t prp1;
    uint64_t rsvd8;
    uint16_t sqid;
    uint16_t qsize;
    uint16_t sq_flags;
    uint16_t cqid;
    uint32_t rsvd12[4];
};

/**
 * Specific structure for Delete Q command
 */
struct nvme_del_q {
    uint8_t  opcode;
    uint8_t  flags;
    uint16_t command_id;
    uint32_t rsvd1[9];
    uint16_t qid;
    uint16_t rsvd10;
    uint32_t rsvd11[5];
};

/**
 * Interface structure for setting the desired IRQ type.
 * works for all type of interrupt scheme expect PIN based.
 */
struct interrupts {
    uint16_t           num_irqs;        /* total no. of irqs req by tnvme */
    enum nvme_irq_type irq_type;        /* Active IRQ scheme for this dev */
};

/**
 * Public interface for the nvme device parameters. These parameters are
 * copied to user on request through an IOCTL interface GET_DEVICE_METRICS.
 */
struct public_metrics_dev {
    struct interrupts irq_active;  /* Active IRQ state of the nvme device */
};

/**
 * Describes bits/bytes within an existing SQ indicating a new value for any
 * cmd dword. This is only allowed for those cmds for which the doorbell hasn't
 * already rung.
 */
struct backdoor_inject {
    uint16_t q_id;        /* SQ ID where the cmd is residing */
    uint16_t cmd_ptr;     /* [0 -> (CreateIOSQ.DW10.SIZE-1)] which cmd in SQ? */
    uint8_t  dword;       /* [0 -> (CC.IOSQES-1)] which DWORD in the cmd */
    uint32_t value_mask;  /* Bitmask indicates which 'value' bits to use */
    uint32_t value;       /* Extract spec'd bits; overwrite those exact bits */
};

#endif
