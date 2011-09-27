#ifndef _DNVME_REG_H_
#define _DNVME_REG_H_

#include "dnvme_interface.h"

/**
* nvme_ctrl_reg defines the register space for the
* nvme controller registers as defined in NVME Spec 1.0b.
*/
struct nvme_ctrl_reg {
    __u64    cap;    /* Controller Capabilities */
    __u32    vs;     /* Version */
    __u32    intms;  /* Interrupt Mask Set */
    __u32    intmc;  /* Interrupt Mask Clear */
    __u32    cc;     /* Controller Configuration */
    __u32    rsvd1;  /* Reserved */
    __u32    csts;   /* Controller Status */
    __u32    rsvd2;  /* Reserved */
    __u32    aqa;    /* Admin Queue Attributes */
    __u64    asq;    /* Admin SQ Base Address */
    __u64    acq;    /* Admin CQ Base Address */
};

/**
* nvme_dev_entry shows a device entry in the global
* linked list which has one pci_dev and its corresponding
* bar mapped value.
*/
struct nvme_dev_entry {
    struct pci_dev *pdev; /* Pointer to pci dev                      */
    u8 *bar0mapped;       /* Bar 0 IO remapped value                 */
    struct dma_pool *prp_page_pool; /* Mem for PRP List */
    struct nvme_ctrl_reg __iomem *nvme_ctrl_space;
                /* Pointer to NVME controller space      */
    u8 init_flag;         /* indicates initiazation for this struct  */
    u32 __iomem *dbs;     /* Door Bell stride                        */
    u32 ctrl_config;      /* Controller Configuration Value 32 bit   */
};

/**
* nvme_space specifies the bar remapped value.
*/
struct nvme_space {
    struct nvme_ctrl_reg __iomem *bar_dev;
};

/**
* read_nvme_reg_generic function is a generic function which
* reads data from the controller registers of the nvme with
* user specified offset and bytes. Copies data back to udata
* pointer which points to user space buffer.
*/
int read_nvme_reg_generic(
    struct nvme_space nvme_ctrl_reg_space,
    u8 *udata,
    int nbytes,
    int offset,
    enum nvme_acc_type acc_type
);


/**
* write_nvme_reg_generic function is a generic function which
* writes data to the controller registers of the nvme with
* user specified offset and bytes.
*/
int write_nvme_reg_generic(
    struct nvme_space nvme_ctrl_reg_space,
    u8 *u8data,
    int nbytes,
    int offset,
    enum nvme_acc_type acc_type
);

#endif
