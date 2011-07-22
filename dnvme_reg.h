#ifndef _DNVME_REG_H_
#define _DNVME_REG_H_

struct nvme_ctrl_reg {
	__u64	cap;    /* Controller Capabilities */
	__u32	vs;     /* Version */
	__u32	intms;  /* Interrupt Mask Set */
	__u32	intmc;  /* Interrupt Mask Clear */
	__u32	cc;     /* Controller Configuration */
	__u32	rsvd1;  /* Reserved */
	__u32	csts;   /* Controller Status */
	__u32	rsvd2;  /* Reserved */
	__u32	aqa;    /* Admin Queue Attributes */
	__u64	asq;    /* Admin SQ Base Address */
	__u64	acq;    /* Admin CQ Base Address */
};

struct nvme_dev_entry {
	struct pci_dev *pdev;
	u8 *bar0mapped;
};

struct nvme_space {
	struct nvme_ctrl_reg __iomem *bar_dev;
};

void read_nvme_registers(
	struct nvme_space nvme_ctrl_reg_space,
	char *udata
	);
#endif
