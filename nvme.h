/*
 * Definitions for the NVM Express interface
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

#ifndef _LINUX_NVME_H
#define _LINUX_NVME_H

#include <linux/types.h>

#pragma pack(push, 1)

#ifdef CHATHAM
/* Slight difference in the older version of the spec */
struct nvme_bar {
	__u64			cap;	/* Controller Capabilities */
	__u32			vs;	/* Version */
	__u32			is;	/* Interrupt Status */
	__u32			cc;	/* Controller Configuration */
	__u32			csts;	/* Controller Status */
	__u32			pad;	/* Padding */
	__u32			aqa;	/* Admin Queue Attributes */
	__u64			asq;	/* Admin SQ Base Address */
	__u64			acq;	/* Admin CQ Base Address */
};
#else
/* Compliant with 1.0a spec NOT 1.0 Gold - see size of cc reg */
struct nvme_bar {
	__u64			cap;	/* Controller Capabilities */
	__u32			vs;	/* Version */
	__u32			intms;	/* Interrupt Mask Set */
	__u32			intmc;	/* Interrupt Mask Clear */
	__u32			cc;	/* Controller Configuration */
	__u32			rsvd1;	/* Reserved */
	__u32			csts;	/* Controller Status */
	__u32			rsvd2;	/* Reserved */
	__u32			aqa;	/* Admin Queue Attributes */
	__u64			asq;	/* Admin SQ Base Address */
	__u64			acq;	/* Admin CQ Base Address */
};
#endif
#pragma pack(pop)
enum {
/* Section3.1.5 */
	NVME_CC_ENABLE		= 1 << 0,
	NVME_CC_CSS_NVM		= 0 << 4,
	NVME_CC_MPS_SHIFT	= 7,
	NVME_CC_ARB_RR		= 0 << 11,
	NVME_CC_ARB_WRRU	= 1 << 11,
	NVME_CC_ARB_VS		= 3 << 11,
	NVME_CC_SHN_NONE	= 0 << 14,
	NVME_CC_SHN_NORMAL	= 1 << 14,
	NVME_CC_SHN_ABRUPT	= 2 << 14,
/* 6 = 64 byte sq size 4 = 16 byte cq size */
	NVME_CC_IOSQES		= 6 << 16, 
	NVME_CC_IOCQES		= 4 << 20,
/* Section3.1.5 */
	NVME_CSTS_RDY		= 1 << 0,
	NVME_CSTS_CFS		= 1 << 1,
	NVME_CSTS_SHST_NORMAL	= 0 << 2,
	NVME_CSTS_SHST_OCCUR	= 1 << 2,
	NVME_CSTS_SHST_CMPLT	= 2 << 2,
};

#define NVME_VS(major, minor)	(major << 16 | minor)

//__u8 psd[32];
struct pwr_state_desc {
	__le16 mp;
	__le16 rsvd1;
	__le32 enlat;
	__le32 exlat;
	__u8 rrt:5,
	     rsvd2:3;
	__u8 rrl:5,
	     rsvd3:3;
	__u8 rwt:5,
	     rsvd4:3;
	__u8 rwl:5,
	     rsvd5:3;
	__u8 rsvd6[16];
};

#define MAX_POWER_STATE		32
struct nvme_id_ctrl {
	__le16 vid;		/* [0-1] PCI Vendor ID*/
	__le16 ssvid;		/* [2-3] PCI Subsystem Vendor ID */
	char sn[20];		/* [4-23] Serial Number */
	char mn[40];		/* [24-63] Model Number */
	char fr[8];		/* [64-71] Firmware Number */
	__u8 rab;		/* [72] Recommended Arbitration Burst */
	__u8 rsvd73[183];	/* [73-255] Reserved */

	__le16 oacs;		/* [256-257] Optional Admin Command Support */
	__u8 acl;		/* [258] Abort Command Limit */
	__u8 aerl;		/* [259] Asynchronous Event Request Limit */
	__u8 frmw;		/* [260] Firmware Updates */
	__u8 lpa;		/* [261] Log Page Attributes */
	__u8 elpe;		/* [262] Error Log Page Entries */
	__u8 npss;		/* [263] Number of Power States Support */
	__u8 rsvd264[248];	/* [264-511] Reserved */

	__u8 sqes;		/* [512] Submission Queue Entry Size */
	__u8 cqes;		/* [513] Completion Queue Entry Size */
	__u8 rsvd514[2];	/* [514-515] Reserved */
	__le32 nn;		/* [516-519] Number of Namespaces */
	__le16 oncs;		/* [520-521] Optional NVM Command Support */
	__le16 fuses;		/* [522-523] Fused Operation Support */
	__u8 fna;		/* [524] Format NVM Attributes */
	__u8 vwc;		/* [525] Volatile Write Cache*/
	__le16 awun;		/* [526-527] Atomic Write Unit Normal */
	__le16 awupf;		/* [528-529] Atomic Write Unit Power Fail */

	__u8 rsvd530[174];	/* [530-703] Reserved */
	__u8 rsvd704[1344];	/* [704-2047] Reserved */

	struct pwr_state_desc psd[MAX_POWER_STATE];		/* [2048-3071] Power State 0-31 Descriptor */
	__u8 vs[1024];		/* [3072-4095] Vendor Specific */
};

struct nvme_lbaf {
	__le16			ms;
	__u8			ds;
	__u8			rp;
};

struct nvme_id_ns {
	__le64			nsze;
	__le64			ncap;
	__le64			nuse;
	__u8			nsfeat;
	__u8			nlbaf;
	__u8			flbas;
	__u8			mc;
	__u8			dpc;
	__u8			dps;
	__u8			rsvd30[98];
	struct nvme_lbaf	lbaf[16];
	__u8			rsvd192[192];
	__u8			vs[3712];
};

enum {
	NVME_NS_FEAT_THIN	= 1 << 0,
	NVME_LBAF_RP_BEST	= 0,
	NVME_LBAF_RP_BETTER	= 1,
	NVME_LBAF_RP_GOOD	= 2,
	NVME_LBAF_RP_DEGRADED	= 3,
};

struct nvme_lba_range_type {
	__u8			type;
	__u8			attributes;
	__u8			rsvd2[14];
	__u64			slba;
	__u64			nlb;
	__u8			guid[16];
	__u8			rsvd48[16];
};

enum {
	NVME_LBART_TYPE_FS	= 0x01,
	NVME_LBART_TYPE_RAID	= 0x02,
	NVME_LBART_TYPE_CACHE	= 0x03,
	NVME_LBART_TYPE_SWAP	= 0x04,

	NVME_LBART_ATTRIB_TEMP	= 1 << 0,
	NVME_LBART_ATTRIB_HIDE	= 1 << 1,
};

/* I/O commands */

enum nvme_opcode {
	nvme_cmd_flush		= 0x00,
	nvme_cmd_write		= 0x01,
	nvme_cmd_read		= 0x02,
	nvme_cmd_write_uncor	= 0x04,
	nvme_cmd_compare	= 0x05,
	nvme_cmd_dsm		= 0x09,
};

#ifdef CHATHAM
struct nvme_common_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__le64			metadata;
	__le64			prp1;
	__le64			prp2;
	__u32			rsvd10[8];
};

struct nvme_rw_command {
	__u8			opcode;		/* DW 0 */
	__u8			flags;
	__u16			command_id;
	__le32			nsid;		/* DW 1 */
	__le64			metadata;	/* DW 2 */
	__le64			prp1;		/* DW 4 */
	__le64			prp2;		/* DW 6 */
	__le64			slba;		/* DW 8 */
	__le16			length;		/* DW 10 */
	__le16			control;
	__le32			dsmgmt;		/* DW 11 */
	__le32			reftag;		/* DW 12 */
	__le16			apptag;		/* DW 13 */
	__le16			appmask;
	__u32			rsvd14[2];
};
#else
struct nvme_common_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2;
	__le64			metadata;
	__le64			prp1;
	__le64			prp2;
	__u32			rsvd10[6];
};

struct nvme_rw_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2;
	__le64			metadata;
	__le64			prp1;
	__le64			prp2;
	__le64			slba;
	__le16			length;
	__le16			control;
	__le32			dsmgmt;
	__le32			reftag;
	__le16			apptag;
	__le16			appmask;
};
#endif

enum {
	NVME_RW_LR			= 1 << 15,
	NVME_RW_FUA			= 1 << 14,
	NVME_RW_DSM_FREQ_UNSPEC		= 0,
	NVME_RW_DSM_FREQ_TYPICAL	= 1,
	NVME_RW_DSM_FREQ_RARE		= 2,
	NVME_RW_DSM_FREQ_READS		= 3,
	NVME_RW_DSM_FREQ_WRITES		= 4,
	NVME_RW_DSM_FREQ_RW		= 5,
	NVME_RW_DSM_FREQ_ONCE		= 6,
	NVME_RW_DSM_FREQ_PREFETCH	= 7,
	NVME_RW_DSM_FREQ_TEMP		= 8,
	NVME_RW_DSM_LATENCY_NONE	= 0 << 4,
	NVME_RW_DSM_LATENCY_IDLE	= 1 << 4,
	NVME_RW_DSM_LATENCY_NORM	= 2 << 4,
	NVME_RW_DSM_LATENCY_LOW		= 3 << 4,
	NVME_RW_DSM_SEQ_REQ		= 1 << 6,
	NVME_RW_DSM_COMPRESSED		= 1 << 7,
};

/* Admin commands */

enum nvme_admin_opcode {
	nvme_admin_delete_sq		= 0x00,
	nvme_admin_create_sq		= 0x01,
	nvme_admin_get_log_page		= 0x02,
	nvme_admin_delete_cq		= 0x04,
	nvme_admin_create_cq		= 0x05,
	nvme_admin_identify		= 0x06,
	nvme_admin_abort_cmd		= 0x08,
	nvme_admin_set_features		= 0x09,
	nvme_admin_get_features		= 0x0a,
	nvme_admin_async_event		= 0x0c,
	nvme_admin_activate_fw		= 0x10,
	nvme_admin_download_fw		= 0x11,
	nvme_admin_format_nvm		= 0x80,
	nvme_admin_security_send	= 0x81,
	nvme_admin_security_recv	= 0x82,
};

enum {
	NVME_QUEUE_PHYS_CONTIG	= (1 << 0),
	NVME_CQ_IRQ_ENABLED	= (1 << 1),
	NVME_SQ_PRIO_URGENT	= (0 << 1),
	NVME_SQ_PRIO_HIGH	= (1 << 1),
	NVME_SQ_PRIO_MEDIUM	= (2 << 1),
	NVME_SQ_PRIO_LOW	= (3 << 1),
/* Figure 53: Feature Identifier */
	NVME_FEAT_ARBITRATION	= 0x01,
	NVME_FEAT_POWER_MGMT	= 0x02,
	NVME_FEAT_LBA_RANGE	= 0x03,
	NVME_FEAT_TEMP_THRESH	= 0x04,
	NVME_FEAT_ERR_RECOVERY	= 0x05,
	NVME_FEAT_VOLATILE_WC	= 0x06,
	NVME_FEAT_NUM_QUEUES	= 0x07,
	NVME_FEAT_IRQ_COALESCE	= 0x08,
	NVME_FEAT_IRQ_CONFIG	= 0x09,
	NVME_FEAT_WRITE_ATOMIC	= 0x0a,
	NVME_FEAT_ASYNC_EVENT	= 0x0b,
	NVME_FEAT_SW_PROGRESS	= 0x80,
};

struct nvme_identify {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	__le64			prp1;
	__le64			prp2;
	__le32			cns;
	__u32			rsvd11[5];
};

struct nvme_features {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	__le64			prp1;
	__le64			prp2;
	__le32			fid;
	__le32			dword11;
	__u32			rsvd12[4];
};

#ifdef CHATHAM
struct nvme_create_cq {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[3];
	__le64			prp1;
	__u64			rsvd6;
	__le16			cqid;
	__le16			qsize;
	__le16			cq_flags;
	__le16			irq_vector;
	__u32			rsvd10[6];
};

struct nvme_create_sq {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[3];
	__le64			prp1;
	__u64			rsvd6;
	__le16			sqid;
	__le16			qsize;
	__le16			sq_flags;
	__le16			cqid;
	__u32			rsvd10[6];
};
#else
struct nvme_create_cq {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__u64			rsvd8;
	__le16			cqid;
	__le16			qsize;
	__le16			cq_flags;
	__le16			irq_vector;
	__u32			rsvd12[4];
};

struct nvme_create_sq {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__u64			rsvd8;
	__le16			sqid;
	__le16			qsize;
	__le16			sq_flags;
	__le16			cqid;
	__u32			rsvd12[4];
};
#endif

struct nvme_delete_queue {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[9];
	__le16			qid;
	__u16			rsvd10;
	__u32			rsvd11[5];
};

struct nvme_download_firmware {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__u32			rsvd1[5];
	__le64			prp1;
	__le64			prp2;
	__le32			numd;
	__le32			offset;
	__u32			rsvd12[4];
};

struct nvme_abort {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__u64			rsvd2[2];
	__le64			prp1;
	__le64			prp2;
	__le16			sqid;
	__le16		        cmdid;
};

struct nvme_command {
	union {
		struct nvme_common_command common;
		struct nvme_rw_command rw;
		struct nvme_identify identify;
		struct nvme_features features;
		struct nvme_create_cq create_cq;
		struct nvme_create_sq create_sq;
		struct nvme_delete_queue delete_queue;
		struct nvme_download_firmware dlfw;
	        struct nvme_abort abort;
	};
};

/* Figure 15: Status Code Types */
enum {
	NVME_SCT_GENERIC		= 0x0000,
	NVME_SCT_SPECIFIC_ERRS		= 0x0100,
	NVME_SCT_MMEDIA_ERRS		= 0x0200,
	NVME_SCT_VENDOR_ERRS		= 0x0700,
};

#define NVME_SCT_MASK			0x0700

/* XXX: Sync with spec */
enum {
	NVME_SC_SUCCESS			= 0x0,
	NVME_SC_INVALID_OPCODE		= 0x1,
	NVME_SC_INVALID_FIELD		= 0x2,
	NVME_SC_CMDID_CONFLICT		= 0x3,
	NVME_SC_DATA_XFER_ERROR		= 0x4,
	NVME_SC_POWER_LOSS		= 0x5,
	NVME_SC_INTERNAL		= 0x6,
	NVME_SC_ABORT_REQ		= 0x7,
	NVME_SC_ABORT_QUEUE		= 0x8,
	NVME_SC_FUSED_FAIL		= 0x9,
	NVME_SC_FUSED_MISSING		= 0xa,
	NVME_SC_INVALID_NS		= 0xb,
	NVME_SC_LBA_RANGE		= 0x80,
	NVME_SC_CAP_EXCEEDED		= 0x81,
	NVME_SC_NS_NOT_READY		= 0x82,
	NVME_SC_CQ_INVALID		= 0x100,
	NVME_SC_QID_INVALID		= 0x101,
	NVME_SC_QUEUE_SIZE		= 0x102,
	NVME_SC_ABORT_LIMIT		= 0x103,
	NVME_SC_ABORT_MISSING		= 0x104,
	NVME_SC_ASYNC_LIMIT		= 0x105,
	NVME_SC_FIRMWARE_SLOT		= 0x106,
	NVME_SC_FIRMWARE_IMAGE		= 0x107,
	NVME_SC_INVALID_VECTOR		= 0x108,
	NVME_SC_INVALID_LOG_PAGE	= 0x109,
	NVME_SC_INVALID_FORMAT		= 0x10a,
	NVME_SC_BAD_ATTRIBUTES		= 0x180,
	NVME_SC_WRITE_FAULT		= 0x280,
	NVME_SC_READ_ERROR		= 0x281,
	NVME_SC_GUARD_CHECK		= 0x282,
	NVME_SC_APPTAG_CHECK		= 0x283,
	NVME_SC_REFTAG_CHECK		= 0x284,
	NVME_SC_COMPARE_FAILED		= 0x285,
	NVME_SC_ACCESS_DENIED		= 0x286,
};

/* Figure 18: Status Code – Specific Command Errors Values */
enum {
	NVME_SC_CQUEUE_INVALID		= 0x00,
	NVME_SC_INVALID_QID		= 0x01,
	NVME_MAX_QUEUE_SIZE_EXCEEDED	= 0x02,
	NVME_SC_CMD_LIMIT_EXCEEDED	= 0x03,
	NVME_SC_CMD_REQ_NOT_FOUND	= 0x04,
	NVME_ASYNC_EVENT_LIMIT_EXCEEDED	= 0x05,
	NVME_INVALID_FIRMWARE_SLOT	= 0x06,
	NVME_INVALID_FIRMWARE_IMAGE	= 0x07,
	NVME_INVALID_INTERRUPT_VECTOR	= 0x08,
	NVME_INVALID_LOG_PAGE		= 0x09,
	NVME_INVALID_FORMAT		= 0x0a,

	NVME_CMD_NVM_ERR_CONFLICT	= 0x80,
};
#define NVME_SC_MASK			0x00FF

struct nvme_completion {
	__le32	result;		/* Used by admin commands to return data */
	__u32	rsvd;
	__le16	sq_head;	/* how much of this queue may be reclaimed */
	__le16	sq_id;		/* submission queue that generated this entry */
	__u16	command_id;	/* of the command which completed */
	__le16	status;		/* did the command fail, and if so, why? */
};

struct nvme_user_io {
	__u8	opcode;
	__u8	flags;
	__u16	control;
	__u32	nsid;
	__u64	metadata;
	__u64	addr;
	__u64	slba;
	__u16	nblocks;
	__u16	block_shift;
	__u32	dsmgmt;
	__u32	reftag;
	__u16	apptag;
	__u16	appmask;
	__u32	result;
};

struct nvme_dlfw {
	__u64	addr;
	__u32	length;	/* In dwords */
	__u32	offset;	/* In dwords */
};

#define NVME_IOCTL_IDENTIFY_NS	_IOW('e', 0x60, struct nvme_id_ns)
#define NVME_IOCTL_IDENTIFY_CTRL _IOW('e', 0x61, struct nvme_id_ctrl)
#define NVME_IOCTL_GET_RANGE_TYPE _IOW('e', 0x62, struct nvme_lba_range_type)
#define NVME_IOCTL_SUBMIT_IO	_IOWR('e', 0x63, struct nvme_rw_command)
#define NVME_IOCTL_DOWNLOAD_FW	_IOR('e', 0x64, struct nvme_dlfw)
#define NVME_IOCTL_ACTIVATE_FW	_IO('e', 0x65)

#ifdef __KERNEL__
#include <linux/list.h>
#include <linux/pci.h>
struct bio;

#define PLX_MSIX_COUNT 32
#define PLX_MSIX_OFFSET	(4 * 64 * 1024)

/*
 * Represents an NVM Express device.  Each nvme_dev is a PCI function.
 * currently unused
struct nvme_dev {
	struct list_head node;
	struct nvme_queue **queues;
	u32 __iomem *dbs;
	struct pci_dev *pci_dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	int instance;
	int queue_count;
	u32 ctrl_config;
	struct msix_entry *entry;
	struct nvme_bar __iomem *bar;
	struct list_head namespaces;
	char serial[20];
	char model[40];
	char firmware_rev[8];
};
*/

/*
 * An NVM Express namespace is equivalent to a SCSI LUN
 * currently unused
struct nvme_ns {
	struct list_head list;

	struct nvme_dev *dev;
	struct request_queue *queue;
	struct gendisk *disk;

	int ns_id;
	int lba_shift;
};
*/

/*
 * some functions
 * currently unused
int nvme_submit_admin_cmd(struct nvme_dev *, struct nvme_command *, u32 *result);
int nvme_read_page(struct nvme_ns *ns, struct page *page);
int nvme_write_page(struct nvme_ns *ns, struct page *page, int sync);
int nvme_submit_bio(struct nvme_ns *ns, struct bio *bio);
*/
extern int nvme_major;
#endif

#endif /* _LINUX_NVME_H */
