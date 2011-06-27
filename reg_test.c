#include <linux/module.h>
#include <linux/pci.h>

/* project specific includes */
#include "test_driver.h"
#include "definitions.h"

struct register_entry {
	int offset;
	int size;
	uint64_t rmask;
	uint64_t wmask;
	uint64_t init_val;
	uint64_t vmask;
	char *name;
};
/* bitmask macro */
#define BM(e, s) (((uint64_t)(1LLU << (((e)-(s)+1)))-1)  << (s))
/* initial value of controller capabilities register */
#define CAP_VAL		(BM(23, 19) | BM(36, 32) | BM(47, 41) | BM(63, 56))

struct register_entry register_map[] = {
	{0, 0, 0, 0, 0, 0, NULL},
	{0x00, 8, BM(63, 0), 0x0, 0x0, CAP_VAL, "Controller Capabilities"},
	{0x08, 4, BM(31, 0), 0x0, NVME_VS(1, 0), BM(31, 0), "Version"},
	{0x0C, 4, BM(31, 0), 0x00, 0x00, 0x00, "Interrupt Mask Set"},
	{0x10, 4, BM(31, 0), 0x00, 0x00, 0x00, "Interrupt Mask Clear"},
	/* this reg below is compliant with 1.0a spec NOT 1.0 Gold */
	{0x14, 4, BM(31, 0), BM(23, 4) | BM(0, 0), 0x0, BM(31, 0),
		"Controller Configuration"},
	{0x1C, 4, BM(31, 0), 0x0, 0x0, BM(31, 0), "Controller Status"},
	{0x24, 4, BM(31, 0), BM(27, 16) | BM(11, 0), 0x0, BM(31, 0),
		"Admin Queue Attributes"},
	{0x28, 8, BM(63, 0), BM(63, 12), 0x0, BM(11, 0),
		"Admin Submission Queue Base Address"},
	{0x30, 8, BM(63, 0), BM(63, 12), 0x0, BM(11, 0),
		"Admin Completion Queue Base Address"},
};

uint64_t read_reg(int index, u8 *bar)
{
	int offset;
	union reg {
		uint64_t u64;
		uint32_t u32[2];
	} reg;

	reg.u64 = 0;

	offset = register_map[index].offset;
	if (register_map[index].size == 8) {
		reg.u32[0] = ioread32((u32 *)(bar + offset));
		reg.u32[1] = ioread32((u32 *)(bar + offset + 4));
	} else if (register_map[index].size == 4)
		reg.u32[0] = ioread32((u32 *)(bar + offset));
	else
		LOG_NORMAL("[Nvme_Drv]Unsupported size read operation\n");
	return reg.u64;
}

void write_reg(int index, u8 *bar, uint64_t value)
{
	int offset;

	offset = register_map[index].offset;

	if (register_map[index].size == 8) {
		iowrite32(value & BM(31, 0), (u32 *)(bar + offset));
		iowrite32((value >> 32) & BM(31, 0), (u32 *)(bar + offset + 4));
	} else if (register_map[index].size == 4)
		iowrite32(value & BM(31, 0), (u32 *)(bar + offset));
	else
		LOG_NORMAL("[Nvme_Drv]Unsupported size write operation\n");
}

int bit_compare(int index, uint64_t exp_value, uint64_t reg_value)
{
	int i;
	uint64_t bmask;
	uint64_t init_value;
	uint64_t vmask;

	for (i = 0; i < (register_map[index].size * 8); i++) {
		bmask = BM(i, i);

		/* check if readable */
		if (!((bmask & register_map[index].rmask)))
			continue;
		if ((bmask & register_map[index].wmask)) {
			/* register is writable */
			if ((exp_value & bmask) != (reg_value & bmask)) {
				LOG_NORMAL("[Nvme_Drv]Write %08llx failure "
					" position:%d value %08llx\n", 
					exp_value, i, reg_value);
				return STATUS_NOK;
			}
		} else {
			/* register is not writable,
			compare with initial value */
			init_value = register_map[index].init_val;
			vmask = register_map[index].vmask;
			if ((init_value & bmask & vmask) !=
				(reg_value & bmask & vmask)) {
				LOG_NORMAL("[Nvme_Drv]Write %08llx to RO bit changed"
					" from initial value, position:%d, value %08llx\n", 
					exp_value, i, reg_value);
				return STATUS_NOK;
			}
		}
	}

	return STATUS_OK;
}


void reset(struct test_ctx *test_ctx)
{

	u32 val;
	val = readl(&test_ctx->bar_dev->cc);
	LOG_NORMAL("[Nvme_Drv]RESET!!!!!\n");

	/* CC.EN -> 0 */
	val &= ~NVME_CC_ENABLE;
	writel(val, &test_ctx->bar_dev->cc); 
	msleep(200);

	/* CC.EN -> 1 */
	val |= NVME_CC_ENABLE;
	writel(val, &test_ctx->bar_dev->cc); 

	val = 0;
	/* CSTS.RDY ? 1 */
	while (!(readl(&test_ctx->bar_dev->csts) & NVME_CSTS_RDY))
		msleep(5);
}

int verify_reg_map(struct test_ctx *test_ctx)
{
	int i;
	int reg_entries;
	int offset;
	u8 *bar0;
	uint64_t reg_value;

	/* FIXME - 
	 * Q: should we do a RESET? ; 
	 * A: TBD, p/f on occasion based expected initial values */
	/*reset(test_ctx);	*/

	reg_entries = sizeof(register_map)/sizeof(struct register_entry);
	bar0 = (u8 *)test_ctx->bar_dev;

	/* select register based on the test number */
	i = test_ctx->test_num;

	if ((i == 0) || (i >= reg_entries))
		return STATUS_TEST_NOT_DEFINED;
		

	/* read register */
	offset = register_map[i].offset;
	reg_value = read_reg(i, bar0);

	LOG_DEBUG("[Nvme_Drv]Register '%s' verify offset: 0x%03X, size: %d\n",
		register_map[i].name, register_map[i].offset, register_map[i].size);

	LOG_DEBUG("\treg_value : %016llX\n", reg_value);
	LOG_DEBUG("\trmask     : %016llX\n", register_map[i].rmask);
	LOG_DEBUG("\twmask     : %016llX\n", register_map[i].wmask);
	LOG_DEBUG("\tinit_val  : %016llX\n", register_map[i].init_val);
	LOG_DEBUG("\tvmask     : %016llX\n", register_map[i].vmask);


	/* verify initial value */
	if ((reg_value & register_map[i].vmask) !=
		(register_map[i].init_val & register_map[i].vmask)) {
		LOG_DEBUG("[Nvme_Drv]Initial register value %08llX differs, should be "
			"%08llX\n", (uint64_t)reg_value,
			(uint64_t)register_map[i].init_val);
		if (test_ctx->test_flags & TEST_MMIO_INITAL_VALUE)
			return STATUS_NOK;
	}
	if ((test_ctx->test_flags & TEST_MMIO_RW) == 0)
		return STATUS_OK;

	/* verify read/write access */
	/* write 1's */
	write_reg(i, bar0, ULLONG_MAX);
	/* compare with wmask & rmask & 1's */
	reg_value = read_reg(i, bar0);
	if (bit_compare(i, ULLONG_MAX, reg_value))
		return STATUS_NOK;

	/* write 0's */
	write_reg(i, bar0, 0);
	/* compare with wmask & rmask & 0's */
	reg_value = read_reg(i, bar0);
	if (bit_compare(i, 0, reg_value))
		return STATUS_NOK;

	return STATUS_OK;
}
