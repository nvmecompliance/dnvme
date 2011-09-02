#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_reg.h"
#include "definitions.h"
#include "sysdnvme.h"

/* Conditional compilation for QEMU related modifications. */
#ifdef QEMU
/*
* if QEMU is defined then we do 64 bit write in two 32 bit writes using
* writel's otherwise directly call writeq.
*/
static inline __u64 READQ(const volatile void __iomem *addr)
{
    const volatile u32 __iomem *p = addr;
    u32 low, high;
    low = readl(p);
    high = readl(p + 1);
    return low + ((u64)high << 32);
}
/*
* if QEMU is defined then we do 64 bit write in two 32 bit writes using
* writel's otherwise directly call writeq.
*/
static inline void WRITEQ(__u64 val, volatile void __iomem *addr)
{
   writel(val, addr);
   writel(val >> 32, addr + 4);
}
#else
static inline void WRITEQ(__u64 val, volatile void __iomem *addr)
{
   writeq(val, addr);
}
static inline __u64 READQ(const volatile void __iomem *addr)
{
   return readq(addr);
}
#endif

/*
* read_nvme_reg_generic  - Function to read the controller registers
* located in the MLBAR/MUBAR (PCI BAR 0 and 1) that are mapped to
* memory area which supports in-order access.
* The limitation of this reading of nvme space in QEMU dictates
* accessing individual register rather than looping with index.
*/
int read_nvme_reg_generic(
			struct nvme_space nvme_ctrl_reg_space,
			u8 *udata,
			int nbytes,
			int offset,
			enum nvme_acc_type acc_type
			)
{
   int index = 0; /* index to loop for total bytes in count of 4 */
   u32 u32data; /* temp data buffer for readl call. */
   unsigned int *bar; /* base address for nvme space from cap */

   /*
   * Assign base address of the Controller Register to bar as the
   * starting point.
   */
   bar = (unsigned int *)&nvme_ctrl_reg_space.bar_dev->cap;

   /* Compute the offset requested by user */
   bar = bar + (offset/sizeof(unsigned int));

   /* loop until user requested nbytes are read. */
   for (index = 0; index < nbytes;) {
	/* Read data from NVME space */
	if (acc_type == DWORD_LEN) {
		u32data = readl(bar);

		LOG_NRM("NVME Reg Reading Data at 0x%0X:0x%0X", *bar, u32data);
		/* Increments in multiples of 4 (size of int) */
		bar++;
		/* Copy data to user buffer. */
		memcpy((u8 *)&udata[index], &u32data, sizeof(u32));

		index += 4;

	} else if (acc_type == QUAD_LEN) {
		/* Directly copy into buffer */
		udata[index] = (u64)READQ(bar);

		/* increment address by 8 bytes */
		bar += 2;

		/* move index by 8 bytes */
		index += 8;
	} else {
		LOG_ERR("Unknown access width specified");
		return -EINVAL;
	}
   }
   return 0;
}

/*
* write_nvme_reg_generic  - Function to write the controller registers
* located in the MLBAR/MUBAR (PCI BAR 0 and 1) that are mapped to
* memory area which supports in-order access.
*/
int write_nvme_reg_generic(
			struct nvme_space nvme_ctrl_reg_space,
			u8 *u8data,
			int nbytes,
			int offset,
			enum nvme_acc_type acc_type
			)
{
   int index = 0; /* index to loop for total bytes in count of 4 */
   unsigned int *bar; /* base address for nvme space from cap */
   u32 u32data;
   /*
   * Assign base address of the Controller Register to bar as the
   * starting point.
   */
   bar = (unsigned int *)&nvme_ctrl_reg_space.bar_dev->cap;

   /* Compute the offset requested by user */
   bar = bar + offset/(sizeof(unsigned int));

   /* loop until user requested nbytes are written. */
   for (index = 0; index < nbytes;) {

	/* Copy u32 type data variable */
	if (acc_type == DWORD_LEN) {
		memcpy((u8 *)&u32data, &u8data[index], sizeof(u32));

		LOG_NRM("NVME Space Writing at Addr:Val::0x%lX:0x%X",
				(unsigned long int)bar, u32data);
		/* Write data to NVME space */
		writel(u32data, bar);

		/* increment by offset 4 as bar is unsigned int */
		bar++;

		/* Move index by 4 bytes */
		index += 4;
	} else if (acc_type == QUAD_LEN) {
		/* Write 8 bytes of data */
		WRITEQ((u64)u8data[index], bar);

		/* Increment address by 8 bytes */
		bar += 2;

		/* Move index by 8 bytes */
		index += 8;
	}
   }

   return 0;
}
