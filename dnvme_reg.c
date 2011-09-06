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
    u64 u64data;

    LOG_DBG("ADDR=0x%llx:p=0x%llx", (u64)addr, (u64)p);
    low = readl(p);
    high = readl(p + 1);

    u64data = high;
    u64data = (u64data << 32) + low;
    LOG_DBG("High:low:u64data = 0x%x:0x%x:0x%llx", high, low, u64data);
    return u64data;
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
   u64 u64data;
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

		LOG_NRM("NVME Read Dword at 0x%0X:0x%0X", *bar, u32data);

		/* Increments in multiples of 4 (size of int) */
		bar++;
		/* Copy data to user buffer. */
		memcpy((u8 *)&udata[index], &u32data, sizeof(u32));

		index += 4;

	} else if (acc_type == QUAD_LEN) {
		/* Directly copy into buffer */
		u64data = READQ(bar);

		/* Copy data to user buffer. */
		memcpy((u8 *)&udata[index], &u64data, sizeof(u64));

		LOG_NRM("NVME Read Quad 0x%llX:0x%llX",
					(u64)bar, (u64)udata[index]);

		/* increment address by 8 bytes */
		bar++;
		bar++;

		/* move index by 8 bytes */
		index += 8;
	} else {
		LOG_ERR("Unknown access width specified, use only DWORD/QUAD");
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
   u64 u64data;
   /*
   * Assign base address of the Controller Register to bar as the
   * starting point.
   */
   bar = (unsigned int *)&nvme_ctrl_reg_space.bar_dev->cap;

   /* Compute the offset requested by user */
   bar = bar + offset/(sizeof(unsigned int));

   /* loop until user requested nbytes are written. */
   for (index = 0; index < nbytes;) {

	/* Check the acc_type and do write as per the access type  */
	if (acc_type == DWORD_LEN) {

		/* Dword access so do 32 bit access */
		memcpy((u8 *)&u32data, &u8data[index], sizeof(u32));

		LOG_NRM("NVME Writing DWORD at Addr:Val::0x%lX:0x%X",
				(unsigned long int)bar, u32data);
		/* Write 32 bit data to NVME space */
		writel(u32data, bar);

		/* increment offset by 4 as bar is unsigned int */
		bar++;

		/* Move index by 4 bytes */
		index += 4;
	} else if (acc_type == QUAD_LEN) {

		/* QUAD access so do 64 bit access */
		memcpy((u8 *)&u64data, &u8data[index], sizeof(u64));

		LOG_NRM("NVME Writing QUAD at Addr:Val::0x%lX:0x%llX",
				(unsigned long int)bar, u64data);

		/* Write 8 bytes of data */
		WRITEQ(u64data, bar);

		/* Increment address by 8 bytes */
		bar++;
		bar++;

		/* Move index by 8 bytes */
		index += 8;
	} else {
		LOG_ERR("Unknown access width specified, use only DWORD/QUAD");
		return -EINVAL;
	}
   }

   return 0;
}
