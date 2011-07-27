#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_reg.h"
#include "definitions.h"
#include "sysdnvme.h"

/*
* read_nvme_reg_generic  - Function to read the controller registers
* located in the MLBAR/MUBAR (PCI BAR 0 and 1) that are mapped to
* memory area which supports in-order access.
* The limitation of this reading of nvme space in QEMU dictates
* accessing individual register rather than looping with index.
*/
void read_nvme_reg_generic(
			struct nvme_space nvme_ctrl_reg_space,
			u8 *udata,
			int nbytes,
			int offset
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
   for (index = 0; index < nbytes; index += 4) {
	/* Read data from NVME space */
	u32data = readl(bar);

	LOG_NRM("NVME Space Reading Data at 0x%0X:0x%0X\n", *bar, u32data);
	/* Increments in multiples of 4 (size of int) */
	bar++;
	/* Copy data to user buffer. */
	memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   }
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
			int offset
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
   for (index = 0; index < nbytes; index += 4) {

	/* Copy u32 type data variable */
	memcpy((u8 *)&u32data, &u8data[index], sizeof(u32));

	LOG_NRM("NVME Space Writing at Addr:Val::0x%lX:0x%X\n",
				(unsigned long int)bar, u32data);
	/* Write data to NVME space */
	writel(u32data, bar);

	/* increment by offset 4 as bar is unsinged int */
	bar++;
   }

   return 0;
}
