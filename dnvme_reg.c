#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_reg.h"
#include "definitions.h"
/*
* read_nvme_registers  - Function to read the controller registers
* located in the MLBAR/MUBAR (PCI BAR 0 and 1) that are mapped to
* memory area which supports in-order access.
* The limitation of this reading of nvme space in QEMU dictates
* accessing individual register rather than looping with index.
*/
void read_nvme_registers(
			struct nvme_space nvme_ctrl_reg_space,
			char *udata
			)
{
   int index = 0;
   u32 u32data;
   u8 __iomem *bar;

   index = 0;
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->cap);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;
   LOG_DEBUG("Reg 0 = %d\n", u32data);
   
   bar = (void __iomem *)nvme_ctrl_reg_space.bar_dev->cap;
   bar += 4;
   
   u32data = readl(&bar);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;
   LOG_DEBUG("Reg 1 = %d\n", u32data);

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->vs);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   LOG_DEBUG("Reg 2= %d\n", u32data);
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->intms);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   LOG_DEBUG("Reg 3= %d\n", u32data);
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->intmc);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   LOG_DEBUG("Reg 4= %d\n", u32data);
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->cc);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   LOG_DEBUG("Reg 5= %d\n", u32data);
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->rsvd1);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;
   LOG_DEBUG("Reg 6= %d\n", u32data);
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->csts);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   LOG_DEBUG("Reg 7= %d\n", u32data);
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->rsvd2);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   LOG_DEBUG("Reg 8= %d\n", u32data);
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->aqa);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   LOG_DEBUG("Reg 9= %d\n", u32data);
   readl(&nvme_ctrl_reg_space.bar_dev->asq);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;
   
   bar = (void __iomem *)nvme_ctrl_reg_space.bar_dev->asq;
   bar += 4;
   
   u32data = readl(&bar);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   LOG_DEBUG("Reg 10= %d\n", u32data);
   readl(&nvme_ctrl_reg_space.bar_dev->acq);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;
   
   LOG_DEBUG("Reg 11= %d\n", u32data);
   bar = (void __iomem *)nvme_ctrl_reg_space.bar_dev->acq;
   bar += 4;
   
   u32data = readl(&bar);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;
   LOG_DEBUG("Reg 12= %d\n", u32data);
}


/*
* read_nvme_reg_generic  - Function to read the controller registers
* located in the MLBAR/MUBAR (PCI BAR 0 and 1) that are mapped to
* memory area which supports in-order access.
* The limitation of this reading of nvme space in QEMU dictates
* accessing individual register rather than looping with index.
*/
void read_nvme_reg_generic(
			struct nvme_space nvme_ctrl_reg_space,
			char *udata,
			int nbytes
			)
{
   int index = 0;
   u32 u32data;
   u8 __iomem *bar;

   bar = (u8 __iomem *)nvme_ctrl_reg_space.bar_dev;

   for(index = 0; index < nbytes; index+=4)
   {
	u32data = readl(&bar);
	LOG_DEBUG("Bar Location = 0x%x. Reg Location = %d Data = %x\n", bar, index, u32data);

	bar+=4;
   }
}
