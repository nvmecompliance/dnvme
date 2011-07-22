#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>

#include "dnvme_reg.h"

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

   index = 0;
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->cap);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->vs);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->intms);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->intmc);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->cc);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->rsvd1);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;
   u32data = readl(&nvme_ctrl_reg_space.bar_dev->csts);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->rsvd2);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   u32data = readl(&nvme_ctrl_reg_space.bar_dev->aqa);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   readl(&nvme_ctrl_reg_space.bar_dev->asq);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));
   index += 4;

   index += 4;

   readl(&nvme_ctrl_reg_space.bar_dev->acq);
   memcpy((u8 *)&udata[index], &u32data, sizeof(u32));

}

