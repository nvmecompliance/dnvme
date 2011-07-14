#ifndef _DNVME_IOCTLS_H_
#define _DNVME_IOCTLS_H_
#endif

/** 
*  Enumerations for read type
*/
enum read_type_t {
	NVME_PCI_HEADER = 0,
        NVME_PCI_BAR0, 
};

struct nvme_read_generic{
   enum read_type_t type;
   u32		offset;
   u32  	nBytes;
   char 	*readBuffer;
};

/* Driver IOCTL Function prototypes */

int driver_generic_read(struct file *file,
                        unsigned long buffer,
                        struct nvme_read_generic *data_usr,
			struct pci_dev *pdev);
