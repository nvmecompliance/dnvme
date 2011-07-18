#ifndef _DNVME_IOCTLS_H_
#define _DNVME_IOCTLS_H_
#endif

/**
* These are the enum types used for branching to
* required offset as specified in the struct nvme_
* read_generic type parameter.
*/
enum type_t {
   NVME_PCI_HEADER = 0,
   NVME_PCI_BAR01,
};

/**
* This struct is the basic structure which has important
* parameter for the generic read funtion to seek the correct
* offset and length while reading data from nvme card.
*/
struct nvme_read_generic {
   enum type_t type;
   u32	offset;
   u32  nBytes;
   unsigned char *rdBuffer;
};

/**
* This struct is the basic structure which has important
* parameter for the generic write funtion to seek the correct
* offset and length while serving the write request.
*/
struct nvme_write_generic {
   enum type_t type;
   u32	offset;
   u32  nBytes;
   unsigned char *wrBuffer;
};

/**
* This is a structure that defines all the PCI
* header information laid out in NVME SPec 1.0a
* PCI Header section.
*/
struct nvme_pci_header {
   u32 ID;
   u16 CMD;
   u16 STS;
   u8  RID;
   u32 CC;
   u8  CLS;
   u8  MLT;
   u8  HTYPE;
   u8  BIST;
   u32 BAR0;
   u32 BAR1;
   u32 BAR2;
   u32 BAR3;
   u32 BAR4;
   u32 BAR5;
   u32 CCPTR;
   u32 SS;
   u32 EPROM;
   u64 CAP;
   u16 INTR;
   u8  MGNT;
   u8  MLAT;
};

/**
* driver_generic_read is a function that is called from
* driver IOCTL when user want to read data from the
* NVME card. The read parameter like offset and length
* etc are specified from the struct nvme_read_generic
* @param file Pass the file descriptor of the device opened.
* @param buffer buffer to store data
* @data_user Structure with different user required parameters.
* @pdev pointer to the device opened.
* @return read success or failure.
*/
int driver_generic_read(struct file *file,
			struct nvme_read_generic *data_usr,
			struct pci_dev *pdev);
/**
* driver_generic_write is a function that is called from
* driver IOCTL when user want to write data to the
* NVME card. The write parameters offset and length
* etc are specified from the struct nvme_write_generic
* @param file Pass the file descriptor of the device opened.
* @param buffer buffer to store data
* @data_user Structure with different user required parameters.
* @pdev pointer to the device opened.
* @return read success or failure.
*/
int driver_generic_write(struct file *file,
			struct nvme_write_generic *data_usr,
			struct pci_dev *pdev);

