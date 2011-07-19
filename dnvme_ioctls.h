#ifndef _DNVME_IOCTLS_H_
#define _DNVME_IOCTLS_H_
#endif

/**
* @def PCI_DEVICE_STATUS
* define the offset for STS register
* from the start of PCI config space as specified in the
* NVME_Comliance 1.0a. offset 06h:STS - Device status.
* This register has error status for NVME PCI Exress
* Card. After reading data from this reagister, the driver
* will identify if any error is set during the operation and
* report as kernel alert message.
*/
#define PCI_DEVICE_STATUS               0x6

/** 
* @def DEV_ERR_MASK
* The bit positions that are set in this 16 bit word
* implies that the error is defined for those poistionis in
* STS register. The bits that are 0 are non error positions.
*/
#define DEV_ERR_MASK			0xC100

/**
* @def DPE
* This bit position indicates data parity error.
* Set to 1 by h/w when the controlller detects a 
* parity error on its interface.
*/
#define DPE				0x8000

/**
* @def SSE
* This bit position indicates Signaled System Error.
* Not Supported vy NVM Express.
*/
#define SSE				0x4000

/**
* @def DPD
* This bit position indicates Master data parity error.
* Set to 1 by h/w if parity error is set or parity
* error line is asserted and parity error response bit
* in CMD.PEE is set to 1.
*/
#define DPD				0x0100

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
* @param data_usr Structure with different user required parameters.
* @param pdev pointer to the device opened.
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
* @param data_usr Structure with different user required parameters.
* @param pdev pointer to the device opened.
* @return read success or failure.
*/
int driver_generic_write(struct file *file,
			struct nvme_write_generic *data_usr,
			struct pci_dev *pdev);

/**
* device_status_chk  - Generic error checking function
* which checks error registers and set kernel
* alert if a error is detected.
* @param pdev
* @param status
*/
void device_status_chk(struct pci_dev *pdev,
                        int status);
