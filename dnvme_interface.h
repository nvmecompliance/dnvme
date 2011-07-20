#ifndef _DNVME_INTERFACE_H_
#define _DNVME_INTERFACE_H_

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
   unsigned int  offset;
   unsigned int  nBytes;
   unsigned char *rdBuffer;
};

/**
* This struct is the basic structure which has important
* parameter for the generic write funtion to seek the correct
* offset and length while serving the write request.
*/
struct nvme_write_generic {
   enum type_t type;
   unsigned int  offset;
   unsigned int  nBytes;
   unsigned char *wrBuffer;
};

#endif
