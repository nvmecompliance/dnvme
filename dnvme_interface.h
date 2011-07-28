#ifndef _DNVME_INTERFACE_H_
#define _DNVME_INTERFACE_H_

/**
* These are the enum types used for branching to
* required offset as specified in the struct nvme_
* read_generic type parameter.
*/
enum nvme_io_space {
   NVMEIO_PCI_HDR,
   NVMEIO_BAR01,
   NVMEIO_FENCE	/* always must be the last element */
};

/**
* This struct is the basic structure which has important
* parameter for the generic read  and write funtion to seek the correct
* offset and length while reading or writing to nvme card.
*/
struct rw_generic {
   enum nvme_io_space type;
   unsigned int  offset;
   unsigned int  nBytes;
   unsigned char *buffer;
};

#endif
