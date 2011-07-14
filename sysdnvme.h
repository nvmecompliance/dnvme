#ifndef  _DNVME_H_
#define _DNVME_H_

#define LEVEL	"dnvme"
#define LOG_NORM(msg, ...)   \
    fprintf(stdout, "%s: %s\n", LEVEL, msg);
#define LOG_ERR(msg)    \
    fprintf(stderr, "%s-ERR:%s:%d:"msg"\n", LEVEL, __FILE__,__LINE__##__VA_ARGS__);
#ifdef DEBUG
#define LOG_DBG(msg)    \
    fprintf(stderr, "%s-DBG:__FILE__:__LINE__: %s\n", LEVEL, msg);
#else
#define LOG_DBG(msg);
#endif

/*!
*  NVME Express Device Structure Parameters.
*
*
*/
struct nvme_device_entry {
    struct list_head list;
    struct pci_dev *pdev;
    int bus;
    int slot;
    int func;
    struct gendisk *disk;
    u32    *bar;
    struct list_head namespaces;
    char   serial[20];
    char   model[40];
    char   firmware_rev[8];
};


/*! \mainpage Device Driver Documentation for NVME Specification 1.0a
   This manual describes the different files in Device Driver for
   NVME Compilance Suite 1.0a.

- \subpage Introduction
- \subpage Usage
*/
/*---------------------------------------------------------------------------*/
/*! \page Introduction Introduction to NVME Device Driver Complinace Suite
This page describes the user specific details.
*/
/*---------------------------------------------------------------------------*/
/*! \page Usage How to Use this Device driver to test NVME Device
This page describes the details on how to invoke this device driver
and work with Complinace suite.
*/
#endif
