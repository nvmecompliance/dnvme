#ifndef _DNVME_H_
#define _DNVME_H_

#define APPNAME         "dnvme"
#define LEVEL           APPNAME
#define LOG_NRM(fmt, ...)    \
    printk("%s: " fmt "\n", LEVEL, ## __VA_ARGS__)
#define LOG_ERR(fmt, ...)    \
    printk("%s-err:%s:%d: " fmt "\n", \
	LEVEL, __FILE__, __LINE__, ## __VA_ARGS__)
#ifdef DEBUG
#define LOG_DBG(fmt, ...)    \
    printk("%s-dbg:%s:%d: " fmt "\n", \
	LEVEL, __FILE__, __LINE__, ## __VA_ARGS__)
#else
#define LOG_DBG(fmt, ...)
#endif

/**
* This definition is intended for making modification or quirks necesary to
* make the driver work with QEMU. This can be commented when an actual NVME
* hardware device is being tested.
*/
#define QEMU

/**
*  NVME Express Device Structure Parameters.
*  the list of devices is maintained in this stucture
*  using kernel linked list structure list_head
*/
struct nvme_device_entry {
    struct list_head list; /** linked list head pointer */
    struct pci_dev *pdev; /** pointer to pci device */
    int bus; /** bus number of the pci device */
    int slot; /** slot number of this pci device */
    int func; /** function no. of this pci device*/
    struct gendisk *disk; /** if block device then gendisk type */
    u32    *bar;   /**base address 0 for this device */
    struct list_head namespaces; /** list head linked list for namespaces. */
    char   serial[20]; /** Serial no. for the PCI device. */
    char   model[40]; /** Modle no. for this device. */
    char   firmware_rev[8]; /** Firmware revision of NMVE device */
};

#endif /* sysdnvme.h*/
