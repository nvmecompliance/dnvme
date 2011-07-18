#ifndef _DNVME_H_
#define _DNVME_H_

#define LEVEL	"dnvme"
#define LOG_NORM(msg, ...)   \
    fprintf(stdout, "%s: %s\n", LEVEL, msg);
#define LOG_ERR(msg)    \
    fprintf(stderr, "%s-ERR:%s:%d:"msg"\n", LEVEL, __FILE__, __LINE__##__VA_ARGS__);
#ifdef DEBUG
#define LOG_DBG(msg)    \
    fprintf(stderr, "%s-DBG:__FILE__:__LINE__: %s\n", LEVEL, msg);
#else
#define LOG_DBG(msg);
#endif

/*!
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

#endif
