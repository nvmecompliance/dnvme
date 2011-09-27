#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/types.h>

#include "sysdnvme.h"
#include "definitions.h"
#include "dnvme_reg.h"
#include "dnvme_cmds.h"

/* Declaration of static functions belonging to Submitting 64Bytes Command */
static int pages_to_sg(struct page **,
    __u32, __u32, __u32, struct scatterlist **);
static int map_user_pg_to_dma(struct nvme_dev_entry *, __u8,
    unsigned long, __u32, struct scatterlist **);
static void unmap_user_pg_to_dma(struct nvme_dev_entry *, __u8,
    unsigned long, __u32, struct scatterlist *);
static int setup_prps(struct nvme_dev_entry *, struct scatterlist *,
    __s32, struct nvme_prps *, __u8);
static void free_prp_pool(struct nvme_dev_entry *, struct nvme_prps *, __u32);

/*
 * submit_command:
 * Entry point for Submitting 64Bytes Command
 * TODO:
 * Add the implementation logic of complete ioctl and update the
 * function arguments accordingly
 */
int submit_command(struct nvme_dev_entry *nvme_dev, __u16 q_id,
    __u8 *buf_addr, __u32 buf_len)
{
    int err; /* Error code return values */
    struct scatterlist *sg; /* Pointer to SG List */
    struct nvme_prps prps; /* Pointer to PRP List */
    unsigned long addr; /* Buf Addr typecasted to unsigned long */

#ifdef TEST_PRP_DEBUG
    int last_prp, i, j;
    __le64 *prp_vlist;
    __s32 num_prps;
#endif

    /* Buf Addr typecasted to unsigned long */
    addr = (unsigned long) buf_addr;

    /* buf_addr not 4 bytes aligned or Invalid Args */
    if ((addr & 3) || (!buf_len) || (NULL == nvme_dev) || (!addr)) {
        LOG_ERR("Invalid Arguments");
        return -EINVAL;
    }

    /* Mapping user pages to dma memory */
    err = map_user_pg_to_dma(nvme_dev, WRITE_DEV, addr, buf_len, &sg);
    if (err < 0) {
        return err;
    }

    /* Setting up PRP's */
    /* TODO change function arguments while implementing command support */
    err = setup_prps(nvme_dev, sg, buf_len, &prps, DISCONTG_IO_Q);
    if (err < 0) {
        /* Unampping PRP's and User pages */
        unmap_user_pg_to_dma(nvme_dev, WRITE_DEV, addr, buf_len, sg);
        return err;
    }

#ifdef TEST_PRP_DEBUG
    last_prp = PAGE_SIZE / PRP_Size - 1;

    num_prps = DIV_ROUND_UP(buf_len, PAGE_SIZE);

    if (prps.type == (PRP1 | PRP_List) || prps.type == (PRP2 | PRP_List)) {
        if (!(prps.v_list)) {
            LOG_ERR("Creation of PRP failed");
            return -ENOMEM;
        }
        prp_vlist = prps.v_list[0];
        if (prps.type == (PRP2 | PRP_List)) {
            LOG_DBG("P1 Entry: %llx", (unsigned long long) prps.prp1);
        }
        for (i = 0, j = 0; i < num_prps - 1; i++) {

            if (j < (prps.npages - 1) && i == last_prp) {
                j++;
                num_prps -= i;
                i = 0 ;
                prp_vlist = prps.v_list[j];
                LOG_NRM("Physical address of next PRP Page: %llx",
                    (__le64) prp_vlist);
            }

            LOG_DBG("PRP List: %llx", (unsigned long long) prp_vlist[i]);
        }

    } else if (prps.type == PRP1) {
        LOG_DBG("P1 Entry: %llx", (unsigned long long) prps.prp1);
    } else {
        LOG_DBG("P1 Entry: %llx", (unsigned long long) prps.prp1);
        LOG_DBG("P2 Entry: %llx", (unsigned long long) prps.prp2);
    }
#endif

    /* Unampping PRP's and User pages */
    unmap_user_pg_to_dma(nvme_dev, WRITE_DEV, addr, buf_len, sg);
    /* TODO remove the last function arg */
    free_prp_pool(nvme_dev, &prps, prps.npages);
    return 0;
}

/*
 * destroy_dma_pool:
 * Destroy's the dma pool
 * Returns void
 */
void destroy_dma_pool(struct nvme_dev_entry *nvme_dev)
{
    /* Destroy the DMA pool */
    dma_pool_destroy(nvme_dev->prp_page_pool);
}

/*
 * map_user_pg_to_dma:
 * Maps User pages to DMA via SG List
 * Returns Error codes or number of physical segments mapped
 */
static int map_user_pg_to_dma(struct nvme_dev_entry *nvme_dev, __u8 write,
    unsigned long buf_addr, __u32 buf_len, struct scatterlist **sgp)
{
    __u32 offset, count; /* Offset inside Page, No. of pages */
    struct page **pages; /* List of pointers to user space pages */
    int err, index; /* Err code return values, index for loop */

#ifdef TEST_PRP_DEBUG
    struct scatterlist *sg_test;
#endif
    offset = offset_in_page(buf_addr);

    /* TODO: Use CC.MPS instead of PAGE_SIZE */
    count = DIV_ROUND_UP(offset + buf_len, PAGE_SIZE);

    /* Allocating conitguous memory for pointer to pages */
    pages = kcalloc(count, sizeof(*pages), GFP_KERNEL);
    if (NULL == pages) {
        LOG_ERR("Memory allocation for pages failed");
        return -ENOMEM;
    }

    /* Pinning user pages in memory */
    err = get_user_pages_fast(buf_addr, count, WRITE_PG, pages);
    if (err < count) {
        count = err;
        err = -EFAULT;
        LOG_ERR("Pinning user pages in memory failed");
        /* Clean up and Return Error Code */
        goto error;
    }

    /* Generate SG List from pinned down pages */
    err = pages_to_sg(pages, count, offset, buf_len, sgp);
    if (err < 0) {
        LOG_ERR("Generation of SG Lists failed");
        /* Clean up and Return Error Code */
        goto error;
    }

#ifdef TEST_PRP_DEBUG
    for (index = 0, sg_test = *sgp; index < count;
        index++, sg_test = sg_next(sg_test)) {
        LOG_DBG("SG Page offset: %u, SG Page len: %u",
            sg_test->offset, sg_test->length);
    }
#endif

    /* Mapping SG List to DMA */
    err = dma_map_sg(&nvme_dev->pdev->dev, *sgp, count,
                write ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
    if (!err) {
        LOG_ERR("Mapping of SG List failed");
        err = -ENOMEM;
        goto error;
    }
    kfree(pages);
    return err;

error:
    LOG_ERR("Error in map_user_pg_to_dma function: %d", err);
    for (index = 0; index < count; index++) {
        put_page(pages[index]);
    }
    /*Freeing the list of pointers */
    kfree(pages);
    return err;
}
/*
 * pages_to_sg:
 * Create a scatterlist from an array of page pointers.
 * Returns Error Codes
 */
static int pages_to_sg(struct page **pages,
    __u32 nr_pages, __u32 offset, __u32 len, struct scatterlist **sglstp)
{
    int index;
    struct scatterlist *sglist;

    /* Allocating conitguous memory for SG List */
    sglist = kcalloc(nr_pages, sizeof(*sglist), GFP_KERNEL);
    if (NULL == sglist) {
        LOG_ERR("Memory allocation for SG List failed");
        return -ENOMEM;
    }
    sg_init_table(sglist, nr_pages);

    /* Building the SG List */
    /* TODO: Use CC.MPS instead of PAGE_SIZE */
    for (index = 0; index < nr_pages; index++) {
        if (NULL == pages[index]) {
            kfree(sglist);
            return -EFAULT;
        }
        sg_set_page(&sglist[index], pages[index],
            min_t(unsigned long, PAGE_SIZE - offset, len), offset);
        len -= (PAGE_SIZE - offset);
        offset = 0;
    }
    *sglstp = sglist;
    return 0;
}

/*
 * unmap_user_pg_to_dma:
 * Unmaps mapped DMA pages and frees the pinned down pages
 */
static void unmap_user_pg_to_dma(struct nvme_dev_entry *dev, __u8 write,
    unsigned long buf_addr, __u32 buf_len, struct scatterlist *sg)
{
    int i, count;
    count = DIV_ROUND_UP(offset_in_page(buf_addr) + buf_len, PAGE_SIZE);
    dma_unmap_sg(&dev->pdev->dev, sg, count,
        write ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

    for (i = 0; i < count; i++) {
        put_page(sg_page(&sg[i]));
    }
    kfree(sg);
}

/*
 * setup_prps:
 * Sets up PRP'sfrom DMA'ed memory
 * Returns Error codes
 * TODO: Handle Create IO CQ/SQ case
 */
static int setup_prps(struct nvme_dev_entry *nvme_dev, struct scatterlist *sg,
    __s32 buf_len, struct nvme_prps *prps, __u8 cr_io_q)
{
    dma_addr_t prp_dma, dma_addr;
    __s32 dma_len; /* Length of DMA'ed SG */
    __le64 *prp_list; /* Pointer to PRP List */
    __u32 offset;
    __u32 num_prps, num_pg, prp_page;
    int index, err;
    struct dma_pool *prp_page_pool;

    dma_addr = sg_dma_address(sg);
    dma_len = sg_dma_len(sg);
    offset = offset_in_page(dma_addr);

    /* Create IO CQ/SQ's */
    if (cr_io_q) {
        /* Specifies PRP1 entry is a PRP_List */
        prps->type = (PRP1 | PRP_List);
        goto prp_list;
    }

    LOG_DBG("PRP1 Entry: Buf_len %d", buf_len);
    LOG_DBG("PRP1 Entry: dma_len %u", dma_len);
    LOG_DBG("PRP1 Entry: PRP entry %llx", (unsigned long long) dma_addr);

    prps->prp1 = cpu_to_le64(dma_addr);
    buf_len -= (PAGE_SIZE - offset);
    if (buf_len <= 0) {
        prps->type = PRP1;
        return 0;
    }

    dma_len -= (PAGE_SIZE - offset);


    /* If pages were contiguous in memory use same SG Entry */
    if (dma_len) {
        dma_addr += (PAGE_SIZE - offset);
    } else {
        sg = sg_next(sg);
        dma_addr = sg_dma_address(sg);
        dma_len = sg_dma_len(sg);
    }

    if (buf_len <= PAGE_SIZE) {
        prps->prp2 = cpu_to_le64(dma_addr);
        prps->type = (PRP1 | PRP2);
        LOG_DBG("PRP2 Entry: Type %u", prps->type);
        LOG_DBG("PRP2 Entry: Buf_len %d", buf_len);
        LOG_DBG("PRP2 Entry: dma_len %u", dma_len);
        LOG_DBG("PRP2 Entry: PRP entry %llx", (unsigned long long) dma_addr);
        return 0;
    }

    /* Specifies PRP2 entry is a PRP_List */
    prps->type = (PRP2 | PRP_List);

prp_list:
    /* Generate PRP List */
    num_prps = DIV_ROUND_UP(buf_len, PAGE_SIZE);
    /* Taking into account the last entry of PRP Page */
    num_pg = DIV_ROUND_UP(PRP_Size * num_prps, PAGE_SIZE - PRP_Size);

    prps->v_list = kmalloc(sizeof(__le64 *) *num_pg, GFP_ATOMIC);
    if (NULL == prps) {
        LOG_ERR("Memory allocation for virtual list failed");
        return -ENOMEM;
    }

    LOG_NRM("No. of PRP Entries inside PRPList: %u", num_prps);

    prp_page = 0;
    prp_page_pool = nvme_dev->prp_page_pool;

    prp_list = dma_pool_alloc(prp_page_pool, GFP_ATOMIC, &prp_dma);
    if (NULL == prp_list) {
        LOG_ERR("Memory allocation for prp page failed");
        return -ENOMEM;
    }
    prps->v_list[prp_page++] = prp_list;
    prps->first_dma = prp_dma;
    if (prps->type == (PRP2 | PRP_List)) {
        /* TODO: Verify Non-Zero offset */
        prps->prp2 = cpu_to_le64(prp_dma);
        LOG_DBG("PRP2 Entry: %llx", (unsigned long long) prps->prp2);
    } else if (prps->type == (PRP1 | PRP_List)) {
        /* TODO: Verify Zero offset */
        prps->prp1 = cpu_to_le64(prp_dma);
        LOG_DBG("PRP1 Entry: %llx", (unsigned long long) prps->prp1);
    } else {
        err = -EFAULT;
        goto error;
    }

    index = 0;
    for (;;) {
        if ((index == PAGE_SIZE / PRP_Size - 1) && (buf_len > PAGE_SIZE)) {
            __le64 *old_prp_list = prp_list;
            prp_list = dma_pool_alloc(prp_page_pool, GFP_ATOMIC, &prp_dma);
            if (NULL == prp_list) {
                LOG_ERR("Memory allocation for prp page failed");
                err = -ENOMEM;
                goto error;
            }
            prps->v_list[prp_page++] = prp_list;
            old_prp_list[index] = cpu_to_le64(prp_dma);
            index = 0;
        }
        prp_list[index++] = cpu_to_le64(dma_addr);
        dma_len -= PAGE_SIZE;
        dma_addr += PAGE_SIZE;
        buf_len -= PAGE_SIZE;

        LOG_DBG("PRP List: Buf_len %d", buf_len);
        LOG_DBG("PRP List: dma_len %u", dma_len);
        LOG_DBG("PRP List: PRP entry %llx",
            (unsigned long long) (dma_addr - PAGE_SIZE));

        if (buf_len <= 0) {
            LOG_DBG("No. of PRP Pages: %u", prp_page);
            prps->npages = prp_page;
            break;
        }
        if (dma_len > 0) {
            continue;
        } else if (dma_len < 0) {
            err = -EFAULT;
            goto error;
        } else {
            sg = sg_next(sg);
            dma_addr = sg_dma_address(sg);
            dma_len = sg_dma_len(sg);
        }

    }
    return 0;

error:
    LOG_ERR("Error in setup_prps function: %d", err);
    free_prp_pool(nvme_dev, prps, prp_page);
    return err;
}

/*
 * free_prp_pool:
 * Free's PRP List and virtual List
 */
static void free_prp_pool(struct nvme_dev_entry *dev,
    struct nvme_prps *prps, __u32 npages)
{
    const int last_prp = PAGE_SIZE / PRP_Size - 1;
    int i;
    dma_addr_t prp_dma, next_prp_dma = 0;
    __le64 *prp_vlist;

    if (!prps) {
        return;
    }

    if (prps->type == (PRP1 | PRP_List) || prps->type == (PRP2 | PRP_List)) {
        prp_dma = prps->first_dma;

        for (i = 0; i < npages; i++) {
            prp_vlist = prps->v_list[i];
            if (i < (npages - 1)) {
                next_prp_dma = le64_to_cpu(prp_vlist[last_prp]);
            }
            dma_pool_free(dev->prp_page_pool, prp_vlist, prp_dma);
            prp_dma = next_prp_dma;
        }
        kfree(prps->v_list);
    }
}
