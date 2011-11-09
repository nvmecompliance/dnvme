#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <ctype.h>
#include <malloc.h>
#include <stdint.h>
#include <sys/mman.h>

#include "../dnvme_interface.h"
#include "../dnvme_ioctls.h"

#include "test_metrics.h"

struct cq_completion {
        uint32_t cmd_specifc;       /* DW 0 all 32 bits     */
        uint32_t reserved;          /* DW 1 all 32 bits     */
        uint16_t sq_head_ptr;       /* DW 2 lower 16 bits   */
        uint16_t sq_identifier;     /* DW 2 higher 16 bits  */
        uint16_t cmd_identifier;    /* Cmd identifier       */
        uint8_t  phase_bit:1;       /* Phase bit            */
        uint16_t status_field:15;   /* Status field         */
    };

void ioctl_prep_sq(int file_desc, uint16_t sq_id, uint16_t cq_id, uint16_t elem, uint8_t contig)
{
    int ret_val = -1;
    struct nvme_prep_sq prep_sq;

    prep_sq.sq_id = sq_id;
    prep_sq.cq_id = cq_id;
    prep_sq.elements = elem;
    prep_sq.contig = contig;

    printf("\tCalling Prepare SQ Creation...\n");
    printf("\tSQ ID = %d\n", prep_sq.sq_id);
    printf("\tAssoc CQ ID = %d\n", prep_sq.cq_id);
    printf("\tNo. of Elem = %d\n", prep_sq.elements);
    printf("\tContig(Y|N=(1|0)) = %d\n", prep_sq.contig);

    ret_val = ioctl(file_desc, NVME_IOCTL_PREPARE_SQ_CREATION, &prep_sq);

    if(ret_val < 0) {
        printf("\tSQ ID = %d Preparation failed!\n", prep_sq.sq_id);
    } else {
        printf("\tSQ ID = %d Preparation success\n", prep_sq.sq_id);
    }
}

void ioctl_prep_cq(int file_desc, uint16_t cq_id, uint16_t elem, uint8_t contig)
{
    int ret_val = -1;
    struct nvme_prep_cq prep_cq;

    prep_cq.cq_id = cq_id;
    prep_cq.elements = elem;
    prep_cq.contig = contig;

    printf("\tCalling Prepare CQ Creation...\n");
    printf("\tCQ ID = %d\n", prep_cq.cq_id);
    printf("\tNo. of Elem = %d\n", prep_cq.elements);
    printf("\tContig(Y|N=(1|0)) = %d\n", prep_cq.contig);

    ret_val = ioctl(file_desc, NVME_IOCTL_PREPARE_CQ_CREATION, &prep_cq);

    if(ret_val < 0) {
        printf("\tCQ ID = %d Preparation failed!\n", prep_cq.cq_id);
    } else {
        printf("\tCQ ID = %d Preparation success\n", prep_cq.cq_id);
    }
}

void ioctl_reap_inquiry(int file_desc, int cq_id)
{
    int ret_val = -1;
    struct nvme_reap_inquiry rp_inq;

    rp_inq.q_id = cq_id;

    ret_val = ioctl(file_desc, NVME_IOCTL_REAP_INQUIRY, &rp_inq);
    if(ret_val < 0) {
        printf("\nreap inquiry failed!\n");
    }
    else {
        printf("\t\tReaped on CQ ID = %d, Num_Remaining = %d\n",
                rp_inq.q_id, rp_inq.num_remaining);
    }
}

void display_cq_data(unsigned char *cq_buffer, int reap_ele)
{
    struct cq_completion *cq_entry;
    while (reap_ele) {
        cq_entry = (struct cq_completion *)cq_buffer;
        printf("\n\t\tCmd Id = 0x%x", cq_entry->cmd_identifier);
        printf("\n\t\tCmd Spec = 0x%x", cq_entry->cmd_specifc);
        printf("\n\t\tPhase Bit = %d", cq_entry->phase_bit);
        printf("\n\t\tSQ Head Ptr = %d", cq_entry->sq_head_ptr);
        printf("\n\t\tSQ ID = 0x%x", cq_entry->sq_identifier);
        printf("\n\t\tStatus = %d\n", cq_entry->status_field);
        reap_ele--;
        cq_buffer += sizeof(struct cq_completion);
    }
}

void ioctl_reap_cq(int file_desc, int cq_id, int elements, int size, int display)
{
    struct nvme_reap rp_cq;
    int ret_val;

    rp_cq.q_id = cq_id;
    rp_cq.elements = elements;
    rp_cq.size = size; //CE entry size is 16 on CQ
    rp_cq.buffer = malloc(sizeof(char) * rp_cq.size);

    ret_val = ioctl(file_desc, NVME_IOCTL_REAP, &rp_cq);
    if(ret_val < 0) {
        printf("\nreap inquiry failed!\n");
    }
    else {

        printf("\n\tCQ ID = %d, No Request = %d, No Reaped = %d No Rem = %d",
                rp_cq.q_id, rp_cq.elements, rp_cq.num_reaped,
                rp_cq.num_remaining);
        if (display)
            display_cq_data(rp_cq.buffer, rp_cq.num_reaped);
    }
}

void set_admn(int file_desc)
{
    ioctl_disable_ctrl(file_desc, ST_DISABLE);
    ioctl_create_acq(file_desc);
    ioctl_create_asq(file_desc);
    ioctl_enable_ctrl(file_desc);
}

void test_meta(int file_desc, int log)
{
    int size;
    int ret_val;
    int meta_id;
    uint64_t *kadr;
    char *tmpfile12 = "/tmp/file_name12.txt";
    char *tmpfile13 = "/tmp/file_name13.txt";

    size = 4096;
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_CREATE, size);
    if(ret_val < 0) {
        printf("\nMeta data creation failed!\n");
    }
    else {
        printf("Meta Data creation success!!\n");
    }
    getchar();
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_CREATE, size);
    if(ret_val < 0) {
        printf("\nMeta data creation failed!\n");
    }
    else {
        printf("Meta Data creation success!!\n");
    }
    size = 1024 * 16 + 10;
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_CREATE, size);
    if(ret_val < 0) {
        printf("\nMeta data creation failed!\n");
    }
    else {
        printf("Meta Data creation success!!\n");
    }
    getchar();

    for (meta_id = 0; meta_id < 20; meta_id++) {
        ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_ALLOC, meta_id);
        if(ret_val < 0) {
            printf("\nMeta Id = %d allocation failed!\n", meta_id);
        }
        else {
            printf("Meta Id = %d allocation success!!\n", meta_id);
        }
    }
    if (log != 0)
        ioctl_dump(file_desc, tmpfile12);

    meta_id = 5;
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_DELETE, meta_id);
    if(ret_val < 0) {
        printf("\nMeta Id = %d deletion failed!\n", meta_id);
    }
    else {
        printf("Meta Id = %d deletion success!!\n", meta_id);
    }
    meta_id = 6;
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_DELETE, meta_id);
    if(ret_val < 0) {
        printf("\nMeta Id = %d deletion failed!\n", meta_id);
    }
    else {
        printf("Meta Id = %d deletion success!!\n", meta_id);
    }
    meta_id = 6;
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_DELETE, meta_id);
    if(ret_val < 0) {
        printf("\nMeta Id = %d deletion failed!\n", meta_id);
    }
    else {
        printf("Meta Id = %d deletion success!!\n", meta_id);
    }
    meta_id = 5;
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_DELETE, meta_id);
    if(ret_val < 0) {
        printf("\nMeta Id = %d deletion failed!\n", meta_id);
    }
    else {
        printf("Meta Id = %d deletion success!!\n", meta_id);
    }

    meta_id = 6;
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_ALLOC, meta_id);
    if(ret_val < 0) {
        printf("\nMeta Id = %d allocation failed!\n", meta_id);
    }
    else {
        printf("Meta Id = %d allocation success!!\n", meta_id);
    }

    meta_id = 0x80004;
    printf("\nTEST 3.1: Call to Mmap encoded Meta Id = 0x%x\n", meta_id);
    kadr = mmap(0, 4096, PROT_READ, MAP_SHARED, file_desc, 4096 * meta_id);
    if (!kadr) {
        printf("mapping failed\n");
        exit(-1);
    }

    munmap(kadr, 4096);
    if (log != 0)
        ioctl_dump(file_desc, tmpfile13);
}

uint32_t test_meta_buf(int file_desc) {
    int ret_val;
    uint32_t meta_id = 0;
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_CREATE, 4096);
    if(ret_val < 0) {
        printf("\nMeta data creation failed!\n");
    } else {
        printf("Meta Data creation success!!\n");
    }
    ret_val = ioctl(file_desc, NVME_IOCTL_METABUF_ALLOC, meta_id);
    if(ret_val < 0) {
        printf("\nMeta Id = %d allocation failed!\n", meta_id);
    } else {
        printf("Meta Id = %d allocation success!!\n", meta_id);
    }
    return meta_id;
}
