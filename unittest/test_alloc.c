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

#include "../dnvme_interface.h"
#include "../dnvme_ioctls.h"

#include "test_metrics.h"

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
        printf("reap inquiry failed!\n");
    }
    else {
        printf("Reaped Cq = %d Successfully, num_remaining = %d\n",
                rp_inq.q_id, rp_inq.num_remaining);
    }
}
