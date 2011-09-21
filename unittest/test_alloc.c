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

void ioctl_alloc_sq(int file_desc, uint16_t sq_id, uint16_t cq_id, uint16_t elem)
{
    int ret_val = -1;
    struct nvme_alloc_contig_sq alloc_sq;

    alloc_sq.sq_id = sq_id;
    alloc_sq.cq_id = cq_id;
    alloc_sq.elements = elem;

    printf("User App Calling Alloc Contig SQ\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_ALLOCATE_CONTIG_SQ, &alloc_sq);

    if(ret_val < 0) {
        printf("SQ Allocation failed!\n");
    } else {
        printf("SQ Allocation success:\n");
    }
}
