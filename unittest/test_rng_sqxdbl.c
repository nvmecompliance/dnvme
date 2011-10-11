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

void ioctl_tst_ring_dbl(int file_desc, int sq_id)
{
    int ret_val = -1;
    struct nvme_ring_sqxtdbl ring_sqxtdbl;

    ring_sqxtdbl.sq_id = sq_id;

    printf("\n\tRequested to Ring Doorbell of SQ ID = %d\n", ring_sqxtdbl.sq_id);

    ret_val = ioctl(file_desc, NVME_IOCTL_RING_SQ_DOORBELL, &ring_sqxtdbl);
    if(ret_val < 0)
        printf("\n\t\tRing Doorbell Failed!\n");
    else
        printf("\n\t\tRing Doorbell SUCCESS\n");

}
