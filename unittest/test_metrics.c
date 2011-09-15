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
/*
 * Functions for the ioctl calls
*/
void ioctl_get_q_metrics(int file_desc, int q_id, int q_type)
{
    int ret_val = -1;
    struct nvme_get_q_metrics get_q_metrics;

    printf("User App Calling Get Q Metrics...\n");

    get_q_metrics.q_id = q_id;
    get_q_metrics.type = q_type;
    get_q_metrics.buffer = malloc(sizeof(uint8_t) *
            sizeof(struct nvme_gen_sq));

    ret_val = ioctl(file_desc, NVME_IOCTL_GET_Q_METRICS, &get_q_metrics);

    if(ret_val < 0)
        printf("Q metrics could not be checked!\n");
    else {
        if (q_type == 1) {
            printf("\nMetrics for SQ Id = %d\n", (uint16_t)get_q_metrics.buffer[0]);
            printf("\tCQ Id = %d\n", (uint16_t)get_q_metrics.buffer[2]);
            printf("\tTail Ptr = %d\n", (uint16_t)get_q_metrics.buffer[4]);
            printf("\tTail_Ptr_Virt = %d\n", (uint16_t)get_q_metrics.buffer[6]);
            printf("\tHead Ptr = %d\n", (uint16_t)get_q_metrics.buffer[8]);
            printf("\tElements = %d\n", (uint16_t)get_q_metrics.buffer[10]);
        } else {
            printf("\nMetrics for CQ Id = %d\n", (uint16_t)get_q_metrics.buffer[0]);
            printf("\tTail_Ptr = %d\n", (uint16_t)get_q_metrics.buffer[2]);
            printf("\tHead Ptr = %d\n", (uint16_t)get_q_metrics.buffer[4]);
            printf("\tElements = %d\n", (uint16_t)get_q_metrics.buffer[6]);
            printf("\tIrq Enabled = %d\n", (uint8_t)get_q_metrics.buffer[8]);
        }
    }

}
