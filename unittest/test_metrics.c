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
void ioctl_get_q_metrics(int file_desc, int q_id, int q_type, int size)
{
    int ret_val = -1;
    uint16_t tmp;
    struct nvme_get_q_metrics get_q_metrics;

    printf("User App Calling Get Q Metrics...\n");

    get_q_metrics.q_id = q_id;
    get_q_metrics.type = q_type;
    get_q_metrics.nBytes = size;

    if (q_type == 1) {
        get_q_metrics.buffer = malloc(sizeof(uint8_t) *
            sizeof(struct nvme_gen_sq));
    } else {
        get_q_metrics.buffer = malloc(sizeof(uint8_t) *
                    sizeof(struct nvme_gen_cq));
    }

    ret_val = ioctl(file_desc, NVME_IOCTL_GET_Q_METRICS, &get_q_metrics);

    if(ret_val < 0)
        printf("\tQ metrics could not be checked!\n");
    else {
        if (q_type == 1) {
            memcpy(&tmp, &get_q_metrics.buffer[0], sizeof(uint16_t));
            printf("\nMetrics for SQ Id = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[2], sizeof(uint16_t));
            printf("\tCQ Id = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[4], sizeof(uint16_t));
            printf("\tTail Ptr = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[6], sizeof(uint16_t));
            printf("\tTail_Ptr_Virt = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[8], sizeof(uint16_t));
            printf("\tHead Ptr = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[10], sizeof(uint16_t));
            printf("\tElements = %d\n", tmp);
        } else {
            memcpy(&tmp, &get_q_metrics.buffer[0], sizeof(uint16_t));
            printf("\nMetrics for CQ Id = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[2], sizeof(uint16_t));
            printf("\tTail_Ptr = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[4], sizeof(uint16_t));
            printf("\tHead Ptr = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[6], sizeof(uint16_t));
            printf("\tElements = %d\n", tmp);
            memcpy(&tmp, &get_q_metrics.buffer[8], sizeof(uint16_t));
            printf("\tIrq Enabled = %d\n", tmp);
        }
    }

}

void test_drv_metrics(int file_desc)
{
    struct metrics_driver get_drv_metrics;
    int ret_val = -1;

    ret_val = ioctl(file_desc, NVME_IOCTL_GET_DRIVER_METRICS, &get_drv_metrics);
    if (ret_val < 0) {
        printf("\tDrv metrics failed!\n");
    }
    printf("\nDrv Version = 0x%X\n", get_drv_metrics.driver_version);
    printf("Api Version = 0x%X\n", get_drv_metrics.api_version);
}
