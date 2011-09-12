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
void ioctl_get_q_metrics(int file_desc)
{
    int ret_val = -1;
    int status = -1;

    printf("User App Calling Get Q Metrics...");

    ret_val = ioctl(file_desc, NVME_IOCTL_GET_Q_METRICS, &status);
    if(status < 0)
        printf("Q metrics could not be checked!\n");
    else
        printf("Q metrics checked see the log msgs\n");
}
