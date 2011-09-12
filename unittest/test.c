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

#define _TNVME_H_

#include "../dnvme_interface.h"
#include "../dnvme_ioctls.h"

#include "test_metrics.h"

#define DEVICE_FILE_NAME "/dev/qnvme0"

/* 
 * Functions for the ioctl calls 
*/
void ioctl_read_data(int file_desc)
{
    int ret_val;
    int i;
    struct rw_generic test_data;

    test_data.type = NVMEIO_BAR01;
    test_data.offset = 8;
    test_data.nBytes = 0x4;
    test_data.acc_type = BYTE_LEN;

    test_data.buffer = malloc(sizeof(char) * test_data.nBytes);

    printf("Reading Test Application...\n");
    ret_val = ioctl(file_desc, NVME_IOCTL_READ_GENERIC, &test_data);

        if (ret_val < 0) {
            printf("ioctl_set_msg failed:%d\n", ret_val);
            exit(-1);
        }

    i = 0;

    printf("Reading::offset:Data\n");
    while (test_data.nBytes) {
        printf("0x%x:0x%x\n", i, test_data.buffer[i]);
    i++;
    test_data.nBytes--;
    }

}

/* 
 * Functions for the ioctl calls 
*/
void ioctl_write_data(int file_desc)
{
    int ret_val;
    //int message;
    struct rw_generic test_data;

    test_data.type = NVMEIO_BAR01;
    test_data.offset = 0x8;
    test_data.nBytes = 4;
    test_data.acc_type = DWORD_LEN;
    test_data.buffer[0] = 0xaa;
    test_data.buffer[1] = 0xbb;
    test_data.buffer[2] = 0xcc;
    test_data.buffer[3] = 0xdd;
    test_data.buffer[4] = 0xee;
    test_data.buffer[5] = 0xbb;
    test_data.buffer[6] = 0xcc;
    test_data.buffer[7] = 0xdd;


    printf("writing Test Application...\n");
    ret_val = ioctl(file_desc, NVME_IOCTL_WRITE_GENERIC, &test_data);

    if (ret_val < 0) {
            printf("ioctl_set_msg failed:%d\n", ret_val);
            exit(-1);
   }
}

void ioctl_create_acq(int file_desc)
    {
    int ret_val = -1;
    struct nvme_acq_gen acq_data;

    acq_data.acq_size = 0x10;

    printf("User Call to Create Admin Completion Q:\n");
    printf("User Admin CQ Size = 0x%x\n", acq_data.acq_size);

    ret_val = ioctl(file_desc, NVME_IOCTL_CREATE_ADMN_CQ, &acq_data);
    if(ret_val < 0)
        printf("Creation of ACQ Failed!\n");
    else
        printf("ACQ Creation SUCCESS\n");
}

void ioctl_create_asq(int file_desc)
{
    int ret_val = -1;
    struct nvme_asq_gen asq_data;

    asq_data.asq_size = 0x50;

    printf("User Call to Create Admin Submission Q:\n");
    printf("User Admin SQ Size = 0x%x\n", asq_data.asq_size);

    ret_val = ioctl(file_desc, NVME_IOCTL_CREATE_ADMN_SQ, &asq_data);
    if(ret_val < 0)
        printf("Creation of ASQ Failed!\n");
    else
        printf("ASQ Creation SUCCESS\n");
}

void ioctl_check_device(int file_desc)
{
    int ret_val = -1;
    int status = -1;

    printf("Inside checking device\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_ERR_CHK, &status);
    if(status < 0)
        printf("Device Status FAILED!\n");
    else
        printf("Device Status SUCCESS\n");
}

void ioctl_enable_ctrl(int file_desc)
{
    int ret_val = -1;
    struct nvme_ctrl_enum ctrl_data;

    ctrl_data.nvme_status = NVME_CTLR_ENABLE;

    printf("User Call to Enable Ctrlr:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_CTLR_STATE, &ctrl_data);
       if(ret_val < 0)
        printf("enable Failed!\n");
    else
        printf("enable SUCCESS\n");
}

void ioctl_disable_ctrl(int file_desc)
{
    int ret_val = -1;
    struct nvme_ctrl_enum ctrl_data;

    ctrl_data.nvme_status = NVME_CTLR_DISABLE;

    printf("User Call to Disable Ctrlr:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_CTLR_STATE, &ctrl_data);
       if(ret_val < 0)
        printf("Diable Failed!\n");
    else
        printf("Disable SUCCESS\n");
}

int main(void)
{
    int file_desc;

    printf("Ensure you have permissions to device..\n\
    else \n do \"chmod 777 /dev/qnvme0\" \n");
    printf("Starting Test Application...\n");

    file_desc = open(DEVICE_FILE_NAME, 0);
    if (file_desc < 0) {
        printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
        exit(-1);
    }

    printf("Device File Succesfully Opened = %d\n", file_desc);

    //ioctl_check_device(file_desc);
    //ioctl_read_data(file_desc);
    //ioctl_write_data(file_desc);
    //ioctl_read_data(file_desc);
    //ioctl_check_device(file_desc);
    //ioctl_disable_ctrl(file_desc);
    //ioctl_create_acq(file_desc);
    //ioctl_create_asq(file_desc);
    //ioctl_enable_ctrl(file_desc);
    ioctl_get_q_metrics(file_desc);

    close(file_desc);
    return 0;
}
