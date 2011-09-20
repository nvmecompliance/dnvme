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

#include "../dnvme/dnvme_interface.h"
#include "../dnvme/dnvme_ioctls.h"

#define DEVICE_FILE_NAME "/dev/qnvme0"

void ioctl_create_prp_more_than_one_page(int file_desc)
{
    int ret_val = -1;
    
    struct nvme_64b_send user_cmd;
    void * addr = (void *) malloc(sizeof(char) * 8200);
    if(addr == NULL) {
        printf("Malloc Failed");
        return;
    }
    user_cmd.queue_id =0;
    user_cmd.cmd_set = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = NULL;
    user_cmd.data_buf_size = 8200; //more than 2 page
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;
    

    printf("User Call to Create PRP more than one page:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if(ret_val < 0)
        printf("Creation of PRP Failed!\n");
    else
        printf("PRP Creation SUCCESS\n");

    free(addr);
}

void ioctl_create_prp_less_than_one_page(int file_desc)
{
    int ret_val = -1;
    
    struct nvme_64b_send user_cmd;
    void * addr = (void *) malloc(sizeof(char) * 95);
    if(addr == NULL) {
        printf("Malloc Failed");
        return;
    }
    user_cmd.queue_id =0;
    user_cmd.cmd_set = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = NULL;
    user_cmd.data_buf_size = 95;
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;
    

    printf("User Call to Create PRP less than one page:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if(ret_val < 0)
        printf("Creation of PRP Failed!\n");
    else
        printf("PRP Creation SUCCESS\n");

    free(addr);
}

void ioctl_create_prp_one_page(int file_desc)
{
    int ret_val = -1;
    
    struct nvme_64b_send user_cmd;
    void * addr = (void *) malloc(sizeof(char) * 4096);
    if(addr == NULL) {
        printf("Malloc Failed");
        return;
    }
    user_cmd.queue_id =0;
    user_cmd.cmd_set = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = NULL;
    user_cmd.data_buf_size = 4096;
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;
    

    printf("User Call to Create PRP single page:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if(ret_val < 0)
        printf("Creation of PRP Failed!\n");
    else
        printf("PRP Creation SUCCESS\n");

    free(addr);
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

    ioctl_create_prp_one_page(file_desc);
    ioctl_create_prp_less_than_one_page(file_desc);
    ioctl_create_prp_more_than_one_page(file_desc);
    close(file_desc);
    return 0;
}
