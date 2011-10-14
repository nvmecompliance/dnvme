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

void ioctl_create_prp_more_than_two_page(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    void *addr = (void *) malloc(8200);
    if (addr == NULL) {
        printf("Malloc Failed");
        return;
    }
    user_cmd.q_id = 0;
    user_cmd.cmd_set = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = NULL;
    user_cmd.data_buf_size = 8200; /* more than 2 page */
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;

    printf("User Call to Create PRP more than one page:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Creation of PRP Failed!\n");
    } else {
        printf("PRP Creation SUCCESS\n");
    }
    free(addr);
}

void ioctl_create_prp_less_than_one_page(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    void *addr = (void *) malloc(95);
    if (addr == NULL) {
        printf("Malloc Failed");
        return;
    }
    user_cmd.q_id = 0;
    user_cmd.cmd_set = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = NULL;
    user_cmd.data_buf_size = 95;
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;

    printf("User Call to Create PRP less than one page:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Creation of PRP Failed!\n");
    } else {
        printf("PRP Creation SUCCESS\n");
    }
    free(addr);
}

void ioctl_create_prp_one_page(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    void *addr = (void *) malloc(4096);
    if (addr == NULL) {
        printf("Malloc Failed");
        return;
    }
    user_cmd.q_id = 0;
    user_cmd.cmd_set = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = NULL;
    user_cmd.data_buf_size = 4096;
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;
    printf("User Call to Create PRP single page:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Creation of PRP Failed!\n");
    } else {
        printf("PRP Creation SUCCESS\n");
    }
    free(addr);
}

void ioctl_create_list_of_prp(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    /* 1 page of PRP List filled completley and 1 more page
     * of PRP List containg only one entry */
    void *addr = (void *) malloc((512 * 4096) + 4096);
    if (addr == NULL) {
        printf("Malloc Failed");
        return;
    }
    user_cmd.q_id = 0;
    user_cmd.cmd_set = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = NULL;
    user_cmd.data_buf_size = (512 * 4096) + 4096;
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;
    printf("User Call to Create Lists of PRP's\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Creation of PRP Failed!\n");
    } else {
        printf("PRP Creation SUCCESS\n");
    }
    free(addr);
}

void ioctl_create_fill_list_of_prp(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    /* 2 pages of PRP Lists filled completley */
    void *addr = (void *) malloc(1023 * 4096);
    if (addr == NULL) {
        printf("Malloc Failed");
        return;
    }
    user_cmd.q_id = 0;
    user_cmd.cmd_set = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = NULL;
    user_cmd.data_buf_size = 1023 * 4096;
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;
    printf("User Call to Create Lists of PRP's\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Creation of PRP Failed!\n");
    } else {
        printf("PRP Creation SUCCESS\n");
    }
    free(addr);
}

void ioctl_create_discontig_ioq(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    struct nvme_create_sq create_sq_cmd;
    /* 2 pages of PRP Lists filled completley */
    void *addr = (void *) malloc(4 * 4096);
    if (addr == NULL) {
        printf("Malloc Failed");
        return;
    }

    /* Fill the command (random values put) */
    create_sq_cmd.opcode = 0x01;
    create_sq_cmd.sqid = 0x01;

    /* Fill the user command */
    user_cmd.q_id = 0;
    user_cmd.bit_mask = 1;
    user_cmd.cmd_buf_ptr = (u_int8_t *) &create_sq_cmd;
    user_cmd.data_buf_size = 4 * 4096;
    user_cmd.data_buf_ptr = addr;
    user_cmd.meta_buf_size = 0;
    user_cmd.meta_buf_ptr = NULL;
    user_cmd.cmd_set = CMD_ADMIN;
    user_cmd.data_dir = 1;

    printf("User Call to Create Lists of PRP's\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Creation of PRP Failed!\n");
    } else {
        printf("PRP Creation SUCCESS\n");
    }
    free(addr);
}
