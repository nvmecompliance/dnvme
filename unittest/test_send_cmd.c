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
#include <stdlib.h>

#define _TNVME_H_

#include "../dnvme_interface.h"
#include "../dnvme_ioctls.h"
#include "test_send_cmd.h"
#include "test_metrics.h"

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

    printf("User Call to Create Lists of PRP's\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Creation of PRP Failed!\n");
    } else {
        printf("PRP Creation SUCCESS\n");
    }
    free(addr);
}

/* CMD to create discontig IOSQueue spanning multiple pages of PRP lists*/
void ioctl_create_discontig_iosq(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    struct nvme_create_sq create_sq_cmd;

    /* Maximum possible entries */
    void *addr = (void *) malloc(1023 * 4096);
    if (addr == NULL) {
        printf("Malloc Failed");
        return;
    }

    /* Fill the command for create discontig IOSQ*/
    create_sq_cmd.opcode = 0x01;
    create_sq_cmd.sqid = 0x01;
    create_sq_cmd.qsize = 65472;
    create_sq_cmd.cqid = 0x01;
    create_sq_cmd.sq_flags = 0x00;

    /* Fill the user command */
    user_cmd.q_id = 0;
    user_cmd.bit_mask = MASK_PRP1_LIST;
    user_cmd.cmd_buf_ptr = (u_int8_t *) &create_sq_cmd;
    user_cmd.data_buf_size = 1023 * 4096;
    user_cmd.data_buf_ptr = addr;

    user_cmd.cmd_set = CMD_ADMIN;
    user_cmd.data_dir = 1;

    printf("User Call to send command\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Sending of Command Failed!\n");
    } else {
        printf("Command sent succesfully\n");
    }
    free(addr);
}

/* CMD to delete IO Queue */
void ioctl_delete_ioq(int file_desc, uint8_t opcode, uint16_t qid)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    struct nvme_del_q del_q_cmd;
    /* Fill the command for create discontig IOSQ*/
    del_q_cmd.opcode = opcode;
    del_q_cmd.qid = qid;

    /* Fill the user command */
    user_cmd.q_id = 0;
    user_cmd.bit_mask = 0;
    user_cmd.cmd_buf_ptr = (u_int8_t *) &del_q_cmd;
    user_cmd.data_buf_size = 0;
    user_cmd.data_buf_ptr = NULL;

    user_cmd.cmd_set = CMD_ADMIN;
    user_cmd.data_dir = 1;

    printf("User Call to send command\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Sending of Command Failed!\n");
    } else {
        printf("Command sent succesfully\n");
    }
}

/* CMD to create contig IOSQueue */
void ioctl_create_contig_iosq(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    struct nvme_create_sq create_sq_cmd;

    /* Fill the command for create discontig IOSQ*/
    create_sq_cmd.opcode = 0x01;
    create_sq_cmd.sqid = 0x02;
    create_sq_cmd.qsize = 256;
    create_sq_cmd.cqid = 0x01;
    create_sq_cmd.sq_flags = 0x01;

    /* Fill the user command */
    user_cmd.q_id = 0;
    user_cmd.bit_mask = (MASK_PRP1_PAGE);
    user_cmd.cmd_buf_ptr = (u_int8_t *) &create_sq_cmd;
    user_cmd.data_buf_size = 0;
    user_cmd.data_buf_ptr = NULL;

    user_cmd.cmd_set = CMD_ADMIN;
    user_cmd.data_dir = 1;

    printf("User Call to send command\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Sending of Command Failed!\n");
    } else {
        printf("Command sent succesfully\n");
    }
}

/* CMD to create contig IOCQueue spanning multiple pages of PRP lists*/
void ioctl_create_contig_iocq(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    struct nvme_create_cq create_cq_cmd;

    /* Fill the command for create discontig IOSQ*/
    create_cq_cmd.opcode = 0x05;
    create_cq_cmd.cqid = 0x01;
    create_cq_cmd.qsize = 20;
    create_cq_cmd.cq_flags = 0x01;

    /* Fill the user command */
    user_cmd.q_id = 0;
    user_cmd.bit_mask = (MASK_PRP1_PAGE);
    user_cmd.cmd_buf_ptr = (u_int8_t *) &create_cq_cmd;
    user_cmd.data_buf_size = 0;
    user_cmd.data_buf_ptr = NULL;

    user_cmd.cmd_set = CMD_ADMIN;
    user_cmd.data_dir = 0;

    printf("User Call to send command\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Sending of Command Failed!\n");
    } else {
        printf("Command sent succesfully\n");
    }
}

/* CMD to send Identify command*/
void ioctl_send_identify_cmd(int file_desc, void* addr)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    struct nvme_identify nvme_identify;

    /* Writing 0's to first page */
    memset(addr, 0, READ_BUFFER_SIZE/2);

    /* Fill the command for create discontig IOSQ*/
    nvme_identify.opcode = 0x06;
    nvme_identify.nsid = 0;
    nvme_identify.cns = 1;

    /* Fill the user command */
    user_cmd.q_id = 0;
    user_cmd.bit_mask = (MASK_PRP1_PAGE | MASK_PRP2_PAGE);
    user_cmd.cmd_buf_ptr = (u_int8_t *) &nvme_identify;
    user_cmd.data_buf_size = 4096;
    user_cmd.data_buf_ptr = addr;

    user_cmd.cmd_set = CMD_ADMIN;
    user_cmd.data_dir = 0;

    printf("User Call to send command\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Sending of Command Failed!\n");
    } else {
        printf("Command sent succesfully\n");
    }
}

/* CMD to send NVME IO write command */
void ioctl_send_nvme_write(int file_desc)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    struct nvme_user_io nvme_write;
    void *addr;

    if (posix_memalign(&addr, 4096, READ_BUFFER_SIZE)) {
        printf("Memalign Failed");
        return;
    }

    /* Writing 1's to first page */
    memset(addr, 1, READ_BUFFER_SIZE/2);
    /* Writing 2's to second page */
    memset((addr + 4096), 2, READ_BUFFER_SIZE/2);

    /* Fill the command for create discontig IOSQ*/
    nvme_write.opcode = 0x01;
    nvme_write.flags = 0;
    nvme_write.control = 0;
    nvme_write.nsid = 0;
    nvme_write.metadata = 0;
    nvme_write.slba = 0;
    nvme_write.nlb = 15;
    nvme_write.cmd_flags = 0;
    nvme_write.dsm = 0;
    nvme_write.ilbrt = 0;
    nvme_write.lbat = 0;
    nvme_write.lbatm = 0;

    /* Fill the user command */
    user_cmd.q_id = 1;
    user_cmd.bit_mask = (MASK_PRP1_PAGE | MASK_PRP1_LIST |
        MASK_PRP2_PAGE | MASK_PRP2_LIST);
    user_cmd.cmd_buf_ptr = (u_int8_t *) &nvme_write;
    user_cmd.data_buf_size = READ_BUFFER_SIZE;
    user_cmd.data_buf_ptr = addr;

    user_cmd.cmd_set = CMD_NVME;
    user_cmd.data_dir = 1;

    printf("User Call to send command\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Sending of Command Failed!\n");
    } else {
        printf("Command sent succesfully\n");
    }

    free(addr);
}

/* CMD to send NVME IO write command */
void ioctl_send_nvme_read(int file_desc, void* addr)
{
    int ret_val = -1;
    struct nvme_64b_send user_cmd;
    struct nvme_user_io nvme_read;

    /* Fill the command for create discontig IOSQ*/
    nvme_read.opcode = 0x02;
    nvme_read.flags = 0;
    nvme_read.control = 0;
    nvme_read.nsid = 0;
    nvme_read.metadata = 0;
    nvme_read.slba = 0;
    nvme_read.nlb = 15;
    nvme_read.cmd_flags = 0;
    nvme_read.dsm = 0;
    nvme_read.ilbrt = 0;
    nvme_read.lbat = 0;
    nvme_read.lbatm = 0;

    /* Fill the user command */
    user_cmd.q_id = 1;
    user_cmd.bit_mask = (MASK_PRP1_PAGE | MASK_PRP1_LIST |
        MASK_PRP2_PAGE | MASK_PRP2_LIST);
    user_cmd.cmd_buf_ptr = (u_int8_t *) &nvme_read;
    user_cmd.data_buf_size = READ_BUFFER_SIZE;
    user_cmd.data_buf_ptr = addr;

    user_cmd.cmd_set = CMD_NVME;
    user_cmd.data_dir = 0;

    printf("User Call to send command\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
    if (ret_val < 0) {
        printf("Sending of Command Failed!\n");
    } else {
        printf("Command sent succesfully\n");
    }

}
