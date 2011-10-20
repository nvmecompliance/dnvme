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

void ioctl_create_acq(int file_desc)
{
    int ret_val = -1;
    struct nvme_create_admn_q aq_data;

    aq_data.elements = 30;
    aq_data.type = ADMIN_CQ;

    //printf("\tUser Call to Create Admin Q:\n");
    printf("\tACQ No. of Elements = %d\n", aq_data.elements);

    ret_val = ioctl(file_desc, NVME_IOCTL_CREATE_ADMN_Q, &aq_data);
    if(ret_val < 0)
        printf("\tCreation of ACQ Failed!\n");
    else
        printf("\tACQ Creation SUCCESS\n");
}

void ioctl_create_asq(int file_desc)
{
    int ret_val = -1;
    struct nvme_create_admn_q aq_data;

    aq_data.elements = 15;
    aq_data.type = ADMIN_SQ;

    //printf("User Call to Create Admin SQ:\n");
    printf("\tAdmin SQ No. of Elements = %d\n", aq_data.elements);

    ret_val = ioctl(file_desc, NVME_IOCTL_CREATE_ADMN_Q, &aq_data);
    if(ret_val < 0)
        printf("\tCreation of ASQ Failed!\n");
    else
        printf("\tASQ Creation SUCCESS\n");
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
    struct nvme_ctrl_state ctrl_data;

    ctrl_data.new_state = ST_ENABLE;

    printf("User Call to Enable Ctrlr:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_DEVICE_STATE, &ctrl_data);
       if(ret_val < 0)
        printf("enable Failed!\n");
    else
        printf("enable SUCCESS\n");
}

void ioctl_disable_ctrl(int file_desc, enum nvme_state new_state)
{
    int ret_val = -1;
    struct nvme_ctrl_state ctrl_data;

    ctrl_data.new_state = new_state;

    printf("User Call to Disable Ctrlr:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_DEVICE_STATE, &ctrl_data);
       if(ret_val < 0)
        printf("Disable Failed!\n");
    else
        printf("Disable SUCCESS\n");
}

void test_admin(int file_desc)
{
    /* Test Case 1 */
    printf("\nTEST 1: Create Admin CQ...\n");
    ioctl_create_acq(file_desc);
    printf("\nTEST 2: Create Admin SQ...\n");
    ioctl_create_asq(file_desc);
}

void test_prep_sq(int file_desc)
{
    printf("\nTEST 3: Allocating SQ 1 to 3 with different sizes...\n");
    printf("\nTEST 3: Contiguous SQ Case...\n");
    printf("\n\tSD_ID : CQ ID = 1 : 1\n");
    ioctl_prep_sq(file_desc, 1, 1, 256, 0);
#if 0
    printf("\nPress any key to continue..");
    getchar();
    printf("\n\tSD_ID : CQ ID = 2 : 3\n");
    ioctl_prep_sq(file_desc, 2, 3, 200, 1);
    printf("\nPress any key to continue..");
    getchar();
    printf("\n\tSD_ID : CQ ID = 3 : 6\n");
    ioctl_prep_sq(file_desc, 3, 6, 120, 1);
    printf("\nPress any key to continue..");
    getchar();
    printf("\nTEST 4: Allocating SQ 1 to 3 with different sizes...\n");
    printf("\nTEST 4: Non Contiguous SQ Case...\n");
    printf("\n\tSD_ID : CQ ID = 4 : 6\n");
    ioctl_prep_sq(file_desc, 4, 6, 10, 0);
    printf("\nPress any key to continue..");
    getchar();
    printf("\n\tSD_ID : CQ ID = 5 : 1\n");
    ioctl_prep_sq(file_desc, 5, 1, 15, 0);
    printf("\nPress any key to continue..");
    getchar();
    printf("\n\tSD_ID : CQ ID = 6 : 3\n");
    ioctl_prep_sq(file_desc, 6, 3, 32, 0);
    printf("\nPress any key to continue..");
    getchar();
#endif
}

void ioctl_write_data(int file_desc)
{
    int ret_val;
    //int message;
    struct rw_generic test_data;

    test_data.type = NVMEIO_BAR01;
    test_data.offset = 0x14;
    test_data.nBytes = 4;
    test_data.acc_type = DWORD_LEN;

    test_data.buffer = malloc(sizeof(char) * test_data.nBytes);
    test_data.buffer[0] = 0x01;
    test_data.buffer[1] = 0x00;
    test_data.buffer[2] = 0x46;
    test_data.buffer[3] = 0x00;

    printf("\nwriting Test Application...\n");
    ret_val = ioctl(file_desc, NVME_IOCTL_WRITE_GENERIC, &test_data);

    if (ret_val < 0) {
            printf("ioctl_set_msg failed:%d\n", ret_val);
            exit(-1);
   }
}

void test_prep_cq(int file_desc)
{
    printf("\nTEST 5: Preparing CQ's with different sizes...\n");
    printf("\nTEST 5: Contiguous CQ Case...\n");
    printf("\n\tCQ ID = 1\n");
    ioctl_prep_cq(file_desc, 1, 20, 1);
    printf("\nPress any key to continue..");
    getchar();
    printf("\n\tCQ ID = 3\n");
    ioctl_prep_cq(file_desc, 3, 200, 1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nTEST 6: Preparing CQ 1 to 3 with different sizes...\n");
    printf("\nTEST 6: Non Contiguous SQ Case...\n");
    printf("\n\tCQ ID = 6\n");
    ioctl_prep_cq(file_desc, 6, 10, 0);
    printf("\nPress any key to continue..");
    getchar();
    printf("\n\tCQ ID = 5\n");
    ioctl_prep_cq(file_desc, 5, 15, 0);
    printf("\nPress any key to continue..");
    getchar();
}

void test_metrics(int file_desc)
{
    printf("\nTEST 4: Get metrics\n");
    /* ACQ Metrics */
    printf("Get ACQ Metrics:\n\n");
    ioctl_get_q_metrics(file_desc, 0, 0, sizeof(struct nvme_gen_cq));
    printf("\nPress any key to continue..");
    getchar();

    /* ASQ Metrics */
/*    printf("Get ASQ Metrics:\n\n");
    ioctl_get_q_metrics(file_desc, 0, 1, sizeof(struct nvme_gen_sq));
    printf("\nPress any key to continue..");
    getchar();
*/
    printf("Get IO_SQ = 2 (exists) Metrics: \n\n");
    ioctl_get_q_metrics(file_desc, 2, 1, sizeof(struct nvme_gen_sq) + 10);
    printf("\nPress any key to continue..");
    getchar();

    printf("Get IO_SQ = 1 (exists) Metrics: \n\n");
    ioctl_get_q_metrics(file_desc, 1, 1, sizeof(struct nvme_gen_sq));
    printf("\nPress any key to continue..");
    getchar();

    printf("Get IO_SQ = 6 (does not exist. No metrics): \n\n");
    ioctl_get_q_metrics(file_desc, 6, 1, sizeof(struct nvme_gen_sq));
    printf("\nPress any key to continue..");
    getchar();

    printf("Get IO_SQ = 3 (exists but no space to copy. No metrics): \n\n");
    ioctl_get_q_metrics(file_desc, 3, 1, sizeof(struct nvme_gen_sq) - 5);
    printf("\nPress any key to continue..");
    getchar();
}

void tst_ring_dbl(int file_desc)
{
    ioctl_tst_ring_dbl(file_desc, 1);
    ioctl_tst_ring_dbl(file_desc, 10);
    ioctl_tst_ring_dbl(file_desc, 5);
    ioctl_tst_ring_dbl(file_desc, 0);
}

void ioctl_dump(int file_desc, char *tmpfile)
{
    int ret_val = -1;
    struct nvme_file pfile;

    pfile.flen = strlen(tmpfile);

    printf("size = %d\n", pfile.flen);

    pfile.file_name = malloc(sizeof(char) * pfile.flen);
    strcpy((char *)pfile.file_name, tmpfile);

    printf("File name = %s\n", pfile.file_name);

    ret_val = ioctl(file_desc, NVME_IOCTL_DUMP_METRICS, &pfile);
    if(ret_val < 0)
        printf("Dump Metircs failed!\n");
    else
        printf("Dump Metrics SUCCESS\n");
}

int test_prp(int file_desc)
{
    ioctl_create_prp_one_page(file_desc);
    ioctl_create_prp_less_than_one_page(file_desc);
    ioctl_create_prp_more_than_two_page(file_desc);
    ioctl_create_list_of_prp(file_desc);
    ioctl_create_fill_list_of_prp(file_desc);
    return 0;
}

int main(int argc, char *argv[])
{
    int file_desc,test_case;
    char *tmpfile1 = "/tmp/file_name1.txt";
    char *tmpfile2 = "/tmp/file_name2.txt";
    char *tmpfile3 = "/tmp/file_name3.txt";
    char *tmpfile4 = "/tmp/file_name4.txt";

    if (argc != 2) {
        printf("Usage: %s test_case_num \n",argv[0]);
        return 1;
    }

    test_case = atoi(argv[1]); /* convert strings to integers */
    printf("\n*****\t Demo \t*****\n");

    printf("Starting Test Application...\n");

    file_desc = open(DEVICE_FILE_NAME, 0);
    if (file_desc < 0) {
        printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
        exit(-1);
    }

    printf("Device File Succesfully Opened = %d\n", file_desc);

    switch(test_case) {
    case 1:
        printf("Test1: Initializing the state of the device to Run tests\n");
        printf("Calling Contoller State to set to Disable state\n");
        ioctl_disable_ctrl(file_desc, ST_DISABLE);
        test_admin(file_desc);
        printf("\n.Test PASS if creation is success.");
        printf("Calling Contoller State to set to Enable state\n");
        ioctl_enable_ctrl(file_desc);
        printf("Writing the Registers of NVME space\n");
        ioctl_write_data(file_desc);
        printf("Test to initialize the state of controller done\n");
        break;
    case 2:
        printf("Test2: Sending Create Discontig IOSQ with ID 1"
        " and contig IOCQ with ID 1\n");
        printf("\n Preparing contig CQ with ID 1\n");
        printf("\n\tCQ ID = 1\n");
        ioctl_prep_cq(file_desc, 1, 20, 1);

        printf("\n Preparing Discontig SQ with ID 1\n");
        printf("\n\tSD_ID : CQ ID = 1 : 1\n");
        ioctl_prep_sq(file_desc, 1, 1, 65472, 0);

        printf("Calling Dump Metrics to tmpfile1\n");
        ioctl_dump(file_desc, tmpfile1);

        printf("Executing SEND 64 byte command both for SQ and CQ\n");
        ioctl_create_contig_iocq(file_desc);
        ioctl_create_discontig_iosq(file_desc);

        printf("Calling Dump Metrics to tmpfile2\n");
        ioctl_dump(file_desc, tmpfile2);

        printf("Ringing Doorbell for SQID 0\n");
        ioctl_tst_ring_dbl(file_desc, 0);
        printf("Test to Create Discontig/Contig IO Queues Done\n");
        break;
    case 3:
        printf("Test3: Sending Create contig IOSQ with ID 2 and linking "
        "to already created CQ ID1\n");

        printf("\n Preparing contig SQ with ID 2\n");
        printf("\n\tSD_ID : CQ ID = 2 : 1\n");
        ioctl_prep_sq(file_desc, 2, 1, 256, 1);

        printf("Calling Dump Metrics to tmpfile1\n");
        ioctl_dump(file_desc, tmpfile1);

        printf("Executing SEND 64 byte command\n");
        ioctl_create_contig_iosq(file_desc);

        printf("Calling Dump Metrics to tmpfile2\n");
        ioctl_dump(file_desc, tmpfile2);

        printf("Ringing Doorbell for SQID 0\n");
        ioctl_tst_ring_dbl(file_desc, 0);
        printf("Test to Create contig IOSQ Done\n");
        break;
    case 4: /* Delete the Queues */
        printf("Test4: Sending Delete IOSQ for ID 1 and 2 "
        "also deleteing IOCQ ID1\n");

        printf("Executing SEND 64 byte commands 3 at a time!\n");
        printf("Deleting IOSQ 1\n");
        ioctl_delete_ioq(file_desc, 0x00, 1);
        printf("Deleting IOSQ 2\n");
        ioctl_delete_ioq(file_desc, 0x00, 2);
        printf("Deleting IOCQ 1\n");
        ioctl_delete_ioq(file_desc, 0x04, 1);

        printf("Ringing Doorbell for SQID 0\n");
        ioctl_tst_ring_dbl(file_desc, 0);
        printf("Test to Create contig IOSQ Done\n");
        break;
    case 5: /* Send the identify command */
        printf("Test5: Sending Identify Command\n");
        ioctl_send_identify_cmd(file_desc);
        printf("Ringing Doorbell for SQID 0\n");
        ioctl_tst_ring_dbl(file_desc, 0);
        printf("Test to send identify command Done\n");
        break;

    case 6: /* Send an IO command */
        printf("Test6: Sending IO Command\n");

        printf("Test to send IO command Done\n");
        break;
    default:
         printf("Default: Disabling the controller completley\n");
         ioctl_disable_ctrl(file_desc, ST_DISABLE_COMPLETELY);
         printf("Calling Dump Metrics to tmpfile4\n");
         ioctl_dump(file_desc, tmpfile4);
         /* NOTE:- Disable Controller not called in unit tests since Disable
         * is an asyn request to the HW and as of now we dont have any means
         * to wait till CQ entries are posted
         * TODO : will be called when IOCTL_REAP is implemented
         */
    }
    printf("\n\n****** END OF DEMO ******\n\n");
    return 0;
}
