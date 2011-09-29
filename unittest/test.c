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
    ioctl_prep_sq(file_desc, 1, 1, 20, 1);
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
    printf("Get ACQ Metrics: \n\n");
    ioctl_get_q_metrics(file_desc, 0, 0, sizeof(struct nvme_gen_cq));
    printf("\nPress any key to continue..");
    getchar();

    /* ASQ Metrics */
    //printf("Get ASQ Metrics: \n\n");
    //ioctl_get_q_metrics(file_desc, 0, 1, sizeof(struct nvme_gen_sq));
    //printf("\nPress any key to continue..");
    //getchar();

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

int main(void)
{
    int file_desc;
    char *tmpfile1 = "/tmp/file_name1.txt";
    char *tmpfile2 = "/tmp/file_name2.txt";

    printf("\n*****\t Demo \t*****\n");

    /*printf("Ensure you have permissions to device..\n\
    else \n do \"chmod 777 /dev/qnvme0\" \n");*/
    printf("Starting Test Application...\n");

    file_desc = open(DEVICE_FILE_NAME, 0);
    if (file_desc < 0) {
        printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
        exit(-1);
    }

    printf("Device File Succesfully Opened = %d\n", file_desc);

    ioctl_dump(file_desc, tmpfile1);
    test_admin(file_desc);
//    printf("\n...Test PASS if creation is success.");
//    printf("\nPress any key to continue..");
//    getchar();

    test_prep_sq(file_desc);
//    printf("\n...Test PASS if all Preparation success...");
//    printf("\nPress any key to continue..");
//    getchar();

    test_prep_cq(file_desc);
//    printf("\n...Test PASS if all Preparation success...");
//    printf("\nPress any key to continue..");
//    getchar();

    ioctl_dump(file_desc, tmpfile2);

//    tst_ring_dbl(file_desc);

//    test_admin(file_desc);
//    printf("\n...Test PASS if creation is not successful.");
//    printf("\nPress any key to continue..");
//    getchar();

//    test_metrics(file_desc);


    //ioctl_check_device(file_desc);
    //ioctl_read_data(file_desc);
    //ioctl_write_data(file_desc);
    //ioctl_read_data(file_desc);
    //ioctl_check_device(file_desc);
    //ioctl_disable_ctrl(file_desc);
    //ioctl_create_acq(file_desc);
    //ioctl_create_asq(file_desc);
    //ioctl_enable_ctrl(file_desc);

    /* ACQ Metrics */
    //ioctl_get_q_metrics(file_desc, 0, 0);
    /* ASQ Metrics */
    //ioctl_get_q_metrics(file_desc, 0, 1);

    //ioctl_get_q_metrics(file_desc, 1, 1);
    //ioctl_get_q_metrics(file_desc, 20, 0);

    //ioctl_alloc_sq(file_desc, 1, 1, 20);
    //ioctl_alloc_sq(file_desc, 2, 3, 200);
    //ioctl_alloc_sq(file_desc, 3, 6, 120);
    //ioctl_alloc_sq(file_desc, 4, 2, 2320);
    //ioctl_alloc_sq(file_desc, 5, 6, 5620);
    //ioctl_alloc_sq(file_desc, 6, 5, 4220);

    //ioctl_create_acq(file_desc);
    //ioctl_create_asq(file_desc);

    close(file_desc);
    printf("\nEnd of Testing...");
    getchar();
    printf("\n\n****** END OF DEMO ****** \n\n");
    return 0;
}
