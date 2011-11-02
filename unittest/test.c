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
#include <sys/mman.h>

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
    test_data.offset = 0x14;
    test_data.nBytes = 4;
    test_data.acc_type = DWORD_LEN;
    test_data.buffer = NULL;

    test_data.buffer = malloc(test_data.nBytes);
    if (test_data.buffer == NULL) {
        printf("malloc failed!");
        goto err;
    }
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

   free(test_data.buffer);

err:
    return;
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
    enum nvme_state new_state = ST_ENABLE;

    printf("User Call to Enable Ctrlr:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_DEVICE_STATE, &new_state);
    if(ret_val < 0)
        printf("enable Failed!\n");
    else
        printf("enable SUCCESS\n");
}

void ioctl_disable_ctrl(int file_desc, enum nvme_state new_state)
{
    int ret_val = -1;
    printf("User Call to Disable Ctrlr:\n");

    ret_val = ioctl(file_desc, NVME_IOCTL_DEVICE_STATE, &new_state);
    if(ret_val < 0)
        printf("Disable Failed!\n");
    else
        printf("Disable SUCCESS\n");
}

void test_admin(int file_desc)
{
    /* Test Case 1 */
    printf("\nTEST 1.1: Create Admin CQ...\n");
    ioctl_create_acq(file_desc);
    printf("\nTEST 1.2: Create Admin SQ...\n");
    ioctl_create_asq(file_desc);
}

void test_prep_sq(int file_desc)
{
    printf("\nTEST 2.2.1: Allocating 3 IO Contiguous SQs with different sizes...\n");
    printf("\n\tSD_ID : CQ ID = 1 : 1\n");
    ioctl_prep_sq(file_desc, 1, 1, 256, 0);

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
    printf("\nTEST 2.2.2: Allocating 3 Non-Contiguous IO SQs with different sizes...\n");
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
    printf("\nTEST 2.3.1: Preparing IO Contiguous CQ's with 100 elements each...\n");
    printf("\n\tCQ ID = 1\n");
    ioctl_prep_cq(file_desc, 1, 100, 1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\n\tCQ ID = 2\n");
    ioctl_prep_cq(file_desc, 2, 100, 1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\n\tCQ ID = 3\n");
    ioctl_prep_cq(file_desc, 3, 100, 1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\n\tCQ ID = 4\n");
    ioctl_prep_cq(file_desc, 4, 100, 1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\n\tCQ ID = 5\n");
    ioctl_prep_cq(file_desc, 5, 100, 1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\n\tCQ ID = 6\n");
    ioctl_prep_cq(file_desc, 6, 100, 1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nTEST 2.3.2: Preparing Non-Contiguous IO CQ's with different sizes...\n");
    printf("\n\tCQ ID = 16\n");
    ioctl_prep_cq(file_desc, 16, 10, 0);
    printf("\nPress any key to continue..");
    getchar();
    printf("\n\tCQ ID = 15\n");
    ioctl_prep_cq(file_desc, 15, 15, 0);
    printf("\nPress any key to continue..");
    getchar();
}

int ioctl_ut_reap_inq(int file_desc)
{
    uint16_t tmp;

    tmp = 0; // Reap Inquiry Unit Test setup.
    if (ioctl(file_desc, IOCTL_UNIT_TESTS, tmp) < 0) {
        printf("\n\nTest = %d Setup failed...", tmp);
        return -1;
    }
    return 0;
}


int ioctl_ut_reap(int file_desc)
{
    uint16_t tmp;

    tmp = 2; // Reap Inquiry Unit Test setup.
    if (ioctl(file_desc, IOCTL_UNIT_TESTS, tmp) < 0) {
        printf("\n\nTest = %d Setup failed...", tmp);
        return -1;
    }
    return 0;
}


int ioctl_ut_mmap(int file_desc)
{
    uint16_t tmp;

    tmp = 1; // Mmap Unit Test setup.
    if (ioctl(file_desc, IOCTL_UNIT_TESTS, tmp) < 0) {
        printf("\n\nTest = %d Setup failed...", tmp);
        return -1;
    }
    return 0;
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

    //printf("size = %d\n", pfile.flen);

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

void test_reap_inquiry(int file_desc)
{
    //printf("\tReap inquiry on Admin CQ...\n");
    //ioctl_reap_inquiry(file_desc, 0);
    printf("\tReap inquiry on CQ = 1...\n");
    ioctl_reap_inquiry(file_desc, 1);
    printf("\tReap inquiry on CQ = 2...\n");
    ioctl_reap_inquiry(file_desc, 2);
    printf("\tReap inquiry on CQ = 3...\n");
    ioctl_reap_inquiry(file_desc, 3);
    printf("\tReap inquiry on CQ = 4...\n");
    ioctl_reap_inquiry(file_desc, 4);
    printf("\tReap inquiry on CQ = 5...\n");
    ioctl_reap_inquiry(file_desc, 5);
    printf("\tReap inquiry on CQ = 6...\n");
    ioctl_reap_inquiry(file_desc, 6);
}

void display_contents(uint8_t *kadr, int elem)
{
    int i;
    for (i = 0; i < elem; i++) {
        //printf("Addr:Val::0x%lx:0x%lx\n", (uint64_t)kadr, *kadr);
        display_cq_data((unsigned char *)kadr, 1);
        kadr += 16;
    }
}

int test_regression(int file_desc)
{
    uint32_t sq_id = 0;
    char *tmpfile1 = "/tmp/regression_file1.txt";
    char *tmpfile2 = "/tmp/regression_file2.txt";
    char *tmpfile3 = "/tmp/regression_file3.txt";
    uint8_t *kadr;
    int fd3;

    printf("\n******\t Sprint 2 Demo \t******\n");

    /*printf("Ensure you have permissions to device..\n\
    else \n do \"chmod 777 /dev/qnvme0\" \n");*/
    printf("Starting Test Application...\n");

    printf("Try opening an already opened device...\n");
    fd3 = open(DEVICE_FILE_NAME, 0);
    if (fd3 > 0) {
        printf("Should not open an already opened device...");
        exit(-1);
    }
    printf("Device not opened as expected...\n");
    printf("\nPress any key to continue..");
    getchar();


    printf("Calling Controller State to set to Disable state\n");
    ioctl_disable_ctrl(file_desc, ST_DISABLE);
    printf("\nPress any key to continue..");
    getchar();


    test_admin(file_desc);
    printf("\n...Test PASS if creation is success.");
    printf("\nPress any key to continue..");
    getchar();

    printf("\nTEST 2.1: Calling Controller State to set to Enable state\n");
    ioctl_enable_ctrl(file_desc);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nSet IO Q Size before proceeding....\n");
    ioctl_write_data(file_desc);
    printf("\nPress any key to continue..");
    getchar();

    test_prep_sq(file_desc);
    printf("\n...Test PASS if all Preparation success...");
    printf("\nPress any key to continue..");
    getchar();

    test_prep_cq(file_desc);
    printf("\n...Test PASS if all Preparation success...");
    printf("\nPress any key to continue..");
    getchar();

    test_prep_sq(file_desc);
    printf("\n...Test PASS if all Preparation fails...");
    printf("\nPress any key to continue..");
    getchar();

    test_prep_cq(file_desc);
    printf("\n...Test PASS if all Preparation fails...");
    printf("\nPress any key to continue..");
    getchar();

    printf("\nSet Up IO Q's to actually have some data to be reaped...\n");
    ioctl_ut_reap_inq(file_desc);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nTest 2.4: Testing Reap Inquiry...\n");
    test_reap_inquiry(file_desc);
    printf("\nPress any key to continue..");
    getchar();

    printf("\n Test 2.5: Call Ring Doorbell\n");
    tst_ring_dbl(file_desc);
    printf("\nPress any key to continue..");
    getchar();

    ioctl_ut_mmap(file_desc);
    printf("\nPress any key to continue..");
    getchar();

    sq_id = 0x10000;
    printf("\nTEST 3.1: Call to Mmap SQ 0\n");
    kadr = mmap(0, 4096, PROT_READ, MAP_SHARED, file_desc, 4096 * sq_id);
    if (!kadr) {
        printf("mapping failed\n");
        return -1;
    }
    display_contents(kadr, 15);
    printf("\nPress any key to continue..");
    getchar();

    munmap(kadr, 4096);

    sq_id = 0x0;
    printf("\nTEST 3.2: Calling to mmap CQ 0\n");
    kadr = mmap(0, 4096, PROT_READ, MAP_SHARED, file_desc, 4096 * sq_id);
    if (!kadr) {
        printf("mapping failed\n");
        return -1;
    }
    display_contents(kadr, 15);
    printf("\nPress any key to continue..");
    getchar();
    munmap(kadr, 4096);

    test_admin(file_desc);
    printf("\n...Test PASS if creation is not successful.");
    printf("\nPress any key to continue..");
    getchar();

    test_meta(file_desc, 0);

    printf("\nTest 2.6.1: Calling Dump Metrics to tmpfile1\n");
    ioctl_dump(file_desc, tmpfile1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nTest 2.7: Calling Controller State to set to Disable state\n");
    ioctl_disable_ctrl(file_desc, ST_DISABLE);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nTest 2.6.2 : Calling Dump Metrics to tmpfile2 after DISABLE...\n");
    ioctl_dump(file_desc, tmpfile2);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nTest 2.8: Calling Controller State to set to ST_DISABLE_COMPLETELY state\n");
    ioctl_disable_ctrl(file_desc, ST_DISABLE_COMPLETELY);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nTest 2.6.3 :Calling Dump Metrics to tmpfile3 After DISABLE_COMPLETELY...\n");
    ioctl_dump(file_desc, tmpfile3);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nEnd of Regression Testing...");
    printf("\nPress any key to continue..");
    getchar();
    return 0;
}

void set_reap_cq(int file_desc, int cq_id, int elements, int size, int disp)
{
    ioctl_reap_cq(file_desc, cq_id, elements, size, disp);
}

void test_reap(int file_desc)
{
    int size, elements;
    int cq_id;
    char *tmpfile1 = "/tmp/file_name5.txt";
    char *tmpfile2 = "/tmp/file_name6.txt";

    set_admn(file_desc);

    printf("\nSet IO Q Size before proceeding....\n");
    ioctl_write_data(file_desc);

    test_prep_sq(file_desc);
    printf("\n...Test PASS if all Preparation fails...");
    printf("\nPress any key to continue..");
    getchar();

    ioctl_prep_cq(file_desc, 4, 100, 1);
    ioctl_prep_cq(file_desc, 5, 100, 1);

    //test_prp(file_desc);

    printf("\nTest 2.6.1: Calling Dump Metrics to tmpfile1\n");
    ioctl_dump(file_desc, tmpfile1);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nSet Up IO Q's to actually have some data to be reaped...\n");
    ioctl_ut_reap_inq(file_desc);
    ioctl_ut_reap(file_desc);

    // Admin Reaping.
    // Num Could Reap = 4.
    cq_id = 0;
    elements = 2;
    size = elements * 16; // 16 CE entry
    set_reap_cq(file_desc, cq_id, elements, size, 1);
    set_reap_cq(file_desc, cq_id, elements, size, 1);

    cq_id = 5;
    elements = 2;
    size = elements * 16; // 16 CE entry
    set_reap_cq(file_desc, cq_id, elements, size, 1);
    set_reap_cq(file_desc, cq_id, elements, size, 1);

    cq_id = 5;
    elements = 5;
    size = elements * 16; // 16 CE entry
    set_reap_cq(file_desc, cq_id, elements, size, 0);
    set_reap_cq(file_desc, cq_id, elements, size, 0);

    cq_id = 4;
    elements = 1;
    size = elements * 16; // 16 CE entry
    set_reap_cq(file_desc, cq_id, elements, size, 0); // Reap 1
    set_reap_cq(file_desc, cq_id, elements, size, 0); // Reap next

    printf("\nTest 2.6.1: Calling Dump Metrics to tmpfile2\n");
    ioctl_dump(file_desc, tmpfile2);
    printf("\nPress any key to continue..");
    getchar();

    printf("\nCheck Reap Outputs...");
    printf("\nPress any key to continue..");
    getchar();
}

void reset_for_reap(int file_desc)
{
    ioctl_disable_ctrl(file_desc, ST_DISABLE_COMPLETELY);
    set_admn(file_desc);
    ioctl_write_data(file_desc);
    ioctl_prep_cq(file_desc, 4, 100, 1);
    ioctl_prep_cq(file_desc, 5, 100, 1);

    printf("\nSet Up IO Q's to actually have some data to be reaped...\n");
    ioctl_ut_reap_inq(file_desc);
    ioctl_ut_reap(file_desc);
}

void reap_params_mod16(int file_desc, int elements)
{
    int size;
    int cq_id;

    reset_for_reap(file_desc);

    // Num Could Reap = 51. Size is equal
     cq_id = 5;
     size = (elements * 16); // 16 CE entry
     set_reap_cq(file_desc, cq_id, elements, size, 1);
     printf("\nPress any key to continue..");
     getchar();

     reset_for_reap(file_desc);

     // Num Could Reap = 51. Size is less %16
     cq_id = 5;
     size = (elements * 16) - 16 * 10; // 16 CE entry
     set_reap_cq(file_desc, cq_id, elements, size, 1);
     printf("\nPress any key to continue..");
     getchar();

     reset_for_reap(file_desc);

     // Num Could Reap = 51. Size is more %16
     cq_id = 5;
     size = (elements * 16) + 16 * 2; // 16 CE entry
     set_reap_cq(file_desc, cq_id, elements, size, 1);
     printf("\nPress any key to continue..");
     getchar();
}

void reap_params_mod17(int file_desc, int elements)
{
    int size;
    int cq_id;

    reset_for_reap(file_desc);

    // Num Could Reap = 51. Size is equal
    cq_id = 5;
    size = (elements * 16); // 16 CE entry
    set_reap_cq(file_desc, cq_id, elements, size, 1);
    printf("\nPress any key to continue..");
    getchar();

    reset_for_reap(file_desc);
    // Num Could Reap = 51. Size is less %17
    cq_id = 5;
    size = (elements * 16) - 17 * 10; // 16 CE entry
    set_reap_cq(file_desc, cq_id, elements, size, 1);
    printf("\nPress any key to continue..");
    getchar();

    reset_for_reap(file_desc);

    // Num Could Reap = 51. Size is more %17
    cq_id = 5;
    size = (elements * 16) + 17 * 2; // 16 CE entry
    set_reap_cq(file_desc, cq_id, elements, size, 1);
    printf("\nPress any key to continue..");
    getchar();
}

void test_reap_regression(int file_desc)
{
    // Num Could Reap = 51
    reap_params_mod16(file_desc, 53);
    printf("\nPress any key to continue..");
    getchar();

    reap_params_mod16(file_desc, 50);
    printf("\nPress any key to continue..");
    getchar();

    reap_params_mod16(file_desc, 51);
    printf("\nPress any key to continue..");
    getchar();

    // Num Could Reap = 51
    reap_params_mod17(file_desc, 53);
    printf("\nPress any key to continue..");
    getchar();

    reap_params_mod17(file_desc, 50);
    printf("\nPress any key to continue..");
    getchar();

    reap_params_mod17(file_desc, 51);
    printf("\nPress any key to continue..");
    getchar();

}

int main()
{
    int file_desc, test_case, i;
    char *tmpfile1 = "/tmp/temp_file1.txt";
    char *tmpfile2 = "/tmp/temp_file2.txt";
    char *tmpfile3 = "/tmp/temp_file3.txt";
    char *tmpfile4 = "/tmp/temp_file4.txt";
    char *tmpfile14 = "/tmp/temp_file14.txt";
    char *tmpfile15 = "/tmp/temp_file15.txt";

    /* Maximum possible entries */
    void *read_buffer;
    void *identify_buffer;
    uint8_t *kadr;

    printf("\n*****\t Demo \t*****\n");

    printf("Starting Test Application...\n");

    file_desc = open(DEVICE_FILE_NAME, 0);
    if (file_desc < 0) {
        printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
        exit(-1);
    }
    printf("Device File Successfully Opened = %d\n", file_desc);

    if (posix_memalign(&read_buffer, 4096, READ_BUFFER_SIZE)) {
        printf("Memalign Failed");
        return 0;
    }

    identify_buffer = malloc(4096);
    if (identify_buffer == NULL) {
        printf("Malloc Failed");
        return 0;
    }

    test_drv_metrics(file_desc);
    do {
        printf("\nEnter a valid test case number:");
        scanf ("%d", &test_case);
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
            printf("Test2: Disabling the controller completley\n");
            ioctl_disable_ctrl(file_desc, ST_DISABLE_COMPLETELY);
            /* NOTE:- Disable Controller not called in unit tests since Disable
            * is an asyn request to the HW and as of now we dont have any means
            * to wait till CQ entries are posted
            * TODO : will be called when IOCTL_REAP is implemented
            */
            break;
        case 3:
            printf("Test3: Sending Create Discontig IOSQ with ID 1"
            " and contig IOCQ with ID 1\n");
            printf("\n Preparing contig CQ with ID 1\n");
            printf("\n\tCQ ID = 1\n");
            ioctl_prep_cq(file_desc, 1, 20, 1);

            printf("\n Preparing Discontig SQ with ID 1\n");
            printf("\n\tSD_ID : CQ ID = 1 : 1\n");
            ioctl_prep_sq(file_desc, 1, 1, 65472, 0);

            printf("Executing SEND 64 byte command both for SQ and CQ\n");
            ioctl_create_contig_iocq(file_desc);
            ioctl_create_discontig_iosq(file_desc);

            printf("\nCalling Dump Metrics to tmpfile1\n");
            ioctl_dump(file_desc, tmpfile1);

            printf("Ringing Doorbell for SQID 0\n");
            ioctl_tst_ring_dbl(file_desc, 0);

            printf("Test to Create Discontig/Contig IO Queues Done\n");
            break;
        case 4:
            printf("Test4: Sending Create contig IOSQ with ID 2 and linking "
            "to already created CQ ID1\n");

            printf("\n Preparing contig SQ with ID 2\n");
            printf("\n\tSD_ID : CQ ID = 2 : 1\n");
            ioctl_prep_sq(file_desc, 2, 1, 256, 1);

            printf("Executing SEND 64 byte command\n");
            ioctl_create_contig_iosq(file_desc);

            printf("\nCalling Dump Metrics to tmpfile2\n");
            ioctl_dump(file_desc, tmpfile2);
            printf("Ringing Doorbell for SQID 0\n");
            ioctl_tst_ring_dbl(file_desc, 0);
            printf("Test to Create contig IOSQ Done\n");
        break;
        case 5: /* Delete the Queues */
            printf("Test5: Sending Delete IOSQ for ID 1 and 2 "
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
            printf("\nCalling Dump Metrics to tmpfile3\n");
            ioctl_dump(file_desc, tmpfile3);
            break;
        case 6: /* Send the identify command */
            printf("Test6: Sending Identify Command\n");
            ioctl_send_identify_cmd(file_desc, read_buffer);
            printf("Ringing Doorbell for SQID 0\n");
            ioctl_tst_ring_dbl(file_desc, 0);
            printf("Test to send identify command Done\n");
            printf("\nCalling Dump Metrics to tmpfile4\n");
            ioctl_dump(file_desc, tmpfile4);
            break;
        case 7: /* Send an IO write command */
            printf("Test7: Sending IO Write Command\n");
            ioctl_send_nvme_write(file_desc);
            printf("Ringing Doorbell for SQID 2\n");
            ioctl_tst_ring_dbl(file_desc, 2);
            printf("Test to send IO Write command Done\n");
            break;
        case 8: /* Send an IO read command */
            printf("Test8: Sending IO Read Command\n");
            ioctl_send_nvme_read(file_desc, read_buffer);
            printf("Ringing Doorbell for SQID 2\n");
            ioctl_tst_ring_dbl(file_desc, 2);
            printf("Test to send IO Read command Done\n");
            break;
        case 9: /* Reading contents of the read buffer */
            printf("\nReadin Data:\n");
            for (i = 0; i < READ_BUFFER_SIZE; i++) {
               printf("%x ",*(uint8_t *)(read_buffer +i) );
            }
            break;
        case 10: /* reap on Admin CQ for 2 elems */
            printf("\nCalling Dump Metrics to tmpfile4\n");
            ioctl_dump(file_desc, tmpfile14);
            set_reap_cq(file_desc, 0, 2, 32, 1);
            printf("\nCalling Dump Metrics to tmpfile4\n");
            ioctl_dump(file_desc, tmpfile15);
            break;
        case 11: /* reap on IO CQ for 2 elems */
            set_reap_cq(file_desc, 1, 2, 32, 1);
            break;
        case 12: /* Display ACQ Contents */
            kadr = mmap(0, 4096, PROT_READ, MAP_SHARED, file_desc, 0);
            if (!kadr) {
                printf("mapping failed\n");
                return -1;
            }
            display_contents(kadr, 20);
            munmap(kadr, 4096);
            break;
        case 13: /* Reading contents of the Identify buffer */
            printf("\nIdentify Data:\n");
            for (i = 0; i < READ_BUFFER_SIZE/2; i++) {
                 printf("%x ",*(uint8_t *)(identify_buffer +i));
            }
            break;
        case 14: /* Test Reap */
            test_reap(file_desc);
            break;
        case 15: /* Test Meta */
            test_meta(file_desc, 1);
            break;
        case 16: /* Regression testing */
            test_regression(file_desc);
            break;
        case 17: /* Reap Regression Testing */
            test_reap_regression(file_desc);
            break;
        default:
            printf("Undefined case!\n");
        }
    } while (test_case < 18);
    free(read_buffer);
    free(identify_buffer);
    printf("\n\n****** END OF DEMO ******\n\n");
    close(file_desc);
    return 0;
}
