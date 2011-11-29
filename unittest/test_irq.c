#include <stdio.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>

#include "../dnvme_interface.h"
#include "../dnvme_ioctls.h"

#include "test_metrics.h"
#include "test_irq.h"

void test_interrupts(int fd) /* Call with 16 number */
{
    int ret_val;
    struct interrupts new_irq;

    printf("Setting MSI X IRQ Type...\n");
    printf("Press any key to continue...\n");
    getchar();

    /* MSI SINGLE Testing */
    new_irq.num_irqs = 32;
    new_irq.irq_type = INT_MSIX;

    ret_val = ioctl(fd, NVME_IOCTL_SET_IRQ, &new_irq);
    if(ret_val < 0) {
        printf("\nSet IRQ MSI-X failed!\n");
    }
    else {
        printf("\nSet IRQ MSI-X success!!\n");
    }
    printf("Press any key to continue...\n");
    getchar();


    printf("Setting NONE IRQ Type...\n");
    /* Setting to INT_NONE Testing */
    new_irq.num_irqs = 1;
    new_irq.irq_type = INT_NONE;

    ret_val = ioctl(fd, NVME_IOCTL_SET_IRQ, &new_irq);
    if(ret_val < 0) {
        printf("\nSet IRQ NONE failed!\n");
    }
    else {
        printf("\nSet IRQ NONE success!!\n");
    }
    printf("Press any key to continue...\n");
    getchar();
}

void set_irq_none(int fd)
{
    int ret_val;
    struct interrupts new_irq;

    printf("Setting NONE IRQ Type...\n");
    /* Setting to INT_NONE Testing */
    new_irq.num_irqs = 0;
    new_irq.irq_type = INT_NONE;

    ret_val = ioctl(fd, NVME_IOCTL_SET_IRQ, &new_irq);
    if(ret_val < 0) {
        printf("\nSet IRQ NONE failed!\n");
    }
    else {
        printf("\nSet IRQ NONE success!!\n");
    }
    printf("Press any key to continue...\n");
    getchar();
}

void set_irq_msix(int fd)
{
    int ret_val;
    struct interrupts new_irq;
    printf("Setting MSI X IRQ Type...\n");

    printf("Enter desired num_irqs = ");
    scanf("%hu", &new_irq.num_irqs);

    new_irq.irq_type = INT_MSIX;

    ret_val = ioctl(fd, NVME_IOCTL_SET_IRQ, &new_irq);
    if(ret_val < 0) {
        printf("\nSet IRQ MSI-X failed!\n");
    }
    else {
        printf("\nSet IRQ MSI-X success!!\n");
    }
}

void test_loop_irq(int fd)
{
    int i, j;
    void *rd_buffer;
    if (posix_memalign(&rd_buffer, 4096, READ_BUFFER_SIZE)) {
        printf("Memalign Failed");
        return;
    }
    /* Loop twice */
    for (i = 0; i < 2; i++) {
        printf("\nIRQ Loop Test = %d\n", i + 1);
        ioctl_disable_ctrl(fd, ST_DISABLE_COMPLETELY);
        set_irq_msix(fd);
        test_admin(fd);
        ioctl_enable_ctrl(fd);
        ioctl_write_data(fd);
        /* Submit 10 cmds */
        for (j = 0; j < 10; j++) {
            ioctl_send_identify_cmd(fd, rd_buffer);
            ioctl_tst_ring_dbl(fd, 0); /* Ring Admin Q Doorbell */
            while (ioctl_reap_inquiry(fd, 0) != j + 1);
        }
    }

    printf("\nCalling Dump Metrics to irq_loop_test\n");
    ioctl_dump(fd, "/tmp/irq_loop_test.txt");
    printf("\nPressAny key..\n");
    getchar();
    getchar();
}


void admin_create_iocq_irq(int fd, int cq_id, int irq_vec, int cq_flags)
{
     struct nvme_64b_send user_cmd;
     struct nvme_create_cq create_cq_cmd;
     int ret_val;

     ioctl_prep_cq(fd, cq_id, 20, 1);

     /* Fill the command for create contig IOSQ*/
     create_cq_cmd.opcode = 0x05;
     create_cq_cmd.cqid = cq_id;
     create_cq_cmd.qsize = 20;
     create_cq_cmd.cq_flags = cq_flags;
     create_cq_cmd.irq_vector = irq_vec;

     /* Fill the user command */
     user_cmd.q_id = 0;
     user_cmd.bit_mask = (MASK_PRP1_PAGE);
     user_cmd.cmd_buf_ptr = (u_int8_t *) &create_cq_cmd;
     user_cmd.data_buf_size = 0;
     user_cmd.data_buf_ptr = NULL;

     user_cmd.cmd_set = CMD_ADMIN;
     user_cmd.data_dir = 0;

     ret_val = ioctl(fd, NVME_IOCTL_SEND_64B_CMD, &user_cmd);
     if (ret_val < 0) {
         printf("Sending Admin Command Create IO Q Failed!\n");
     } else {
         printf("Admin Command Create IO Q SUCCESS\n\n");
     }
}

