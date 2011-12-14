#include "test_send_cmd.h"

#define PAGE_SIZE_I 4096

void test_interrupts(int fd);
void set_irq_msix(int fd);
void set_irq_none(int fd);
void test_loop_irq(int fd);
int admin_create_iocq_irq(int fd, int cq_id, int irq_vec, int cq_flags);
void set_cq_irq(int fd, void *p_dcq_buf);
void set_sq_irq(int fd, void *addr);
int irq_for_io_discontig(int file_desc, int cq_id, int irq_no, int cq_flags, void *addr);
int irq_for_io_contig(int file_desc, int cq_id, int irq_no, int cq_flags);
void test_contig_io_irq(int fd, void *addr);
void test_discontig_io_irq(int fd, void *addr);
void test_irq_delete(int fd);
