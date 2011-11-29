
void test_interrupts(int fd);
void set_irq_msix(int fd);
void set_irq_none(int fd);
void test_loop_irq(int fd);
void admin_create_iocq_irq(int fd, int cq_id, int irq_vec, int cq_flags);
