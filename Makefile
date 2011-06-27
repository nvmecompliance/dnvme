KDIR := /lib/modules/2.6.33.3-85.fc13.x86_64/build/
EXTRA_CFLAGS += -I$(PWD)/

obj-m := dnvme.o
dnvme-objs += test_driver.o reg_test.o cmd_test.o comm_func.o

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
