# Modify the Makefile to point to Linux build tree.
KDIR := /lib/modules/2.6.35-30-server/build/
CDIR := /usr/src/linux-source-2.6.35/scripts/
SOURCE := ~/Projects/Nvme_Project/
DRV_DIR := dnvme/

SUBDIR := \
	doc

SOURCES := \
	sysdnvme.c \
	dnvme_ioctls.c

FLAG = -DDEBUG
EXTRA_CFLAGS += $(FLAG)
EXTRA_CFLAGS += -I$(PWD)/

obj-m := dnvme.o
dnvme-objs += sysdnvme.o dnvme_ioctls.o

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

doc: GOAL=doc

doc: $(SUBDIRS)
	doxygen doxygen.conf > doxygen.log

$(SUBDIRS):
	$(MAKE) -C $@ $(GOAL)

chksrc:
	$(CDIR)checkpatch.pl --file --terse $(SOURCE)$(DRV_DIR)*.c
	$(CDIR)checkpatch.pl --file --terse $(SOURCE)$(DRV_DIR)*.h

.PHONY: all clean doc
