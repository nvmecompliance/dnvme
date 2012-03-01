#
# NVM Express Compliance Suite
# Copyright (c) 2011, Intel Corporation.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#

# Modify the Makefile to point to Linux build tree.
DIST ?= $(shell uname -r)
KDIR:=/lib/modules/$(DIST)/build/
CDIR:=/usr/src/linux-source-2.6.35/scripts/
SOURCE:=$(shell pwd)
DRV_NAME:=dnvme
# QEMU_ON should be used when running the driver within QEMU, which forces
# dnvme to convert 8B writes to 2 4B writes patchin a QEMU deficiency
QEMU_ON:=-DQEMU
DBG_ON:=-g -DDEBUG

EXTRA_CFLAGS+=$(QEMU_ON) $(DBG_ON) -I$(PWD)/


SOURCES := \
	dnvme_reg.c \
	sysdnvme.c \
	dnvme_ioctls.c \
	dnvme_sts_chk.c \
	dnvme_queue.c \
	dnvme_cmds.c \
	dnvme_ds.c \
	dnvme_irq.c \
	ut_reap_inq.c

#
# RPM build parameters
#
RPMBASE=$(DRV_NAME)
MAJOR=$(shell awk 'FNR==29' $(PWD)/version.h)
MINOR=$(shell awk 'FNR==32' $(PWD)/version.h)
SOFTREV=$(MAJOR).$(MINOR)
RPMFILE=$(RPMBASE)-$(SOFTREV)
RPMCOMPILEDIR=$(PWD)/rpmbuild
RPMSRCFILE=$(PWD)/$(RPMFILE)
RPMSPECFILE=$(RPMBASE).spec
SRCDIR?=./src

obj-m := dnvme.o
dnvme-objs += sysdnvme.o dnvme_ioctls.o dnvme_reg.o dnvme_sts_chk.o dnvme_queue.o dnvme_cmds.o dnvme_ds.o dnvme_irq.o ut_reap_inq.o

all:
	make -C $(KDIR) M=$(PWD) modules

rpm: rpmzipsrc rpmbuild

clean:
	make -C $(KDIR) M=$(PWD) clean
	rm -f doxygen.log
	rm -rf $(SRCDIR)
	rm -rf $(RPMFILE)
	rm -rf $(RPMCOMPILEDIR)
	rm -rf $(RPMSRCFILE)
	rm -f $(RPMSRCFILE).tar*

clobber: clean
	rm -rf Doc/HTML
	rm -f $(DRV_NAME)

doc: all
	doxygen doxygen.conf > doxygen.log

# Specify a custom source c:ompile dir: "make src SRCDIR=../compile/dir"
# If the specified dir could cause recursive copies, then specify w/o './'
# "make src SRCDIR=src" will copy all except "src" dir.
src:
	rm -rf $(SRCDIR)
	mkdir -p $(SRCDIR)
	(git archive HEAD) | tar xf - -C $(SRCDIR)

install:
	# typically one invokes this as "sudo make install"
	mkdir -p $(DESTDIR)/lib/modules/$(DIST)
	install -p $(DRV_NAME).ko $(DESTDIR)/lib/modules/$(DIST)
	install -p etc/55-$(RPMBASE).rules $(DESTDIR)/etc/udev/rules.d
ifeq '$(DESTDIR)' ''
	# DESTDIR only defined when installing to generate an RPM, i.e. psuedo
	# install thus don't update /lib/modules/xxx/modules.dep file
	/sbin/depmod -a
endif

rpmzipsrc: SRCDIR:=$(RPMFILE)
rpmzipsrc: clobber src
	rm -f $(RPMSRCFILE).tar*
	tar cvf $(RPMSRCFILE).tar $(RPMFILE)
	gzip $(RPMSRCFILE).tar

rpmbuild: rpmzipsrc
	# Build the RPM and then copy the results local
	./build.sh $(RPMCOMPILEDIR) $(RPMSPECFILE) $(RPMSRCFILE)
	rm -rf ./rpm
	mkdir ./rpm
	cp -p $(RPMCOMPILEDIR)/RPMS/x86_64/*.rpm ./rpm
	cp -p $(RPMCOMPILEDIR)/SRPMS/*.rpm ./rpm

.PHONY: all clean clobber doc src install rpmzipsrc rpmbuild
