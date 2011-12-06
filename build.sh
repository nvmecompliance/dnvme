#!/bin/bash
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

RPMCOMPILEDIR=$1
RPMSPECFILE=$2
RPMSRCFILE=$3

Usage() {
echo "usage...."
echo "  $0 <RPM_compile> <RPM_spec> <RPM_src>"
echo "    <RPM_compile>  Specify full path to the base RPM compilation dir"
echo "    <RPM_spec>     Specify filename only of the RPM spec file (*.spec)"
echo "    <RPM_src>      Specify full path to the RPM source file (*.tar.gz)"
echo ""
}

# RPM build errors under ROOT a potentially catastrophic
if [[ $EUID -eq 0 ]]; then
  echo Running as 'root' is not supported
  exit
fi

if [ -z $RPMCOMPILEDIR ] || [ -z $RPMSPECFILE ] || [ -z $RPMSRCFILE ]; then
  Usage
  exit
fi

# Setup a fresh RPM build environment, and then build
MAJOR=`awk 'FNR == 29' version.h`
MINOR=`awk 'FNR == 32' version.h`
rm -rf $RPMCOMPILEDIR
mkdir -p $RPMCOMPILEDIR/{BUILDROOT,BUILD,RPMS,S{OURCE,PEC,RPM}S}
cp -p $RPMSRCFILE.tar.gz  $RPMCOMPILEDIR
cp -p $RPMSRCFILE.tar.gz  $RPMCOMPILEDIR/SOURCES
cp -p ${RPMSPECFILE} $RPMCOMPILEDIR/SPECS
cd $RPMCOMPILEDIR/SPECS
rpmbuild --define "_topdir ${RPMCOMPILEDIR}" --define "_major $MAJOR" --define "_minor $MINOR" -ba ${RPMSPECFILE}

exit
