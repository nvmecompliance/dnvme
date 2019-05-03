/*
 * NVM Express Compliance Suite
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Specify the software release version numbers on their own line for use with
 * awk and the creation of RPM's while also being compatible with building
 * the binaries via the Makefile with *.cpp source code.
 * If the line numbers within this file change by the result of editing, then
 * you must modify both the Makefile and build.sh for awk parsing. Additionally
 * test this modification by running the Makefile rpm target.
 */

#define VER_MAJOR    \
2

#define VER_MINOR    \
15
