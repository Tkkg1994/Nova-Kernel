#!/bin/bash
#
# Shell functions for the rest of the scripts.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
#
# Copyright (C) IBM Corporation, 2013
#
# Authors: Paul E. McKenney <paulmck@linux.vnet.ibm.com>

# bootparam_hotplug_cpu bootparam-string
#
# Returns 1 if the specified boot-parameter string tells rcutorture to
# test CPU-hotplug operations.
bootparam_hotplug_cpu () {
	echo "$1" | grep -q "rcutorture\.onoff_"
}

# configfrag_boot_params bootparam-string config-fragment-file
#
# Adds boot parameters from the .boot file, if any.
configfrag_boot_params () {
	if test -r "$2.boot"
	then
		echo $1 `grep -v '^#' "$2.boot" | tr '\012' ' '`
	else
		echo $1
	fi
}

# configfrag_hotplug_cpu config-fragment-file
#
# Returns 1 if the config fragment specifies hotplug CPU.
configfrag_hotplug_cpu () {
	if test ! -r "$1"
	then
		echo Unreadable config fragment "$1" 1>&2
		exit -1
	fi
	grep -q '^CONFIG_HOTPLUG_CPU=y$' "$1"
}

# identify_qemu builddir
#
# Returns our best guess as to which qemu command is appropriate for
# the kernel at hand.  Override with the RCU_QEMU_CMD environment variable.
identify_qemu () {
	local u="`file "$1"`"
	if test -n "$RCU_QEMU_CMD"
	then
		echo $RCU_QEMU_CMD
	elif echo $u | grep -q x86-64
	then
		echo qemu-system-x86_64
	elif echo $u | grep -q "Intel 80386"
	then
		echo qemu-system-i386
	elif uname -a | grep -q ppc64
	then
		echo qemu-system-ppc64
	else
		echo Cannot figure out what qemu command to use! 1>&2
		# Usually this will be one of /usr/bin/qemu-system-*
		# Use RCU_QEMU_CMD environment variable or appropriate
		# argument to top-level script.
		exit 1
	fi
}
