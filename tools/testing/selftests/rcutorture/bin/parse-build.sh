#!/bin/sh
#
# Check the build output from an rcutorture run for goodness.
# The "file" is a pathname on the local system, and "title" is
# a text string for error-message purposes.
#
# The file must contain kernel build output.
#
# Usage:
#	sh parse-build.sh file title
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
# Copyright (C) IBM Corporation, 2011
#
# Authors: Paul E. McKenney <paulmck@linux.vnet.ibm.com>

T=$1
title=$2

if grep -q CC < $T
then
	:
else
	echo $title no build
	exit 1
fi

if egrep -q "error:|rcu[^/]*\.c.*warning:|rcu.*\.h.*warning:" < $T
then
	echo $title build errors:
	egrep "error:|rcu[^/]*\.c.*warning:|rcu.*\.h.*warning:" < $T
	exit 2
fi
exit 0
