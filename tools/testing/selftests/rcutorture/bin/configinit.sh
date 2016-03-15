#!/bin/sh
#
# sh configinit.sh config-spec-file [ build output dir ]
#
# Create a .config file from the spec file.  Run from the kernel source tree.
# Exits with 0 if all went well, with 1 if all went well but the config
# did not match, and some other number for other failures.
#
# The first argument is the .config specification file, which contains
# desired settings, for example, "CONFIG_NO_HZ=y".  For best results,
# this should be a full pathname.
#
# The second argument is a optional path to a build output directory,
# for example, "O=/tmp/foo".  If this argument is omitted, the .config
# file will be generated directly in the current directory.

T=/tmp/configinit.sh.$$
trap 'rm -rf $T' 0
mkdir $T

# Capture config spec file.

c=$1
buildloc=$2
builddir=
if test -n $buildloc
then
	if echo $buildloc | grep -q '^O='
	then
		builddir=`echo $buildloc | sed -e 's/^O=//'`
		if test ! -d $builddir
		then
			mkdir $builddir
		fi
	else
		echo Bad build directory: \"$builddir\"
		exit 2
	fi
fi

sed -e 's/^\(CONFIG[0-9A-Z_]*\)=.*$/grep -v "^# \1" |/' < $c > $T/u.sh
sed -e 's/^\(CONFIG[0-9A-Z_]*=\).*$/grep -v \1 |/' < $c >> $T/u.sh
grep '^grep' < $T/u.sh > $T/upd.sh
echo "cat - $c" >> $T/upd.sh
make mrproper
make $buildloc distclean > $builddir/Make.distclean 2>&1
make $buildloc defconfig > $builddir/Make.defconfig.out 2>&1
mv $builddir/.config $builddir/.config.sav
sh $T/upd.sh < $builddir/.config.sav > $builddir/.config
cp $builddir/.config $builddir/.config.new
yes '' | make $buildloc oldconfig > $builddir/Make.modconfig.out 2>&1

# verify new config matches specification.
configcheck.sh $builddir/.config $c

exit 0
