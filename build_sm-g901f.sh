#!/bin/bash

export ARCH=arm
export CROSS_COMPILE=/Kernel_Folder/Toolchain_5.3_a15/bin/arm-cortex_a15-linux-gnueabihf-

BUILD_WHERE=$(pwd)
BUILD_KERNEL_DIR=$BUILD_WHERE
BUILD_ROOT_DIR=$BUILD_KERNEL_DIR
BUILD_KERNEL_OUT_DIR=$BUILD_ROOT_DIR

BOARD_KERNEL_BASE=0x00000000
BOARD_KERNEL_PAGESIZE=4096
BOARD_KERNEL_TAGS_OFFSET=0x02400000
BOARD_RAMDISK_OFFSET=0x02600000
BOARD_KERNEL_CMDLINE="console=null androidboot.hardware=qcom user_debug=23 msm_rtb.filter=0x3b7 dwc3_msm.cpu_to_affin=1 lpm_levels.sleep_disabled=1"

KERNEL_ZIMG=$BUILD_KERNEL_OUT_DIR/arch/arm/boot/zImage
DTC=$BUILD_KERNEL_OUT_DIR/scripts/dtc/dtc

FUNC_CLEAN_DTB()
{
	if ! [ -d $BUILD_KERNEL_OUT_DIR/arch/arm/boot/dts ] ; then
		echo "no directory : "$BUILD_KERNEL_OUT_DIR/arch/arm/boot/dts""
	else
		echo "rm files in : "$BUILD_KERNEL_OUT_DIR/arch/arm/boot/dts/*.dtb""
		rm $BUILD_KERNEL_OUT_DIR/arch/arm/boot/dts/*.dtb
		rm $BUILD_KERNEL_OUT_DIR/arch/arm/boot/dt.img
		rm $BUILD_KERNEL_OUT_DIR/arch/arm/boot/zImage
	fi
}

INSTALLED_DTIMAGE_TARGET=$BUILD_KERNEL_DIR/arch/arm/boot/dt.img
DTBTOOL=$BUILD_KERNEL_DIR/tools/dtbTool

FUNC_BUILD_DTIMAGE_TARGET()
{
	echo ""
	echo "================================="
	echo "START : FUNC_BUILD_DTIMAGE_TARGET"
	echo "================================="
	echo ""
	echo "DT image target : $INSTALLED_DTIMAGE_TARGET"
	
	if ! [ -e $DTBTOOL ] ; then
		if ! [ -d $BUILD_ROOT_DIR/android/out/host/linux-x86/bin ] ; then
			mkdir -p $BUILD_ROOT_DIR/android/out/host/linux-x86/bin
		fi
		cp $BUILD_ROOT_DIR/kernel/tools/dtbTool $DTBTOOL
	fi

	echo "$DTBTOOL -o $INSTALLED_DTIMAGE_TARGET -s $BOARD_KERNEL_PAGESIZE \
						-p $BUILD_KERNEL_OUT_DIR/scripts/dtc/ $BUILD_KERNEL_OUT_DIR/arch/arm/boot/dts/"
	$DTBTOOL -o $INSTALLED_DTIMAGE_TARGET -s $BOARD_KERNEL_PAGESIZE \
						-p $BUILD_KERNEL_OUT_DIR/scripts/dtc/ $BUILD_KERNEL_OUT_DIR/arch/arm/boot/dts/

	chmod a+r $INSTALLED_DTIMAGE_TARGET

	echo ""
	echo "================================="
	echo "END   : FUNC_BUILD_DTIMAGE_TARGET"
	echo "================================="
	echo ""
}

FUNC_CLEAN_DTB

cp -f $BUILD_WHERE/arch/arm/configs/0Nova-Kernel_SM-G901F_defconfig $BUILD_WHERE

mv -f $BUILD_WHERE/0Nova-Kernel_SM-G901F_defconfig $BUILD_WHERE/.config

make -j3

/Kernel_Folder/Toolchain_5.3_a15/bin/arm-cortex_a15-linux-gnueabihf-strip --strip-unneeded $BUILD_WHERE/drivers/staging/qcacld-2.0/wlan.ko

FUNC_BUILD_DTIMAGE_TARGET

mv $BUILD_WHERE/arch/arm/boot/zImage $BUILD_WHERE/arch/arm/boot/boot.img-zImage
mv $BUILD_WHERE/arch/arm/boot/dt.img $BUILD_WHERE/arch/arm/boot/boot.img-dtb
