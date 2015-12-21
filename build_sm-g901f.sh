#!/bin/bash
{
	make mrproper
	make 0Nova-Kernel_SM-G901F_defconfig
        make -j5
	/Kernel_Folder/Toolchain_5.2_a15/bin/arm-cortex_a15-linux-gnueabihf-strip --strip-unneeded drivers/staging/qcacld-2.0/wlan.ko
}
