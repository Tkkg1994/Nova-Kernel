################################################################################

1. How to Build
	- get Toolchain
		From android git server , codesourcery and etc ..
		 - arm-eabi-4.7
  - you can choise defconfig file for skt, kt, lgt
   		
		$ export CROSS_COMPILE=(compiler path)
		$ export ARCH=arm
		$ mkdir output
		$ make apq8084_sec_defconfig VARIANT_DEFCONFIG=apq8084_sec_lentislte_skt_defconfig
		$ make

2. Output files
	- Kernel : output/arch/arm/boot/zImage
	- module : output/drivers/*/*.ko

3. How to Clean	
		$ cd output
		$ make clean
################################################################################
