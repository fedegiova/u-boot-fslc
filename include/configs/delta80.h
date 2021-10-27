/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 */

#ifndef __DELTA80_CONFIG_H
#define __DELTA80_CONFIG_H

#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV		"ttymxc0"

#include "mx6_common.h"

#define CONFIG_IMX_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      0





#define CONFIG_EXTRA_ENV_SETTINGS ""\
		"console=serial\0"						\
		"netdev=eth0\0"						\
        "panel=Ampire\0" \
        "splashimage=0x11000000\0" \
        "splashsize=0x80000\0" \
        "splashpos=m,m\0" \
        "splashsource=mmc_raw\0" \
		"ethprime=FEC0\0"					\
		"uboot=u-boot.bin\0"			\
		"kernel=uImage\0"				\
        "ipaddr=172.31.4.253\0"\
        "netmask=255.255.255.0\0"\
        "ethaddr=00:0C:0C:A0:02:b5\0"   \
        "memtest_iter=50\0"   \
		"nfsroot=/home/federico/ltib/ltib/rootfs\0"				\
		"bootargs_base=setenv bootargs console=ttymxc0,115200 video=mxcfb0:dev=ldb,LDB-SVGA,if=RGB666 ldb=sin0 vmalloc=192M setenv vram=1:3M,2:3M,3:3M,4:3M,5:3M\0"\
		"bootargs_nfs=setenv bootargs ${bootargs} root=/dev/nfs "\
		"ip=${ipaddr}:${serverip}:${gateway}:255.255.0.0:delta80:eth1 nfsroot=${serverip}:${nfsroot},v3,tcp\0"\
		"bootcmd_net=run bootargs_base bootargs_nfs; "		\
		"tftpboot ${loadaddr} ${kernel}; bootm\0"	\
		"bootargs_mmc=setenv bootargs ${bootargs}"     \
		" root=/dev/mmcblk0p1 rootwait ip=none enable_wait_mode=off\0"                \
		"bootcmd_mmc=run bootargs_base bootargs_mmc; "   \
		"mmc dev 0; "	\
		"mmc read ${loadaddr} 0x800 0x3000; bootm\0"	\
		"bootcmd_mmc_nfs=run bootargs_base bootargs_nfs; "   \
		"mmc dev 0; "	\
		"mmc read ${loadaddr} 0x800 0x3000; bootm\0"	\
        "bootcmd_installer=run bootargs_base; sf probe; sf read ${loadaddr} 0x82000 0x380000; sf read 0x12800000 0x400000 0x400000; bootm ${loadaddr}  0x12800000\0" \
		"bootcmd=run bootcmd_mmc\0"                             \
        "bootcmd_mtest=sf probe; sf read  0x00907000 0x60000 0x20000;go 0x00907000\0" \
        "bootcmd_memtest=mtest 0x11000000 0x1f000000 0 ${memtest_iter}; mark_mtest_complete $?; setenv bootargs_base \"${bootargs_base} memtest_mode cma=64M\"; run bootcmd_mmc\0"\

#define CONFIG_ARP_TIMEOUT     200UL

/* Physical Memory Map */
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VGA_AS_SINGLE_DEVICE /*<< Set fb as an output only device*/
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_LOGO
//#define CONFIG_VIDEO_BMP_LOGO
/*	Since we have modified the clock routing in delta80.c to use PLL2_PDF0 at 280MHz
 *	to generate a 40MHz display clock with a /7 divisor the LDB_CLOCK
 *	changed from the default of 65Mhz to 40Mhz (ipu_common.c) and we need
 *	to make the driver aware of it
 * */
#define CONFIG_SYS_LDB_CLOCK 40000000
#define CONFIG_SYS_IPUV3_CLK 280000000
#define CONFIG_IMX_VIDEO_SKIP

#define CONFIG_USBD_HS

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* need to enable this since adapters are into an array so addressing won't work if the previous element is not enabled*/
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  100000

/* RTC */
#define CONFIG_SYS_RTC_BUS_NUM     1   //enable bootcounter over rtc
#define CONFIG_SYS_I2C_RTC_ADDR    0x6f
#define CONFIG_SYS_BOOTCOUNT_ADDR  0x20
#define CONFIG_BOOTCOUNT_ALEN      1

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */
#endif


#endif                         /* __DELTA80_CONFIG_H */
