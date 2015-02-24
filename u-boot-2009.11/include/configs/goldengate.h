/*
 * Copyright (c) 2011 by Cortina Systems Incorporated.
 *
 * Configuration settings for the GoldenGate Board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#ifndef __CONFIG_H
#define __CONFIG_H

// g2autoconf.h is created automatically during the openwrt
// build process.  It will contain #defines for every enabled
// CONFIG_UBOOT_* option in the openwrt .config file.
//
#include "g2autoconf.h"

/*
 * Board info register
 */
#define SYS_ID  (0x10000000)
#define ARM_SUPPLIED_REVISION_REGISTER SYS_ID

#ifdef CONFIG_MK_FPGA
	#define CONFIG_SYSTEM_CLOCK		(50*1000000)
	#define CONFIG_IDENT_STRING		" GoldenGate SoC FPGA"
#else
	#define CONFIG_SYSTEM_CLOCK		(100*1000000)
	#define CONFIG_IDENT_STRING		" GoldenGate SoC ASIC"
#endif

/*
 * High Level Configuration Options
 */
#define CONFIG_GOLDENGATE		1
#define CONFIG_FM_A9_SMP		1

#define CONFIG_SHOW_BOOT_PROGRESS	1

/* Timer Configuration */
#define CONFIG_SYS_HZ			(1000)

/* GLOBAL_STRAP register bit 0 for secure mode indication */
#define GLOBAL_STRAP			0xf000000c

/* global register 2 for S/W use */
#define	GLOBAL_SOFTWARE2		0xf00000bc

#define CONFIG_CMDLINE_TAG		1 /* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_MISC_INIT_R		1 /* call misc_init_r during start up */

#define CONFIG_MD5

/*
 * Number of bytes reserved for initial data
 */
#define CONFIG_SYS_GBL_DATA_SIZE	128

/*
 * Hardware drivers
 */
#define CONFIG_NET_MULTI

#define CONFIG_GOLDENGATE_SPI		1

/*
 * G2 Serial Configuration
 */
#define CONFIG_GOLDENGATE_SERIAL

#define CONFIG_CONS_INDEX		0

#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*
 * Command line configuration.
 */
#define CONFIG_CMD_BDI
#define CONFIG_CMD_BOOTD
#if defined(CONFIG_MTD_CORTINA_CS752X_NAND) || defined(CONFIG_MTD_CORTINA_CS752X_SERIAL)
#else
#define CONFIG_CMD_FLASH
#endif

#define CONFIG_CMD_IMI
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_NET
#define CONFIG_CMD_PING
#define CONFIG_CMD_SAVEENV
#define CONFIG_CMD_LOADB
#define CONFIG_CMD_HTTP
#define CONFIG_CMD_DHCP

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME
#define CONFIG_BOOTP_SUBNETMASK

#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_BOOTDELAY		0

#define CONFIG_BOOTCOMMAND "run mtd_boot;run switch_boot_partition;run mtd_boot;fail"

/*
 * Static configuration when assigning fixed address
 */
#define CONFIG_ETHADDR			00:50:C2:11:22:33	/* talk on MY local MAC address */
#define CONFIG_ETH1ADDR		00:50:C2:44:55:66
#define CONFIG_ETH2ADDR		00:50:C2:77:88:99
#define CONFIG_NETMASK			255.255.254.0		/* talk on MY local net */
#define CONFIG_IPADDR			10.0.11.194		/* static IP I currently own */
#define CONFIG_SERVERIP		10.0.11.171		/* current IP of my dev pc */
#define CONFIG_GATEWAYIP		10.0.10.1		/* current gateway IP */
#define CONFIG_BOOTFILE		uImage

/* Allow overwrite of ethaddr
 */
#define CONFIG_ENV_OVERWRITE

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP			/* undef to save memory */
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O Buffer Size */

/* Monitor Command Prompt	 */
#define CONFIG_SYS_PROMPT		"U-BOOT # "
#define CONFIG_CMDLINE_EDITING		/* add command line history	*/
#define CONFIG_AUTO_COMPLETE		/* add autocompletion support	*/

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE	\
			(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		32		/* max number of command args */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE /* Boot Argument Buffer Size */

#ifdef CONFIG_MK_FPGA
	#define CONFIG_SYS_LOAD_ADDR		0xe1600000	/* default load address */
#else
	#define CONFIG_SYS_LOAD_ADDR		0x01600000	/* default load address */
#endif

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE			(256 * 1024)	/* regular stack */
#ifdef CONFIG_USE_IRQ
	#define CONFIG_STACKSIZE_IRQ		(4 * 1024)	/* IRQ stack */
	#define CONFIG_STACKSIZE_FIQ		(4 * 1024)	/* FIQ stack */
#endif

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#ifdef CONFIG_MK_FPGA
	#define CONFIG_NR_DRAM_BANKS		1		/* we have 1 bank of DRAM */
	#define PHYS_SDRAM_1			0xe0000000	/* SDRAM Bank #1 */
	#define PHYS_SDRAM_1_SIZE		0x08000000	/* we can only use maximum 128 MB */
	#define CONFIG_SYS_MEMTEST_START	0xe0100000
	#define CONFIG_SYS_MEMTEST_END		0xf0000000
#else
	#define CONFIG_NR_DRAM_BANKS		1		/* we have 1 bank of DRAM */
	#define PHYS_SDRAM_1			0x00000000	/* SDRAM Bank #1 */
	#define PHYS_SDRAM_1_SIZE		0x10000000	/* 256 MB */
	#define CONFIG_SYS_MEMTEST_START	0x00100000
	#define CONFIG_SYS_MEMTEST_END		0xf0000000
#endif

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
/*
 * Note that CONFIG_SYS_MAX_FLASH_SECT allows for a parameter block
 * i.e. the bottom "sector" (bottom boot), or top "sector"
 *      (top boot), is a seperate erase region divided into
 *      4 (equal) smaller sectors. This, notionally, allows
 *      quicker erase/rewrire of the most frequently changed
 *      area......
 *      CONFIG_SYS_MAX_FLASH_SECT is padded up to a multiple of 4
 */
#ifdef CONFIG_MK_FPGA
	#define CONFIG_SYS_FLASH_BASE		0xe8000000
	#define PHYS_FLASH_SIZE			0x04000000	/* 64MB */
#else
	#define CONFIG_SYS_FLASH_BASE		0xe0000000
	#define PHYS_FLASH_SIZE			0x08000000	/* 128MB */
#endif

#define PHYS_FLASH_1			(CONFIG_SYS_FLASH_BASE)

#define CONFIG_SYS_MAX_FLASH_BANKS	1		/* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT	(512 * 2)


/*
 * Environment settings
 */
#if defined(CONFIG_CMD_FLASH) || defined(CONFIG_MTD_CORTINA_CS752X_SERIAL)
	#undef 	CONFIG_MTD_CORTINA_CS752X_NAND
#endif

#if defined(CONFIG_MTD_CORTINA_CS752X_NAND)
	#define CONFIG_NAND_CS75XX		1
	#define	CONFIG_NAND_G2			1
	#define	CONFIG_CMD_NAND			1
	#define CONFIG_CS75XX_NAND_REGS_BASE	0xf0050000
	#define CONFIG_SYS_MAX_NAND_DEVICE	1
	#define CONFIG_SYS_NAND_MAX_CHIPS	2
	#define CONFIG_SYS_NAND_BASE		0xE0000000
	#define CONFIG_SYS_NAND_BASE_LIST	{ CONFIG_SYS_NAND_BASE }
	/* #define CONFIG_JFFS2_NAND */
	#define CONFIG_MTD_NAND_VERIFY_WRITE    1
	#define CONFIG_CS75XX_NAND_HWECC
	/* The following parameters defined in g2autocon.h : */
	/*#define	CONFIG_CS75XX_NAND_HWECC_HAMMING_256 */
	/*#define	CONFIG_CS75XX_NAND_HWECC_HAMMING_512 */
	/*#define	CONFIG_CS75XX_NAND_HWECC_BCH_8       */
	/*#define	CONFIG_CS75XX_NAND_HWECC_BCH_12      */

	#define CONFIG_SYS_NO_FLASH
	#undef CONFIG_ENV_IS_IN_FLASH

	#define CONFIG_ENV_IS_IN_NAND
	#define PHYS_FLASH_1			(CONFIG_SYS_FLASH_BASE)

	#ifdef G2_UBOOT_ENV_OFFSET
		#define CONFIG_ENV_OFFSET	(G2_UBOOT_ENV_OFFSET)
	#else
		#define CONFIG_ENV_OFFSET	(0x100000)
	#endif

	#ifdef G2_UBOOT_ENV_SIZE
		#define CONFIG_ENV_SIZE		(G2_UBOOT_ENV_SIZE)
	#else
		#define CONFIG_ENV_SIZE		(0x20000)
	#endif

	#define CONFIG_ENV_OFFSET_REDUND    	(0x180000)

	#ifdef G2_UBOOT_ENV_RANGE
		#define CONFIG_ENV_RANGE	(G2_UBOOT_ENV_RANGE)
	#else
		#define CONFIG_ENV_RANGE	(0x100000)
	#endif

	#define CONFIG_ENV_SECT_SIZE		(CONFIG_ENV_SIZE)
	#define CONFIG_ENV_ADDR 		(PHYS_FLASH_1 + CONFIG_ENV_OFFSET)
	#define CONFIG_SYS_MONITOR_BASE		(PHYS_FLASH_1)
	#define CONFIG_SYS_MONITOR_LEN		(0x100000)	/* Reserve 1MB */
#endif

#if defined(CONFIG_MTD_CORTINA_CS752X_SERIAL)
	#define CONFIG_CMD_SF
	#define CONFIG_SYS_NO_FLASH
	#undef CONFIG_ENV_IS_IN_FLASH
	#define CONFIG_ENV_IS_IN_SPI_FLASH
	#define PHYS_FLASH_1			(CONFIG_SYS_FLASH_BASE)

	#define CONFIG_ENV_SPI_MAX_HZ 		1000000
	#define CONFIG_ENV_SPI_MODE 		SPI_MODE_3
	#define CONFIG_ENV_SPI_BUS 		0
	#define CONFIG_ENV_SPI_CS 		1

	#ifdef G2_UBOOT_ENV_OFFSET
		#define CONFIG_ENV_OFFSET	(G2_UBOOT_ENV_OFFSET)
	#else
		#define CONFIG_ENV_OFFSET	(0x40000)
	#endif

	#ifdef G2_UBOOT_ENV_SIZE
		#define CONFIG_ENV_SIZE		(G2_UBOOT_ENV_SIZE)
	#else
		#define CONFIG_ENV_SIZE		(0x40000)
	#endif

	#ifdef G2_UBOOT_ENV_RANGE
		#define CONFIG_ENV_RANGE	(G2_UBOOT_ENV_RANGE)
	#else
		#define CONFIG_ENV_RANGE	(0x40000)
	#endif

	#define CONFIG_ENV_SECT_SIZE		(CONFIG_ENV_SIZE)
	#define CONFIG_ENV_ADDR 		(PHYS_FLASH_1 + CONFIG_ENV_OFFSET)
	#define CONFIG_SYS_MONITOR_BASE		(PHYS_FLASH_1)
	#define CONFIG_SYS_MONITOR_LEN		(0x100000)	/* Reserve 1MB */
#endif

#ifdef CONFIG_CMD_FLASH
	#undef CONFIG_SYS_NO_FLASH
	#define CONFIG_ENV_IS_IN_FLASH		1
	#define CONFIG_ENV_SIZE			(0x20000)
	#define CONFIG_ENV_SECT_SIZE		(CONFIG_ENV_SIZE)
	#define CONFIG_ENV_ADDR 		(PHYS_FLASH_1 + 0x00040000)
	#define CONFIG_SYS_MONITOR_BASE		(PHYS_FLASH_1)
	#define CONFIG_SYS_MONITOR_LEN		(256 * 1024)	/* Reserve 256KiB */

	/* Timeout values are in ticks */
	/* Lengthened for bad boards */
	#define CONFIG_SYS_FLASH_ERASE_TOUT	(20 * CONFIG_SYS_HZ) /* Erase Timeout */
	#define CONFIG_SYS_FLASH_WRITE_TOUT	(20 * CONFIG_SYS_HZ) /* Write Timeout */
	#define CONFIG_SYS_FLASH_PROTECTION	/* The devices have real protection */
	#define CONFIG_SYS_FLASH_EMPTY_INFO	/* flinfo indicates empty blocks */

	/*
	 * Use the CFI flash driver for ease of use
	 */
	#define CONFIG_SYS_FLASH_CFI
	#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE 	1	/* Use buffered writes (~10x faster) */
	#define CONFIG_FLASH_CFI_DRIVER

	//#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
	#define CONFIG_FLASH_PART_BOOTLOADER0_LOC      (CONFIG_SYS_FLASH_BASE + 0x000000)
	#define CONFIG_FLASH_PART_BOOTENV0_LOC         (CONFIG_SYS_FLASH_BASE + 0x040000)
	#define CONFIG_FLASH_PART_BOOTLOADER1_LOC      (CONFIG_SYS_FLASH_BASE + 0x060000)
	#define CONFIG_FLASH_PART_BOOTENV1_LOC         (CONFIG_SYS_FLASH_BASE + 0x0a0000)
	#define CONFIG_FLASH_PART_SUPER0_LOC           (CONFIG_SYS_FLASH_BASE + 0x0c0000)
	#define CONFIG_FLASH_PART_SUPER1_LOC           (CONFIG_SYS_FLASH_BASE + 0x0e0000)
	#define CONFIG_FLASH_PART_KERNEL0_LOC          (CONFIG_SYS_FLASH_BASE + 0x100000)
	#define CONFIG_FLASH_PART_KERNEL1_LOC          (CONFIG_SYS_FLASH_BASE + 0xb00000)
#endif

#define CONFIG_UBOOT_DEF_ENV										\
	"hostname=bhr4\0"										\
	"user_dir=bhr4\0"										\
	"rootpath=/nfsroot/bhr4/root\0"									\
	"netdev=eth2\0"											\
	"raddr_ramdisk=0x1000000\0"									\
	"raddr_tftp=0x5000000\0"									\
	"rsize_tftp=0x5000000\0"									\
	"raddr_kernel=0x5000000\0"									\
	"naddr_uboot=0x20000\0"										\
	"nsize_uboot=0x40000\0"										\
	"naddr_uboot_env_pri=0x100000\0"								\
	"nsize_uboot_env=0x100000\0"									\
	"naddr_uboot_env_sec=0x180000\0"								\
	"naddr_kernel_even=0x200000\0"									\
	"naddr_kernel_odd=0xc00000\0"									\
	"nsize_kernel=0xA00000\0"									\
	"naddr_ubi=0x1600000\0"										\
	"nsize_ubi=0xEA00000\0"										\
	"uboot_upd=bcm_init && tftp ${raddr_tftp} ${user_dir}/bootloader.bin"				\
		" && nand erase ${naddr_uboot} ${nsize_uboot}"						\
		" && nand write ${raddr_tftp} ${naddr_uboot} ${filesize}\0"				\
	"addip=setenv bootargs ${bootargs} "                                     		      	\
		"ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}"                      		\
		":${hostname}:${netdev}:off panic=1\0"                                  		\
	"addtty=setenv bootargs ${bootargs} console=ttyS0,${baudrate}\0"				\
	"addmisc=setenv bootargs ${bootargs} SB_PHY=PPPS qm_int_buff=${QM_INT_BUFF}\0"			\
	"addmacaddr=setenv bootargs ${bootargs} ethaddr0=${ethaddr} ethaddr1=${eth1addr} ethaddr2=${eth2addr} "	\
		"wifiaddr0=${wifiaddr0} wifiaddr1=${wifiaddr1}\0"					\
	"addmtdparts=setenv bootargs ${bootargs} mtdparts=cs752x_nand_flash:256K@${naddr_uboot}(bootloader),512k@${naddr_uboot_env_pri}(uboot_env_pri),512k@${naddr_uboot_env_sec}(uboot_env_sec),10M@${naddr_kernel_even}(kernel_initrd_even),10M@${naddr_kernel_odd}(kernel_initrd_odd),224M@${naddr_ubi}(ubi)\0"		\
	"mtd_boot=run set_current_kernel; nboot ${raddr_kernel} 0 ${current_kernel}; run ramargs addtty addmisc addmacaddr addmtdparts; bootm ${raddr_kernel}\0"	\
	"ramdisk_file=uRamdisk\0"									\
	"ramargs=setenv bootargs init=/init root=/dev/ram rw\0"						\
	"net_boot=bcm_init && run net_kernel_load net_ramdisk_load && run ramargs addtty addmisc addmacaddr addmtdparts "		\
		" && bootm ${raddr_kernel} ${raddr_ramdisk}\0"						\
	"net_kernel_load=tftp ${raddr_kernel} ${user_dir}/${bootfile}\0"				\
	"net_ramdisk_load=tftp ${raddr_ramdisk} ${user_dir}/${ramdisk_file}\0"				\
	"kernel_upd=run set_current_kernel;"								\
		"bcm_init && tftp ${raddr_tftp} ${user_dir}/kernel_initrd.img"				\
		" && nand erase ${current_kernel} ${nsize_kernel}"					\
		" && nand write ${raddr_tftp} ${current_kernel} ${filesize};\0"				\
	"ubi_upd=bcm_init && tftp ${raddr_tftp} ${user_dir}/ubifs.img"					\
		" && nand erase ${naddr_ubi} ${nsize_ubi}"						\
		" && nand write ${raddr_tftp} ${naddr_ubi} ${filesize}\0"				\
	"erase_env=echo Erasing U-Boot env;nand erase clean ${naddr_uboot_env_pri} ${nsize_uboot_env};\0"	\
	"boot_partition=0\0"										\
	"switch_boot_partition=if test ${boot_partition} -eq 0;"					\
		"then setenv boot_partition 1;"								\
		"else setenv boot_partition 0;"								\
		"fi;\0"											\
	"set_current_kernel=if test ${boot_partition} -eq 0;"						\
		"then setenv current_kernel ${naddr_kernel_even};"					\
		"else setenv current_kernel ${naddr_kernel_odd};"					\
		"fi\0"											\


#define CONFIG_EXTRA_ENV_SETTINGS									\
	CONFIG_UBOOT_DEF_ENV
/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN		(4*CONFIG_ENV_SIZE + 128 * 1024)

/*
 * I2C Settings
 */
#if 0
	#define CONFIG_SYS_I2C_PORT         	I2C2_BASE_ADDR
	#define CONFIG_HARD_I2C			1 /* To enable I2C support	*/
	#undef	CONFIG_SOFT_I2C			  /* I2C bit-banged		*/
	#define CONFIG_I2C_CMD_TREE    		1
	#define CONFIG_CMD_I2C         		1
	#define CONFIG_SYS_I2C_SPEED		100000//400000		/* I2C speed and slave address	*/
	#define CONFIG_SYS_I2C_SLAVE		0x10
#endif

// If you intend to support "initramfs" style root file system (where
// the rootfs is compiled into the kernel image), then you must define:
#define CONFIG_INITRD_TAG   		1

// Run is needed for multi-image support.  We'll use the run
// command at the u-boot prompt as part of the multi-image
// boot process
#define CONFIG_CMD_RUN
#define CONFIG_SYS_HUSH_PARSER
#ifdef	CONFIG_SYS_HUSH_PARSER
#define	CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#endif


#if defined(CONFIG_MTD_CORTINA_CS752X_NAND) || defined(CONFIG_MTD_CORTINA_CS752X_SERIAL)
#else
	#define CONFIG_CMD_JFFS2
	#define CONFIG_SYS_JFFS2_FIRST_BANK 0
	#define CONFIG_SYS_JFFS2_NUM_BANKS  1
	// #define CONFIG_SYS_JFFS2_SORT_FRAGMENTS
	#define CONFIG_SYS_JFFS_CUSTOM_PART

/*====================================================================*/
/* These two are the critical parameters telling                      */
/* where the jffs2 partition is carved out.  u-boot must              */
/* know where the JFFS2 file system starts and how big it is.         */
/*                                                                    */
/* THESE VALUES MUST MATCH THE VALUES IN                              */
/*   openwrt-2.4.2011-trunk/target/linux/g2/image/flash-config.yml    */
/* for                                                                */
/*   jffs_sector:          136                                        */
/*   jffs_size:            40M                                        */
/*====================================================================*/

	#define CONFIG_JFFS2_PART_OFFSET 	(CONFIG_ENV_SECT_SIZE * 40)
	#define CONFIG_JFFS2_PART_SIZE   	(CONFIG_ENV_SECT_SIZE * 320)
#endif /* CONFIG_MTD_CORTINA_CS752X_NAND */

#endif	/* __CONFIG_H */

