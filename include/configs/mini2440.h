/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 * Gary Jennejohn <gj@denx.de>
 * David Mueller <d.mueller@elsoft.ch>
 *
 * (C) Copyright 2009-2010
 * Michel Pollet <buserror@gmail.com>
 *
 * (C) Copyright 2012
 * Gabriel Huau <contact@huau-gabriel.fr>
 *
 * Configuation settings for the MINI2440 board.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* Enable debug */
#define DEBUG

/*
 u-boot.mini2440
 SMDK2440 has 1 bank of 64 MB DRAM
 3000'0000 to 3400'0000
 
 Linux-Kernel is expected to be at 3000'8000, entry 3000'8000
 optionally with a ramdisk at 3080'0000

 we load ourself to 33F8'0000
 download area is 3300'0000
 
 TEXT_BASE = 0x33F80000
*/
/* Start of U-Boot */
#define CONFIG_SYS_TEXT_BASE 0x33F80000
#define CONFIG_S3C2440_GPIO

/*
 * High Level Configuration Options
 */
#define CONFIG_ARM920T			/* This is an ARM920T Core	*/
#define CONFIG_S3C24X0			/* in a SAMSUNG S3C24X0 SoC */
#define CONFIG_S3C2440			/* in a SAMSUNG S3C2440 SoC */
#define CONFIG_MINI2440			/* on a MIN2440 Board       */

#define MACH_TYPE_MINI2440	1999
#define CONFIG_MACH_TYPE	MACH_TYPE_MINI2440

/*
 * We don't use lowlevel_init
 */
#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_BOARD_EARLY_INIT_F

/*
 * input clock of PLL
 */
/* MINI2440 has 12.0000MHz input clock */
#define CONFIG_SYS_CLK_FREQ	12000000

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN	(CONFIG_ENV_SIZE + 2048*1024)

/*
 * Hardware drivers
 */
#define CONFIG_DRIVER_DM9000
#define CONFIG_DM9000_BASE				0x20000300
#define DM9000_IO		CONFIG_DM9000_BASE
#define DM9000_DATA		(CONFIG_DM9000_BASE+4)

/*
 * select serial console configuration
 */
#define CONFIG_S3C24X0_SERIAL
#define CONFIG_SERIAL1

/*
 * allow to overwrite serial and ethaddr
 */
#define CONFIG_ENV_OVERWRITE

/*
 * Command definition
 */
#include <config_cmd_default.h>

#define CONFIG_CMD_DHCP
#define CONFIG_CMD_PORTIO
#define CONFIG_CMD_REGINFO
#define CONFIG_CMD_SAVES
#define CONFIG_CMD_SAVEENV
#define CONFIG_CMD_MEMINFO
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_DEVICE

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT	"MINI2440 => "
#define CONFIG_SYS_CBSIZE	256
#define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16)
#define CONFIG_SYS_MAXARGS	32
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START	0x30000000
#define CONFIG_SYS_MEMTEST_END		0x34000000	/* 64MB in DRAM	*/

/* default load address	*/
#define CONFIG_SYS_LOAD_ADDR		0x32000000

/* boot parameters address */
#define CONFIG_BOOT_PARAM_ADDR		0x30000100

/*
 * the PWM TImer 4 uses a counter of 15625 for 10 ms, so we need
 * it to wrap 100 times (total 1562500) to get 1 sec.
 */
#define CONFIG_SYS_HZ			1562500

/*
 * valid baudrates
 */
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }
#define CONFIG_BAUDRATE		115200

/*
 * Stack sizes
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128*1024)	/* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(8*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4*1024)	/* FIQ stack */
#endif

/*
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS        1          /* we have 1 bank of DRAM */
#define PHYS_SDRAM_SIZE             (64*1024*1024) /* 64MB of DRAM */
#define CONFIG_SYS_SDRAM_BASE       0x30000000
#define CONFIG_SYS_FLASH_BASE		0x0

/*
 * Stack should be on the SRAM because
 * DRAM is not init
 */
#define CONFIG_SYS_INIT_SP_ADDR		(0x40001000 - GENERATED_GBL_DATA_SIZE)

/*
 * Config for NAND flash
 */
#define CONFIG_CMD_NAND
#define CONFIG_SYS_MAX_NAND_DEVICE	1	/* Max number of NAND */
#define NAND_MAX_CHIPS			1

/* nand (un)lock commands	?? */
#if 0
#define CONFIG_CMD_NAND_LOCK_UNLOCK
#endif

/*
 * Two NAND Model can be on the board
 * nand_id == 0xec76  Samsung K91208  
 * nand_id == 0xad76  Hynix HY27US08121A 
 */
/* Select driver s3c2440_nand.c */
#define CONFIG_NAND_S3C2440
/* physical address to access nand (@NAND FLASH CONFIGURATION REGISTER) */
#define CONFIG_SYS_NAND_BASE		0x4E000000 /* S3C2440_NAND_BASE (nGCS0)*/
/* Use NAND BBT */
#define CONFIG_S3C2440_NAND_BBT

/* Timing used in u-boot 1.3.2 mini2440 */
#if 1
#define CONFIG_S3C24XX_CUSTOM_NAND_TIMING
#define CONFIG_S3C24XX_TACLS	8
#define CONFIG_S3C24XX_TWRPH0	8
#define CONFIG_S3C24XX_TWRPH1	8
#endif

#if 0
/* S3C2440_NAND_HWECC seems to be broken in u-boot */
#define CONFIG_S3C2440_NAND_HWECC		/* this works for generation, not verification */
#endif

/******************* SELECT NAND or NOR BOOT ************************************/

/*
 * It is possible to have u-boot save it's environment in NOR, however,
 * reember it is incompatible with booting from NAND as the NOR is not
 * available at that point. So use this only if you use nand as storage
 * and will never boot from it
 */
/* #define CONFIG_MINI2440_NOR_ENV */

/*
 * u-boot environmnet
 */
#ifndef CONFIG_MINI2440_NOR_ENV
/* Declare no flash (NOR/SPI), SW2 must select NAND */
/* #define CONFIG_SYS_NO_FLASH */

/* dont define for CONFIG_ENV_IS_IN_FLASH */
/* Define this if you have a NAND device which you want to use for the environment. */
#define CONFIG_ENV_IS_IN_NAND
/* This size must be the size of a common denominator for the NAND erase block */
#define CONFIG_ENV_SIZE		0x20000		/* 128k Total Size of Environment Sector */

/*
 * Enables support for dynamically retrieving the offset of the
 * environment from block zero's out-of-band data.  The
 * "nand env.oob" command can be used to record this offset.
 * Currently, CONFIG_ENV_OFFSET_REDUND is not supported when
 * using CONFIG_ENV_OFFSET_OOB.
*/
/*#define CONFIG_ENV_OFFSET_OOB 
 TODO: à retester, entre temps le driver nand a été fixé.
No dynamic environment marker in OOB block 0
Error reading env offset in OOB
 */

/* addr of environment */
#ifndef CONFIG_ENV_OFFSET_OOB
#define CONFIG_ENV_OFFSET	(0x40000)
#endif

#define MTDIDS_DEFAULT		"nand0=mini2440-nand"

/* We can't access NOR flash when SW2 in on NAND position */
#define CONFIG_SYS_NO_FLASH
/* must be undef since NOR FLASH is not selected */
#undef CONFIG_CMD_IMLS

#else
/*
 * NOR FLASH organization
 * Now uses the standard CFI interface
 * FLASH and environment organization
 */
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
#define CONFIG_SYS_MONITOR_BASE		0x0
/* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_BANKS	1
/* 512 * 4096 sectors, or 32 * 64k blocks */
#define CONFIG_SYS_MAX_FLASH_SECT	512
#define CONFIG_FLASH_SHOW_PROGRESS  1

/*
 * Config for NOR flash
 */
#define CONFIG_ENV_IS_IN_FLASH
#define CONFIG_MY_ENV_OFFSET	0x40000
/* addr of environment */
#define CONFIG_ENV_ADDR		(CONFIG_SYS_FLASH_BASE + CONFIG_MY_ENV_OFFSET)
/* 16k Total Size of Environment Sector */
#define CONFIG_ENV_SIZE		0x4000
#endif

/******************* CONFIG_SPL_BUILD ***************************************/
#if 1
#define CONFIG_SPL
#endif

/*
 * Final target image containing SPL and payload.  Some SPLs
 * use an arch-specific makefile fragment instead, for
 * example if more than one image needs to be produced.
 */
#if 0
#define CONFIG_SPL_TARGET
#endif

/*
 * Enable the SPL framework under common/.  This framework
 * supports MMC, NAND and YMODEM loading of U-Boot and NAND
 * NAND loading of the Linux Kernel.
 */
#define CONFIG_SPL_FRAMEWORK

/*
 * CONFIG_SPL_NAND_SIMPLE: 
 * Active le driver drivers/mtd/nand/nand_spl_simple.c
 * Celui-ci peut être suffisant pour booter
 * 
 * Support for NAND boot using simple NAND drivers that
 * expose the cmd_ctrl() interface.
 * 
 */
#if 1
#define CONFIG_SPL_NAND_SIMPLE
#endif

/*
 * Include standard software ECC in the SPL
 */
#if 0
#define CONFIG_SPL_NAND_ECC
#endif

/*
 * Include nand_base.c in the SPL.  Requires
 * CONFIG_SPL_NAND_DRIVERS.
 */
#define CONFIG_SPL_NAND_BASE

#if 1
#define CONFIG_SPL_NAND_DRIVERS
#endif

/*
 * CONFIG_SPL_NAND_LOAD (drivers/mtd/nand/nand_spl_load.o)
 */
#if 1
#define CONFIG_SPL_NAND_LOAD
#endif

/*
 * CONFIG_SPL_NAND_SUPPORT (drivers/mtd/nand/libnand.o)
 */
#define CONFIG_SPL_NAND_SUPPORT

/* Adress of the start of the stack SPL will use */
#define CONFIG_SPL_STACK 		0xF00

/* TEXT_BASE for linking the SPL binary. */

#define CONFIG_SPL_TEXT_BASE		0x0

/* Maximum binary size (text, data and rodata) of the SPL binary. */
#define CONFIG_SPL_MAX_SIZE		(4 * 1024)	/* 4 KB for stack */

/*
 * CONFIG_SPL_SERIAL_SUPPORT (drivers/serial/libserial.o)
 * define serial_putc */
#if 1
#define CONFIG_SPL_SERIAL_SUPPORT
#endif

#if 0
/* Support for drivers/mtd/spi/libspi_flash.o in SPL binary */
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#endif

/* essayer de ce passer de ces options pour diminuer la taille de u-boot-spl.bin */
/* memset, memcpy */
#define CONFIG_SPL_LIBGENERIC_SUPPORT

#if 0
/* function like memset, memcpy, printf are undef */
/* printf puts... */
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#endif

#if 0
/* "It's not always fitting to use CPU's start.S" --Marek Vasut */
#define CONFIG_SPL_START_S_PATH		"arch/arm/cpu/arm920t/s3c24x0"
#endif

#ifdef CONFIG_SPL
/* K9F1G08U0B */
/* NAND chip page size		*/
#define CONFIG_SYS_NAND_PAGE_SIZE	2048
/* NAND chip block size		*/
#define CONFIG_SYS_NAND_BLOCK_SIZE	(128 * 1024) /* (128k + 4k) Byte */
/* NAND chip page per block count  */
#define CONFIG_SYS_NAND_PAGE_COUNT	64
/* Location of the bad-block label */
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	0	/* ??  CHECK IT */
/* Extra address cycle for A25 */
/* #define CONFIG_SYS_NAND_5_ADDR_CYCLE */
#endif

/* On gagne de la place 3696 => 3520 => 3512 */ 
#if !CONFIG_SPL_BUILD
/* Use NAND BBT */
#define CONFIG_S3C2440_NAND_BBT
#endif

/* Size of the block protected by one OOB (Spare Area in Samsung terminology) */
/* Il y a 512 octets pour proteger un block */
#define CONFIG_SYS_NAND_ECCSIZE		CONFIG_SYS_NAND_PAGE_SIZE	/* OK */
/* Number of ECC bytes per OOB - S3C2440 calculates 3 bytes ECC */
/* four ECC (Error Correction Code) modules ?? */
#define CONFIG_SYS_NAND_ECCBYTES	3	/* CHECK IT */
/* Size of a single OOB region */
#define CONFIG_SYS_NAND_OOBSIZE		16	/* OK */
/* ECC byte positions */
#define CONFIG_SYS_NAND_ECCPOS		{0, 1, 2}	/* CHECK IT */
/* linux/drivers/mtd/nand/s3c2410.c
 * new oob placement block for use with hardware ecc generation
 * static struct nand_ecclayout nand_hw_eccoob = {
 *	.eccbytes = 3,
 *	.eccpos = {0, 1, 2},
 *	.oobfree = {{8, 8}}
 * };
 */

/* Exemple d'utilisation de ces définitions
 * drivers/mtd/nand/nand_spl_simple.c
 * static int nand_ecc_pos[] = CONFIG_SYS_NAND_ECCPOS;
 * 	u_char oob_data[CONFIG_SYS_NAND_OOBSIZE];
 * #define ECCSTEPS	(CONFIG_SYS_NAND_PAGE_SIZE / CONFIG_SYS_NAND_ECCSIZE) == 1
 * #define ECCTOTAL	(ECCSTEPS * CONFIG_SYS_NAND_ECCBYTES) == 3
 * 
 *	Pick the ECC bytes out of the oob data
 *	for (i = 0; i < ECCTOTAL; i++)
 *		ecc_code[i] = oob_data[nand_ecc_pos[i]];
 */

/* Location in NAND to read U-Boot from */
#define CONFIG_SYS_NAND_U_BOOT_OFFS	(4 * 1024)	/* Offset to RAM U-Boot image */
/* Location in memory to load U-Boot to */
#define CONFIG_SYS_NAND_U_BOOT_DST	0x33F80000	/* NUB load-addr      */
/* Entry point in loaded image to jump to */
#define CONFIG_SYS_NAND_U_BOOT_START	CONFIG_SYS_NAND_U_BOOT_DST	/* NUB start-addr     */
/* size of u-boot, for NAND loading */
#define CONFIG_SYS_NAND_U_BOOT_SIZE	(256 * 1024)	/* Size of RAM U-Boot image   */

/******************* END CONFIG_SPL_BUILD ************************************/

/* ATAG configuration */
#define CONFIG_INITRD_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_CMDLINE_TAG
#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE

#define CONFIG_BOOTDELAY	3
#define CONFIG_BOOTARGS    	"root=/dev/mtdblock3 rootfstype=jffs2 console=ttySAC0,115200"
#define CONFIG_ETHADDR	        08:08:11:18:12:27
#define CONFIG_NETMASK         255.255.255.0
#define CONFIG_IPADDR		10.0.0.111
#define CONFIG_SERVERIP		10.0.0.4

#define CONFIG_BOOTCOMMAND	""

#define CONFIG_EXTRA_ENV_SETTINGS	\
	"usbtty=cdc_acm\0" \
	"mtdparts=mtdparts=mini2440-nand:256k@0(u-boot),128k(env),5m(kernel),-(root)\0" \
	"mini2440=mini2440=3tb\0" \
	"bootargs_base=console=ttySAC0,115200 noinitrd\0" \
	"bootargs_init=init=/sbin/init\0" \
	"root_nand=root=/dev/mtdblock3 rootfstype=jffs2\0" \
	"root_mmc=root=/dev/mmcblk0p2 rootdelay=2\0" \
	"root_nfs=/mnt/nfs\0" \
	"set_root_nfs=setenv root_nfs root=/dev/nfs rw nfsroot=${serverip}:${root_nfs}\0" \
	"ifconfig_static=run setenv ifconfig ip=${ipaddr}:${serverip}::${netmask}:mini2440:eth0\0" \
	"ifconfig_dhcp=run setenv ifconfig ip=dhcp\0" \
	"ifconfig=ip=dhcp\0" \
	"set_bootargs_mmc=setenv bootargs ${bootargs_base} ${bootargs_init} ${mini2440} ${root_mmc}\0" \
	"set_bootargs_nand=setenv bootargs ${bootargs_base} ${bootargs_init} ${mini2440} ${root_nand}\0" \
	"set_bootargs_nfs=run set_root_nfs\; setenv bootargs ${bootargs_base} ${bootargs_init} ${mini2440} ${root_nfs} ${ifconfig}\0" \
	""

#endif	/* __CONFIG_H */
