/*
 * (C) Copyright 2006 OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
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

#include <common.h>

#include <nand.h>
#include <asm/arch/s3c24x0_cpu.h>
#include <asm/io.h>

/* from linux/arch/arm/plat-samsung/include/plat/regs-nand.h */
#define S3C2440_NFCONF_BUSWIDTH_8	(0<<0)
#define S3C2440_NFCONF_BUSWIDTH_16	(1<<0)
#define S3C2440_NFCONF_ADVFLASH		(1<<3)
#define S3C2440_NFCONF_TACLS(x)		((x)<<12)
#define S3C2440_NFCONF_TWRPH0(x)	((x)<<8)
#define S3C2440_NFCONF_TWRPH1(x)	((x)<<4)

#define S3C2440_NFCONT_LOCKTIGHT	(1<<13)
#define S3C2440_NFCONT_SOFTLOCK		(1<<12)
#define S3C2440_NFCONT_ILLEGALACC_EN	(1<<10)
#define S3C2440_NFCONT_RNBINT_EN	(1<<9)
#define S3C2440_NFCONT_RN_FALLING	(1<<8)
#define S3C2440_NFCONT_SPARE_ECCLOCK	(1<<6)
#define S3C2440_NFCONT_MAIN_ECCLOCK	(1<<5)
#define S3C2440_NFCONT_INITECC		(1<<4)
#define S3C2440_NFCONT_nFCE		(1<<1)
#define S3C2440_NFCONT_ENABLE		(1<<0)

#define S3C2440_NFSTAT_READY		(1<<0)
#define S3C2440_NFSTAT_nCE		(1<<1)
#define S3C2440_NFSTAT_RnB_CHANGE	(1<<2)
#define S3C2440_NFSTAT_ILLEGAL_ACCESS	(1<<3)

/* Nand base 0x4E000000 */
#define S3C2440_ADDR_NALE 0x8   /* NFCMD */
#define S3C2440_ADDR_NCLE 0xc   /* NFADDR */
#define S3C2440_ADDR_NDATA 0x10 /* NFDATA */

#ifdef CONFIG_SPL_BUILD

/* in the early stage of NAND flash booting, printf() is not available */
#define printf(fmt, args...)

/* 
 * nand_read_buf() is already defined in include/linux/mtd/nand.h
 */

#endif

static void s3c2440_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	struct s3c2440_nand *nand = s3c2440_get_base_nand();

	debug("hwcontrol(): 0x%02x 0x%02x\n", cmd, ctrl);

	if (ctrl & NAND_CTRL_CHANGE) {
		ulong IO_ADDR_W = (ulong)nand;
		switch (ctrl & 0x6){ /* bit 2, 1 */
		  case NAND_CLE:
			IO_ADDR_W |= S3C2440_ADDR_NALE; /* NFCMD */
		    break;
		  case NAND_ALE:
			IO_ADDR_W |= S3C2440_ADDR_NCLE; /* NFADDR */
		    break;
		  default:
			IO_ADDR_W |= S3C2440_ADDR_NDATA; /* NFDATA */
		}

		chip->IO_ADDR_W = (void *)IO_ADDR_W;

		if (ctrl & NAND_NCE){
			/* Force nFCE to low (Enable chip select) */
			writel(readl(&nand->nfcont) & ~S3C2440_NFCONT_nFCE,
			       &nand->nfcont);
		}
		else{
#ifdef CONFIG_S3C2440_NAND_HWECC
// NFCONT = (NFCONT & ~((1 << 5) | NFCONF_nFCE)) | S3C2440_NFCONT_INITECC;
			/* Force nFCE to high (Disable chip select) */
			writel(readl(&nand->nfcont) | S3C2440_NFCONT_nFCE,
			       &nand->nfcont);
#else
			/* Force nFCE to low (Enable chip select) */
			writel(readl(&nand->nfcont) & ~S3C2440_NFCONT_nFCE,
			       &nand->nfcont);
#endif
		}
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, chip->IO_ADDR_W);
}

static int s3c2440_dev_ready(struct mtd_info *mtd)
{
	struct s3c2440_nand *nand = s3c2440_get_base_nand();
	debug("dev_ready\n");
	return readl(&nand->nfstat) & S3C2440_NFSTAT_READY;
}

#ifdef CONFIG_S3C2440_NAND_HWECC
/* 
 * s3c2440 : 28-bit ECC Parity Code = 22-bit Line parity + 6bit Column Parity 
 * s3c2410 : 24-bit ECC Parity Code = 18-bit Line parity + 6-bit Column Parity
 */
void s3c2440_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct s3c2440_nand *nand = s3c2440_get_base_nand();
	debug("s3c2440_nand_enable_hwecc(%p, %d)\n", mtd, mode);
	/* & ~(1 << 5): Unlock main data area ECC generation */
	writel( (readl(&nand->nfcont) & ~(1 << 5)) | S3C2440_NFCONT_INITECC, &nand->nfcont);
}

static int s3c2440_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				      u_char *ecc_code)
{
	struct s3c2440_nand *nand = s3c2440_get_base_nand();
 	ecc_code[0] = readb(&nand->nfeccd0);
 	ecc_code[1] = readb(&nand->nfeccd0 + 1);
 	ecc_code[2] = readb(&nand->nfeccd0 + 2);
	debug("s3c2440_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[0], ecc_code[1], ecc_code[2]);

	return 0;
}

static int s3c2440_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	if (read_ecc[0] == calc_ecc[0] &&
	    read_ecc[1] == calc_ecc[1] &&
	    read_ecc[2] == calc_ecc[2])
		return 0;

	printf("s3c2440_nand_correct_data: not implemented\n");
	return -1;
}
#endif

int board_nand_init(struct nand_chip *nand)
{
	u_int32_t set;
	u_int8_t tacls, twrph0, twrph1;
	struct s3c24x0_clock_power *clk_power = s3c24x0_get_base_clock_power();
	struct s3c2440_nand *nand_reg = s3c2440_get_base_nand();

	debug("board_nand_init()\n");

	writel(readl(&clk_power->clkcon) | (1 << 4), &clk_power->clkcon);

	/* initialize hardware */
#if defined(CONFIG_S3C24XX_CUSTOM_NAND_TIMING)
	debug("s3c24x0 NAND: use custom NAND timing\n");
	tacls  = CONFIG_S3C24XX_TACLS;
	twrph0 = CONFIG_S3C24XX_TWRPH0;
	twrph1 =  CONFIG_S3C24XX_TWRPH1;
#else
	debug("s3c24x0 NAND: use default NAND timing\n");
	tacls = 4; // tacls_max
	twrph0 = 8;
	twrph1 = 8;
#endif

	set  = S3C2440_NFCONF_TACLS(tacls - 1);
	set |= S3C2440_NFCONF_TWRPH0(twrph0 - 1);
	set |= S3C2440_NFCONF_TWRPH1(twrph1 - 1);
	writel(set, &nand_reg->nfconf);
	debug("s3c24x0 NAND: nfconf = %x\n", nand_reg->nfconf);

	/* enable the controller and de-assert nFCE */
	set  = S3C2440_NFCONT_SPARE_ECCLOCK;
	set |= S3C2440_NFCONT_MAIN_ECCLOCK;
	set |= S3C2440_NFCONT_INITECC;
	set |= S3C2440_NFCONT_nFCE;
	set |= S3C2440_NFCONT_ENABLE;
	writel(set, &nand_reg->nfcont);
	debug("s3c24x0 NAND: nfcont = %x\n", nand_reg->nfcont);

	/* initialize nand_chip data structure */
	nand->IO_ADDR_R = (void *)&nand_reg->nfdata;
	nand->IO_ADDR_W = (void *)&nand_reg->nfdata;
	debug("s3c24x0 NAND: @nfdata = 0x%x\n",&nand_reg->nfdata);
	
	nand->select_chip = NULL;

	/* read_buf and write_buf are default */
	/* read_byte and write_byte are default */
#ifdef CONFIG_SPL_BUILD
	nand->read_buf = nand_read_buf;
#endif

	/* hwcontrol always must be implemented */
	nand->cmd_ctrl = s3c2440_hwcontrol;

	nand->dev_ready = s3c2440_dev_ready;

#ifdef CONFIG_S3C2440_NAND_HWECC
	debug("s3c24x0 NAND: use HWECC\n");
	nand->ecc.hwctl = s3c2440_nand_enable_hwecc;
	nand->ecc.calculate = s3c2440_nand_calculate_ecc;
	nand->ecc.correct = s3c2440_nand_correct_data;
	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.size = CONFIG_SYS_NAND_ECCSIZE;
	nand->ecc.bytes = CONFIG_SYS_NAND_ECCBYTES;
#else
	debug("s3c24x0 NAND: use ECC_SOFT\n");
	nand->ecc.mode = NAND_ECC_SOFT;
#endif

#ifdef CONFIG_S3C2440_NAND_BBT
	/* If you use u-boot BBT creation code, specifying this flag will
	 * let the kernel fish out the BBT from the NAND, and also skip the
	 * full NAND scan that can take 1/2s or so. Little things... */
	nand->options = NAND_USE_FLASH_BBT;
	debug("s3c24x0 NAND: use NAND_USE_FLASH_BBT\n");
#else
	nand->options = 0;
#endif

	debug("end of nand_init\n");

	return 0;
}
