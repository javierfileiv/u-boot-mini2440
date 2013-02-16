/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
 *
 * (C) Copyright 2009
 * Michel Pollet <buserror@gmail.com>
 *
 * (C) Copyright 2012
 * Gabriel Huau <contact@huau-gabriel.fr>
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

#include <common.h>
#include <asm/arch/s3c2440.h>
#include <asm/arch/iomux.h>
#include <asm/arch/gpio.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <netdev.h>
#include "mini2440.h"

DECLARE_GLOBAL_DATA_PTR;

static inline void pll_delay(unsigned long loops)
{
	__asm__ volatile ("1:\n"
	  "subs %0, %1, #1\n"
	  "bne 1b" : "=r" (loops) : "0" (loops));
}

int board_early_init_f(void)
{
	struct s3c24x0_clock_power * const clk_power =
					s3c24x0_get_base_clock_power();

	/* to reduce PLL lock time, adjust the LOCKTIME register */
	clk_power->locktime = 0xFFFFFF; /* Max PLL Lock time count */
	clk_power->clkdivn = CLKDIVN_VAL;

	/* configure UPLL */
	clk_power->upllcon = ((U_M_MDIV << 12) + (U_M_PDIV << 4) + U_M_SDIV);
	/* some delay between MPLL and UPLL */
	pll_delay(100);

	/* configure MPLL */
	clk_power->mpllcon = ((M_MDIV << 12) + (M_PDIV << 4) + M_SDIV);

	/* some delay between MPLL and UPLL */
	pll_delay(10000);

	return 0;
}

/*
 * Miscellaneous platform dependent initialisations
 */
int board_init(void)
{
	debug("MINI2440: board_init()\n");
	struct s3c24x0_gpio * const gpio = s3c24x0_get_base_gpio();
	
	gpio->gpacon = 0x007FFFFF;	/* Port A is all "special" */

// 	// port B outputs reconfigured
// 	gpio->gpbcon = 	
// 		(0x1 <<  0) | // GPB0	OUT	TOUT0		PWM Buzzer
// 		(0x2 <<  2) | // GPB1	OUT	TOUT1		LCD Backlight
// 		(0x1 <<  4) | // GPB2	OUT	L3MODE
// 		(0x1 <<  6) | // GBP3	OUT	L3DATA
// 		(0x1 <<  8) | // GBP4	OUT	L3CLOCK
// 		(0x1 << 10) | // GBP5	OUT	LED1
// 		(0x1 << 12) | // GBP6	OUT	LED2
// 		(0x1 << 14) | // GBP7	OUT	LED3
// 		(0x1 << 16) | // GBP8	OUT	LED4
// 		(0x2 << 18) | // GBP9	---	nXDACK0		CON5 EBI
// 		(0x2 << 20) | // GBP10	---	nXDREQ0		CON5 EBI
// 		0;
// 	gpio->gpbup	= (1 << 10) - 1; // disable pullup on all 10 pins
// 	gpio->gpbdat	= 	
// 		(0 << 5) | /* turn LED 1 on */
// 		(1 << 6) | /* turn LED 1 off */
// 		(1 << 7) | /* turn LED 1 off */
// 		(1 << 8) | /* turn LED 1 off */
// 		0;

	// lcd signals on C and D
	gpio->gpccon	= (0xAAAAAAAA &		/* all default IN but ... */
				~(0x3 << 10)) |	/* not pin 5 ... */
				(0x1 << 10);	/* that is output (USBD) */
	gpio->gpcup	= 0xFFFFFFFF;
	gpio->gpcdat	= 0;

	gpio->gpdcon	= 0xAAAAAAAA;
	gpio->gpdup	= 0xFFFFFFFF;
	//port E is set for all it's special functions (i2c, spi etc)
    	gpio->gpecon 	= 0xAAAAAAAA;
	gpio->gpeup	= 0x0000FFFF;

	gpio->gpfcon 	= 
		(0x1 <<  0) | // GPG0	EINT0	OUT
		(0x1 <<  2) | // GPG1	EINT1	OUT
		(0x1 <<  4) | // GPG2	EINT2	OUT
		(0x1 <<  6) | // GPG3	EINT3	OUT
		(0x1 <<  8) | // GPG4	EINT4	OUT
		(0x1 << 10) | // GPG5	EINT5	OUT
		(0x1 << 12) | // GPG6	EINT6	OUT
		(0x0 << 14) | // GPG7	EINT7	IN	DM9000
		0;
	gpio->gpfdat	= 0;
	gpio->gpfup	= 
		((1 << 7) - 1) // all disabled
		& ~( 1 << 7 ) // but for the ethernet one, we need it.
		;

	gpio->gpgcon 	=
		(0x0 <<  0) | // GPG0	EINT8	IN	Key1
		(0x1 <<  2) | // GPG1	EINT9	OUT		Con5
		(0x1 <<  4) | // GPG2	EINT10	OUT
		(0x0 <<  6) | // GPG3	EINT11	IN	Key2
		(0x0 <<  8) | // GPG4	EINT12	IN	Smart Screen Interrupt
		(0x0 << 10) | // GPG5	EINT13	IN	Key3
		(0x0 << 12) | // GPG6	EINT14	IN	Key4
		(0x0 << 14) | // GPG7	EINT15	IN	Key5
		(0x1 << 16) | // GPG8	EINT16	OUT	nCD_SD
		(0x1 << 18) | // GPG9	EINT17	OUT
		(0x1 << 20) | // GPG10	EINT18	OUT
		(0x0 << 22) | // GPG11	EINT19	IN	Key6
		(0x0 << 24) | // GPG12	EINT18	IN	// GPG[12..15] need to be inputs
		(0x0 << 26) | // GPG13	EINT18	IN	// hard pullups
		(0x0 << 28) | // GPG14	EINT18	IN
		(0x0 << 30) | // GPG15	EINT18	IN
		0;
	gpio->gpgup = (1 << 15) -1;	// disable pullups for all pins*/


	/* IOMUX Port H : UART Configuration */
	gpio->gphcon = IOMUXH_nCTS0 | IOMUXH_nRTS0 | IOMUXH_TXD0 | IOMUXH_RXD0 |
		IOMUXH_TXD1 | IOMUXH_RXD1 | IOMUXH_TXD2 | IOMUXH_RXD2;

	//gpio->gphup = (1 << 10) - 1; // disable pullups for all pins

// 	gpio->extint0=0x22222222;
// 	gpio->extint1=0x22222222;
// 	gpio->extint2=0x22222222;

	gpio_direction_output(GPH8, 0);
	gpio_direction_output(GPH9, 0);
	gpio_direction_output(GPH10, 0);

	/* adress of boot parameters */
	gd->bd->bi_boot_params = CONFIG_BOOT_PARAM_ADDR;

	return 0;
}

int dram_init(void)
{
	struct s3c24x0_memctl *memctl = s3c24x0_get_base_memctl();

	/*
	 * Configuring bus width and timing
	 * Initialize clocks for each bank 0..5
	 * Bank 3 and 4 are used for DM9000
	 */
	writel(BANK_CONF, &memctl->bwscon);
	writel(B0_CONF, &memctl->bankcon[0]);
	writel(B1_CONF, &memctl->bankcon[1]);
	writel(B2_CONF, &memctl->bankcon[2]);
	writel(B3_CONF, &memctl->bankcon[3]);
	writel(B4_CONF, &memctl->bankcon[4]);
	writel(B5_CONF, &memctl->bankcon[5]);

	/* Bank 6 and 7 are used for DRAM */
	writel(SDRAM_64MB, &memctl->bankcon[6]);
	writel(SDRAM_64MB, &memctl->bankcon[7]);

// 	gd->bd->bi_dram[0].start = PHYS_SDRAM_SIZE;
// 	gd->bd->bi_dram[0].size = 64*1024*1024;

	writel(MEM_TIMING, &memctl->refresh);
	writel(BANKSIZE_CONF, &memctl->banksize);
	writel(B6_MRSR, &memctl->mrsrb6);
	writel(B7_MRSR, &memctl->mrsrb7);

	gd->ram_size = get_ram_size((void *) CONFIG_SYS_SDRAM_BASE,
			PHYS_SDRAM_SIZE);
	return 0;
}

int board_eth_init(bd_t *bis)
{
#ifdef CONFIG_DRIVER_DM9000
	return dm9000_initialize(bis);
#else
	return 0;
#endif
}
