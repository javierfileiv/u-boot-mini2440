/*
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.
 */


#include <common.h>
//#include "cpu.h"
#include <spl.h>

#include <asm/io.h>
//#include <asm/arch/clock.h>
//#include <asm/arch/pinmux.h>
//#include <asm/arch/tegra.h>
//#include <asm/arch-tegra/board.h>
#include <asm/arch/spl.h>

void spl_board_init(void)
{
// 	struct pmux_tri_ctlr *pmt = (struct pmux_tri_ctlr *)NV_PA_APB_MISC_BASE;
// 
// 	/* enable JTAG */
// 	writel(0xC0, &pmt->pmt_cfg_ctl);
// 
// 	board_init_uart_f();
// 
// 	/* Initialize periph GPIOs */
// 	gpio_early_init_uart();
// 
// 	clock_early_init();
// 	preloader_console_init();
}

u32 spl_boot_device(void)
{
	return 0; //BOOT_DEVICE_RAM;
}

void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	debug("image entry point: 0x%X\n", spl_image->entry_point);

// 	start_cpu((u32)spl_image->entry_point);
// 	halt_avp();
}