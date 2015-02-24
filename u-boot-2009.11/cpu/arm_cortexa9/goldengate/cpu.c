/*
 * (C) Copyright 2010 ST-Ericsson SA
 * Author: Rabin Vincent <rabin.vincent <at> stericsson.com>
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
#include <asm/io.h>
////#include <asm/arch/clock.h>
#include <nomadik.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_NOMADIK_GPIO
unsigned long nmk_gpio_base[CONFIG_NOMADIK_GPIO_NUM_BANKS] = {
	U8500_GPIO0_BASE,
	U8500_GPIO1_BASE,
	U8500_GPIO2_BASE,
	U8500_GPIO3_BASE,
	U8500_GPIO4_BASE,
	U8500_GPIO5_BASE,
	U8500_GPIO6_BASE,
	U8500_GPIO7_BASE,
	U8500_GPIO8_BASE
};
#endif

void reset_cpu(unsigned long ignored)
{
////	struct prcmu *prcmu = (struct prcmu *) U8500_PRCMU_BASE;

	//writel(0x1, &prcmu->ape_softrst);

	while (1)
		; /* infinite loop until reset */
}

