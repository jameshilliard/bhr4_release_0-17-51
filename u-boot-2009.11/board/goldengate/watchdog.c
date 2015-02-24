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
#include <common.h>
#include <asm/io.h>
#include <asm/arch/registers.h>

#ifdef CONFIG_MK_FPGA
#define GOLDENGATE_TWD_BASE	(0x1E000600)	/* CPU Timer & watch dog */
#else
#define GOLDENGATE_TWD_BASE	(0xF8000600)	/* CPU Timer & watch dog */
#endif

#define GOLDENGATE_WDT0_BASE	(GOLDENGATE_TWD_BASE)

#define G2_TIMER_LOAD 			0x00
#define G2_TIMER_COUNTER		0x04
#define G2_TIMER_CONTROL		0x08
#define G2_TIMER_INTSTAT		0x0C

#define G2_WDOG_LOAD			0x20
#define G2_WDOG_COUNTER			0x24
#define G2_WDOG_CONTROL			0x28
#define G2_WDOG_INTSTAT			0x2C
#define G2_WDOG_RESETSTAT		0x30
#define G2_WDOG_DISABLE			0x34

#define G2_TIMER_CONTROL_ENABLE		(1 << 0)
#define G2_TIMER_CONTROL_ONESHOT	(0 << 1)
#define G2_TIMER_CONTROL_PERIODIC	(1 << 1)
#define G2_TIMER_CONTROL_IT_ENABLE	(1 << 2)

#define TIMER_MARGIN	30

static unsigned int g2_timer_rate = (100*1024*1024);
static int g2_margin = TIMER_MARGIN;
static unsigned int	perturb;

#define ONLY_TESTING	0
static int g2_noboot = ONLY_TESTING;

extern void get_platform_clk(unsigned int *cpu_clk, unsigned int *apb_clk, 
			unsigned int *axi_clk);

int g2_wdt_set_heartbeat(int t)
{

	if (t < 0x0001 || t > 0xFFFF)
		return -1;

	g2_margin = t;
	return 0;
}

int g2_wdt_init(void)
{
	static unsigned int cpu_clk;
	static unsigned int apb_clk;
	static unsigned int axi_clk;
        unsigned int reg_v;

        /* Reset all block & subsystem */
        reg_v = __raw_readl(GLOBAL_GLOBAL_CONFIG);
        /* enable axi & L2 reset */
        reg_v &= ~0x00000300;

        /* wd_enable are exclusive with wd0_reset_subsys_enable */
        reg_v &= ~0x0000000E;

        /* reset remap, all block & subsystem */
        reg_v |= 0x000000F0;
        __raw_writel(reg_v, GLOBAL_GLOBAL_CONFIG);

	/* get APB clock rate */
	get_platform_clk(&cpu_clk, &apb_clk, &axi_clk);

	g2_timer_rate = apb_clk;
	
	/*
	 * Check that the margin value is within it's range;
	 * if not reset to the default
	 */
	if (g2_wdt_set_heartbeat(g2_margin)) {
		g2_wdt_set_heartbeat(TIMER_MARGIN);
	}

	return 0;
}

void g2_wdt_keepalive(void)
{
	unsigned int count=0;

	/* Assume prescale is set to 256 */
	count = (g2_timer_rate / 256) * g2_margin;

	/* Reload the counter */
	__raw_writel(count + perturb, GOLDENGATE_WDT0_BASE + G2_WDOG_LOAD);
	perturb = perturb ? 0 : 1;
}

void g2_wdt_start(void)
{

	/* This loads the count register but does NOT start the count yet */
	g2_wdt_keepalive();

	if (g2_noboot) {
		/* Enable watchdog - prescale=256, watchdog mode=0, enable=1 */
		__raw_writel(0x0000FF01, GOLDENGATE_WDT0_BASE + G2_WDOG_CONTROL);
	} else {
		/* Enable watchdog - prescale=256, watchdog mode=1, enable=1 */
		__raw_writel(0x0000FF09, GOLDENGATE_WDT0_BASE + G2_WDOG_CONTROL);
	}

	return;
}

void g2_wdt_stop(void)
{

	/* switch from watchdog mode to timer mode */
	__raw_writel(0x12345678, GOLDENGATE_WDT0_BASE + G2_WDOG_DISABLE);
	__raw_writel(0x87654321, GOLDENGATE_WDT0_BASE + G2_WDOG_DISABLE);
	/* watchdog is disabled */
	__raw_writel(0x0, GOLDENGATE_WDT0_BASE + G2_WDOG_CONTROL);

	return;
}

#if CONFIG_DEBUG
int do_watchdog(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	switch (argc) {
	case 2:	/* watchdog disable or enable */
		if(strcmp ("enable", argv[1]) == 0 ){
			g2_wdt_start();
		} else if (strcmp ("disable", argv[1]) == 0 ){
			g2_wdt_stop();
		} else {
			printf ("Usage:\n%s\n", cmdtp->usage);
			return 1;
		}
		break;
	
	case 3:
		if(strcmp ("set", argv[1]) == 0 ){
			g2_wdt_stop();
			g2_margin = simple_strtoul(argv[2], NULL, 16);
			g2_wdt_start();
		} else {
	  		printf ("Usage:\n%s\n", cmdtp->usage);
	  		return 1;
		}
		break;
	
	default:
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	
	return 0;

}

U_BOOT_CMD(
	wd,	3,		1,	do_watchdog,
	"wd      - watchdog enable/disable/set time\n",
        "wd [disable|enable|set time_val]\n"
);
#endif

