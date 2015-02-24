/*
 * arch/arm/mach-goldengate/pm.c
 * CS75xx Power Management
 *
 * Copyright (C) 2011 Cortina-Systems Co. LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/atomic.h>
//#include <asm/mach/time.h>
//#include <asm/mach/irq.h>

//#include <mach/cs75xx_pmc.h>
//#include <mach/gpio.h>
#include <mach/platform.h>
#include <mach/hardware.h>
#include <asm/hardware/gic.h>
//#include "generic.h"
//#include "pm.h"


u64 gic_wakeups0;
u64 gic_backups0;
u16 gic_wakeups1;
u16 gic_backups1;

u16 regbus_wakeups;
u16 regbus_backups;

void cs75xx_irq_suspend(void)
{
	unsigned int gic_dist_base;

	if(regbus_wakeups != 0)
		gic_wakeups0 |= (u64)1 << IRQ_PERI_REGBUS;

	/* Anyway, let UART0 can wake up system */
	gic_wakeups0 |= (u64)1 << GOLDENGATE_IRQ_UART0;

	/* 1. Disable unnessary peripheral interrupt */
	regbus_backups = readl(IO_ADDRESS(PER_PERIPHERAL_INTENABLE_0));
	writel(regbus_wakeups, IO_ADDRESS(PER_PERIPHERAL_INTENABLE_0));


	/* 2. Disable unnessary interrupt in GIC */
	gic_dist_base = IO_ADDRESS(GOLDENGATE_GIC_DIST_BASE);
	gic_backups0 = readl(gic_dist_base + GIC_DIST_ENABLE_SET);
	gic_backups0 |= (u64)readl(gic_dist_base + GIC_DIST_ENABLE_SET + 4) << 32;
	gic_backups1 = readl(gic_dist_base + GIC_DIST_ENABLE_SET + 8);

	writel(0xffff0000 & gic_backups0, gic_dist_base + GIC_DIST_ENABLE_CLEAR);
	writel(0xFFFFFFFF, gic_dist_base + GIC_DIST_ENABLE_CLEAR + 4);
	writel(0xFFFF, gic_dist_base + GIC_DIST_ENABLE_CLEAR + 8);

	gic_wakeups0 |= gic_backups0 & 0xFFFF;
	writel(gic_wakeups0, gic_dist_base + GIC_DIST_ENABLE_SET);
	writel(gic_wakeups0 >> 32, gic_dist_base + GIC_DIST_ENABLE_SET + 4);
	writel(gic_wakeups1, gic_dist_base + GIC_DIST_ENABLE_SET + 8);

}

void cs75xx_irq_resume(void)
{
	unsigned int gic_dist_base;

	/* Enable GIC */
	gic_dist_base = IO_ADDRESS(GOLDENGATE_GIC_DIST_BASE);
	writel(gic_backups0, gic_dist_base + GIC_DIST_ENABLE_SET);
	writel(gic_backups0 >> 32, gic_dist_base + GIC_DIST_ENABLE_SET + 4);
	writel(gic_backups1, gic_dist_base + GIC_DIST_ENABLE_SET + 8);

	/* Enable peripheral IRQ */
	writel(regbus_backups, IO_ADDRESS(PER_PERIPHERAL_INTENABLE_0));

	/* Clear setting for next suspend */
	gic_wakeups0 = 0;
	gic_wakeups1 = 0;
	regbus_wakeups = 0;

}

static int cs75xx_pm_valid_state(suspend_state_t state)
{
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;

		default:
			return 0;
	}
}


static suspend_state_t target_state;

/*
 * Called after processes are frozen, but before we shutdown devices.
 */
static int cs75xx_pm_begin(suspend_state_t state)
{
	target_state = state;
	return 0;
}

 int cs75xx_pm_enter(suspend_state_t state)
{
//	u32 saved_lpr;
//	cs75xx_gpio_suspend();
	cs75xx_irq_suspend();

//	pr_debug("AT91: PM - wake mask %08x, pm state %d\n",
//			/* remember all the always-wake irqs */
//			(cs75xx_sys_read(AT91_PMC_PCSR)
//					| (1 << AT91_ID_FIQ)
//					| (1 << AT91_ID_SYS)
//					| (cs75xx_extern_irq))
//				& cs75xx_sys_read(AT91_AIC_IMR),
//			state);

	switch (state) {
		/*
		 * Suspend-to-RAM is like STANDBY plus slow clock mode, so
		 * drivers must suspend more deeply:  only the master clock
		 * controller may be using the main oscillator.
		 */
		case PM_SUSPEND_MEM:
			/*
			 * Ensure that clocks are in a valid state.
			 */

			/*
			 * Enter slow clock mode by switching over to clk32k and
			 * turning off the main oscillator; reverse on wakeup.
			 */

		/*
		 * STANDBY mode has *all* drivers suspended; ignores irqs not
		 * marked as 'wakeup' event sources; and reduces DRAM power.
		 * But otherwise it's identical to PM_SUSPEND_ON:  cpu idle, and
		 * nothing fancy done with main or cpu clocks.
		 */
		case PM_SUSPEND_STANDBY:
			/*
			 * NOTE: the Wait-for-Interrupt instruction needs to be
			 * in icache so no SDRAM accesses are needed until the
			 * wakeup IRQ occurs and self-refresh is terminated.
			 */
//			asm("b 1f; .align 5; 1:");
//			asm("mcr p15, 0, r0, c7, c10, 4");	/* drain write buffer */
//			saved_lpr = sdram_selfrefresh_enable();
//			asm("mcr p15, 0, r0, c7, c0, 4");	/* wait for interrupt */
//			sdram_selfrefresh_disable(saved_lpr);
//			break;

		case PM_SUSPEND_ON:
			asm("wfi");	/* wait for interrupt */
			break;

		default:
			pr_debug("CS75xx: PM - bogus suspend state %d\n", state);
			goto error;
	}

//	pr_debug("AT91: PM - wakeup %08x\n",
//			cs75xx_sys_read(AT91_AIC_IPR) & cs75xx_sys_read(AT91_AIC_IMR));

error:
	target_state = PM_SUSPEND_ON;
	cs75xx_irq_resume();
	return 0;
}

/*
 * Called right prior to thawing processes.
 */
static void cs75xx_pm_end(void)
{
	target_state = PM_SUSPEND_ON;
}


static struct platform_suspend_ops cs75xx_pm_ops ={
	.valid	= cs75xx_pm_valid_state,
	.begin	= cs75xx_pm_begin,
	.enter	= cs75xx_pm_enter,
	.end	= cs75xx_pm_end,
};

static int __init cs75xx_pm_init(void)
{
	gic_wakeups0 = 0;
	gic_backups0 = 0;
	gic_wakeups1 = 0;
	gic_backups1 = 0;

	regbus_wakeups = 0;
	regbus_backups = 0;

	suspend_set_ops(&cs75xx_pm_ops);

	return 0;
}
arch_initcall(cs75xx_pm_init);
