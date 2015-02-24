/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
 *
 * (C) Copyright 2003
 * Texas Instruments, <www.ti.com>
 * Kshitij Gupta <Kshitij@ti.com>
 *
 * (C) Copyright 2004
 * ARM Ltd.
 * Philippe Robin, <philippe.robin@arm.com>
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
/*
 * The RealView Emulation BaseBoard provides timers and soft reset
 * - the cpu code does not need to provide these.
 */
#include <common.h>
#include <asm/system.h>
#include <asm/io.h>
#include <netdev.h>
#include <asm/arch/registers.h>

/*middle
#include "everify.h"
*/
#ifdef CONFIG_MK_FPGA
#define GOLDENGATE_TWD_BASE	(0x1E000600)/* CPU Timer & watch dog */
#else
#define GOLDENGATE_TWD_BASE	(0xF8000600)/* CPU Timer & watch dog */
#endif

#define	MAX_CLK_NUM		7

DECLARE_GLOBAL_DATA_PTR;

static u32 __attribute__((aligned(16384))) page_table[4096];

static void mmu_setup(void);

extern int g2_eth_initialize(bd_t *);
extern int g2_wdt_init(void);
extern int g2_wdt_set_heartbeat(int t);
extern void g2_wdt_start(void);

#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
	/* printf("Boot reached stage %d\n", progress); */
}
#endif

void get_platform_clk(unsigned int *cpu_clk, unsigned int *apb_clk,
			unsigned int *axi_clk)
{
	unsigned int	platform_clk[MAX_CLK_NUM][3] = {
				{ 400*1000000, 100*1000000, 133333333},
				{ 600*1000000, 100*1000000, 150*1000000},
				{ 700*1000000, 100*1000000, 140*1000000},
				{ 800*1000000, 100*1000000, 160*1000000},
				{ 900*1000000, 100*1000000, 150*1000000},
				{ 750*1000000, 150*1000000, 150*1000000},
				{ 850*1000000, 170*1000000, 141666667}
			};
	unsigned int 	reg_v;

#ifdef CONFIG_MK_FPGA
	*cpu_clk = 400 * 1000000;
	*apb_clk = 50 * 1000000;
	*axi_clk = 104 * 1000000;
#else
	reg_v = __raw_readl(GLOBAL_STRAP);
	reg_v = (reg_v >> 1) & 0x07;

	*cpu_clk = platform_clk[reg_v][0];
	*apb_clk = platform_clk[reg_v][1];
	*axi_clk = platform_clk[reg_v][2];

#endif
}

static volatile void gpio_mux_init(void)
{
#ifndef CONFIG_MK_FPGA
	/* GPIO_0 is shared with Pflash/nflash/sflash */
	__raw_writel(0xff80ff00, GLOBAL_GPIO_MUX_0);

	/* GPIO_1 is shared with TS/I2C/SPI/SSP/UART0 */
	__raw_writel(0xfffffc88, GLOBAL_GPIO_MUX_1);

	/* GPIO_2 is shared with TS/UART1/UART2/UART3/SD */
	__raw_writel(0xffffffff, GLOBAL_GPIO_MUX_2);

	/* GPIO_3 is shared with GMAC1/GMAC2 */
	__raw_writel(0x00000000, GLOBAL_GPIO_MUX_3);

	/* GPIO_4 is shared with LCD */
	__raw_writel(0xffffffff, GLOBAL_GPIO_MUX_4);
#endif
}

static void disable_remap(void)
{
#ifndef CONFIG_MK_FPGA
	unsigned int	reg_val;

	/* Disable ROM map */
	reg_val = __raw_readl(GLOBAL_GLOBAL_CONFIG);
	reg_val = reg_val | 0x0001;
	__raw_writel(reg_val, GLOBAL_GLOBAL_CONFIG);

	/* L2 filter start address */
	__raw_writel(0x00000001, 0xf5010c00);
#endif
}

static inline void delay (unsigned long loops)
{
	__asm__ volatile ("1:\n"
		"subs %0, %1, #1\n"
		"bne 1b":"=r" (loops):"0" (loops));
}

/*
 * Miscellaneous platform dependent initialisations
 */

int board_init (void)
{

	/* adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x00000100;

	gd->flags = 0;

	disable_remap();

	gpio_mux_init();

	return 0;
}

/* We know all the init functions have been run now */
int board_eth_init(bd_t *bis)
{
	g2_eth_initialize(bis);

	return 0;
}

int misc_init_r (void)
{
	GLOBAL_PIN_MUX_t gpm;
	unsigned int reg_val;

	setenv("verify", "n");

	g2_wdt_init();

	g2_wdt_set_heartbeat(300);

	g2_wdt_start();

#ifdef CONFIG_UBOOT_PWC	
	cs75xx_pwr_init();
#endif	

	/* configure Power LED as output and turn off Failure LED (red) */
	reg_val = __raw_readl(PER_GPIO2_CFG);
	reg_val &= ~(0x1 << 7);
	reg_val &= ~(0x1 << 5);
	__raw_writel(reg_val, PER_GPIO2_CFG);
	reg_val = __raw_readl(PER_GPIO2_IN);
	reg_val &= ~(0x1 << 7);
	reg_val |= (0x1 << 5);
	__raw_writel(reg_val, PER_GPIO2_OUT);

	/* slow flashing at 2Hz (0.5s) for Power LED by programming PWM */
	__raw_writel(75000000, PER_PWM_TIMER_PERIOD);
	__raw_writel(37500000, PER_PWM_TIMER_COMP);
	gpm.wrd = __raw_readl(GLOBAL_PIN_MUX);
	gpm.bf.pwm_nf = 1;
	gpm.bf.pwm_pf = 1;
	gpm.bf.pwm_sf = 1;
	__raw_writel(gpm.wrd, GLOBAL_PIN_MUX);

	return (0);
}

/******************************
 Routine:
 Description:
******************************/
int dram_init (void)
{
	unsigned int	size;

	size = __raw_readl(GLOBAL_SOFTWARE2);
	size = size & 0xfff00000;

	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = size; /*PHYS_SDRAM_1_SIZE; */

	mmu_setup();

	icache_enable ();

	/* middle just for test */
	/* do_verify(((unsigned char *)0xf6a00000), 0x30000); */

	return 0;
}

int interrupt_init (void)
{
	return 0;
}

/*
 * Write the system control status register to cause reset
 */
void reset_cpu(ulong addr)
{
        /*
        * To reset, use watchdog to reset whole system
        */
        unsigned int reg_v;

        /* do external reset */
	/* delay in POST, not in here - hoangtran */
	/*
        reg_v = __raw_readl(GLOBAL_SCRATCH);
        reg_v = reg_v | 0x400;
        __raw_writel(reg_v, GLOBAL_SCRATCH);
        udelay(500);
        reg_v = reg_v & (~0x400);
        __raw_writel(reg_v, GLOBAL_SCRATCH);
	*/
        
        /* Reset all block & subsystem */
        reg_v = __raw_readl(GLOBAL_GLOBAL_CONFIG);
        /* enable axi & L2 reset */
        reg_v &= ~0x00000200;

        /* wd_enable are exclusive with wd0_reset_subsys_enable */
        reg_v &= ~0x0000000E;

        /* reset remap, all block & subsystem */
        reg_v |= 0x000000F0;
        __raw_writel(reg_v, GLOBAL_GLOBAL_CONFIG);

        /* Fire */
        __raw_writel(0, GOLDENGATE_TWD_BASE + 0x28); /* Disable WD */
        __raw_writel(10, GOLDENGATE_TWD_BASE + 0x20); /* LOAD */

        /* Enable watchdog - prescale=256, watchdog mode=1, enable=1 */
        __raw_writel(0x0000FF09, GOLDENGATE_TWD_BASE + 0x28); /* Enable WD */

}

static void mmu_setup(void)
{
	bd_t *bd = gd->bd;
	int i, j;
	u32 reg;

	/* Set up an identity-mapping for all 4GB, rw for everyone */
	for (i = 0; i < 4096; i++)
		page_table[i] = i << 20 | (3 << 10) | 0x12;

	/* Then, enable cacheable and bufferable for RAM only */
	for (j = 0; j < CONFIG_NR_DRAM_BANKS; j++) {
		for (i = bd->bi_dram[j].start >> 20;
			i < (bd->bi_dram[j].start + bd->bi_dram[j].size) >> 20;
			i++) {
			page_table[i] = i << 20 | (3 << 10) | 0x1e; /* C=1 B=1*/
		}
	}

	/* Copy the page table address to cp15 */
	asm volatile("mcr p15, 0, %0, c2, c0, 0"
		     : : "r" (page_table) : "memory");

	/* Set the access control to all-supervisor */
	asm volatile("mcr p15, 0, %0, c3, c0, 0"
		     : : "r" (~0));

	/* and enable the mmu */
	reg = get_cr();	/* get control reg. */
	delay(100);
	set_cr(reg | CR_M);
}

static int do_fail (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned int reg_val;

	/* turn off power white LED and turn on fault red LED */
	reg_val = __raw_readl(PER_GPIO2_IN);
	reg_val |= (0x1 << 7);
	reg_val &= ~(0x1 << 5);
	__raw_writel(reg_val, PER_GPIO2_OUT);

	/* no flashing */
	__raw_writel(37500000, PER_PWM_TIMER_COMP);

}

U_BOOT_CMD(
	fail, 1, 0,	do_fail,
	"Simulate failure ...",
	""
);
