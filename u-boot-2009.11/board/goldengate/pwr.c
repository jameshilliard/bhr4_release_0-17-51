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
#include <asm/io.h>
#include <asm/arch/registers.h>


#define CS75XX_RTC_BASE 	RTC_RTCON
#define CS75XX_CIR_BASE 	CIR_PWRCTRL_CIR_ID


/* Register Map */
#define CS75XX_RTC_RTCON	0x00
#define CS75XX_RTC_WKUPPEND	0x4C


#define	CS75XX_CIR_ID		0x00
#define	CS75XX_CIR_CTRL0	0x04
#define	CS75XX_CIR_CTRL1	0x08
#define	CS75XX_CIR_INT_ST	0x0C
#define	CS75XX_CIR_INT_EN	0x10
#define	CS75XX_CIR_DATA		0x14
#define	CS75XX_CIR_DATAEXT	0x18
#define	CS75XX_CIR_POWER	0x1C
#define	CS75XX_CIR_POWEREXT	0x20
#define	CS75XX_PWR_CTRL1	0x28
#define CS75XX_PWR_INT_EN	0x30


#define VCR_KEY_POWER		0x613E609F
#define TV1_KEY_POWER		0x40040100
#define TV1_KEY_POWER_EXT	0xBCBD
#define DVB_KEY_POWER		0x38000000

#define VCR_H_ACT_PER		(16-1)
#define VCR_L_ACT_PER		(8-1)
#define VCR_DATA_LEN		(32-1)
#define TV1_H_ACT_PER		(8-1)
#define TV1_L_ACT_PER		(4-1)
#define TV1_DATA_LEN		(48-1)

#define VCR_BAUD		540 /* us */
#define TV1_BAUD		430 /* us */
#define DVB_BAUD		830 /* us */

#ifdef CONFIG_CORTINA_FPGA
#define	EXT_CLK_SRC		104 /* MHz */
#define	EXT_CLK_DIV		12
#else
#define	EXT_CLK			24  /* MHz */
#endif

#define KUC100_VCR_33		0x0 /* KOKA KUC-100 VCR-33 */
#define KUC100_TV1_26		0x1 /* KOKA KUC-100 TV1-26 */
#define WB_DVB			0x2 /* white brand DVB */


#define DEFAULT_RC_CFG		WB_DVB


/*
 * At first power connect, init CIR to support Power-On by CIR and shut-down
 * the device to wait for Power-On.
 */
int cs75xx_pwr_init(void)
{
	CIR_PWRCTRL_CIR_RXCTRL0_t reg_ctrl0;
	CIR_PWRCTRL_CIR_RXCTRL1_t reg_ctrl1;
	CIR_PWRCTRL_CIR_PWRKEY_t reg_pwrkey;
	CIR_PWRCTRL_CIR_PWRKEY_EXT_t reg_pwrkey_ext;

	reg_ctrl0.wrd = __raw_readl(CS75XX_CIR_BASE + CS75XX_CIR_CTRL0);

	switch (DEFAULT_RC_CFG) {
	case KUC100_VCR_33:
		reg_ctrl0.bf.demod_en       = 0;
		reg_ctrl0.bf.pos            = 0;
		reg_ctrl0.bf.cir_protocol   = 1;
		reg_ctrl0.bf.rc5_stopBit_en = 0;
		reg_ctrl0.bf.rc5_extend     = 0;
		reg_ctrl0.bf.head_lo_t      = VCR_L_ACT_PER;
		reg_ctrl0.bf.head_hi_t      = VCR_H_ACT_PER;
#ifdef CONFIG_CORTINA_FPGA
		reg_ctrl0.bf.baud_div       = (VCR_BAUD*EXT_CLK_SRC/EXT_CLK_DIV);
#else
		reg_ctrl0.bf.baud_div       = (VCR_BAUD*EXT_CLK);
#endif
		__raw_writel(reg_ctrl0.wrd, CS75XX_CIR_BASE + CS75XX_CIR_CTRL0);

		reg_ctrl1.bf.data_len_b     = VCR_DATA_LEN;
		reg_ctrl1.bf.data_compare   = 0;
		reg_ctrl1.bf.pwrKeyIRQCpu   = 0;
		__raw_writel(reg_ctrl1.wrd, CS75XX_CIR_BASE + CS75XX_CIR_CTRL1);

		reg_pwrkey.bf.pwr_code1     = VCR_KEY_POWER;
		__raw_writel(reg_pwrkey.wrd, CS75XX_CIR_BASE + CS75XX_CIR_POWER);
		reg_pwrkey_ext.bf.pwr_code2 = 0;
		__raw_writel(reg_pwrkey_ext.wrd, CS75XX_CIR_BASE + CS75XX_CIR_POWEREXT);

		break;

	case KUC100_TV1_26:
		reg_ctrl0.bf.demod_en       = 0;
		reg_ctrl0.bf.pos            = 0;
		reg_ctrl0.bf.cir_protocol   = 1;
		reg_ctrl0.bf.rc5_stopBit_en = 0;
		reg_ctrl0.bf.rc5_extend     = 0;
		reg_ctrl0.bf.head_lo_t      = TV1_L_ACT_PER;
		reg_ctrl0.bf.head_hi_t      = TV1_H_ACT_PER;
#ifdef CONFIG_CORTINA_FPGA
		reg_ctrl0.bf.baud_div       = (TV1_BAUD*EXT_CLK_SRC/EXT_CLK_DIV);
#else
		reg_ctrl0.bf.baud_div	    = (TV1_BAUD*EXT_CLK);
#endif
		__raw_writel(reg_ctrl0.wrd, CS75XX_CIR_BASE + CS75XX_CIR_CTRL0);

		reg_ctrl1.bf.data_len_b     = TV1_DATA_LEN;
		reg_ctrl1.bf.data_compare   = 0;
		reg_ctrl1.bf.pwrKeyIRQCpu   = 0;
		__raw_writel(reg_ctrl1.wrd, CS75XX_CIR_BASE + CS75XX_CIR_CTRL1);

		reg_pwrkey.bf.pwr_code1     = TV1_KEY_POWER;
		__raw_writel(reg_pwrkey.wrd, CS75XX_CIR_BASE + CS75XX_CIR_POWER);
		reg_pwrkey_ext.bf.pwr_code2 = TV1_KEY_POWER_EXT;
		__raw_writel(reg_pwrkey_ext.wrd, CS75XX_CIR_BASE + CS75XX_CIR_POWEREXT);

		break;

	case WB_DVB:
		reg_ctrl0.bf.demod_en       = 0;
		reg_ctrl0.bf.pos            = 0;
		reg_ctrl0.bf.cir_protocol   = 0;
		reg_ctrl0.bf.rc5_stopBit_en = 0;
		reg_ctrl0.bf.rc5_extend     = 1;
		reg_ctrl0.bf.head_lo_t      = 0;
		reg_ctrl0.bf.head_hi_t      = 0;
#ifdef CONFIG_CORTINA_FPGA
		reg_ctrl0.bf.baud_div       = (DVB_BAUD*EXT_CLK_SRC/EXT_CLK_DIV);
#else
		reg_ctrl0.bf.baud_div       = (DVB_BAUD*EXT_CLK);
#endif
		__raw_writel(reg_ctrl0.wrd, CS75XX_CIR_BASE + CS75XX_CIR_CTRL0);

		reg_ctrl1.bf.data_len_b     = VCR_DATA_LEN;
		reg_ctrl1.bf.data_compare   = 0;
		reg_ctrl1.bf.pwrKeyIRQCpu   = 0;
		__raw_writel(reg_ctrl1.wrd, CS75XX_CIR_BASE + CS75XX_CIR_CTRL1);

		reg_pwrkey.bf.pwr_code1     = DVB_KEY_POWER;
		__raw_writel(reg_pwrkey.wrd, CS75XX_CIR_BASE + CS75XX_CIR_POWER);
		reg_pwrkey_ext.bf.pwr_code2 = 0;
		__raw_writel(reg_pwrkey_ext.wrd, CS75XX_CIR_BASE + CS75XX_CIR_POWEREXT);

		break;

	default:
		printf("Unknown Remote Control Configuration %d\n", DEFAULT_RC_CFG);
		return;
	}

	/* first power connect */
	if (reg_ctrl0.bf.fst_por_ok == 0) {
		CIR_PWRCTRL_CIR_INT_STATUS_t reg_int_st;
		CIR_PWRCTRL_PWR_CTRL1_t reg_pwr_ctrl1;
		CIR_PWRCTRL_PWR_INT_ENABLE_t reg_pwr_inten;

		printf("\n\n\nPlease Press Power Button!!!\n");

		reg_ctrl0.bf.fst_por_ok = 1;
		__raw_writel(reg_ctrl0.wrd, CS75XX_CIR_BASE + CS75XX_CIR_CTRL0);

		/* RTC Write Enable and Clear Wakeup Alarm */
		__raw_writel(2, CS75XX_RTC_BASE + CS75XX_RTC_RTCON);
		__raw_writel(0, CS75XX_RTC_BASE + CS75XX_RTC_WKUPPEND);

		/* Clear CIR Interrupt */
		reg_int_st.wrd = __raw_readl(CS75XX_CIR_BASE + CS75XX_CIR_INT_ST);
		reg_int_st.bf.pwrkey_int_sts = 1;
		reg_int_st.bf.cir_dat_int = 1;
		reg_int_st.bf.repeat_sts = 1;
		__raw_writel(reg_int_st.wrd, CS75XX_CIR_BASE + CS75XX_CIR_INT_ST);

		/* Turn-off Interrupt */
		__raw_writel(0, CS75XX_CIR_BASE + CS75XX_PWR_INT_EN);

		/* Clear PWR Interrupt */
		reg_pwr_ctrl1.wrd = 0;
		reg_pwr_ctrl1.bf.pwr_int_clear = 1;
		__raw_writel(reg_pwr_ctrl1.wrd, CS75XX_CIR_BASE + CS75XX_PWR_CTRL1);

		/* Turn-on Interrupt */
		reg_pwr_inten.wrd = __raw_readl(CS75XX_CIR_BASE + CS75XX_PWR_INT_EN);
		reg_pwr_inten.bf.cir_pwr_on_en = 1;
		reg_pwr_inten.bf.rtc_wake_en = 1;
		reg_pwr_inten.bf.push_btn_wake_en = 1;
		__raw_writel(reg_pwr_inten.wrd, CS75XX_CIR_BASE + CS75XX_PWR_INT_EN);

		/*
		 * To Shut-Down, PWR_CTRL1_INIT_FINISH and PWR_CTRL1_SHUT_DOWN
		 *  can't set at the same time
		 */
		reg_pwr_ctrl1.wrd = 0;
		reg_pwr_ctrl1.bf.sysInitFinish = 1;
		__raw_writel(reg_pwr_ctrl1.wrd, CS75XX_CIR_BASE + CS75XX_PWR_CTRL1);

		udelay(2000);	/* 2ms, Not Remove */

		reg_pwr_ctrl1.wrd = 0;
		reg_pwr_ctrl1.bf.swShutdnEn = 1;
		__raw_writel(reg_pwr_ctrl1.wrd, CS75XX_CIR_BASE + CS75XX_PWR_CTRL1);

		while(1);	/* stop until the system power down */
	}

	return 0;
}

