/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc.
 * Copyright 2008 Sascha Hauer, kernel@pengutronix.de
 * Copyright 2009 Ilya Yanok, <yanok@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <common.h>
#include <nand.h>

#include <malloc.h>
#include <watchdog.h>
#include <linux/err.h>
#include <linux/mtd/compat.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>

#ifdef CONFIG_MTD_PARTITIONS
#include <linux/mtd/partitions.h>
#endif

#include <asm/io.h>
#include <asm/errno.h>

#ifdef CONFIG_JFFS2_NAND
#include <jffs2/jffs2.h>
#endif

#include <asm/arch/registers.h>

#include "cs75xx_nand.h"

#define DRIVER_NAME "cs75xx_nand"

#define ENOTSUPP	524	/* Operation is not supported */

#ifndef CONFIG_SYS_NAND_BASE_LIST
#define CONFIG_SYS_NAND_BASE_LIST { CONFIG_SYS_NAND_BASE }
#endif

DECLARE_GLOBAL_DATA_PTR;

#define OWN_DMA	0
#define OWN_SW	1



//static int nand_curr_device = -1;
//nand_info_t nand_info[CONFIG_SYS_MAX_NAND_DEVICE];

static struct nand_chip nand_chip[CONFIG_SYS_MAX_NAND_DEVICE];
static ulong base_address[CONFIG_SYS_MAX_NAND_DEVICE] = CONFIG_SYS_NAND_BASE_LIST;

static const char default_nand_name[] = "cs75xx_nand";
static __attribute__((unused)) char dev_name[CONFIG_SYS_MAX_NAND_DEVICE][8];

/* OOB placement block for use with hardware ecc generation */
/*#ifdef CONFIG_CS75XX_NAND_HWECC*/
static struct nand_ecclayout cs75xx_nand_ecclayout;

#ifndef CONFIG_SYS_NAND_RESET_CNT
#define CONFIG_SYS_NAND_RESET_CNT 200000
#endif

#if defined(CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined(CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
  #define CONFIG_CS75XX_NAND_ECC_HW_BCH
#endif

#if defined(CONFIG_CS752X_NAND_ECC_HW_HAMMING_256) || defined(CONFIG_CS752X_NAND_ECC_HW_HAMMING_512)
  #define CONFIG_CS75XX_NAND_ECC_HW_HAMMING
#endif

/* Define default oob placement schemes for large and small page devices */
#ifdef	CONFIG_CS75XX_NAND_ECC_HW_BCH

static struct nand_ecclayout cs75xx_nand_bch_oob_16 = {
	.eccbytes = 13,
	.eccpos = {0, 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14},
	.oobfree = {
		{.offset = 15,
		 /* . length = 1}}  resever 1 for erase tags: 1 - 1 = 0*/
		 . length = 0}} /* resever 1 for erase tags: 1 - 1 = 0 */
};

#else

static struct nand_ecclayout cs75xx_nand_oob_8 = {
	.eccbytes = 3,
	.eccpos = {0, 1, 2},
	.oobfree = {
		{.offset = 3,
		 .length = 2},
		{.offset = 6,
		 .length = 2}}
};

static struct nand_ecclayout cs75xx_nand_oob_16 = {
	.eccbytes = 6,
	.eccpos = {0, 1, 2, 3, 6, 7},
	.oobfree = {
		{.offset = 8,
		 . length = 8}}
};

#endif


struct cs75xx_nand_host {
	struct mtd_info		mtd;
	struct nand_chip	*nand;
	void __iomem		*io_base;
	struct device		*dev;
	unsigned int		col_addr;
	unsigned int		page_addr;
};


static struct cs75xx_nand_host cs75xx_host;
static struct cs75xx_nand_host *host = &cs75xx_host;


static unsigned int CHIP_EN;
static unsigned int *pread, *pwrite;
GLOBAL_STRAP_t	global_strap;
FLASH_TYPE_t flash_type;
FLASH_STATUS_t			flash_status;
FLASH_NF_ACCESS_t nf_access;
FLASH_NF_COUNT_t nf_cnt;
FLASH_NF_COMMAND_t nf_cmd;
FLASH_NF_ADDRESS_1_t nf_addr1;
FLASH_NF_ADDRESS_2_t	nf_addr2;
FLASH_NF_DATA_t	nf_data;
FLASH_NF_ECC_STATUS_t	ecc_sts;
FLASH_NF_ECC_CONTROL_t ecc_ctl;
FLASH_NF_ECC_OOB_t	ecc_oob;
FLASH_NF_ECC_GEN0_t	ecc_gen0;
FLASH_NF_ECC_GEN1_t	ecc_gen1;
FLASH_NF_ECC_GEN2_t	ecc_gen2;
FLASH_NF_ECC_GEN3_t	ecc_gen3;
FLASH_NF_ECC_GEN4_t	ecc_gen4;
FLASH_NF_ECC_GEN5_t	ecc_gen5;
FLASH_NF_ECC_GEN6_t	ecc_gen6;
FLASH_NF_ECC_GEN7_t	ecc_gen7;
FLASH_NF_ECC_GEN8_t	ecc_gen8;
FLASH_NF_ECC_GEN9_t	ecc_gen9;
FLASH_NF_ECC_GEN10_t	ecc_gen10;
FLASH_NF_ECC_GEN11_t	ecc_gen11;
FLASH_NF_ECC_GEN12_t	ecc_gen12;
FLASH_NF_ECC_GEN13_t	ecc_gen13;
FLASH_NF_ECC_GEN14_t	ecc_gen14;
FLASH_NF_ECC_GEN15_t	ecc_gen15;
FLASH_NF_FIFO_CONTROL_t	fifo_ctl;
FLASH_NF_FIFO_STATUS_t	fifo_sts;
FLASH_NF_FIFO_ADDRESS_t	fifo_addr;
FLASH_NF_FIFO_DATA_t	dido_data;
FLASH_FLASH_ACCESS_START_t flash_start;
FLASH_NF_ECC_RESET_t	ecc_reset;
FLASH_FLASH_INTERRUPT_t	flash_int_sts;
FLASH_FLASH_MASK_t	flash_int_mask;
FLASH_NF_BCH_STATUS_t	bch_sts;
FLASH_NF_BCH_ERROR_LOC01_t	bch_err_loc01;
FLASH_NF_BCH_ERROR_LOC23_t	bch_err_loc23;
FLASH_NF_BCH_ERROR_LOC45_t	bch_err_loc45;
FLASH_NF_BCH_ERROR_LOC67_t	bch_err_loc67;
FLASH_NF_BCH_CONTROL_t	bch_ctrl;
FLASH_NF_BCH_OOB0_t	bch_oob0;
FLASH_NF_BCH_OOB1_t	bch_oob1;
FLASH_NF_BCH_OOB2_t	bch_oob2;
FLASH_NF_BCH_OOB3_t	bch_oob3;
FLASH_NF_BCH_OOB4_t	bch_oob4;
FLASH_NF_BCH_GEN0_0_t	bch_gen00;
FLASH_NF_BCH_GEN0_1_t	bch_gen01;
FLASH_NF_BCH_GEN0_2_t	bch_gen02;
FLASH_NF_BCH_GEN0_3_t	bch_gen03;
FLASH_NF_BCH_GEN0_3_t	bch_gen04;
FLASH_NF_BCH_GEN1_0_t	bch_gen10;
FLASH_NF_BCH_GEN1_1_t	bch_gen11;
FLASH_NF_BCH_GEN1_2_t	bch_gen12;
FLASH_NF_BCH_GEN1_3_t	bch_gen13;
FLASH_NF_BCH_GEN2_0_t	bch_gen20;
FLASH_NF_BCH_GEN2_1_t	bch_gen21;
FLASH_NF_BCH_GEN2_2_t	bch_gen22;
FLASH_NF_BCH_GEN2_3_t	bch_gen23;
FLASH_NF_BCH_GEN3_0_t	bch_gen30;
FLASH_NF_BCH_GEN3_1_t	bch_gen31;
FLASH_NF_BCH_GEN3_2_t	bch_gen32;
FLASH_NF_BCH_GEN3_3_t	bch_gen33;
FLASH_NF_BCH_GEN4_0_t	bch_gen40;
FLASH_NF_BCH_GEN4_1_t	bch_gen41;
FLASH_NF_BCH_GEN4_2_t	bch_gen42;
FLASH_NF_BCH_GEN4_3_t	bch_gen43;
FLASH_NF_BCH_GEN5_0_t	bch_gen50;
FLASH_NF_BCH_GEN5_1_t	bch_gen51;
FLASH_NF_BCH_GEN5_2_t	bch_gen52;
FLASH_NF_BCH_GEN5_3_t	bch_gen53;
FLASH_NF_BCH_GEN6_0_t	bch_gen60;
FLASH_NF_BCH_GEN6_1_t	bch_gen61;
FLASH_NF_BCH_GEN6_2_t	bch_gen62;
FLASH_NF_BCH_GEN6_3_t	bch_gen63;
FLASH_NF_BCH_GEN7_0_t	bch_gen70;
FLASH_NF_BCH_GEN7_1_t	bch_gen71;
FLASH_NF_BCH_GEN7_2_t	bch_gen72;
FLASH_NF_BCH_GEN7_3_t	bch_gen73;

/*DMA regs*/
/*#ifndef NAND_DIRECT_ACCESS*/
DMA_DMA_SSP_RXDMA_CONTROL_t 		dma_rxdma_ctrl;
DMA_DMA_SSP_TXDMA_CONTROL_t			dma_txdma_ctrl;
DMA_DMA_SSP_TXQ5_CONTROL_t 			dma_txq5_ctrl;
DMA_DMA_SSP_RXQ5_PKTCNT_READ_t		dma_rxq5_pktcnt_read;
DMA_DMA_SSP_TXQ5_PKTCNT_READ_t		dma_txq5_pktcnt_read;
DMA_DMA_SSP_RXQ5_BASE_DEPTH_t		dma_rxq5_base_depth;
DMA_DMA_SSP_RXQ5_WPTR_t				dma_rxq5_wptr;
DMA_DMA_SSP_RXQ5_RPTR_t				dma_rxq5_rptr;
DMA_DMA_SSP_TXQ5_BASE_DEPTH_t		dma_txq5_base_depth;
DMA_DMA_SSP_TXQ5_WPTR_t				dma_txq5_wptr;
DMA_DMA_SSP_TXQ5_RPTR_t				dma_txq5_rptr;
DMA_DMA_SSP_RXQ5_FULL_THRESHOLD_t	dma_rxq5_threshold;
DMA_DMA_SSP_RXQ5_PKTCNT_t			dma_rxq5_pktcnt;
DMA_DMA_SSP_RXQ5_FULL_DROP_PKTCNT_t dma_rxq5_drop_pktcnt;
DMA_DMA_SSP_TXQ5_PKTCNT_t			dma_txq5_pktcnt;
DMA_DMA_SSP_DMA_SSP_INTERRUPT_0_t	dma_ssp0_intsts;
DMA_DMA_SSP_DMA_SSP_INTENABLE_0_t	dma_ssp0_int_enable;
DMA_DMA_SSP_DMA_SSP_INTERRUPT_1_t	dma_ssp1_intsts;
DMA_DMA_SSP_DMA_SSP_INTENABLE_1_t	dma_ssp1_int_enable;
DMA_DMA_SSP_DESC_INTERRUPT_t		dma_ssp_desc_intsts;
DMA_DMA_SSP_DESC_INTENABLE_t		dma_ssp_desc_int_enable;
DMA_DMA_SSP_RXQ5_INTERRUPT_t		dma_ssp_rxq5_intsts;
DMA_DMA_SSP_RXQ5_INTENABLE_t		dma_ssp_rxq5_int_enable;
DMA_DMA_SSP_TXQ5_INTERRUPT_t		dma_ssp_txq5_intsts;
DMA_DMA_SSP_TXQ5_INTENABLE_t		dma_ssp_txq5_int_enable;


FLASH_FLASH_ACCESS_START_t tmp_access;

DMA_SSP_TX_DESC_T *tx_desc;
DMA_SSP_RX_DESC_T *rx_desc;
/*#endif*/


static int cs75xx_nand_block_checkbad(struct mtd_info *mtd, loff_t ofs, int getchip, int allowbbt);
static int cs75xx_nand_get_device(struct nand_chip *chip, struct mtd_info *mtd, int new_state);
static uint8_t *cs75xx_nand_fill_oob(struct nand_chip *chip, uint8_t *oob, struct mtd_oob_ops *ops);
static int cs75xx_nand_do_write_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops);
static int cs75xx_nand_do_write_ops(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops);
static int cs75xx_nand_do_read_ops(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops);
int cs75xx_nand_erase_nand(struct mtd_info *mtd, struct erase_info *instr, int allowbbt);
static int cs75xx_nand_check_wp (struct mtd_info *mtd);
static unsigned int reg_wait(unsigned int regaddr, unsigned int mask, unsigned int val, int timeout);
static void cs75xx_nand_wait_ready(struct mtd_info *mtd);



#define NAND_MAX_CHIPS CONFIG_SYS_NAND_MAX_CHIPS
#define BBT_PAGE_MASK	0xffffff3f
#define	SZ_32M 0x8000
#define	SZ_128M 0x20000

static int cs75xx_ecc_check = 0;


static unsigned int read_flash_ctrl_reg(unsigned int ofs)
{
    unsigned int *base;

    base = (unsigned int *)( ofs);
    return (*base);
}

static void write_flash_ctrl_reg(unsigned int ofs,unsigned int data)
{
    unsigned int *base;

    base = (unsigned int *)( ofs);
      *base = data;
}

static unsigned int read_dma_ctrl_reg(unsigned int ofs)
{
    unsigned int *base;

    base = (unsigned int *)( ofs);
    return (*base);
}

static void write_dma_ctrl_reg(unsigned int ofs,unsigned int data)
{

    unsigned int *base;

    base = (unsigned int *)( ofs);
      *base = data;
}

/**
 * cs75xx_nand_get_device - [GENERIC] Get chip for selected access
 * @chip:	the nand chip descriptor
 * @mtd:	MTD device structure
 * @new_state:	the state which is requested
 *
 * Get the device and lock it for exclusive access
 */
#if 0
static int
cs75xx_nand_get_device(struct nand_chip *chip, struct mtd_info *mtd, int new_state)
{


 retry:


	/* Hardware controller shared among independent devices */
	if (!chip->controller->active)
		chip->controller->active = chip;

	if (chip->controller->active == chip && chip->state == FL_READY) {
		chip->state = new_state;

		return 0;
	}
	if (new_state == FL_PM_SUSPENDED) {

		return (chip->state == FL_PM_SUSPENDED) ? 0 : -EAGAIN;
	}

	goto retry;
}

#else
static int cs75xx_nand_get_device (struct nand_chip *this, struct mtd_info *mtd, int new_state)
{
	this->state = new_state;
	return 0;
}
#endif

/**
 * cs75xx_nand_do_write_oob - [MTD Interface] NAND write out-of-band
 * @mtd:	MTD device structure
 * @to:		offset to write to
 * @ops:	oob operation description structure
 *
 * NAND write out-of-band
 */
static int cs75xx_nand_do_write_oob(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
	int chipnr, page, status, len;
	struct nand_chip *chip = mtd->priv;

#ifdef CS75XX_NAND_DEBUG
	printf("%s: to = 0x%08x, len = %i\n",
			 __func__, (unsigned int)to, (int)ops->ooblen);
#endif

	if (ops->mode == MTD_OOB_AUTO)
		len = chip->ecc.layout->oobavail;
	else
		len= mtd->oobsize;


	/* Do not allow write past end of page */
	if ((ops->ooboffs + ops->ooblen) > len) {
		printf("%s: Attempt to write "
				"past end of page\n", __func__);
		return -EINVAL;
	}

	if (unlikely(ops->ooboffs >= len)) {
		printf("%s: Attempt to start "
				"write outside oob\n", __func__);
		return -EINVAL;
	}

	/* Do not allow reads past end of device */
	if (unlikely(to >= mtd->size ||
		     ops->ooboffs + ops->ooblen >
			((mtd->size >> chip->page_shift) -
			 (to >> chip->page_shift)) * len)) {
		printf("%s: Attempt write beyond "
				"end of device\n", __func__);
		return -EINVAL;
	}

	chipnr = (int)(to >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	/* Shift to get page */
	page = (int)(to >> chip->page_shift);

	/*
	 * Reset the chip. Some chips (like the Toshiba TC5832DC found in one
	 * of my DiskOnChip 2000 test units) will clear the whole data page too
	 * if we don't do this. I have no clue why, but I seem to have 'fixed'
	 * it in the doc2000 driver in August 1999.  dwmw2.
	 */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Check, if it is write protected */
	if (cs75xx_nand_check_wp(mtd))
		return -EROFS;

	/* Invalidate the page cache, if we write to the cached page */
	if (page == chip->pagebuf)
		chip->pagebuf = -1;

	memset(chip->oob_poi, 0xff, mtd->oobsize);
	cs75xx_nand_fill_oob(chip, ops->oobbuf, ops);
	status = chip->ecc.write_oob(mtd, chip, page & chip->pagemask);
	memset(chip->oob_poi, 0xff, mtd->oobsize);

	if (status)
		return status;

	ops->oobretlen = ops->ooblen;

	return 0;
}


/*
 * return register value after "(*reg) & mask == val", with timeout
 */
static unsigned int reg_wait(unsigned int regaddr, unsigned int mask, unsigned int val, int timeout)
{
	unsigned int i, tmp;

	for(i=timeout;i>0;i--)
	{
		tmp = read_flash_ctrl_reg(regaddr);
		if((tmp&mask) == val)
			return 0;//TRUE;
		udelay(5);
	}

	printf("reg_wait error !!  \n");

	return 1;//FALSE;
}



/*#ifdef CONFIG_CS75XX_NAND_HWECC*/

static int cs75xx_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{
#if 0
	struct nand_chip *nand_chip = mtd->priv;
	struct cs75xx_nand_host *host = nand_chip->priv;

	/*
	 * 1-Bit errors are automatically corrected in HW.  No need for
	 * additional correction.  2-Bit errors cannot be corrected by
	 * HW ECC, so we need to return failure
	 */
	unsigned short_t ecc_status = readw(&host->regs->nfc_ecc_status_result);

	if (((ecc_status & 0x3) == 2) || ((ecc_status >> 2) == 2)) {
		printf(
		      "CS75XX_NAND: HWECC uncorrectable 2-bit ECC error\n");
		return -1;
	}
#endif
	return 0;
}

static int cs75xx_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				  u_char *ecc_code)
{
	return 0;
}

static void check_flash_ctrl_status()
{
	int rty=0;

	flash_status.wrd = read_flash_ctrl_reg(FLASH_STATUS);
	while(flash_status.bf.nState)
        {
  	     flash_status.wrd = read_flash_ctrl_reg(FLASH_STATUS);
             udelay(5);
             rty++;
             if(rty > 50000)
             {
             	printk("FLASH_STATUS ERROR: %x\n",flash_status.wrd);
             	return;
             }
        }

}


/**
 * cs75xx_nand_release_device - [GENERIC] release chip
 * @mtd:	MTD device structure
 *
 * Deselect, release chip lock and wake up anyone waiting on the device
 */
/* XXX U-BOOT XXX */
static void cs75xx_nand_release_device (struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	this->select_chip(mtd, -1);	/* De-select the NAND device */
}

static void cs75xx_nand_read_id(int chip_no, unsigned char *id)
{
	unsigned int opcode, i;
	const unsigned int extid=8;
	FLASH_FLASH_ACCESS_START_t tmp_access;

	check_flash_ctrl_status();

	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0x0);

	flash_type.wrd = read_flash_ctrl_reg(FLASH_TYPE);

	/*need to check extid byte counts*/
	nf_cnt.wrd = 0;

	nf_cnt.bf.nflashRegOobCount = NCNT_EMPTY_OOB;
	nf_cnt.bf.nflashRegDataCount = NCNT_DATA_8;
	nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_1;
	nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;

	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);

	nf_cmd.wrd = 0;
	nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READID;
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd);
	nf_addr1.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd);
	nf_addr2.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd);

	/* read maker code */
	nf_access.wrd = 0;
	nf_access.bf.nflashCeAlt = chip_no;

	nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;
	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);


	for( i=0; i < extid; i++) {

		flash_start.wrd = 0;
		flash_start.bf.nflashRegReq = FLASH_GO;
		flash_start.bf.nflashRegCmd = FLASH_RD;
		write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);


		flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
		tmp_access.wrd = 0;
		tmp_access.bf.nflashRegReq = 1;
		reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 2000);


		opcode=read_flash_ctrl_reg(FLASH_NF_DATA);
		id[i] = (unsigned char)(( opcode >> ((i<<3) % 32 ) )& 0xff);
	}


	ecc_reset.wrd = 0;
	ecc_reset.bf.eccClear = ECC_CLR;
	ecc_reset.bf.fifoClear = FIFO_CLR;
	ecc_reset.bf.nflash_reset = NF_RESET;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);
}









/**
 * cs75xx_nand_fill_oob - [Internal] Transfer client buffer to oob
 * @chip:	nand chip structure
 * @oob:	oob data buffer
 * @ops:	oob ops structure
 */
static uint8_t *cs75xx_nand_fill_oob(struct nand_chip *chip, uint8_t *oob,
				  struct mtd_oob_ops *ops)
{
	size_t len = ops->ooblen;

	switch(ops->mode) {

	case MTD_OOB_PLACE:
	case MTD_OOB_RAW:
		memcpy(chip->oob_poi + ops->ooboffs, oob, len);
		return oob + len;

	case MTD_OOB_AUTO: {
		struct nand_oobfree *free = chip->ecc.layout->oobfree;
		uint32_t boffs = 0, woffs = ops->ooboffs;
		size_t bytes = 0;

		for(; free->length && len; free++, len -= bytes) {
			/* Write request not from offset 0 ? */
			if (unlikely(woffs)) {
				if (woffs >= free->length) {
					woffs -= free->length;
					continue;
				}
				boffs = free->offset + woffs;
				bytes = min_t(size_t, len,
					      (free->length - woffs));
				woffs = 0;
			} else {
				bytes = min_t(size_t, len, free->length);
				boffs = free->offset;
			}
			memcpy(chip->oob_poi + boffs, oob, bytes);
			oob += bytes;
		}
		return oob;
	}
	//default:
		//BUG();
	}
	return NULL;
}

/**
 * nand_transfer_oob - [Internal] Transfer oob to client buffer
 * @chip:	nand chip structure
 * @oob:	oob destination address
 * @ops:	oob ops structure
 * @len:	size of oob to transfer
 */
static uint8_t *cs75xx_nand_transfer_oob(struct nand_chip *chip, uint8_t *oob,
				  struct mtd_oob_ops *ops, size_t len)
{
	switch(ops->mode) {

	case MTD_OOB_PLACE:
	case MTD_OOB_RAW:
		memcpy(oob, chip->oob_poi + ops->ooboffs, len);
		return oob + len;

	case MTD_OOB_AUTO: {
		struct nand_oobfree *free = chip->ecc.layout->oobfree;
		uint32_t boffs = 0, roffs = ops->ooboffs;
		size_t bytes = 0;

		for(; free->length && len; free++, len -= bytes) {
			/* Read request not from offset 0 ? */
			if (unlikely(roffs)) {
				if (roffs >= free->length) {
					roffs -= free->length;
					continue;
				}
				boffs = free->offset + roffs;
				bytes = min_t(size_t, len,
					      (free->length - roffs));
				roffs = 0;
			} else {
				bytes = min_t(size_t, len, free->length);
				boffs = free->offset;
			}
			memcpy(oob, chip->oob_poi + boffs, bytes);
			oob += bytes;
		}
		return oob;
	}
	//default:
		//BUG();
	}
	return NULL;
}

/**
 * cs75xx_nand_block_isbad - [MTD Interface] Check if block at offset is bad
 * @mtd:	MTD device structure
 * @offs:	offset relative to mtd start
 */
static int cs75xx_nand_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	int rc;
	/* Check for invalid offset */
	if (offs > mtd->size)
		return -EINVAL;


//	write_flash_ctrl_reg( FLASH_TYPE, 0xf000);

	rc = cs75xx_nand_block_checkbad(mtd, offs, 1, 0);


	return rc;
}

/**
 * cs75xx_nand_block_markbad - [MTD Interface] Mark block at the given offset as bad
 * @mtd:	MTD device structure
 * @ofs:	offset relative to mtd start
 */
static int cs75xx_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	int ret;


//	write_flash_ctrl_reg( FLASH_TYPE, 0xf000);
	if ((ret = cs75xx_nand_block_isbad(mtd, ofs))) {
		/* If it was bad already, return success and do nothing. */


		if (ret > 0)
			return 0;
		return ret;
	}

	ret= chip->block_markbad(mtd, ofs);


	return ret;
}


#if 0 //CONFIG_PM
static int cs75xx_nand_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mfd_cell *cell = (struct mfd_cell *)dev->dev.platform_data;

	struct nand_chip *chip = cs75xx_host->nand_chip;//mtd->priv;

	return cs75xx_nand_get_device(chip, cs75xx_host->mtd, FL_PM_SUSPENDED);
}

static int cs75xx_nand_resume(struct platform_device *dev)
{
	struct nand_chip *chip = cs75xx_host->nand_chip;//mtd->priv;

	if (chip->state == FL_PM_SUSPENDED)
		cs75xx_nand_release_device(cs75xx_host->mtd);
	else
		printf(KERN_ERR "%s called for a chip which is not "
		       "in suspended state\n", __func__);
}
#else
#define cs75xx_nand_suspend NULL
#define cs75xx_nand_resume NULL
#endif

/**
 * cs75xx_nand_sync - [MTD Interface] sync
 * @mtd:	MTD device structure
 *
 * Sync is actually a wait for chip ready function
 */
static void cs75xx_nand_sync(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

#ifdef CS75XX_NAND_DEBUG
	printf( "%s: called\n", __func__);
#endif

	/* Grab the lock and see if the device is available */
	cs75xx_nand_get_device(chip, mtd, FL_SYNCING);
	/* Release it and go back */
	cs75xx_nand_release_device(mtd);
}

/**
 * cs75xx_nand_check_wp - [GENERIC] check if the chip is write protected
 * @mtd:	MTD device structure
 * Check, if the device is write protected
 *
 * The function expects, that the device is already selected
 */
static int cs75xx_nand_check_wp (struct mtd_info *mtd)
{
//	struct nand_chip *this = mtd->priv;
	/* Check the WP bit */
	int ready;

	check_flash_ctrl_status();

	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0x0); //disable ecc gen

	nf_cnt.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = NCNT_EMPTY_OOB;
	nf_cnt.bf.nflashRegDataCount = NCNT_DATA_1;
	nf_cnt.bf.nflashRegAddrCount = NCNT_EMPTY_ADDR;
	nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);

	nf_cmd.wrd = 0;
	nf_cmd.bf.nflashRegCmd0 = NAND_CMD_STATUS;
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd);

	nf_access.wrd = 0;
	nf_access.bf.nflashCeAlt = CHIP_EN;
	//nf_access.bf.nflashDirWr = ;
	nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;
	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

	flash_start.wrd = 0;
	flash_start.bf.nflashRegReq = FLASH_GO;
	flash_start.bf.nflashRegCmd = FLASH_RD;
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

	flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	while(flash_start.bf.nflashRegReq)
         {
  	     flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
                udelay(1);

        }

      	ready = read_flash_ctrl_reg(FLASH_NF_DATA)&0xff;
      	if(ready==0xff)
      		printf("cs75xx_nand_check_wp flash status : %x\n",read_flash_ctrl_reg(FLASH_STATUS));

	return (ready & NAND_STATUS_WP) ? 0 : 1;
}

/**
 * cs75xx_nand_write_oob - [MTD Interface] NAND write data and/or out-of-band
 * @mtd:	MTD device structure
 * @to:		offset to write to
 * @ops:	oob operation description structure
 */
static int cs75xx_nand_write_oob(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow writes past end of device */
	if (ops->datbuf && (to + ops->len) > mtd->size) {
		printf("%s: Attempt write beyond "
				"end of device\n", __func__);
		return -EINVAL;
	}



	cs75xx_nand_get_device(chip, mtd, FL_WRITING);

	switch(ops->mode) {
	case MTD_OOB_PLACE:
	case MTD_OOB_AUTO:
	case MTD_OOB_RAW:
		break;

	default:
		goto out;
	}

	if (!ops->datbuf)
		ret = cs75xx_nand_do_write_oob(mtd, to, ops);
	else
		ret = cs75xx_nand_do_write_ops(mtd, to, ops);

 out:
	cs75xx_nand_release_device(mtd);

	return ret;
}

/**
 * cs75xx_nand_do_read_oob - [Intern] NAND read out-of-band
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @ops:	oob operations description structure
 *
 * NAND read out-of-band data from the spare area
 */
static int cs75xx_nand_do_read_oob(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
	int page, realpage, chipnr, sndcmd = 1;
	struct nand_chip *chip = mtd->priv;
	int blkcheck = (1 << (chip->phys_erase_shift - chip->page_shift)) - 1;
	int readlen = ops->ooblen;
	int len;
	uint8_t *buf = ops->oobbuf;


#ifdef CS75XX_NAND_DEBUG
	printf("%s: from = 0x%08Lx, len = %i\n",
			__func__, (unsigned long long)from, readlen);
#endif

	if (ops->mode == MTD_OOB_AUTO)
		len = chip->ecc.layout->oobavail;
	else
		len = mtd->oobsize;


	if (unlikely(ops->ooboffs >= len)) {
		printf("%s: Attempt to start read "
					"outside oob\n", __func__);
		return -EINVAL;
	}

#if 0
	/* Do not allow read past end of page */
	if ((ops->ooboffs + ops->ooblen) > len) {
		printf("%s: Attempt to read "
				"past end of page\n", __func__);
		return -EINVAL;
	}

#endif

	/* Do not allow reads past end of device */
	if (unlikely(from >= mtd->size ||
		     ops->ooboffs + readlen > ((mtd->size >> chip->page_shift) -
					(from >> chip->page_shift)) * len)) {
		printf("%s: Attempt read beyond end "
					"of device\n", __func__);
		return -EINVAL;
	}

	chipnr = (int)(from >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	/* Shift to get page */
	realpage = (int)(from >> chip->page_shift);
	page = realpage & chip->pagemask;

	while(1) {
		sndcmd = chip->ecc.read_oob(mtd, chip, page, sndcmd);

		len = min(len, readlen);
		buf = cs75xx_nand_transfer_oob(chip, buf, ops, len);

		if (!(chip->options & NAND_NO_READRDY)) {
			/*
			 * Apply delay or wait for ready/busy pin. Do this
			 * before the AUTOINCR check, so no problems arise if a
			 * chip which does auto increment is marked as
			 * NOAUTOINCR by the board driver.
			 */
			if (!chip->dev_ready)
				udelay(chip->chip_delay);
			else
				cs75xx_nand_wait_ready(mtd);
		}

		readlen -= len;
		if (!readlen)
			break;

		/* Increment page address */
		realpage++;

		page = realpage & chip->pagemask;
		/* Check, if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}

		/* Check, if the chip supports auto page increment
		 * or if we have hit a block boundary.
		 */
		if (!NAND_CANAUTOINCR(chip) || !(page & blkcheck))
			sndcmd = 1;
	}

	ops->oobretlen = ops->ooblen;
	return 0;
}

/**
 * cs75xx_nand_read_oob - [MTD Interface] NAND read data and/or out-of-band
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @ops:	oob operation description structure
 *
 * NAND read data and/or out-of-band data
 */
static int cs75xx_nand_read_oob(struct mtd_info *mtd, loff_t from,
			 struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow reads past end of device */
	if (ops->datbuf && (from + ops->len) > mtd->size) {
		printf("%s: Attempt read "
				"beyond end of device\n", __func__);
		return -EINVAL;
	}



	cs75xx_nand_get_device(chip, mtd, FL_READING);

	switch(ops->mode) {
	case MTD_OOB_PLACE:
	case MTD_OOB_AUTO:
	case MTD_OOB_RAW:
		break;

	default:
		goto out;
	}

	if (!ops->datbuf)
		ret = cs75xx_nand_do_read_oob(mtd, from, ops);
	else
		ret = cs75xx_nand_do_read_ops(mtd, from, ops);

 out:
	cs75xx_nand_release_device(mtd);


	return ret;
}


/**
 * cs75xx_nand_write - [MTD Interface] NAND write with ECC
 * @mtd:	MTD device structure
 * @to:		offset to write to
 * @len:	number of bytes to write
 * @retlen:	pointer to variable to store the number of written bytes
 * @buf:	the data to write
 *
 * NAND write with ECC
 */
static int cs75xx_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
			  size_t *retlen, const uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	int ret;

	/* Do not allow reads past end of device */
	if ((to + len) > mtd->size)
		return -EINVAL;
	if (!len)
		return 0;


	cs75xx_nand_get_device(chip, mtd, FL_WRITING);

	chip->ops.len = len;
	chip->ops.datbuf = (uint8_t *)buf;
	chip->ops.oobbuf = NULL;

	ret = cs75xx_nand_do_write_ops(mtd, to, &chip->ops);

	*retlen = chip->ops.retlen;

	cs75xx_nand_release_device(mtd);


	return ret;
}

/**
 * cs75xx_nand_do_read_ops - [Internal] Read data with ECC
 *
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @ops:	oob ops structure
 *
 * Internal function. Called with chip held.
 */
static int cs75xx_nand_do_read_ops(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
	int chipnr, page, realpage, col, bytes, aligned;
	struct nand_chip *chip = mtd->priv;
	struct mtd_ecc_stats stats;
	int blkcheck = (1 << (chip->phys_erase_shift - chip->page_shift)) - 1;
	int sndcmd = 1;
	int ret = 0;
	uint32_t readlen = ops->len;
	uint32_t oobreadlen = ops->ooblen;
	uint8_t *bufpoi, *oob, *buf;

	stats = mtd->ecc_stats;

	chipnr = (int)(from >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	realpage = (int)(from >> chip->page_shift);
	page = realpage & chip->pagemask;

	col = (int)(from & (mtd->writesize - 1));

	buf = ops->datbuf;
	oob = ops->oobbuf;

	while(1) {
		bytes = min(mtd->writesize - col, readlen);
		aligned = (bytes == mtd->writesize);

		/* Is the current page in the buffer ? */
		if (realpage != chip->pagebuf || oob) {
			bufpoi = aligned ? buf : chip->buffers->databuf;

			if (likely(sndcmd)) {
				chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);
				sndcmd = 0;
			}

			/* Now read the page into the buffer */
			if (unlikely(ops->mode == MTD_OOB_RAW))
				ret = chip->ecc.read_page_raw(mtd, chip,
							      bufpoi, page);
			else if (!aligned && NAND_SUBPAGE_READ(chip) && !oob)
				ret = chip->ecc.read_subpage(mtd, chip, col, bytes, bufpoi);
			else
				ret = chip->ecc.read_page(mtd, chip, bufpoi,
							  page);
			if (ret < 0)
				break;

			/* Transfer not aligned data */
			if (!aligned) {
				if (!NAND_SUBPAGE_READ(chip) && !oob)
					chip->pagebuf = realpage;
				memcpy(buf, chip->buffers->databuf + col, bytes);
			}

			buf += bytes;

			if (unlikely(oob)) {
				/* Raw mode does data:oob:data:oob */
				if (ops->mode != MTD_OOB_RAW) {
					int toread = min(oobreadlen,
						chip->ecc.layout->oobavail);
					if (toread) {
						oob = cs75xx_nand_transfer_oob(chip,
							oob, ops, toread);
						oobreadlen -= toread;
					}
				} else
					buf = cs75xx_nand_transfer_oob(chip, buf, ops, mtd->oobsize);
			}

			if (!(chip->options & NAND_NO_READRDY)) {
				/*
				 * Apply delay or wait for ready/busy pin. Do
				 * this before the AUTOINCR check, so no
				 * problems arise if a chip which does auto
				 * increment is marked as NOAUTOINCR by the
				 * board driver.
				 */
				if (!chip->dev_ready)
					udelay(chip->chip_delay);
				else
					cs75xx_nand_wait_ready(mtd);
			}
		} else {
			memcpy(buf, chip->buffers->databuf + col, bytes);
			buf += bytes;
		}

		readlen -= bytes;

		if (!readlen)
			break;

		/* For subsequent reads align to page boundary. */
		col = 0;
		/* Increment page address */
		realpage++;

		page = realpage & chip->pagemask;
		/* Check, if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}

		/* Check, if the chip supports auto page increment
		 * or if we have hit a block boundary.
		 */
		if (!NAND_CANAUTOINCR(chip) || !(page & blkcheck))
			sndcmd = 1;
	}

	ops->retlen = ops->len - (size_t) readlen;
	if (oob)
		ops->oobretlen = ops->ooblen - oobreadlen;

	if (ret)
		return ret;

	if (mtd->ecc_stats.failed - stats.failed)
		return -EBADMSG;

	return  mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;
}

/**
 * cs75xx_nand_read - [MTD Interface] MTD compability function for nand_do_read_ecc
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @len:	number of bytes to read
 * @retlen:	pointer to variable to store the number of read bytes
 * @buf:	the databuffer to put data
 *
 * Get hold of the chip and call nand_do_read
 */

static int cs75xx_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
		     size_t *retlen, uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	int ret;

	/* Do not allow reads past end of device */
	if ((from + len) > mtd->size)
		return -EINVAL;
	if (!len)
		return 0;

	cs75xx_nand_get_device(chip, mtd, FL_READING);

	chip->ops.len = len;
	chip->ops.datbuf = buf;
	chip->ops.oobbuf = NULL;

	ret = cs75xx_nand_do_read_ops(mtd, from, &chip->ops);

	*retlen = chip->ops.retlen;

	cs75xx_nand_release_device(mtd);

	return ret;
}

/**
 * cs75xx_nand_erase_block - [GENERIC] erase a block
 * @mtd:	MTD device structure
 * @page:	page address
 *
 * Erase a block.
 */

static int cs75xx_nand_erase_block(struct mtd_info *mtd, int page)
{
//	int opcode,tst=0,tst1=0,tst2=0;
	struct nand_chip *this = mtd->priv;
	u64 test;
	FLASH_FLASH_ACCESS_START_t tmp_access;

	check_flash_ctrl_status();

	/* Send commands to erase a page */
	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0); //

	nf_cnt.wrd = 0;
	nf_cmd.wrd = 0;
	nf_addr1.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = NCNT_EMPTY_OOB;
	nf_cnt.bf.nflashRegDataCount = NCNT_EMPTY_DATA;
	nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;

	/*
	test = this->chipsize;
	test = test / mtd->writesize;
	if((this->chipsize/mtd->writesize) > 0x10000)
	*/

	test = 0x10000 * mtd->writesize;
	if( this->chipsize  > test) {
	    nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_3;
	} else {
	    nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_2;
	}

	nf_cmd.bf.nflashRegCmd0 = NAND_CMD_ERASE1;
	nf_cmd.bf.nflashRegCmd1 = NAND_CMD_ERASE2;
	nf_addr1.wrd = page;
	nf_addr2.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd);
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd);
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd);

	nf_access.wrd = 0;
	nf_access.bf.nflashCeAlt = CHIP_EN;
	//nf_access.bf.nflashDirWr = ;
	nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;

	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);
	flash_start.wrd = 0;
	flash_start.bf.nflashRegReq = FLASH_GO;
	flash_start.bf.nflashRegCmd = FLASH_RD;  //no data access use read..
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);
	flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	tmp_access.wrd = 0;
	tmp_access.bf.nflashRegReq = 1;
	reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);

	return 0;
}



/**
 * cs75xx_nand_block_checkbad - [GENERIC] Check if a block is marked bad
 * @mtd:	MTD device structure
 * @ofs:	offset from device start
 * @getchip:	0, if the chip is already selected
 * @allowbbt:	1, if its allowed to access the bbt area
 *
 * Check, if the block is bad. Either by reading the bad block table or
 * calling of the scan function.
 */
static int cs75xx_nand_block_checkbad(struct mtd_info *mtd, loff_t ofs, int getchip,
			       int allowbbt)
{
	struct nand_chip *chip = mtd->priv;

	if (!chip->bbt)
		return chip->block_bad(mtd, ofs, getchip);

	/* Return info from the table */
	return nand_isbad_bbt(mtd, ofs, allowbbt);
}

/**
 * cs75xx_nand_erase - [MTD Interface] erase block(s)
 * @mtd:	MTD device structure
 * @instr:	erase instruction
 *
 * Erase one ore more blocks
 */
static int cs75xx_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int rc;



	rc = cs75xx_nand_erase_nand(mtd, instr, 0);

	return rc;

}

/**
 * cs75xx_nand_erase_nand - [Internal] erase block(s)
 * @mtd:	MTD device structure
 * @instr:	erase instruction
 * @allowbbt:	allow erasing the bbt area
 *
 * Erase one ore more blocks
 */
int cs75xx_nand_erase_nand(struct mtd_info *mtd, struct erase_info *instr,
		    int allowbbt)
{
	int page, status, pages_per_block, ret, chipnr;
	struct nand_chip *chip = mtd->priv;
	int rewrite_bbt[NAND_MAX_CHIPS]={0};
	unsigned int bbt_masked_page = 0xffffffff;
	loff_t len;

#ifdef CS75XX_NAND_DEBUG
	printf("%s: start = 0x%012llx, len = %llu\n",
				__func__, (unsigned long long)instr->addr,
				(unsigned long long)instr->len);
#endif

	/* Start address must align on block boundary */
	if (instr->addr & ((1 << chip->phys_erase_shift) - 1)) {
		printf("%s: Unaligned address\n", __func__);
		printf( KERN_ALERT "unaligned_chipptr!!!");
		return -EINVAL;
	}

	/* Length must align on block boundary */
	if (instr->len & ((1 << chip->phys_erase_shift) - 1)) {
		printf("%s: Length not block aligned\n",
					__func__);
		return -EINVAL;
	}

	/* Do not allow erase past end of device */
	if ((instr->len + instr->addr) > mtd->size) {
		printf("%s: Erase past end of device\n",
					__func__);
		return -EINVAL;
	}

	instr->fail_addr = MTD_FAIL_ADDR_UNKNOWN;

	/* Grab the lock and see if the device is available */
	cs75xx_nand_get_device(chip, mtd, FL_ERASING);

	/* Shift to get first page */
	page = (int)(instr->addr >> chip->page_shift);
	chipnr = (int)(instr->addr >> chip->chip_shift);

	/* Calculate pages in each block */
	pages_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);

	/* Select the NAND device */
	chip->select_chip(mtd, chipnr);

	/* Check, if it is write protected */
	if (cs75xx_nand_check_wp(mtd)) {
		printf("%s: Device is write protected!!!\n",
					__func__);
		instr->state = MTD_ERASE_FAILED;
		goto erase_exit;
	}

	/*
	 * If BBT requires refresh, set the BBT page mask to see if the BBT
	 * should be rewritten. Otherwise the mask is set to 0xffffffff which
	 * can not be matched. This is also done when the bbt is actually
	 * erased to avoid recusrsive updates
	 */
	if (chip->options & BBT_AUTO_REFRESH && !allowbbt)
		bbt_masked_page = chip->bbt_td->pages[chipnr] & BBT_PAGE_MASK;

	/* Loop through the pages */
	len = instr->len;

	instr->state = MTD_ERASING;

	while (len) {
		/*
		 * heck if we have a bad block, we do not erase bad blocks !
		 */
		if (cs75xx_nand_block_checkbad(mtd, ((loff_t) page) <<
					chip->page_shift, 0, allowbbt)) {
			printf(KERN_WARNING "%s: attempt to erase a bad block "
					"at page 0x%08x\n", __func__, page);
			instr->state = MTD_ERASE_FAILED;
			goto erase_exit;
		}

		/*
		 * Invalidate the page cache, if we erase the block which
		 * contains the current cached page
		 */
		if (page <= chip->pagebuf && chip->pagebuf <
		    (page + pages_per_block))
			chip->pagebuf = -1;

		//chip->erase_cmd(mtd, page & chip->pagemask);
		cs75xx_nand_erase_block(mtd, page);

		status = chip->waitfunc(mtd, chip);

		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_ERASING,
					       status, page);

		/* See if block erase succeeded */
		if (status & NAND_STATUS_FAIL) {
			printf("%s: Failed erase, "
					"page 0x%08x\n", __func__, page);
			instr->state = MTD_ERASE_FAILED;
			instr->fail_addr =
				((loff_t)page << chip->page_shift);
			goto erase_exit;
		}

		/*
		 * If BBT requires refresh, set the BBT rewrite flag to the
		 * page being erased
		 */
		if (bbt_masked_page != 0xffffffff &&
		    (page & BBT_PAGE_MASK) == bbt_masked_page)
			    rewrite_bbt[chipnr] =
					((loff_t)page << chip->page_shift);

		/* Increment page address and decrement length */
		len -= (1 << chip->phys_erase_shift);
		page += pages_per_block;

		/* Check, if we cross a chip boundary */
		if (len && !(page & chip->pagemask)) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);

			/*
			 * If BBT requires refresh and BBT-PERCHIP, set the BBT
			 * page mask to see if this BBT should be rewritten
			 */
			if (bbt_masked_page != 0xffffffff &&
			    (chip->bbt_td->options & NAND_BBT_PERCHIP))
				bbt_masked_page = chip->bbt_td->pages[chipnr] &
					BBT_PAGE_MASK;
		}
	}
	instr->state = MTD_ERASE_DONE;

 erase_exit:

	ret = instr->state == MTD_ERASE_DONE ? 0 : -EIO;

	/* Deselect and wake up anyone waiting on the device */
	cs75xx_nand_release_device(mtd);

	/* Do call back function */
	if (!ret)
		mtd_erase_callback(instr);

	/*
	 * If BBT requires refresh and erase was successful, rewrite any
	 * selected bad block tables
	 */
	if (bbt_masked_page == 0xffffffff || ret)
		return ret;

	for (chipnr = 0; chipnr < chip->numchips; chipnr++) {
		if (!rewrite_bbt[chipnr])
			continue;
		/* update the BBT for chip */
		printf("%s: nand_update_bbt "
			"(%d:0x%x 0x%0x)\n", __func__, chipnr,
			rewrite_bbt[chipnr], chip->bbt_td->pages[chipnr]);
		nand_update_bbt(mtd, rewrite_bbt[chipnr]);
	}

	/* Return more or less happy */
	return ret;
}

/**
 * cs75xx_nand_write_oob_std - [REPLACABLE] the most common OOB data write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to write
 */
static int cs75xx_nand_write_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
			      int page)
{
	int status = 0, i;
	FLASH_FLASH_ACCESS_START_t tmp_access;
	//const uint8_t *buf = chip->oob_poi;
	//int length = mtd->oobsize;

	check_flash_ctrl_status();

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);
	//chip->write_buf(mtd, buf, length);
	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0x0); //disable ecc gen

	nf_cmd.wrd = 0;
	nf_addr1.wrd = 0;
	nf_cnt.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = mtd->oobsize- 1;
	nf_cnt.bf.nflashRegDataCount = NCNT_EMPTY_DATA;

	nf_addr2.wrd = 0;

	if(chip->chipsize < SZ_32M )
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_3;

		if(mtd->writesize > NCNT_512P_DATA)
		{
			nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
			nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
			nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		}
		else
		{
			nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_3;
			nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READOOB;  //??
			nf_cmd.bf.nflashRegCmd1 = NAND_CMD_SEQIN;
			nf_cmd.bf.nflashRegCmd2 = NAND_CMD_PAGEPROG;
		}
		//read oob need to add page data size to match correct oob ddress
		nf_addr1.wrd = (((page & 0x00ffffff)<<8));
		nf_addr2.wrd = ((page & 0xff000000)>>24);
	}
	else if(chip->chipsize <= SZ_128M )
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_4;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		nf_addr1.wrd = (((page & 0xffff)<<16) + (mtd->writesize&0xffff));
		nf_addr2.wrd = ((page & 0xffff0000)>>16);

	}
	else //if((chip->chipsize > (128 << 20)) ))
	{

		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_5;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		nf_addr1.wrd = (((page & 0xffff)<<16) + (mtd->writesize&0xffff));
		nf_addr2.wrd = ((page & 0xffff0000)>>16);

	}

	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd); //write read id command
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd); //write address 0x0
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd);

	pwrite = (unsigned int *) chip->oob_poi;

	for(i=0;i<((mtd->oobsize/4));i++)
	{
		nf_access.wrd = 0;
		nf_access.bf.nflashCeAlt = CHIP_EN;
		//nf_access.bf.nflashDirWr = ;
		nf_access.bf.nflashRegWidth = NFLASH_WiDTH32;
		write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

		write_flash_ctrl_reg(FLASH_NF_DATA,pwrite[i]);

		flash_start.wrd = 0;
		flash_start.bf.nflashRegReq = FLASH_GO;
		flash_start.bf.nflashRegCmd = FLASH_WT;
		//flash_start.bf.nflash_random_access = RND_ENABLE;
		write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

		tmp_access.wrd = 0;
		tmp_access.bf.nflashRegReq = 1;
		reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);

	}
#if 0
	unsigned int rtmp[80];
	nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;
	nf_cmd.bf.nflashRegCmd1 = NAND_CMD_READSTART;
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd); //write read id command

	for(i=0;i<((mtd->oobsize/4));i++)
	{
		nf_access.wrd = 0;
		nf_access.bf.nflashCeAlt = CHIP_EN;
		//nf_access.bf.nflashDirWr = ;
		nf_access.bf.nflashRegWidth = NFLASH_WiDTH32;
		write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

		flash_start.wrd = 0;
		flash_start.bf.nflashRegReq = FLASH_GO;
		flash_start.bf.nflashRegCmd = FLASH_RD;
		//flash_start.bf.nflash_random_access = RND_ENABLE;
		write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

		flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
		while(flash_start.bf.nflashRegReq)
  	     {
  		    flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
  	         udelay(1);

    	}
			rtmp[i] = read_flash_ctrl_reg(FLASH_NF_DATA);
a
	}
	if (memcmp((unsigned char *)pwrite , (unsigned char *)rtmp, mtd->oobsize))
	   printf("W->R oob error\n");
#endif
	check_flash_ctrl_status();

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

/**
 * cs75xx_nand_read_oob_std - [REPLACABLE] the most common OOB data read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @page:	page number to read
 * @sndcmd:	flag whether to issue read command or not
 */
static int cs75xx_nand_read_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
			     int page, int sndcmd)
{
	int i;
	FLASH_FLASH_ACCESS_START_t tmp_access;

	check_flash_ctrl_status();

	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0x0); //disable ecc gen

	nf_cmd.wrd = 0;
	nf_addr1.wrd = 0;
	nf_addr2.wrd = 0;
	nf_cnt.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = mtd->oobsize -1 ;
	nf_cnt.bf.nflashRegDataCount = NCNT_EMPTY_DATA;

	if(chip->chipsize < (32 << 20))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_3;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
		if(mtd->writesize > NCNT_512P_DATA)
			nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;
		else
			nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READOOB;

		nf_addr1.wrd = ((page & 0x00ffffff)<<8);
		nf_addr2.wrd = ((page & 0xff000000)>>24);
	}
	else if((chip->chipsize >= (32 << 20)) && (chip->chipsize <= (128 << 20)))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_4;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;

		// Jeneng
		if(mtd->writesize > NCNT_512P_DATA){
			nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
			nf_cmd.bf.nflashRegCmd1 = NAND_CMD_READSTART;
		}
		nf_addr1.wrd = (((page & 0xffff)<<16) + (mtd->writesize&0xffff));
		nf_addr2.wrd = ((page & 0xffff0000)>>16);

	}
	else //if((chip->chipsize > (128 << 20)) ))
	{

		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_5;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_READSTART;
		nf_addr1.wrd = (((page & 0xffff)<<16) + (mtd->writesize&0xffff));
		nf_addr2.wrd = ((page & 0xffff0000)>>16);
	}


	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd); //write read id command
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd); //write address 0x0
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd);

	pread = (unsigned int *) chip->oob_poi;

	for(i=0;i< mtd->oobsize/4;i++)
	{
		nf_access.wrd = 0;
		nf_access.bf.nflashCeAlt = CHIP_EN;
		//nf_access.bf.nflashDirWr = ;
		nf_access.bf.nflashRegWidth = NFLASH_WiDTH32;
		write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

		flash_start.wrd = 0;
		flash_start.bf.nflashRegReq = FLASH_GO;
		flash_start.bf.nflashRegCmd = FLASH_RD;
		//flash_start.bf.nflash_random_access = RND_ENABLE;
		write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

		flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);

		tmp_access.wrd = 0;
		tmp_access.bf.nflashRegReq = 1;
		reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);

			pread[i] = read_flash_ctrl_reg(FLASH_NF_DATA);

	}
	return sndcmd;
}

/**
 * cs75xx_nand_write_page_hwecc - [REPLACABLE] hardware ecc based page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void cs75xx_nand_write_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
{

	int i, j, eccsize = chip->ecc.size, page, col;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	/*uint8_t *ecc_calc = chip->buffers->ecccalc;*/
	/*const uint8_t *p = buf;*/
	uint32_t *eccpos = chip->ecc.layout->eccpos, *addr;
	DMA_DMA_SSP_RXQ5_INTERRUPT_t	tmp_dma_ssp_rxq5_intsts;
	DMA_DMA_SSP_TXQ5_INTERRUPT_t	tmp_dma_ssp_txq5_intsts;
	FLASH_FLASH_ACCESS_START_t tmp_access;
#if defined(CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined(CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)

	FLASH_NF_BCH_STATUS_t	tmp_bsc_sts;
#else
	FLASH_NF_ECC_STATUS_t	tmp_ecc_sts;
#endif

	check_flash_ctrl_status();

	page = host->page_addr;
  	col  = host->col_addr;

	ecc_reset.wrd = 3;
	ecc_reset.bf.eccClear = ECC_CLR;
	ecc_reset.bf.fifoClear = FIFO_CLR;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);

	flash_int_sts.bf.regIrq = 1;
	write_flash_ctrl_reg(FLASH_FLASH_INTERRUPT, flash_int_sts.wrd);

	ecc_reset.wrd = 0;
	ecc_reset.bf.eccClear = 1;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);

#ifdef	CONFIG_CS752X_NAND_ECC_HW_BCH_8_512
	bch_ctrl.wrd = 0;
	bch_ctrl.bf.bchEn = BCH_ENABLE;
	bch_ctrl.bf.bchErrCap = BCH_ERR_CAP_8_512;
	bch_ctrl.bf.bchOpcode = BCH_ENCODE;
	write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);
#elif  CONFIG_CS752X_NAND_ECC_HW_BCH_12_512
	bch_ctrl.wrd = 0;
	bch_ctrl.bf.bchEn = BCH_ENABLE;
	bch_ctrl.bf.bchErrCap = BCH_ERR_CAP_12_512;
	bch_ctrl.bf.bchOpcode = BCH_ENCODE;
	write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);
#else

	ecc_ctl.wrd = 0;
	if((eccsize-1) == NCNT_512P_DATA)
		ecc_ctl.bf.eccGenMode = ECC_GEN_512;
	else
		ecc_ctl.bf.eccGenMode = ECC_GEN_256;
	ecc_ctl.bf.eccEn = ECC_ENABLE;
	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, ecc_ctl.wrd);
#endif



	/*disable txq5*/
	dma_txq5_ctrl.bf.txq5_en = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_CONTROL, dma_txq5_ctrl.wrd);
	/*clr tx/rx eof*/
	dma_ssp_txq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT);
	dma_ssp_txq5_intsts.bf.txq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT, dma_ssp_txq5_intsts.wrd);
	dma_ssp_rxq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT);
	dma_ssp_rxq5_intsts.bf.rxq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT, dma_ssp_rxq5_intsts.wrd);


	nf_cnt.wrd = 0;
	nf_cmd.wrd = 0;
	nf_addr1.wrd = 0;
	nf_addr2.wrd = 0;
	nf_cnt.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = mtd->oobsize - 1;
	nf_cnt.bf.nflashRegDataCount = mtd->writesize-1;

	if(chip->chipsize < (32 << 20))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_3;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		nf_addr1.wrd = (((page & 0x00ffffff)<<8));
		nf_addr2.wrd = ((page & 0xff000000)>>24);

	}
	else if((chip->chipsize >= (32 << 20)) && (chip->chipsize <= (128 << 20)))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_4;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		nf_addr1.wrd = (((page & 0xffff)<<16) );
		nf_addr2.wrd = ((page & 0xffff0000)>>16);

	}
	else /*if((chip->chipsize > (128 << 20)) ))*/
	{

		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_5;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		nf_addr1.wrd = (((page & 0xffff)<<16) );
		nf_addr2.wrd = ((page & 0xffff0000)>>16);
	}

	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd);
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd);
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd);

	/*dma_map_single( NULL, (void *)buf, mtd->writesize, DMA_TO_DEVICE);*/

	nf_access.wrd = 0;
	nf_access.bf.nflashCeAlt = CHIP_EN;
	nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;
	/*write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

	write
	prepare dma descriptor
	chip->buffers->databuf
	nf_access.wrd = read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	*/
	nf_access.bf.nflashExtAddr = ((page << chip->page_shift) / 0x8000000);
	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

	addr =  (unsigned int *)((page << chip->page_shift) % 0x8000000);
	addr = (unsigned int *)((unsigned int)addr + (unsigned int)(CONFIG_SYS_FLASH_BASE) );

	/*page data tx desc*/
	dma_txq5_wptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR);
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.own = OWN_DMA;
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.buf_size = mtd->writesize;
	tx_desc[dma_txq5_wptr.bf.index].buf_adr = (unsigned int)(buf);


	/*page data rx desc
	printf("cs75xx_nand_write_page_hwecc : addr : %p  buf: %p",addr, buf);
	*/

	dma_rxq5_rptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR);
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.own = OWN_DMA;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.buf_size = mtd->writesize;
	rx_desc[dma_rxq5_rptr.bf.index].buf_adr = (unsigned int)addr;

	/*oob rx desc*/
	addr = (unsigned int *)((unsigned int)addr + mtd->writesize);
	/*printf("  oob : addr(%p)  chip->oob_poi(%p) \n",addr, chip->oob_poi);*/

	dma_rxq5_rptr.bf.index = (dma_rxq5_rptr.bf.index + 1) % FDMA_DESC_NUM;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.own = OWN_DMA;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.buf_size = mtd->oobsize;
	rx_desc[dma_rxq5_rptr.bf.index].buf_adr = (unsigned int)addr;


	/*update page tx write ptr*/
	dma_txq5_wptr.bf.index = (dma_txq5_wptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR, dma_txq5_wptr.wrd);
	/*set axi_bus_len = 8
	set fifo control*/
	fifo_ctl.wrd = 0;
	fifo_ctl.bf.fifoCmd = FLASH_WT;
	write_flash_ctrl_reg(FLASH_NF_FIFO_CONTROL, fifo_ctl.wrd);

	flash_start.wrd = 0;
	flash_start.bf.fifoReq = FLASH_GO;
	/*flash_start.bf.nflashRegCmd = FLASH_WT;*/
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

	/*enable txq5*/
	dma_txq5_ctrl.bf.txq5_en = 1;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_CONTROL, dma_txq5_ctrl.wrd);

	dma_ssp_rxq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT);
	tmp_dma_ssp_rxq5_intsts.wrd = 0;
	tmp_dma_ssp_rxq5_intsts.bf.rxq5_eof = 1;


	reg_wait(DMA_DMA_SSP_RXQ5_INTERRUPT, tmp_dma_ssp_rxq5_intsts.wrd , tmp_dma_ssp_rxq5_intsts.wrd, 1000);


	dma_ssp_txq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT);
	tmp_dma_ssp_txq5_intsts.wrd = 0;
	tmp_dma_ssp_txq5_intsts.bf.txq5_eof = 1;

	reg_wait(DMA_DMA_SSP_TXQ5_INTERRUPT, tmp_dma_ssp_txq5_intsts.wrd , tmp_dma_ssp_txq5_intsts.wrd, 1000);

	/*clr tx/rx eof*/

	dma_ssp_txq5_intsts.bf.txq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT, dma_ssp_txq5_intsts.wrd);

	dma_ssp_rxq5_intsts.bf.rxq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT, dma_ssp_rxq5_intsts.wrd);



#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
	bch_sts.wrd=read_flash_ctrl_reg(FLASH_NF_BCH_STATUS);
	tmp_bsc_sts.wrd = 0;
	tmp_bsc_sts.bf.bchGenDone = 1;

	reg_wait(FLASH_NF_BCH_STATUS, tmp_bsc_sts.wrd , tmp_bsc_sts.wrd, 1000);

	bch_ctrl.wrd = read_flash_ctrl_reg(FLASH_NF_BCH_CONTROL);  /*disable ecc gen*/
	bch_ctrl.bf.bchEn = BCH_DISABLE;
	write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);

#else



	ecc_sts.wrd=read_flash_ctrl_reg(FLASH_NF_ECC_STATUS);
	tmp_ecc_sts.wrd = 0;
	tmp_ecc_sts.bf.eccDone = 1;

	reg_wait(FLASH_NF_ECC_STATUS, tmp_ecc_sts.wrd , tmp_ecc_sts.wrd, 1000);


	ecc_ctl.wrd = read_flash_ctrl_reg( FLASH_NF_ECC_CONTROL);
	ecc_ctl.bf.eccEn= 0;
	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, ecc_ctl.wrd);  /*disable ecc gen*/

#endif

	/*printf("write page ecc(page %x) : ", page);*/

	for (i = 0,j = 0; eccsteps; eccsteps--, i++, j += eccbytes)
	{
#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)

		bch_gen00.wrd = read_flash_ctrl_reg(FLASH_NF_BCH_GEN0_0 + 20*i);
		chip->oob_poi[eccpos[j]] = bch_gen00.wrd & 0xff;
		chip->oob_poi[eccpos[j+1]] = (bch_gen00.wrd >> 8) & 0xff;
		chip->oob_poi[eccpos[j+2]] = (bch_gen00.wrd >> 16) & 0xff;
		chip->oob_poi[eccpos[j+3]] = (bch_gen00.wrd >> 24) & 0xff;
		bch_gen01.wrd = read_flash_ctrl_reg(FLASH_NF_BCH_GEN0_1 + 20*i);
		chip->oob_poi[eccpos[j+4]] = bch_gen01.wrd & 0xff;
		chip->oob_poi[eccpos[j+5]] = (bch_gen01.wrd >> 8) & 0xff;
		chip->oob_poi[eccpos[j+6]] = (bch_gen01.wrd >> 16) & 0xff;
		chip->oob_poi[eccpos[j+7]] = (bch_gen01.wrd >> 24) & 0xff;
		bch_gen02.wrd = read_flash_ctrl_reg(FLASH_NF_BCH_GEN0_2 + 20*i);
		chip->oob_poi[eccpos[j+8]] = bch_gen02.wrd & 0xff;
		chip->oob_poi[eccpos[j+9]] = (bch_gen02.wrd >> 8) & 0xff;
		chip->oob_poi[eccpos[j+10]] = (bch_gen02.wrd >> 16) & 0xff;
		chip->oob_poi[eccpos[j+11]] = (bch_gen02.wrd >> 24) & 0xff;
		bch_gen03.wrd = read_flash_ctrl_reg(FLASH_NF_BCH_GEN0_3 + 20*i);
		chip->oob_poi[eccpos[j+12]] = bch_gen03.wrd & 0xff;
  #if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
		chip->oob_poi[eccpos[j+13]] = (bch_gen03.wrd >> 8 ) & 0xff;
		chip->oob_poi[eccpos[j+14]] = (bch_gen03.wrd >> 16) & 0xff;
		chip->oob_poi[eccpos[j+15]] = (bch_gen03.wrd >> 24)& 0xff;
		bch_gen04.wrd = read_flash_ctrl_reg( FLASH_NF_BCH_GEN0_4 + 20*i);
		chip->oob_poi[eccpos[j+16]] = bch_gen04.wrd & 0xff;
		chip->oob_poi[eccpos[j+17]] = (bch_gen04.wrd >> 8 ) & 0xff;
		chip->oob_poi[eccpos[j+18]] = (bch_gen04.wrd >> 16) & 0xff;
		chip->oob_poi[eccpos[j+19]] = (bch_gen04.wrd >> 24)& 0xff;

  #endif
#else



		ecc_gen0.wrd = read_flash_ctrl_reg(FLASH_NF_ECC_GEN0 + 4*i);
		chip->oob_poi[eccpos[j]] = ecc_gen0.wrd & 0xff;
		chip->oob_poi[eccpos[j+1]] = (ecc_gen0.wrd >> 8) & 0xff;
		chip->oob_poi[eccpos[j+2]] = (ecc_gen0.wrd >> 16) & 0xff;
		/* printf("%x ", ecc_gen0.wrd);		*/
#endif
	}
	/*printf("\n");*/



#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
	/* jenfeng clear erase tag*/
	*(chip->oob_poi+ chip->ecc.layout->oobfree[0].offset +  chip->ecc.layout->oobfree[0].length)= 0;
#endif

	/*dma_map_single( NULL, (void *)chip->oob_poi, mtd->oobsize, DMA_TO_DEVICE);

	dma_txq5_wptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR);
	*/
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.own = OWN_DMA;
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.buf_size = mtd->oobsize;
	tx_desc[dma_txq5_wptr.bf.index].buf_adr = (unsigned int)(chip->oob_poi);

	/*dma_cache_sync(NULL, chip->oob_poi, mtd->oobsize, DMA_BIDIRECTIONAL);
	update tx write ptr
	*/
	dma_txq5_wptr.bf.index = (dma_txq5_wptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR, dma_txq5_wptr.wrd);

	dma_ssp_rxq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT);
	tmp_dma_ssp_rxq5_intsts.wrd = 0;
	tmp_dma_ssp_rxq5_intsts.bf.rxq5_eof = 1;


	reg_wait(DMA_DMA_SSP_RXQ5_INTERRUPT, tmp_dma_ssp_rxq5_intsts.wrd , tmp_dma_ssp_rxq5_intsts.wrd, 1000);

	dma_ssp_txq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT);
	tmp_dma_ssp_txq5_intsts.wrd = 0;
	tmp_dma_ssp_txq5_intsts.bf.txq5_eof = 1;

	reg_wait(DMA_DMA_SSP_TXQ5_INTERRUPT, tmp_dma_ssp_txq5_intsts.wrd , tmp_dma_ssp_txq5_intsts.wrd, 1000);

	flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);

	tmp_access.wrd = 0;
	tmp_access.bf.fifoReq = 1;
	reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);

	/*update rx read ptr
	dma_rxq5_rptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR);
	*/
	dma_rxq5_rptr.bf.index = (dma_rxq5_rptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR, dma_rxq5_rptr.wrd);



	/*chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);*/


}


/**
 * cs75xx_nand_read_page_hwecc - [REPLACABLE] hardware ecc based page read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 * @page:	page number to read
 *
 * Not for syndrome calculating ecc controllers which need a special oob layout
 */
static int cs75xx_nand_read_page_hwecc(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int page)
{
	int i, eccsize = chip->ecc.size, col;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	/*uint8_t *ecc_calc = chip->buffers->ecccalc;*/
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos, *addr;;
	struct cs75xx_nand_host *host = nand_chip->priv;
	DMA_DMA_SSP_RXQ5_INTERRUPT_t	tmp_dma_ssp_rxq5_intsts;
	DMA_DMA_SSP_TXQ5_INTERRUPT_t	tmp_dma_ssp_txq5_intsts;
	FLASH_FLASH_ACCESS_START_t tmp_access;
#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
	FLASH_NF_BCH_STATUS_t	tmp_bsc_sts;
#else
	FLASH_NF_ECC_STATUS_t	tmp_ecc_sts;
#endif

	check_flash_ctrl_status();

  	col  = host->col_addr;
	p = buf;
	/*printf("%s : page(%x:%x) col(%x)  ---->>\n", __func__, page, (page*mtd->writesize), col);*/
	ecc_reset.wrd = 0;
	ecc_reset.bf.eccClear = ECC_CLR;
	ecc_reset.bf.fifoClear = FIFO_CLR;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);

	flash_int_sts.bf.regIrq = 1;
	write_flash_ctrl_reg(FLASH_FLASH_INTERRUPT, flash_int_sts.wrd);

	ecc_reset.wrd = 0;
	ecc_reset.bf.eccClear = 1;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);

#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512)
	bch_ctrl.wrd = 0;
	bch_ctrl.bf.bchEn = BCH_ENABLE;
	bch_ctrl.bf.bchOpcode = BCH_DECODE;
	bch_ctrl.bf.bchErrCap = BCH_ERR_CAP_8_512;
	write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);
#elif defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512 )
	bch_ctrl.wrd = 0;
	bch_ctrl.bf.bchEn = BCH_ENABLE;
	bch_ctrl.bf.bchOpcode = BCH_DECODE;
	bch_ctrl.bf.bchErrCap = BCH_ERR_CAP_12_512;
	write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);
#else

	ecc_ctl.wrd = 0;
	if((eccsize-1) == NCNT_512P_DATA)
		ecc_ctl.bf.eccGenMode = ECC_GEN_512;
	else
		ecc_ctl.bf.eccGenMode = ECC_GEN_256;
	ecc_ctl.bf.eccEn = ECC_ENABLE;
	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, ecc_ctl.wrd);
#endif



	/*disable txq5*/
	dma_txq5_ctrl.bf.txq5_en = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_CONTROL, dma_txq5_ctrl.wrd);

	dma_ssp_txq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT);
	dma_ssp_txq5_intsts.bf.txq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT, dma_ssp_txq5_intsts.wrd);
	dma_ssp_rxq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT);
	dma_ssp_rxq5_intsts.bf.rxq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT, dma_ssp_rxq5_intsts.wrd);

	/*for indirect access with DMA, because DMA not ready	*/
	nf_cnt.wrd = 0;
	nf_cmd.wrd = 0;
	nf_addr1.wrd = 0;
	nf_addr2.wrd = 0;
	nf_cnt.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = mtd->oobsize - 1;
	nf_cnt.bf.nflashRegDataCount = mtd->writesize-1;

	if(chip->chipsize < (32 << 20))
	{
	nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_3;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;
		nf_addr1.wrd = (((page & 0x00ffffff)<<8));
		nf_addr2.wrd = ((page & 0xff000000)>>24);
	}
	else if((chip->chipsize >= (32 << 20)) && (chip->chipsize <= (128 << 20)))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_4;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;

		if(mtd->writesize > NCNT_512P_DATA) {
			nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
			nf_cmd.bf.nflashRegCmd1 = NAND_CMD_READSTART;
		}
		nf_addr1.wrd = (((page & 0xffff)<<16));
		nf_addr2.wrd = ((page & 0xffff0000)>>16);
	}
	else /*if((chip->chipsize > (128 << 20)) ))*/
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_5;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_READSTART;
		nf_addr1.wrd = (((page & 0xffff)<<16));
		nf_addr2.wrd = ((page & 0xffff0000)>>16);
	}


	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd); /*write read id command*/
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd); /*write address 0x0*/
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd);


	nf_access.wrd = 0;
	nf_access.bf.nflashCeAlt = CHIP_EN;
	nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;
	/*	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);
	prepare dma descriptor
	nf_access.wrd = read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	*/
	nf_access.bf.nflashExtAddr = ((page << chip->page_shift) / 0x8000000);
	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

	addr =  (unsigned int *)((page << chip->page_shift) % 0x8000000);
	addr = (unsigned int *)((unsigned int)addr + (unsigned int)(CONFIG_SYS_FLASH_BASE) );

	/*printf("%s : addr : %p  buf: %p",__func__, addr, buf);
	chip->buffers->databuf
	page tx data desc
	*/
	dma_txq5_wptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR);

	tx_desc[dma_txq5_wptr.bf.index].word0.bf.own = OWN_DMA;
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.buf_size = mtd->writesize;
	tx_desc[dma_txq5_wptr.bf.index].buf_adr = (unsigned int)addr;

	/*oob tx desc*/
	dma_txq5_wptr.bf.index = (dma_txq5_wptr.bf.index + 1) % FDMA_DESC_NUM;

	addr = (unsigned int *)((unsigned int)addr + mtd->writesize);
	/*printf("   oob : addr (%p)  chip->oob_poi (%p) \n", addr, chip->oob_poi);*/
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.own = OWN_DMA;
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.buf_size = mtd->oobsize;
	tx_desc[dma_txq5_wptr.bf.index].buf_adr = (unsigned int)addr;

	/*page data rx desc*/
	dma_rxq5_rptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR);
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.own = OWN_DMA;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.buf_size = mtd->writesize;

	rx_desc[dma_rxq5_rptr.bf.index].buf_adr = (unsigned int)(buf);

	/*oob rx desc*/
	dma_rxq5_rptr.bf.index = (dma_rxq5_rptr.bf.index + 1) % FDMA_DESC_NUM;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.own = OWN_DMA;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.buf_size = mtd->oobsize;
	rx_desc[dma_rxq5_rptr.bf.index].buf_adr =  (unsigned int)(chip->oob_poi);


	/*dma_map_single( NULL, (void *)chip->oob_poi, mtd->oobsize, DMA_FROM_DEVICE);
	dma_map_single( NULL, (void *)buf, mtd->writesize, DMA_FROM_DEVICE);


	set axi_bus_len = 8

	set fifo control
	*/
	fifo_ctl.wrd = 0;
	fifo_ctl.bf.fifoCmd = FLASH_RD;
	write_flash_ctrl_reg(FLASH_NF_FIFO_CONTROL, fifo_ctl.wrd);

	flash_start.wrd = 0;
	flash_start.bf.fifoReq = FLASH_GO;
	/*flash_start.bf.nflashRegCmd = FLASH_RD;*/
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

	/*update tx write ptr*/
	dma_txq5_wptr.bf.index = (dma_txq5_wptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR, dma_txq5_wptr.wrd);
	/*dma_txq5_wptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR);

	enable txq5
	*/
	dma_txq5_ctrl.bf.txq5_en = 1;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_CONTROL, dma_txq5_ctrl.wrd);

	tmp_dma_ssp_rxq5_intsts.wrd = 0;
	tmp_dma_ssp_rxq5_intsts.bf.rxq5_eof = 1;


	reg_wait(DMA_DMA_SSP_RXQ5_INTERRUPT, tmp_dma_ssp_rxq5_intsts.wrd , tmp_dma_ssp_rxq5_intsts.wrd, 1000);

	tmp_dma_ssp_txq5_intsts.wrd = 0;
	tmp_dma_ssp_txq5_intsts.bf.txq5_eof = 1;

	reg_wait(DMA_DMA_SSP_TXQ5_INTERRUPT, tmp_dma_ssp_txq5_intsts.wrd , tmp_dma_ssp_txq5_intsts.wrd, 1000);

	tmp_access.wrd = 0;
	tmp_access.bf.fifoReq = 1;
	reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);


	/**
	  ecc_sts.wrd=read_flash_ctrl_reg(FLASH_NF_ECC_STATUS);
	  while(!ecc_sts.bf.eccDone)
	  {
	  ecc_sts.wrd=read_flash_ctrl_reg(FLASH_NF_ECC_STATUS);
	  udelay(1);

	  }

	  write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0);




	dma_cache_sync(NULL, buf, mtd->writesize, DMA_BIDIRECTIONAL);
	dma_cache_sync(NULL, chip->oob_poi, mtd->oobsize, DMA_BIDIRECTIONAL);
	update rx read ptr
	**/
	dma_rxq5_rptr.bf.index = (dma_rxq5_rptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR, dma_rxq5_rptr.wrd);


#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
	bch_sts.wrd=read_flash_ctrl_reg(FLASH_NF_BCH_STATUS);

	tmp_bsc_sts.wrd = 0;
	tmp_bsc_sts.bf.bchGenDone = 1;

	reg_wait(FLASH_NF_BCH_STATUS, tmp_bsc_sts.wrd , tmp_bsc_sts.wrd, 1000);

	/**write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, 0);  **/

#else
	ecc_sts.wrd=read_flash_ctrl_reg(FLASH_NF_ECC_STATUS);


	tmp_ecc_sts.wrd = 0;
	tmp_ecc_sts.bf.eccDone = 1;

	reg_wait(FLASH_NF_ECC_STATUS, tmp_ecc_sts.wrd , tmp_ecc_sts.wrd, 1000);


	ecc_ctl.wrd = read_flash_ctrl_reg(FLASH_NF_ECC_CONTROL );  /**disable ecc gen**/
	ecc_ctl.bf.eccEn = 0;
	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, ecc_ctl.wrd);  /**disable ecc gen**/
#endif
	/**dma_cache_maint((void *)chip->oob_poi, mtd->oobsize, DMA_FROM_DEVICE);
	dma_cache_maint((void *)buf, mtd->writesize, DMA_FROM_DEVICE);
	**/


#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
	/** jenfeng**/
	if( 0xff ==  *(chip->oob_poi+ chip->ecc.layout->oobfree[0].offset +  chip->ecc.layout->oobfree[0].length)){
		/** Erase tga is on , No needs to check. **/
		goto BCH_EXIT;
	}

#endif
	for (i = 0; i < chip->ecc.total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	/** 	printf("ecc_code(%x) :\n",chip->ecc.total);
		for (i = 0; i < chip->ecc.total; i++)
		  printf(" %x", ecc_code[i]);
	**/
	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {

#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
		int j ; /** (i/eccbytes);**/

		bch_oob0.wrd = ecc_code[i]   | ecc_code[i+1]<<8 | ecc_code[i+2]<<16  | ecc_code[i+3]<<24;
		bch_oob1.wrd = ecc_code[i+4] | ecc_code[i+5]<<8 | ecc_code[i+6]<<16  | ecc_code[i+7]<<24;
		bch_oob2.wrd = ecc_code[i+8] | ecc_code[i+9]<<8 | ecc_code[i+10]<<16 | ecc_code[i+11]<<24;
#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512)
		bch_oob3.wrd = ecc_code[i+12];

		write_flash_ctrl_reg(FLASH_NF_BCH_OOB0, bch_oob0.wrd);
		write_flash_ctrl_reg(FLASH_NF_BCH_OOB1, bch_oob1.wrd);
		write_flash_ctrl_reg(FLASH_NF_BCH_OOB2, bch_oob2.wrd);
		write_flash_ctrl_reg(FLASH_NF_BCH_OOB3, bch_oob3.wrd);
#else
		bch_oob3.wrd = ecc_code[i+12] | ecc_code[i+13]<<8 | ecc_code[i+14]<<16 | ecc_code[i+15]<<24;
		bch_oob4.wrd = ecc_code[i+16] | ecc_code[i+17]<<8 | ecc_code[i+18]<<16 | ecc_code[i+19]<<24;

		write_flash_ctrl_reg(FLASH_NF_BCH_OOB0, bch_oob0.wrd);
		write_flash_ctrl_reg(FLASH_NF_BCH_OOB1, bch_oob1.wrd);
		write_flash_ctrl_reg(FLASH_NF_BCH_OOB2, bch_oob2.wrd);
		write_flash_ctrl_reg(FLASH_NF_BCH_OOB3, bch_oob3.wrd);
		write_flash_ctrl_reg(FLASH_NF_BCH_OOB4, bch_oob4.wrd);
#endif


		/** enable ecc compare**/
		bch_ctrl.wrd = read_flash_ctrl_reg(FLASH_NF_BCH_CONTROL);
		bch_ctrl.bf.bchCodeSel = (i/eccbytes);
		bch_ctrl.bf.bchCompare = 1;
		write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);

		tmp_bsc_sts.wrd = 0;
		tmp_bsc_sts.bf.bchDecDone = 1;

		reg_wait(FLASH_NF_BCH_STATUS, tmp_bsc_sts.wrd , tmp_bsc_sts.wrd, 1000);

		bch_sts.wrd=read_flash_ctrl_reg(FLASH_NF_BCH_STATUS);

		switch(bch_sts.bf.bchDecStatus)
		{
			case BCH_CORRECTABLE_ERR:
				//printf("correctable error(%x)!!\n",bch_sts.bf.bchErrNum);
				for(j=0  ;j<((bch_sts.bf.bchErrNum+1)/2);j++)
				{
					bch_err_loc01.wrd = read_flash_ctrl_reg(FLASH_NF_BCH_ERROR_LOC01 + j*4);

					if ( (j+1)*2 <= bch_sts.bf.bchErrNum ) {
						if( ((bch_err_loc01.bf.bchErrLoc1 & 0x1fff)>>3) < 0x200) {
							/** printf("pdata[%x] %x , ",((i/eccbytes)*chip->ecc.size + ((bch_err_loc01.bf.bchErrLoc1&0x1fff)>>3)), p[(bch_err_loc01.bf.bchErrLoc1&0x1fff)>>3]);  **/
							p[(bch_err_loc01.bf.bchErrLoc1&0x1fff)>>3] ^= (1 << (bch_err_loc01.bf.bchErrLoc1 & 0x07));
							/** printf("pdata_new %x  \n",p[(bch_err_loc01.bf.bchErrLoc1&0x1fff)>>3]);*/
						} else {
							/** printf( "BCH bit error [%x]:[%x]\n", (bch_err_loc01.bf.bchErrLoc1&0x1fff)>>3 - 0x200, bch_err_loc01.bf.bchErrLoc1 & 0x07); **/

						}
					}

					if(((bch_err_loc01.bf.bchErrLoc0 & 0x1fff)>>3) < 0x200)
					{
						/** printf("j: %x  ,pdata[%x] %x , ",j ,((i/eccbytes)*chip->ecc.size + ((bch_err_loc01.bf.bchErrLoc0&0x1fff)>>3)), p[ (bch_err_loc01.bf.bchErrLoc0&0x1fff)>>3]); **/
						p[(bch_err_loc01.bf.bchErrLoc0&0x1fff)>>3] ^= (1 << (bch_err_loc01.bf.bchErrLoc0 & 0x07));
						/** printf("pdata_new %x  \n",p[(bch_err_loc01.bf.bchErrLoc0&0x1fff)>>3 ]); **/
					} else
					{
						/** printf( "BCH bit error [%x]:[%x]\n", (bch_err_loc01.bf.bchErrLoc0&0x1fff)>>3 - 0x200, bch_err_loc01.bf.bchErrLoc0 & 0x07); **/
					}
				}
				break;
			case BCH_UNCORRECTABLE:
				printf("uncorrectable error!!\n");
				mtd->ecc_stats.failed++;

				break;
		}
		/** clear compare sts
		bch_sts.bf.bchDecDone = 0;
		write_flash_ctrl_reg(FLASH_NF_BCH_STATUS, bch_sts.wrd);
		**/
		if ( cs75xx_ecc_check && i == 0 ) {
			unsigned char *free= buf+ mtd->writesize+ mtd->oobsize;
			*free= bch_sts.bf.bchDecStatus;
			*(free+1)= bch_sts.bf.bchErrNum;
		}

		/** disable ecc compare**/
		bch_ctrl.wrd = read_flash_ctrl_reg(FLASH_NF_BCH_CONTROL);
		bch_ctrl.bf.bchCompare = 0;
		write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);

#else

		/** for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) { ***/
		ecc_oob.wrd =	ecc_code[i] | ecc_code[i+1]<<8 | ecc_code[i+2]<<16;
		write_flash_ctrl_reg(FLASH_NF_ECC_OOB, ecc_oob.wrd);

		ecc_ctl.wrd = read_flash_ctrl_reg(FLASH_NF_ECC_CONTROL);
		ecc_ctl.bf.eccCodeSel = (i/eccbytes);
		write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, ecc_ctl.wrd);

		ecc_sts.wrd = read_flash_ctrl_reg(FLASH_NF_ECC_STATUS);

		switch(ecc_sts.bf.eccStatus)
		{
			case ECC_NO_ERR:
				/** printf("ECC no error!!\n");**/
				break;
			case ECC_1BIT_DATA_ERR:
				/* flip the bit */
				p[ecc_sts.bf.eccErrByte] ^= (1 << ecc_sts.bf.eccErrBit);
				/** printf("ecc_code- %x (%x) :\n",i ,chip->ecc.total);**/
				ecc_gen0.wrd = read_flash_ctrl_reg(FLASH_NF_ECC_GEN0 + (4*(i/eccbytes)));
				/** for (i = 0; i < chip->ecc.total; i++)
				  printf(" %x", ecc_code[i]);
				**/
				printf("\nECC one bit data error(%x)!!(org: %x) HW(%xs) page(%x)\n", (i/eccbytes),ecc_oob.wrd, ecc_gen0.wrd, page);
				break;
			case ECC_1BIT_ECC_ERR:
				/** printf("ecc_code- %x (%x) :\n",i ,chip->ecc.total);**/
				ecc_gen0.wrd = read_flash_ctrl_reg(FLASH_NF_ECC_GEN0 + (4*(i/eccbytes)));
				/** for (i = 0; i < chip->ecc.total; i++)
				  printf(" %x", ecc_code[i]);
				**/
				printf("\nECC one bit ECC error(%x)!!(org: %x) HW(%xs) page(%x)\n", (i/eccbytes), ecc_oob.wrd, ecc_gen0.wrd, page);
				break;
			case ECC_UNCORRECTABLE:
				mtd->ecc_stats.failed++;
				ecc_gen0.wrd = read_flash_ctrl_reg(FLASH_NF_ECC_GEN0 + (4*(i/eccbytes)));
				/** printf("ecc_code- %x (%x) :\n",i ,chip->ecc.total);
				for (i = 0; i < chip->ecc.total; i++)
				  printf(" %x", ecc_code[i]);
				**/
				printf("\nECC uncorrectable error(%x)!!(org: %x) HW(%xs) page(%x)\n", (i/eccbytes), ecc_oob.wrd, ecc_gen0.wrd, page);
				break;
		}

		if ( cs75xx_ecc_check && i == 0 ) {
			unsigned char *free= buf+ mtd->writesize+ mtd->oobsize;
			*free=  ecc_sts.bf.eccStatus;
		}

#endif
	} /** for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) **/

	#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
BCH_EXIT:
	/** diasble bch **/
	bch_ctrl.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);
	#endif

#if 0
	dma_txq5_wptr.wrd = read_dma_ctrl_reg( DMA_DMA_SSP_TXQ5_WPTR );
	dma_rxq5_wptr.wrd = read_dma_ctrl_reg( DMA_DMA_SSP_RXQ5_WPTR );
	dma_ssp_rxq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT);
	if( dma_ssp_rxq5_intsts.bf.rxq5_full || ( dma_txq5_wptr.wrd % 2)  || ( dma_rxq5_wptr.wrd %2 ) ) {
		printf( "========= %s full[%d] [%d][%d] =============\n", __func__, dma_ssp_rxq5_intsts.bf.rxq5_full, dma_txq5_wptr.wrd, dma_rxq5_wptr.wrd );
		dma_ssp_rxq5_intsts.bf.rxq5_full= 3;
	}
#endif


	/** printf("%s : page(%x:%x) col(%x)  <<----\n", __func__, page, (page*mtd->writesize), col);**/

	return 0;
}


/**
 * cs75xx_nand_write_page_raw - [Intern] raw page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 *
 * Not for syndrome calculating ecc controllers, which use a special oob layout
 */
static void cs75xx_nand_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
				const uint8_t *buf)
{
	int page, eccsize = chip->ecc.size;
	int i; /** , col;**/
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = (uint8_t *)buf;
	/** uint8_t *ecc_calc = chip->buffers->ecccalc;**/
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos, *addr;
	struct cs75xx_nand_host *host = nand_chip->priv;
	DMA_DMA_SSP_RXQ5_INTERRUPT_t	tmp_dma_ssp_rxq5_intsts;
	DMA_DMA_SSP_TXQ5_INTERRUPT_t	tmp_dma_ssp_txq5_intsts;
	FLASH_FLASH_ACCESS_START_t tmp_access;

	check_flash_ctrl_status();

	/** chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
	**/
	page = host->page_addr;


	ecc_reset.wrd = 3;
	ecc_reset.bf.eccClear = ECC_CLR;
	ecc_reset.bf.fifoClear = FIFO_CLR;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);

	flash_int_sts.bf.regIrq = 1;
	write_flash_ctrl_reg(FLASH_FLASH_INTERRUPT, flash_int_sts.wrd);

	ecc_reset.wrd = 0;
	ecc_reset.bf.eccClear = 1;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);

	/**  Disable ECC function**/
	bch_ctrl.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);
	ecc_ctl.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, ecc_ctl.wrd);


	/** disable txq5 **/
	dma_txq5_ctrl.bf.txq5_en = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_CONTROL, dma_txq5_ctrl.wrd);
	/** clr tx/rx eof **/
	dma_ssp_txq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT);
/** 	dma_ssp_txq5_intsts.bf.txq5_eof = 0;**/
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT, dma_ssp_txq5_intsts.wrd);
	dma_ssp_rxq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT);
/** 	dma_ssp_rxq5_intsts.bf.rxq5_eof = 0; **/
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT, dma_ssp_rxq5_intsts.wrd);


	nf_cnt.wrd = 0;
	nf_cmd.wrd = 0;
	nf_addr1.wrd = 0;
	nf_addr2.wrd = 0;
	nf_cnt.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = mtd->oobsize - 1;
	nf_cnt.bf.nflashRegDataCount = mtd->writesize-1;

	if(chip->chipsize < (32 << 20))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_3;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		nf_addr1.wrd = (((page & 0x00ffffff)<<8));
		nf_addr2.wrd = ((page & 0xff000000)>>24);

	}
	else if((chip->chipsize >= (32 << 20)) && (chip->chipsize <= (128 << 20)))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_4;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		nf_addr1.wrd = (((page & 0xffff)<<16) );
		nf_addr2.wrd = ((page & 0xffff0000)>>16);

	}
	else /** if((chip->chipsize > (128 << 20)) )) **/
	{

		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_5;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_SEQIN;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_PAGEPROG;
		nf_addr1.wrd = (((page & 0xffff)<<16) );
		nf_addr2.wrd = ((page & 0xffff0000)>>16);
	}

	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd);
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd);
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd);

	/** dma_map_single( NULL, (void *)buf, mtd->writesize, DMA_TO_DEVICE);**/

	nf_access.wrd = 0;
	nf_access.bf.nflashCeAlt = CHIP_EN;
	nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;
	/** write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

	write
	prepare dma descriptor
	chip->buffers->databuf
	nf_access.wrd = read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	**/
	nf_access.bf.nflashExtAddr = ((page << chip->page_shift) / 0x8000000);
	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

	addr =  (unsigned int *)((page << chip->page_shift) % 0x8000000);
	addr = (unsigned int *)((unsigned int)addr + (unsigned int)(CONFIG_SYS_FLASH_BASE) );

	/** page data tx desc **/
	dma_txq5_wptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR);
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.own = OWN_DMA;
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.buf_size = mtd->writesize;
	tx_desc[dma_txq5_wptr.bf.index].buf_adr = (unsigned int)(buf);


	/** page data rx desc
	printf("cs75xx_nand_write_page_hwecc : addr : %p  buf: %p",addr, buf);
	**/

	dma_rxq5_rptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR);
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.own = OWN_DMA;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.buf_size = mtd->writesize;
	rx_desc[dma_rxq5_rptr.bf.index].buf_adr = (unsigned int)addr;

	/** oob rx desc**/
	addr = (unsigned int *)((unsigned int)addr + mtd->writesize);
	/** printf("  oob : addr(%p)  chip->oob_poi(%p) \n",addr, chip->oob_poi);**/

	dma_rxq5_rptr.bf.index = (dma_rxq5_rptr.bf.index + 1) % FDMA_DESC_NUM;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.own = OWN_DMA;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.buf_size = mtd->oobsize;
	rx_desc[dma_rxq5_rptr.bf.index].buf_adr = (unsigned int)addr;


	/** update page tx write ptr **/
	dma_txq5_wptr.bf.index = (dma_txq5_wptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR, dma_txq5_wptr.wrd);
	/** set axi_bus_len = 8
	set fifo control
	**/
	fifo_ctl.wrd = 0;
	fifo_ctl.bf.fifoCmd = FLASH_WT;
	write_flash_ctrl_reg(FLASH_NF_FIFO_CONTROL, fifo_ctl.wrd);

	flash_start.wrd = 0;
	flash_start.bf.fifoReq = FLASH_GO;
	/** flash_start.bf.nflashRegCmd = FLASH_WT; **/
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

	/** enable txq5**/
	dma_txq5_ctrl.bf.txq5_en = 1;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_CONTROL, dma_txq5_ctrl.wrd);

	tmp_dma_ssp_rxq5_intsts.wrd = 0;
	tmp_dma_ssp_rxq5_intsts.bf.rxq5_eof = 1;

	/** static UINT32 reg_wait(UINT32 regaddr, UINT32 mask, UINT32 val, int timeout) **/
	reg_wait(DMA_DMA_SSP_RXQ5_INTERRUPT, tmp_dma_ssp_rxq5_intsts.wrd , tmp_dma_ssp_rxq5_intsts.wrd, 1000);

	tmp_dma_ssp_txq5_intsts.wrd = 0;
	tmp_dma_ssp_txq5_intsts.bf.txq5_eof = 1;

	reg_wait(DMA_DMA_SSP_TXQ5_INTERRUPT, tmp_dma_ssp_txq5_intsts.wrd , tmp_dma_ssp_txq5_intsts.wrd, 1000);

	/** clr tx/rx eof **/

	dma_ssp_txq5_intsts.bf.txq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT, dma_ssp_txq5_intsts.wrd);

	dma_ssp_rxq5_intsts.bf.rxq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT, dma_ssp_rxq5_intsts.wrd);

	/**
	//prepare dma descriptor
	//chip->buffers->databuf

	//set axi_bus_len = 8
	//set fifo control

	  fifo_ctl.wrd = 0;
	  fifo_ctl.bf.fifoCmd = FLASH_WT;
	  write_flash_ctrl_reg(FLASH_NF_FIFO_CONTROL, fifo_ctl.wrd);
	//
	flash_start.wrd = 0;
	flash_start.bf.fifoReq = FLASH_GO;
	//flash_start.bf.nflashRegCmd = FLASH_WT;
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

	flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	while(flash_start.bf.fifoReq)
	{
	flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	udelay(1);

	}
	 **/



#if defined( CONFIG_CS752X_NAND_ECC_HW_BCH_8_512) || defined( CONFIG_CS752X_NAND_ECC_HW_BCH_12_512)
	/**  jenfeng clear erase tag**/
	*(chip->oob_poi+ chip->ecc.layout->oobfree[0].offset +  chip->ecc.layout->oobfree[0].length)= 0;
#endif

	/** dma_map_single( NULL, (void *)chip->oob_poi, mtd->oobsize,  DMA_TO_DEVICE);

	dma_txq5_wptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR);
	**/
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.own = OWN_DMA;
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.buf_size = mtd->oobsize;
	tx_desc[dma_txq5_wptr.bf.index].buf_adr = (unsigned int)(chip->oob_poi);

	/** dma_cache_sync(NULL, chip->oob_poi, mtd->oobsize, DMA_BIDIRECTIONAL);
	update tx write ptr
	**/
	dma_txq5_wptr.bf.index = (dma_txq5_wptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR, dma_txq5_wptr.wrd);

	tmp_dma_ssp_rxq5_intsts.wrd = 0;
	tmp_dma_ssp_rxq5_intsts.bf.rxq5_eof = 1;

	reg_wait(DMA_DMA_SSP_RXQ5_INTERRUPT, tmp_dma_ssp_rxq5_intsts.wrd , tmp_dma_ssp_rxq5_intsts.wrd, 1000);

	tmp_dma_ssp_txq5_intsts.wrd = 0;
	tmp_dma_ssp_txq5_intsts.bf.txq5_eof = 1;

	reg_wait(DMA_DMA_SSP_TXQ5_INTERRUPT, tmp_dma_ssp_txq5_intsts.wrd , tmp_dma_ssp_txq5_intsts.wrd, 1000);

	tmp_access.wrd = 0;
	tmp_access.bf.fifoReq = 1;
	reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);



	/** update rx read ptr
	dma_rxq5_rptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR);
	**/
	dma_rxq5_rptr.bf.index = (dma_rxq5_rptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR, dma_rxq5_rptr.wrd);



	/** chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
	**/


}


/**
 * cs75xx_nand_read_page_raw - [Intern] read raw page data without ecc
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 */
static int cs75xx_nand_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
			      uint8_t *buf, int page)
{
	unsigned int *addr;
	DMA_DMA_SSP_RXQ5_INTERRUPT_t	tmp_dma_ssp_rxq5_intsts;
	DMA_DMA_SSP_TXQ5_INTERRUPT_t	tmp_dma_ssp_txq5_intsts;
	FLASH_FLASH_ACCESS_START_t tmp_access;
	/** chip->read_buf(mtd, buf, mtd->writesize);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize); **/

	check_flash_ctrl_status();

	ecc_reset.wrd = 0;
	ecc_reset.bf.eccClear = ECC_CLR;
	ecc_reset.bf.fifoClear = FIFO_CLR;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);

	flash_int_sts.bf.regIrq = 1;
	write_flash_ctrl_reg(FLASH_FLASH_INTERRUPT, flash_int_sts.wrd);

	ecc_reset.wrd = 0;
	ecc_reset.bf.eccClear = 1;
	write_flash_ctrl_reg(FLASH_NF_ECC_RESET, ecc_reset.wrd);


	bch_ctrl.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_BCH_CONTROL, bch_ctrl.wrd);

	ecc_ctl.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, ecc_ctl.wrd);


	/** disable txq5**/
	dma_txq5_ctrl.bf.txq5_en = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_CONTROL, dma_txq5_ctrl.wrd);

	dma_ssp_txq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT);
	dma_ssp_txq5_intsts.bf.txq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_INTERRUPT, dma_ssp_txq5_intsts.wrd);
	dma_ssp_rxq5_intsts.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT);
	dma_ssp_rxq5_intsts.bf.rxq5_eof = 0;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_INTERRUPT, dma_ssp_rxq5_intsts.wrd);

	/** for indirect access with DMA, because DMA not ready	**/
	nf_cnt.wrd = 0;
	nf_cmd.wrd = 0;
	nf_addr1.wrd = 0;
	nf_addr2.wrd = 0;
	nf_cnt.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = mtd->oobsize -1 ;
	nf_cnt.bf.nflashRegDataCount = mtd->writesize-1;

	if(chip->chipsize < (32 << 20))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_3;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;
		nf_addr1.wrd = (((page & 0x00ffffff)<<8));
		nf_addr2.wrd = ((page & 0xff000000)>>24);
	}
	else if((chip->chipsize >= (32 << 20)) && (chip->chipsize <= (128 << 20)))
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_4;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;

		if(mtd->writesize > NCNT_512P_DATA) {
			nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
			nf_cmd.bf.nflashRegCmd1 = NAND_CMD_READSTART;
		}
		nf_addr1.wrd = (((page & 0xffff)<<16));
		nf_addr2.wrd = ((page & 0xffff0000)>>16);
	}
	else /** if((chip->chipsize > (128 << 20)) )) **/
	{
		nf_cnt.bf.nflashRegAddrCount = NCNT_ADDR_5;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_2;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_READ0;
		nf_cmd.bf.nflashRegCmd1 = NAND_CMD_READSTART;
		nf_addr1.wrd = (((page & 0xffff)<<16));
		nf_addr2.wrd = ((page & 0xffff0000)>>16);
	}


	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd); /** write read id command**/
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd); /** write address 0x0**/
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd);


	nf_access.wrd = 0;
	nf_access.bf.nflashCeAlt = CHIP_EN;
	nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;
	/** 	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);
	prepare dma descriptor
	nf_access.wrd = read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	**/
	nf_access.bf.nflashExtAddr = ((page << chip->page_shift) / 0x8000000);
	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);

	addr =  (unsigned int *)((page << chip->page_shift) % 0x8000000);
	addr = (unsigned int *)((unsigned int)addr + (unsigned int)(CONFIG_SYS_FLASH_BASE) );

	/** printf("%s : addr : %p  buf: %p",__func__, addr, buf);
	chip->buffers->databuf
	page tx data desc
	**/
	dma_txq5_wptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR);

	tx_desc[dma_txq5_wptr.bf.index].word0.bf.own = OWN_DMA;
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.buf_size = mtd->writesize;
	tx_desc[dma_txq5_wptr.bf.index].buf_adr = (unsigned int)addr;

	/** oob tx desc **/
	dma_txq5_wptr.bf.index = (dma_txq5_wptr.bf.index + 1) % FDMA_DESC_NUM;

	addr = (unsigned int *)((unsigned int)addr + mtd->writesize);
	/** printf("   oob : addr (%p)  chip->oob_poi (%p) \n", addr, chip->oob_poi); **/
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.own = OWN_DMA;
	tx_desc[dma_txq5_wptr.bf.index].word0.bf.buf_size = mtd->oobsize;
	tx_desc[dma_txq5_wptr.bf.index].buf_adr = (unsigned int)addr;

	/** page data rx desc **/
	dma_rxq5_rptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR);
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.own = OWN_DMA;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.buf_size = mtd->writesize;

	rx_desc[dma_rxq5_rptr.bf.index].buf_adr = (unsigned int)(buf);

	/** oob rx desc **/
	dma_rxq5_rptr.bf.index = (dma_rxq5_rptr.bf.index + 1) % FDMA_DESC_NUM;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.own = OWN_DMA;
	rx_desc[dma_rxq5_rptr.bf.index].word0.bf.buf_size = mtd->oobsize;
	rx_desc[dma_rxq5_rptr.bf.index].buf_adr =  (unsigned int)(chip->oob_poi);


	/** dma_map_single( NULL, (void *)chip->oob_poi, mtd->oobsize,  DMA_FROM_DEVICE);
	dma_map_single( NULL, (void *)buf, mtd->writesize, DMA_FROM_DEVICE);


	set axi_bus_len = 8

	set fifo control
	**/
	fifo_ctl.wrd = 0;
	fifo_ctl.bf.fifoCmd = FLASH_RD;
	write_flash_ctrl_reg(FLASH_NF_FIFO_CONTROL, fifo_ctl.wrd);

	flash_start.wrd = 0;
	flash_start.bf.fifoReq = FLASH_GO;
	/** flash_start.bf.nflashRegCmd = FLASH_RD;**/
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

	/** update tx write ptr **/
	dma_txq5_wptr.bf.index = (dma_txq5_wptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR, dma_txq5_wptr.wrd);
	/** dma_txq5_wptr.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_WPTR);

	enable txq5
	**/
	dma_txq5_ctrl.bf.txq5_en = 1;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_CONTROL, dma_txq5_ctrl.wrd);

	tmp_dma_ssp_rxq5_intsts.wrd = 0;
	tmp_dma_ssp_rxq5_intsts.bf.rxq5_eof = 1;

	/** static UINT32 reg_wait(UINT32 regaddr, UINT32 mask, UINT32 val, int timeout)**/
	reg_wait(DMA_DMA_SSP_RXQ5_INTERRUPT, tmp_dma_ssp_rxq5_intsts.wrd , tmp_dma_ssp_rxq5_intsts.wrd, 1000);

	tmp_dma_ssp_txq5_intsts.wrd = 0;
	tmp_dma_ssp_txq5_intsts.bf.txq5_eof = 1;

	reg_wait(DMA_DMA_SSP_TXQ5_INTERRUPT, tmp_dma_ssp_txq5_intsts.wrd , tmp_dma_ssp_txq5_intsts.wrd, 1000);

	tmp_access.wrd = 0;
	tmp_access.bf.fifoReq = 1;
	reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);



	/**
	  ecc_sts.wrd=read_flash_ctrl_reg(FLASH_NF_ECC_STATUS);
	  while(!ecc_sts.bf.eccDone)
	  {
	  ecc_sts.wrd=read_flash_ctrl_reg(FLASH_NF_ECC_STATUS);
	  udelay(1);

	  }

	  write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0);

	 **/


	/** update rx read ptr**/
	dma_rxq5_rptr.bf.index = (dma_rxq5_rptr.bf.index + 1) % FDMA_DESC_NUM;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_RPTR, dma_rxq5_rptr.wrd);


	return 0;
}

/**
 * cs75xx_nand_write_page - [REPLACEABLE] write one page
 * @mtd:	MTD device structure
 * @chip:	NAND chip descriptor
 * @buf:	the data to write
 * @page:	page number to write
 * @cached:	cached programming
 * @raw:	use _raw version of write_page
 */
static int cs75xx_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int page, int cached, int raw)
{
	int status;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	if (unlikely(raw))
		chip->ecc.write_page_raw(mtd, chip, buf);
	else
		chip->ecc.write_page(mtd, chip, buf);

	/*
	 * Cached progamming disabled for now, Not sure if its worth the
	 * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
	 */
	cached = 0;

	if (!cached || !(chip->options & NAND_CACHEPRG)) {

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_WRITING, status,
					       page);

		if (status & NAND_STATUS_FAIL)
			return -EIO;
	} else {
		chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
	}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	/* Send command to read back the data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	if (chip->verify_buf(mtd, buf, mtd->writesize))
		return -EIO;
#endif
	return 0;
}


/*
 * Get the flash and manufacturer id and lookup if the type is supported
 */
static struct nand_flash_dev *cs75xx_nand_get_flash_type(struct mtd_info *mtd,
						  struct nand_chip *chip,
						  int busw, int *maf_id)
{
	struct nand_flash_dev *type = NULL;
	int i, dev_id, maf_idx;
	unsigned char id[8];
	u16 oobsize_8kp[]= { 0 , 128, 218, 400, 436, 512, 640, 0};

	/* Select the device */
	chip->select_chip(mtd, 0);

	/*
	 * Reset the chip, required by some chips (e.g. Micron MT29FxGxxxxx)
	 * after power-up
	 */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	/* Send the command for reading device ID */
	chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	/* Read manufacturer and device IDs */
	memset( id, 0, sizeof( id));
	cs75xx_nand_read_id(0, &id[0]);
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	*maf_id  = id[0];
	dev_id = id[1];
	/* Try again to make sure, as some systems the bus-hold or other
	 * interface concerns can cause random data which looks like a
	 * possibly credible NAND flash to appear. If the two results do
	 * not match, ignore the device completely.
	 */

	chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	/* Read manufacturer and device IDs */

	/**
	tmp_manf = chip->read_byte(mtd);
	tmp_id = chip->read_byte(mtd);

	if (tmp_manf != *maf_id || tmp_id != dev_id) {
		printf(KERN_INFO "%s: second ID read did not match "
		       "%02x,%02x against %02x,%02x\n", __func__,
		       *maf_id, dev_id, tmp_manf, tmp_id);
		return ERR_PTR(-ENODEV);
	}
	**/
	/* Lookup the flash id */
	for (i = 0; nand_flash_ids[i].name != NULL; i++) {
		if (dev_id == nand_flash_ids[i].id) {
			type =  &nand_flash_ids[i];
			break;
		}
	}

	if (!type)
		return ERR_PTR(-ENODEV);

	if (!mtd->name)
		mtd->name = type->name;

	chip->chipsize = type->chipsize << 20;

	/* Newer devices have all the information in additional id bytes */
	if (!type->pagesize) {
		int extid;
		/*  The 3rd id byte holds MLC / multichip data  */
		chip->cellinfo = id[2];//chip->read_byte(mtd);
		/*  The 4th id byte is the important one  */
		extid = id[3];
		if( id[0] == id[6] && id[1] == id[7] &&
			id[0] == NAND_MFR_SAMSUNG  &&
                                (chip->cellinfo & NAND_CI_CELLTYPE_MSK) &&
                                id[5] != 0x00)  {
			mtd->writesize = 2048 * ( 1 << (extid & 0x3) ) ;

			/* Calc oobsize */
			mtd->oobsize =  oobsize_8kp[ ((extid & 0x40) >> 4) + ((extid >> 2) &0x03)] ;

			/* Calc blocksize. Blocksize is multiples of 128KB */
			mtd->erasesize = ( 1 << (((extid & 0x80) >> 5) + ((extid >> 4) & 0x03) )) * ( 128 * 1024) ;
			busw =  0;
		} else {
			/* Calc pagesize */
			mtd->writesize = 1024 << (extid & 0x3);
			extid >>= 2;
			/* Calc oobsize */
			mtd->oobsize = (8 << (extid & 0x01)) * (mtd->writesize >> 9);
			extid >>= 2;
			/* Calc blocksize. Blocksize is multiples of 64KiB */
			mtd->erasesize = (64 * 1024) << (extid & 0x03);
			extid >>= 2;
			/* Get buswidth information */
			busw = (extid & 0x01) ? NAND_BUSWIDTH_16 : 0;
		}
	} else {
		/*
		 * Old devices have chip data hardcoded in the device id table
		 */
		mtd->erasesize = type->erasesize;
		mtd->writesize = type->pagesize;
		mtd->oobsize = mtd->writesize / 32;
		busw = type->options & NAND_BUSWIDTH_16;
	}

	/* Try to identify manufacturer */
	for (maf_idx = 0; nand_manuf_ids[maf_idx].id != 0x0; maf_idx++) {
		if (nand_manuf_ids[maf_idx].id == *maf_id)
			break;
	}

	/*  if Toshiba TH58NVG4S0FTA20 : 2GB, 4k page size, 256kB block size, 232B oob size
	 *  M_ID=0x98, D_ID=0xD3, ID[3]:[1:0] page size 1,2,4,8k;
	 *  ID[3]:[5:4] block size 64kB,128kB,256kB,512kB
	 *  And Strap pin need to set to 4k page with 224B oob size : flash type : 0x7
	 * ECC level : 4bit ECC for each 512Byte is required. So need to define BCH ECC algorithm.
	 */
	 if(id[0] == NAND_MFR_TOSHIBA && id[1]== 0xD3)
	 {
	 	int extid;
	 	extid = id[3];
	 	mtd->writesize = 1024 << (extid & 0x3);
	 	mtd->oobsize =  224;
	 	extid >>= 4;
	 	mtd->erasesize = (64 * 1024) << (extid & 0x03);
	 	type->name = "NAND 2GiB 3,3V 8-bit";
	 	mtd->name = "NAND 2GiB 3,3V 8-bit";
	 	type->chipsize  = 2048;
	 	chip->chipsize = type->chipsize << 20;
	 	#ifndef CONFIG_CS75XX_NAND_ECC_HW_BCH
	 		printk(KERN_INFO "NAND ECC Level 4bit ECC for each 512Byte is required. \n");
	 	#endif
	 }

/////// middle debug : oob size not 4 bytes alignment
	if(mtd->oobsize%8)
		mtd->oobsize = mtd->oobsize - (mtd->oobsize%8);
///////

	/*
	 * Check, if buswidth is correct. Hardware drivers should set
	 * chip correct !
	 */
	if (busw != (chip->options & NAND_BUSWIDTH_16)) {
		printf(KERN_INFO "NAND device: Manufacturer ID:"
		       " 0x%02x, Chip ID: 0x%02x (%s %s)\n", *maf_id,
		       dev_id, nand_manuf_ids[maf_idx].name, mtd->name);
		printf(KERN_WARNING "NAND bus width %d instead %d bit\n",
		       (chip->options & NAND_BUSWIDTH_16) ? 16 : 8,
		       busw ? 16 : 8);
		return ERR_PTR(-EINVAL);
	}

	/* Calculate the address shift from the page size */
	chip->page_shift = ffs(mtd->writesize) - 1;
	/* Convert chipsize to number of pages per chip -1. */
	chip->pagemask = (chip->chipsize >> chip->page_shift) - 1;

	chip->bbt_erase_shift = chip->phys_erase_shift =
		ffs(mtd->erasesize) - 1;
	if (chip->chipsize & 0xffffffff)
		chip->chip_shift = ffs((unsigned)chip->chipsize) - 1;
	else
		chip->chip_shift = ffs((unsigned)(chip->chipsize >> 32)) + 32 - 1;

	/* Set the bad block position */
	chip->badblockpos = mtd->writesize > 512 ?
		NAND_LARGE_BADBLOCK_POS : NAND_SMALL_BADBLOCK_POS;

	/* Get chip options, preserve non chip based options */
	chip->options &= ~NAND_CHIPOPTIONS_MSK;
	chip->options |= type->options & NAND_CHIPOPTIONS_MSK;

	/*
	 * Set chip as a default. Board drivers can override it, if necessary
	 */
	chip->options |= NAND_NO_AUTOINCR;

	/* Check if chip is a not a samsung device. Do not clear the
	 * options for chips which are not having an extended id.
	 */
	if (*maf_id != NAND_MFR_SAMSUNG && !type->pagesize)
		chip->options &= ~NAND_SAMSUNG_LP_OPTIONS;

#if 0
	/* Check for AND chips with 4 page planes */
	if (chip->options & NAND_4PAGE_ARRAY)
		chip->erase_cmd = multi_erase_cmd;
	else
		chip->erase_cmd = single_erase_cmd;

	/* Do not replace user supplied command function ! */
	if (mtd->writesize > 512 && chip->cmdfunc == cs752x_nand_command)
		chip->cmdfunc = cs752x_nand_command_lp;
#endif

#ifdef CS75XX_NAND_DEBUG
	printf(KERN_INFO "NAND device: Manufacturer ID:"
	       " 0x%02x, Chip ID: 0x%02x (%s %s)\n", *maf_id, dev_id,
	       nand_manuf_ids[maf_idx].name, type->name);
#endif
	return type;
}

/*
 *	hardware specific access to control-lines
 *	ctrl:
 */
static void cs75xx_nand_hwcontrol(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)
{
	/**
	struct sharpsl_nand *sharpsl = mtd_to_sharpsl(mtd);
	struct nand_chip *chip = mtd->priv;
  middle not yet
  nothing to do now.
  **/
}


/**
 * cs75xx_nand_default_block_markbad - [DEFAULT] mark a block bad
 * @mtd:	MTD device structure
 * @ofs:	offset from device start
 *
 * This is the default implementation, which can be overridden by
 * a hardware specific driver.
*/
static int cs75xx_nand_default_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	uint8_t buf[2] = { 0, 0 };
	int block, ret;

	/* Get block number */
	block = (int)(ofs >> chip->bbt_erase_shift);
	if (chip->bbt)
		chip->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	/* Do we have a flash based bad block table ? */
	if (chip->options & NAND_USE_FLASH_BBT)
		ret = nand_update_bbt(mtd, ofs);
	else {
		/* We write two bytes, so we dont have to mess with 16 bit
		 * access
		 */
		cs75xx_nand_get_device(chip, mtd, FL_WRITING);
		ofs += mtd->oobsize;
		chip->ops.len = chip->ops.ooblen = 2;
		chip->ops.datbuf = NULL;
		chip->ops.oobbuf = buf;
		chip->ops.ooboffs = chip->badblockpos & ~0x01;

		ret = cs75xx_nand_do_write_oob(mtd, ofs, &chip->ops);
		cs75xx_nand_release_device(mtd);
	}
	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}

/**
 * cs75xx_nand_block_bad - [DEFAULT] Read bad block marker from the chip
 * @mtd:	MTD device structure
 * @ofs:	offset from device start
 * @getchip:	0, if the chip is already selected
 *
 * Check, if the block is bad.
 */
static int cs75xx_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	int page, chipnr, res = 0;
	struct nand_chip *chip = mtd->priv;
	u16 bad;

	page = (int)(ofs >> chip->page_shift) & chip->pagemask;

	if (getchip) {
		chipnr = (int)(ofs >> chip->chip_shift);

		cs75xx_nand_get_device(chip, mtd, FL_READING);

		/* Select the NAND device */
		chip->select_chip(mtd, chipnr);
	}

	cs75xx_nand_read_oob_std(mtd, chip, page, 0);

	if(chip->oob_poi[chip->badblockpos] != 0xff)
		return 1;

	if (getchip)
		cs75xx_nand_release_device(mtd);

	return res;
}

#define NOTALIGNED(x)	(x & (chip->subpagesize - 1)) != 0
/**
 * cs75xx_nand_do_write_ops - [Internal] NAND write with ECC
 * @mtd:	MTD device structure
 * @to:		offset to write to
 * @ops:	oob operations description structure
 *
 * NAND write with ECC
 */
static int cs75xx_nand_do_write_ops(struct mtd_info *mtd, loff_t to,
			     struct mtd_oob_ops *ops)
{
	int chipnr, realpage, page, blockmask, column;
	struct nand_chip *chip = mtd->priv;
	uint32_t writelen = ops->len;
	uint8_t *oob = ops->oobbuf;
	uint8_t *buf = ops->datbuf;
	int ret, subpage;

	ops->retlen = 0;
	if (!writelen)
		return 0;

	/* reject writes, which are not page aligned */
	if (NOTALIGNED(to) /*|| NOTALIGNED(ops->len)*/) {
		printf(KERN_NOTICE "%s: Attempt to write not "
				"page aligned data\n", __func__);
		return -EINVAL;
	}

	column = to & (mtd->writesize - 1);
	subpage = column || (writelen & (mtd->writesize - 1));

	if (subpage && oob) {
		return -EINVAL;
	}

	chipnr = (int)(to >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	/* Check, if it is write protected */
	if (cs75xx_nand_check_wp(mtd))
		return -EIO;

	realpage = (int)(to >> chip->page_shift);
	page = realpage & chip->pagemask;
	blockmask = (1 << (chip->phys_erase_shift - chip->page_shift)) - 1;

	/* Invalidate the page cache, when we write to the cached page */
	if (to <= (chip->pagebuf << chip->page_shift) &&
	    (chip->pagebuf << chip->page_shift) < (to + ops->len))
		chip->pagebuf = -1;

	/* If we're not given explicit OOB data, let it be 0xFF */
	if (likely(!oob))
		memset(chip->oob_poi, 0xff, mtd->oobsize);

	while(1) {
		int bytes = mtd->writesize;
		int cached = writelen > bytes && page != blockmask;
		uint8_t *wbuf = buf;

		/* Partial page write ? */
		if (unlikely(column || writelen < (mtd->writesize - 1))) {
			cached = 0;
			bytes = min_t(int, bytes - column, (int) writelen);
			chip->pagebuf = -1;
			//chip->ecc.read_page(mtd, chip, chip->buffers->databuf, (page<<chip->page_shift));
			chip->ecc.read_page(mtd, chip, chip->buffers->databuf, page);
			memcpy(&chip->buffers->databuf[column], buf, bytes);
			wbuf = chip->buffers->databuf;
		}

		if (unlikely(oob))
			oob = cs75xx_nand_fill_oob(chip, oob, ops);

		ret = chip->write_page(mtd, chip, wbuf, page, cached,
				       (ops->mode == MTD_OOB_RAW));
		if (ret)
			break;

		writelen -= bytes;
		if (!writelen)
			break;

		column = 0;
		buf += bytes;
		realpage++;

		page = realpage & chip->pagemask;
		/* Check, if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}
	}

	ops->retlen = ops->len - writelen;
	if (unlikely(oob))
		ops->oobretlen = ops->ooblen;
	return ret;
}

/**
 * cs75xx_nand_wait - [DEFAULT]  wait until the command is done
 * @mtd:	MTD device structure
 * @chip:	NAND chip structure
 *
 * Wait for command done. This applies to erase and program only
 * Erase can take up to 400ms and program up to 20ms according to
 * general NAND and SmartMedia specs
 */
/* XXX U-BOOT XXX */
static int cs75xx_nand_wait(struct mtd_info *mtd, struct nand_chip *this)
{
	unsigned long	timeo;
	int state = this->state;

	if (state == FL_ERASING)
		timeo = (CONFIG_SYS_HZ * 400) / 1000;
	else
		timeo = (CONFIG_SYS_HZ * 20) / 1000;

	if ((state == FL_ERASING) && (this->options & NAND_IS_AND))
		this->cmdfunc(mtd, NAND_CMD_STATUS_MULTI, -1, -1);
	else
		this->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);

	/** reset_timer(); **/
	timeo += get_timer(0);

	while (1) {
		if (get_timer(0) > timeo) {
			printf("Timeout!");
			return 0x01;
		}

		if (this->dev_ready) {
			if (this->dev_ready(mtd))
				break;
		} else {
			/** if (this->read_byte(mtd) & NAND_STATUS_READY)**/
				break;
		}
	}
#ifdef PPCHAMELON_NAND_TIMER_HACK
	reset_timer();
	while (get_timer(0) < 10);
#endif /*  PPCHAMELON_NAND_TIMER_HACK */

	state = read_flash_ctrl_reg(FLASH_NF_DATA)&0xff;
	return state;
}


void cs75xx_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	return;
}

/**
 * cs75xx_nand_verify_buf - [DEFAULT] Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 * Default verify function for 8bit buswith
 */
static int cs75xx_nand_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i, page=0;
	struct nand_chip *chip = mtd->priv;
	//u_char *tmpdata, *tmpoob;tmpdata ,tmpoob
	size_t  retlen;
	retlen=0;

	page = host->page_addr;

	memset( chip->buffers->databuf, 0, mtd->writesize);
	chip->ecc.read_page(mtd, chip, chip->buffers->databuf, page);

	if(len==mtd->writesize)
	{
		for (i=0; i<len; i++)
		{
			if (buf[i] != chip->buffers->databuf[i])
			{
				printf("Data verify error -> page: %x, byte: %x, buf[i]:%x  chip->buffers->databuf[i]:%x \n",host->page_addr,i, buf[i], chip->buffers->databuf[i]);
				return i;
			}
		}
	}
	else if(len == mtd->oobsize)
	{
		for (i=0; i<len; i++)
		{
			if (buf[i] != chip->oob_poi[i])
			{
				printf("OOB verify error -> page: %x, byte: %x, buf[i]:%x  chip->oob_poi[i]:%x  \n",host->page_addr,i,buf[i],chip->oob_poi[i]);
				return i;
			}
		}
	}
	else
	{
		printf (KERN_WARNING "verify length not match 0x%08x\n", len);

		return -1;
	}

	return 0;
}

/**
 * cs75xx_nand_read_buf - [DEFAULT] read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 * Default read function for 8bit buswith
 */
static void cs75xx_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int page, col;
	struct nand_chip *this = mtd->priv;
	struct cs75xx_nand_host *host = nand_chip->priv;

	if(len <= (mtd->writesize+ mtd->oobsize))
	{
		page = host->page_addr;
  		col  = host->col_addr;

		this->ecc.read_page(mtd, this, this->buffers->databuf, page);
    	        memcpy(buf, &(this->buffers->databuf[col]), len);

	}

}

/**
 * cs75xx_nand_write_buf - [DEFAULT] write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 * Default write function for 8bit buswith
 */
static void cs75xx_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i, page=0,col=0;
	struct nand_chip *chip = mtd->priv;
	struct cs75xx_nand_host *host = nand_chip->priv;
  	size_t  retlen;
  	retlen=0;


	/** if(len <= (mtd->writesize+mtd->oobsize))**/
	if(len <= (mtd->writesize+ mtd->oobsize))
	{

		page = host->page_addr;
  		col  = host->col_addr;

		chip->ecc.read_page(mtd, chip, chip->buffers->databuf, page);

                for(i=0;i<len;i++)
                        chip->buffers->databuf[col+i] = buf[i];

                chip->ecc.write_page(mtd, chip,	chip->buffers->databuf);
	}
}

/**
 * cs75xx_nand_read_byte - [DEFAULT] read one byte from the chip
 * @mtd:	MTD device structure
 *
 * Default read function for 8bit buswith
 */
static uint8_t cs75xx_nand_read_byte(struct mtd_info *mtd)
{

  struct nand_chip *chip = mtd->priv;
  struct cs75xx_nand_host *host = nand_chip->priv;
  unsigned int    data=0, page=0, col=0;


  page = host->page_addr;
  col  = host->col_addr;

  cs75xx_nand_read_page_raw(mtd, chip, chip->buffers->databuf, page);
  data = *(chip->buffers->databuf+col);

  return data&0xff;
}

/**
 * cs75xx_nand_select_chip - [DEFAULT] control CE line
 * @mtd:	MTD device structure
 * @chipnr:	chipnumber to select, -1 for deselect
 *
 * Default select function for 1 chip devices.
 */
static void cs75xx_nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;

	switch (chipnr) {
	case -1:
		CHIP_EN = NFLASH_CHIP0_EN;
		break;
	case 0:
		CHIP_EN = NFLASH_CHIP0_EN;
		break;
	case 1:
		CHIP_EN = NFLASH_CHIP1_EN;
		break;

	default:
		CHIP_EN = NFLASH_CHIP0_EN;
		/** BUG();**/
	}

}

/*
 * Wait for the ready pin, after a command
 * The timeout is catched later.
 */
static void cs75xx_nand_wait_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	u32 timeo = (CONFIG_SYS_HZ * 20) / 1000;

	/** reset_timer();**/
	timeo += get_timer(0);
	/* wait until command is processed or timeout occures */
	while (get_timer(0) < timeo) {
		if (chip->dev_ready)
			if (chip->dev_ready(mtd))
				break;
	}
}

/**
 * cs75xx_nand_command - [DEFAULT] Send command to NAND device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This function is used for small page
 * devices (256/512 Bytes per page)
 */
static void cs75xx_nand_command(struct mtd_info *mtd, unsigned int command,
			 int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;

	int ctrl = NAND_CTRL_CLE | NAND_CTRL_CHANGE;
	uint32_t rst_sts_cnt = CONFIG_SYS_NAND_RESET_CNT;
	struct cs75xx_nand_host *host = nand_chip->priv;

	FLASH_FLASH_ACCESS_START_t tmp_access;

	/*
	 * Write out the command to the device.
	 */
	if (command == NAND_CMD_SEQIN) {
		int readcmd;

		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		chip->cmd_ctrl(mtd, readcmd, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
	}
	chip->cmd_ctrl(mtd, command, ctrl);

	/*
	 * Address cycle, when necessary
	 */
	ctrl = NAND_CTRL_ALE | NAND_CTRL_CHANGE;
	/* Serially input address */
	if (column != -1) {
		/* Adjust columns for 16 bit buswidth */
		if (chip->options & NAND_BUSWIDTH_16)
			column >>= 1;
		chip->cmd_ctrl(mtd, column, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
		host->col_addr = column;
	}
	if (page_addr != -1) {
		chip->cmd_ctrl(mtd, page_addr, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
		chip->cmd_ctrl(mtd, page_addr >> 8, ctrl);
		/* One more address cycle for devices > 32MiB */
		if (chip->chipsize > (32 << 20))
			chip->cmd_ctrl(mtd, page_addr >> 16, ctrl);

		host->page_addr = page_addr;
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status and sequential in needs no delay
	 */
	switch (command) {

	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_STATUS:
		/*
		 * Write out the command to the device.
		 */
		if (column != -1 || page_addr != -1) {

			/* Serially input address */
			if (column != -1)
				/** FLASH_WRITE_REG(NFLASH_ADDRESS,column);**/
				host->col_addr = column;

			if (page_addr != -1)
				/** FLASH_WRITE_REG(NFLASH_ADDRESS,opcode|(page_addr<<8));**/
				host->page_addr = page_addr;

		}
		return;

	case NAND_CMD_RESET:
		check_flash_ctrl_status();
		udelay(chip->chip_delay);
		write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0x0); /** disable ecc gen**/
		nf_cnt.wrd = 0;
		nf_cnt.bf.nflashRegOobCount = NCNT_EMPTY_OOB;
		nf_cnt.bf.nflashRegDataCount = NCNT_EMPTY_DATA;
		nf_cnt.bf.nflashRegAddrCount = NCNT_EMPTY_ADDR;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
		write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);

		nf_cmd.wrd = 0;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_RESET;
		write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd); /** write read id command**/
		nf_addr1.wrd = 0;
		write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd); /** write address 0x00**/
		nf_addr2.wrd = 0;
		write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd); /** write address 0x00**/

		nf_access.wrd = 0;
		nf_access.bf.nflashCeAlt = CHIP_EN;
		/** nf_access.bf.nflashDirWr = ;**/
		nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;

		write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);
		flash_start.wrd = 0;
		flash_start.bf.nflashRegReq = FLASH_GO;
		flash_start.bf.nflashRegCmd = FLASH_WT;
		write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

		tmp_access.wrd = 0;
		tmp_access.bf.nflashRegReq = 1;
		reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);
		flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);

		udelay(100);

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}
	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	cs75xx_nand_wait_ready(mtd);
}

/**
 * cs75xx_nand_command_lp - [DEFAULT] Send command to NAND large page device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices We dont have the separate regions as we have in the small page
 * devices.  We must emulate NAND_CMD_READOOB to keep the code compatible.
 */
static void cs75xx_nand_command_lp(struct mtd_info *mtd, unsigned int command,
			    int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	uint32_t rst_sts_cnt = CONFIG_SYS_NAND_RESET_CNT;
	struct cs75xx_nand_host *host = nand_chip->priv;
	FLASH_FLASH_ACCESS_START_t tmp_access;

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	chip->cmd_ctrl(mtd, command & 0xff,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (chip->options & NAND_BUSWIDTH_16)
				column >>= 1;
			chip->cmd_ctrl(mtd, column, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			chip->cmd_ctrl(mtd, column >> 8, ctrl);
		}
		if (page_addr != -1) {
			chip->cmd_ctrl(mtd, page_addr, ctrl);
			chip->cmd_ctrl(mtd, page_addr >> 8,
				       NAND_NCE | NAND_ALE);
			/* One more address cycle for devices > 128MiB */
			if (chip->chipsize > (128 << 20))
				chip->cmd_ctrl(mtd, page_addr >> 16,
					       NAND_NCE | NAND_ALE);
		}
	}
	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * program and erase have their own busy handlers
	 * status, sequential in, and deplete1 need no delay
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		/*
		 * Write out the command to the device.
		 */
		if (column != -1 || page_addr != -1) {

			/* Serially input address */
			if (column != -1)
				/** FLASH_WRITE_REG(NFLASH_ADDRESS,column);**/
				host->col_addr = column;

			if (page_addr != -1)
				/** FLASH_WRITE_REG(NFLASH_ADDRESS,opcode|(page_addr<<8));**/
				host->page_addr = page_addr;

		}
		return;

		/*
		 * read error status commands require only a short delay
		 */
	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		udelay(chip->chip_delay);
		return;

	case NAND_CMD_RESET:


		check_flash_ctrl_status();
		udelay(chip->chip_delay);
		write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0x0); /** disable ecc gen**/
		nf_cnt.wrd = 0;
		nf_cnt.bf.nflashRegOobCount = NCNT_EMPTY_OOB;
		nf_cnt.bf.nflashRegDataCount = NCNT_EMPTY_DATA;
		nf_cnt.bf.nflashRegAddrCount = NCNT_EMPTY_ADDR;
		nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;
		write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);

		nf_cmd.wrd = 0;
		nf_cmd.bf.nflashRegCmd0 = NAND_CMD_RESET;
		write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd); /** write read id command**/
		nf_addr1.wrd = 0;
		write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd); /** write address 0x00**/
		nf_addr2.wrd = 0;
		write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd); /** write address 0x00**/

		nf_access.wrd = 0;
		nf_access.bf.nflashCeAlt = CHIP_EN;
		/** nf_access.bf.nflashDirWr = ;**/
		nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;

		write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);
		flash_start.wrd = 0;
		flash_start.bf.nflashRegReq = FLASH_GO;
		flash_start.bf.nflashRegCmd = FLASH_WT;
		write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

		tmp_access.wrd = 0;
		tmp_access.bf.nflashRegReq = 1;
		reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);
		flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);

		udelay(100);
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		chip->cmd_ctrl(mtd, NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		chip->cmd_ctrl(mtd, NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);

	cs75xx_nand_wait_ready(mtd);
}

static int cs75xx_nand_dev_ready(struct mtd_info *mtd)
{
	int ready;
	FLASH_FLASH_ACCESS_START_t tmp_access;

	check_flash_ctrl_status();

	write_flash_ctrl_reg(FLASH_NF_DATA,0xffffffff);
RD_STATUS:
	write_flash_ctrl_reg(FLASH_NF_ECC_CONTROL, 0x0); /** disable ecc gen**/
	nf_cnt.wrd = 0;
	nf_cnt.bf.nflashRegOobCount = NCNT_EMPTY_OOB;
	nf_cnt.bf.nflashRegDataCount = NCNT_DATA_1;
	nf_cnt.bf.nflashRegAddrCount = NCNT_EMPTY_ADDR;
	nf_cnt.bf.nflashRegCmdCount = NCNT_CMD_1;

	write_flash_ctrl_reg(FLASH_NF_COUNT, nf_cnt.wrd);

	nf_cmd.wrd = 0;
	nf_cmd.bf.nflashRegCmd0 = NAND_CMD_STATUS;
	write_flash_ctrl_reg(FLASH_NF_COMMAND, nf_cmd.wrd); /** write read id command**/
	nf_addr1.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_1, nf_addr1.wrd); /** write address 0x00**/
	nf_addr2.wrd = 0;
	write_flash_ctrl_reg(FLASH_NF_ADDRESS_2, nf_addr2.wrd); /** write address 0x00**/

	nf_access.wrd = 0;
	nf_access.bf.nflashCeAlt = CHIP_EN;
	/** nf_access.bf.nflashDirWr = ;**/
	nf_access.bf.nflashRegWidth = NFLASH_WiDTH8;

	write_flash_ctrl_reg(FLASH_NF_ACCESS, nf_access.wrd);
	flash_start.wrd = 0;
	flash_start.bf.nflashRegReq = FLASH_GO;
	flash_start.bf.nflashRegCmd = FLASH_RD;
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, flash_start.wrd);

	flash_start.wrd=read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
	tmp_access.wrd = 0;
	tmp_access.bf.nflashRegReq = 1;
	reg_wait(FLASH_FLASH_ACCESS_START, tmp_access.wrd, 0, 1000);

	check_flash_ctrl_status();

      	ready = read_flash_ctrl_reg(FLASH_NF_DATA)&0xff;
      	if(ready==0xff)
      	{
      		printf("cs75xx_nand_dev_ready : %x\n",read_flash_ctrl_reg(FLASH_STATUS));
	    	goto RD_STATUS;
	}

	return (ready & NAND_STATUS_READY);
}


int board_nand_init(struct nand_chip *this)
{
	struct mtd_info *mtd;
	unsigned short tmp;
	int err = 0;

	/* structures must be linked */
	mtd = &host->mtd;
	mtd->priv = this;
	host->nand = this;

	/* 5 us command delay time */
	this->chip_delay = 20;

	this->priv = host;
	this->dev_ready = cs75xx_nand_dev_ready;
	this->cmdfunc = cs75xx_nand_command;
	this->select_chip = cs75xx_nand_select_chip;
	this->read_byte = cs75xx_nand_read_byte;
	/** this->read_word = cs75xx_nand_read_word;**/
	this->write_buf = cs75xx_nand_write_buf;
	this->read_buf = cs75xx_nand_read_buf;
	this->verify_buf = cs75xx_nand_verify_buf;


/** #ifdef CONFIG_CS75XX_NAND_HWECC**/
	this->ecc.calculate = cs75xx_nand_calculate_ecc;
	this->ecc.hwctl = cs75xx_nand_enable_hwecc;
	this->ecc.correct = cs75xx_nand_correct_data;
	this->ecc.mode = NAND_ECC_HW;

#ifdef CONFIG_CS752X_NAND_ECC_HW_BCH_8_512
	this->ecc.size = 512;
	this->ecc.bytes = 13;
#elif CONFIG_CS752X_NAND_ECC_HW_BCH_12_512
	this->ecc.size = 512;
	this->ecc.bytes = 20;
#elif CONFIG_CS752X_NAND_ECC_HW_HAMMING_512
	this->ecc.size = 512;
	this->ecc.bytes = 3;
#else
	this->ecc.size = 256;
	this->ecc.bytes = 3;
#endif
/** #endif**/

	/* check, if a user supplied wait function given */
	this->waitfunc = cs75xx_nand_wait;
	this->block_bad = cs75xx_nand_block_bad;
	this->block_markbad = cs75xx_nand_default_block_markbad;

	if (!this->scan_bbt)
		this->scan_bbt = nand_default_bbt;


	//this->controller = cs75xx_nand_hwcontrol;

	this->cmd_ctrl = cs75xx_nand_hwcontrol;
	/* Reset NAND */
	this->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	return err;
}

/**
 * cs75xx_nand_scan_tail - [NAND Interface] Scan for the NAND device
 * @mtd:	    MTD device structure
 * @maxchips:	    Number of chips to scan for
 *
 * This is the second phase of the normal nand_scan() function. It
 * fills out all the uninitialized function pointers with the defaults
 * and scans for a bad block table if appropriate.
 */
int cs75xx_nand_scan_tail(struct mtd_info *mtd)
{
	int i, eccStartOffset;
	struct nand_chip *chip = mtd->priv;

	if (!(chip->options & NAND_OWN_BUFFERS))
		chip->buffers = kmalloc(sizeof(*chip->buffers), GFP_KERNEL);
	if (!chip->buffers)
		return -ENOMEM;

	/* Set the internal oob buffer location, just after the page data */
	chip->oob_poi = chip->buffers->databuf + mtd->writesize;

	/*
	 * If no default placement scheme is given, select an appropriate one
	 */
	if (!chip->ecc.layout) {
#ifdef CONFIG_CS752X_NAND_ECC_HW_BCH_8_512
		if( mtd->oobsize == 16) {
			chip->ecc.layout= &cs75xx_nand_bch_oob_16;
		}  else
#elif  (CONFIG_CS752X_NAND_ECC_HW_HAMMING_512 ||  CONFIG_CS752X_NAND_ECC_HW_HAMMING_256)
		if( mtd->oobsize == 8 ) {
			chip->ecc.layout= &cs75xx_nand_oob_8;
		} else if( mtd->oobsize == 16 ) {
			chip->ecc.layout= &cs75xx_nand_oob_16;
		} else
#endif
		{
			memset( &cs75xx_nand_ecclayout, 0, sizeof( cs75xx_nand_ecclayout));
			cs75xx_nand_ecclayout.eccbytes= mtd->writesize/chip->ecc.size*chip->ecc.bytes;
			if( sizeof( cs75xx_nand_ecclayout.eccpos) < 4*cs75xx_nand_ecclayout.eccbytes ) {
				printf(KERN_WARNING "eccpos memory is less than needed eccbytes");
				return 1;
			}

			if ( cs75xx_nand_ecclayout.eccbytes > mtd->oobsize) {
				printf(KERN_WARNING "eccbytes is less than oob size");
				return 1;
			}

			memset( cs75xx_nand_ecclayout.eccpos, 0, sizeof( cs75xx_nand_ecclayout.eccpos));
			eccStartOffset= mtd->oobsize - cs75xx_nand_ecclayout.eccbytes;
			for( i = 0 ; i < cs75xx_nand_ecclayout.eccbytes; ++i) {
				if( ( i+ eccStartOffset ) == chip->badblockpos) {
					continue;
				}
				cs75xx_nand_ecclayout.eccpos[ i ]=  i + eccStartOffset;
			}

			cs75xx_nand_ecclayout.oobfree[0].offset = 2;
			cs75xx_nand_ecclayout.oobfree[0].length = mtd->oobsize - cs75xx_nand_ecclayout.eccbytes - cs75xx_nand_ecclayout.oobfree[0].offset;

#ifdef CONFIG_CS75XX_NAND_ECC_HW_BCH
			/**  BCH algorithm needs one extra byte to tag erase status**/
			if ( cs75xx_nand_ecclayout.oobfree[0].length == 0) {
				printf(KERN_WARNING "eccbytes is less than required");
				return 1;
			};
			cs75xx_nand_ecclayout.oobfree[0].length -= 1;
#endif
			chip->ecc.layout= &cs75xx_nand_ecclayout;
		}
	}

	if (!chip->write_page)
		chip->write_page = cs75xx_nand_write_page;

	/*
	 * check ECC mode, default to software if 3byte/512byte hardware ECC is
	 * selected and we have 256 byte pagesize fallback to software ECC
	 */
	if (!chip->ecc.read_page_raw)
		chip->ecc.read_page_raw = cs75xx_nand_read_page_raw;
	if (!chip->ecc.write_page_raw)
		chip->ecc.write_page_raw = cs75xx_nand_write_page_raw;


		/* Use standard hwecc read page function ? */
		if (!chip->ecc.read_page)
			chip->ecc.read_page = cs75xx_nand_read_page_hwecc;
		if (!chip->ecc.write_page)
			chip->ecc.write_page = cs75xx_nand_write_page_hwecc;
		if (!chip->ecc.read_oob)
			chip->ecc.read_oob = cs75xx_nand_read_oob_std;
		if (!chip->ecc.write_oob)
			chip->ecc.write_oob = cs75xx_nand_write_oob_std;


	/*
	 * The number of bytes available for a client to place data into
	 * the out of band area
	 */
	chip->ecc.layout->oobavail = 0;
	for (i = 0; chip->ecc.layout->oobfree[i].length; i++)
		chip->ecc.layout->oobavail +=
			chip->ecc.layout->oobfree[i].length;
	mtd->oobavail = chip->ecc.layout->oobavail;

	/*
	 * Set the number of read / write steps for one page depending on ECC
	 * mode
	 */
	chip->ecc.steps = mtd->writesize / chip->ecc.size;
	if(chip->ecc.steps * chip->ecc.size != mtd->writesize) {
		printf(KERN_WARNING "Invalid ecc parameters\n");
		BUG();
	}
	chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;

	/*
	 * Allow subpage writes up to ecc.steps. Not possible for MLC
	 * FLASH.
	 */
	if (!(chip->options & NAND_NO_SUBPAGE_WRITE) &&
	    !(chip->cellinfo & NAND_CI_CELLTYPE_MSK)) {
		switch(chip->ecc.steps) {
		case 2:
			mtd->subpage_sft = 1;
			break;
		case 4:
		case 8:
			mtd->subpage_sft = 2;
			break;
		}
	}
	chip->subpagesize = mtd->writesize >> mtd->subpage_sft;

	/* Initialize state */
	chip->state = FL_READY;

	/* De-select the device */
	chip->select_chip(mtd, -1);

	/* Invalidate the pagebuffer reference */
	chip->pagebuf = -1;

	/* Fill in remaining MTD driver data */
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->erase = cs75xx_nand_erase;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = cs75xx_nand_read;
	mtd->write = cs75xx_nand_write;
	mtd->read_oob = cs75xx_nand_read_oob;
	mtd->write_oob = cs75xx_nand_write_oob;
	mtd->sync = cs75xx_nand_sync;
	mtd->lock = NULL;
	mtd->unlock = NULL;
	mtd->suspend = cs75xx_nand_suspend;
	mtd->resume = cs75xx_nand_resume;
	mtd->block_isbad = cs75xx_nand_block_isbad;
	mtd->block_markbad = cs75xx_nand_block_markbad;

	/* propagate ecc.layout to mtd_info */
	mtd->ecclayout = chip->ecc.layout;

	/* Check, if we should skip the bad block table scan */
	if (chip->options & NAND_SKIP_BBTSCAN)
		chip->options |= NAND_BBT_SCANNED;

	return 0;
}


/**
 * cs75xx_nand_scan_ident - [NAND Interface] Scan for the NAND device
 * @mtd:	     MTD device structure
 * @maxchips:	     Number of chips to scan for
 *
 * This is the first phase of the normal nand_scan() function. It
 * reads the flash ID and sets up MTD fields accordingly.
 *
 * The mtd->owner field must be set to the module of the caller.
 */
int cs75xx_nand_scan_ident(struct mtd_info *mtd, int maxchips)
{
	int i, busw, nand_maf_id, nand_dev_id;
	struct nand_chip *chip = mtd->priv;
	struct nand_flash_dev *type;
	unsigned char id[8];

	/* Get buswidth to select the correct functions */
	busw = chip->options & NAND_BUSWIDTH_16;
	/* Set the default functions */
	/** nand_set_defaults(chip, busw);**/

	/* Read the flash type */
	type = cs75xx_nand_get_flash_type(mtd, chip, busw, &nand_maf_id);

	if (IS_ERR(type)) {
#ifndef CONFIG_SYS_NAND_QUIET_TEST
		printf(KERN_WARNING "No NAND device found!!!\n");
#endif
		chip->select_chip(mtd, -1);
		return PTR_ERR(type);
	}

	/* Check for a chip array */
	for (i = 1; i < maxchips; i++) {
		chip->select_chip(mtd, i);
		/* See comment in nand_get_flash_type for reset */
		chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
		/* Send the command for reading device ID */
		cs75xx_nand_read_id(1,id);
		/* Read manufacturer and device IDs */
		if (nand_maf_id != id[0] ||
		    nand_dev_id != id[1])
			break;
	}
#ifdef CS75XX_NAND_DEBUG
	if (i > 1)
		printf("%d NAND chips detected\n", i);
#endif

	/* Store the number of chips and calc total size for mtd */
	chip->numchips = i;
	mtd->size = i * chip->chipsize;

	return 0;
}

static int init_DMA_SSP( void )
{
	int i;
	unsigned char *tmp;
	DMA_DMA_SSP_RXDMA_CONTROL_t dma_rxdma_ctrl;
	DMA_DMA_SSP_TXDMA_CONTROL_t dma_txdma_ctrl;

	DMA_DMA_SSP_RXQ5_BASE_DEPTH_t dma_rxq5_base_depth;
	DMA_DMA_SSP_TXQ5_BASE_DEPTH_t dma_txq5_base_depth;

	dma_rxdma_ctrl.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_RXDMA_CONTROL);
	dma_txdma_ctrl.wrd = read_dma_ctrl_reg(DMA_DMA_SSP_TXDMA_CONTROL);

	if ((dma_rxdma_ctrl.bf.rx_check_own != 1)
	    && (dma_rxdma_ctrl.bf.rx_dma_enable != 1)) {
		dma_rxdma_ctrl.bf.rx_check_own = 1;
		dma_rxdma_ctrl.bf.rx_dma_enable = 1;
		write_dma_ctrl_reg(DMA_DMA_SSP_RXDMA_CONTROL,
				   dma_rxdma_ctrl.wrd);
	}
	if ((dma_txdma_ctrl.bf.tx_check_own != 1)
	    && (dma_txdma_ctrl.bf.tx_dma_enable != 1)) {
		dma_txdma_ctrl.bf.tx_check_own = 1;
		dma_txdma_ctrl.bf.tx_dma_enable = 1;
		write_dma_ctrl_reg(DMA_DMA_SSP_TXDMA_CONTROL,
				   dma_txdma_ctrl.wrd);
	}

	i = 16;
	tx_desc = (DMA_SSP_TX_DESC_T *) ((u32)malloc((sizeof(DMA_SSP_TX_DESC_T) * FDMA_DESC_NUM)+ i -1) & ~(i - 1));
	rx_desc = (DMA_SSP_RX_DESC_T *) ((u32)malloc((sizeof(DMA_SSP_RX_DESC_T) * FDMA_DESC_NUM)+ i -1) & ~(i - 1));
//(void *)(((u32)malloc(size + align) + align - 1) & ~(align - 1));
	if (!rx_desc || !tx_desc) {
		printk("Buffer allocation for failed!\n");
		if (rx_desc) {
			kfree(rx_desc);
		}

		if (tx_desc) {
			kfree(tx_desc);
		}

		return 0;
	}

	/* printk("tx_desc_v: %p , rx_desc_v: %p \n", tx_desc, rx_desc); */
	/* set base address and depth */

	dma_rxq5_base_depth.bf.base = ((unsigned int)rx_desc) >> 4;
	dma_rxq5_base_depth.bf.depth = FDMA_DEPTH;
	write_dma_ctrl_reg(DMA_DMA_SSP_RXQ5_BASE_DEPTH,
			   dma_rxq5_base_depth.wrd);

	dma_txq5_base_depth.bf.base = ((unsigned int)tx_desc) >> 4;
	dma_txq5_base_depth.bf.depth = FDMA_DEPTH;
	write_dma_ctrl_reg(DMA_DMA_SSP_TXQ5_BASE_DEPTH,
			   dma_txq5_base_depth.wrd);

	memset((unsigned char *)tx_desc, 0,
	       (sizeof(DMA_SSP_TX_DESC_T) * FDMA_DESC_NUM));
	memset((unsigned char *)rx_desc, 0,
	       (sizeof(DMA_SSP_RX_DESC_T) * FDMA_DESC_NUM));

	for (i = 0; i < FDMA_DESC_NUM; i++) {
		/* set own by sw */
		tx_desc[i].word0.bf.own = OWN_SW;
		/* enable q5 Scatter-Gather memory copy */
		tx_desc[i].word0.bf.sgm_rsrvd = 0x15;
	}

	return 1;
}

int cs75xx_nand_scan(struct mtd_info *mtd, int maxchips)
{
	int ret;

	/* Many callers got this wrong, so check for it for a while... */
	/* XXX U-BOOT XXX */
#if 0
	if (!mtd->owner && caller_is_module()) {
		printf(KERN_CRIT "nand_scan() called with NULL mtd->owner!\n");
		BUG();
	}
#endif

	ret = cs75xx_nand_scan_ident(mtd, maxchips);
	if (!ret)
		ret = cs75xx_nand_scan_tail(mtd);
	return ret;
}

static void cs75xx_nand_init_chip(struct mtd_info *mtd, struct nand_chip *nand,
			   ulong base_addr)
{
	int maxchips = CONFIG_SYS_NAND_MAX_CHIPS;
	int __attribute__((unused)) i = 0;

	if( init_DMA_SSP() == 0 ) {
		printf("desc alloc error!! \n");
	}

	if (maxchips < 1)
		maxchips = 1;
	mtd->priv = nand;

	/** set to NAND data register**/
	nand->IO_ADDR_R = nand->IO_ADDR_W = (void  __iomem *)FLASH_NF_DATA;/** base_addr;**/
	if (board_nand_init(nand) == 0) {
		if (cs75xx_nand_scan(mtd, maxchips) == 0) {
			if (!mtd->name)
				mtd->name = (char *)default_nand_name;
#ifndef CONFIG_RELOC_FIXUP_WORKS
			else
				mtd->name += gd->reloc_off;
#endif

#ifdef CONFIG_MTD_DEVICE
			/*
			 * Add MTD device so that we can reference it later
			 * via the mtdcore infrastructure (e.g. ubi).
			 */
			sprintf(dev_name[i], "nand%d", i);
			mtd->name = dev_name[i++];
			add_mtd_device(mtd);
#endif
		} else
			mtd->name = NULL;
	} else {
		mtd->name = NULL;
		mtd->size = 0;
	}

}

void cs75xx_nand_init(void)
{
	int i;
	unsigned int size = 0;
	for (i = 0; i < CONFIG_SYS_MAX_NAND_DEVICE; i++) {
		cs75xx_nand_init_chip(&nand_info[i], &nand_chip[i], base_address[i]);
		size += nand_info[i].size / 1024;
		if (nand_curr_device == -1)
			nand_curr_device = i;
	}
	printf("%u MiB\n", size / 1024);

#ifdef CONFIG_SYS_NAND_SELECT_DEVICE
	/*
	 * Select the chip in the board/cpu specific driver
	 */
	board_nand_select_device(nand_info[nand_curr_device].priv, nand_curr_device);
#endif
}
