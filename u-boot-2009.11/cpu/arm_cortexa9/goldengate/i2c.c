/*
 *  i2c Support for Atmel's AT91RM9200 Two-Wire Interface
 *
 *  (c) Rick Bronson
 *
 *  Borrowed heavily from original work by:
 *  Copyright (c) 2000 Philip Edelbrock <phil@stimpy.netroedge.com>
 *
 *  Modified to work with u-boot by (C) 2004 Gary Jennejohn garyj@denx.de
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
*/
#include <common.h>

#ifdef CONFIG_HARD_I2C

#include <i2c.h>
#include <asm/io.h>

#undef G2_I2C_DEBUG

#ifdef G2_I2C_DEBUG
#define debug(fmt,args...)	printf (fmt ,##args)
#else
#define debug(fmt,args...)
#endif /* MKIMAGE_DEBUG */

//#include <at91rm9200_i2c.h>
#define BIT(x)  (1<<(x))

/* define DEBUG */
/* Register Map */
#define G2_BIW_CFG		0x00
#define G2_BIW_CTRL		0x04
#define G2_BIW_TXR		0x08
#define G2_BIW_RXR		0x0C
#define G2_BIW_ACK		0x10
#define G2_BIW_IE0		0x14
#define G2_BIW_INT0		0x18
#define G2_BIW_IE1		0x1C
#define G2_BIW_INT1		0x20
#define G2_BIW_STAT		0x24

/* G2_BIW_CFG BIT */
#define BIW_CFG_CORE_EN		BIT(0)
#define BIW_CFG_PRER_OFF	16
#define BIW_CFG_PRER_MASK	0xFFFF0000

/* G2_BIW_CTRL BIT */
#define BIW_CTRL_DONE		BIT(0)
#define BIW_CTRL_ACK_IN		BIT(3)
#define BIW_CTRL_WRITE		BIT(4)
#define BIW_CTRL_READ		BIT(5)
#define BIW_CTRL_STOP		BIT(6)
#define BIW_CTRL_START		BIT(7)

/* G2_BIW_TXR BIT */
#define BIW_TXR_OFF			0
#define BIW_TXR_MASK		0x000000FF

/* G2_BIW_RXR BIT */
#define BIW_RXR_OFF			0
#define BIW_RXR_MASK		0x000000FF

/* G2_BIW_ACK */
#define BIW_ACK_AL			BIT(0)
#define BIW_ACK_BUSY		BIT(1)
#define BIW_ACK_ACK_OUT		BIT(2)

/* G2_BIW_IE0 BIT */
#define BIW_IE_BIT			BIT(0)

/* G2_BIW_INT0 */
#define BIW_INT_BIT			BIT(0)

/* G2_BIW_STAT */
#define BIW_STAT_BIT		BIT(0)

#define G2_PER_CLOCK		50	/* 20 ns Time period units */
#define G2_I2C_RETRY_CNT	3000


#define I2C_WRITE_CMD			0x00
#define I2C_READ_CMD			0x01
//#define CS4341_I2C_SLAVE_ADDR	0x10	/* prefix 001000 + AD0 */
#define CS_DONE			1
#define CS_OK           0
#define CS_ERROR        -1
#define CS_ERR_ENTRY_NOT_FOUND	-2

#define PER_BIW_CFG                              0xf00700e8

#define read_reg(addr)					(__raw_readl(addr))
#define write_reg(addr, val)			(__raw_writel(val, addr))
#define g2_i2c_read_reg(offset)			(read_reg(PER_BIW_CFG+offset))
#define g2_i2c_write_reg(offset, val)	(write_reg(PER_BIW_CFG+offset, val))


/****************************************************************/
/* $rtn_hdr_start  I2C: SET SPEED                               */
/* CATEGORY   : Device                                          */
/* ACCESS     : private                                         */
/* BLOCK      : Peripheral                                      */
/* CHIP       : Daytona                                         */
/* INPUTS     : o I2C Interface Speed (100 or 400 KHz)          */
/*              o Interface enable/disable ctl (TRUE/FALSE)     */
/* OUTPUTS    : ----                                            */
/* RETURNS    : CS_STATUS or Error Code                         */
/* DESCRIPTION:                                                 */
/* Configures the I2C interface speed.                          */
/* The [speed_khz] parameter is typically specified as 100KHz or*/
/* 400KHz. This parameter will be ignored when [enbl] parameter */
/* is FALSE                                                     */
/* Note that the API will allow the user to set the I2C clock   */
/* speed to something other than 100KHz or 400KHz, in case some */
/* fine-tuning of the speed is needed and/or proprietary I2C    */
/* slave devices are used on the customer board. When set to    */
/* anything other than 100 or 400KHz, the API will print a      */
/* warning message to notify the user that the setting is not   */
/* typical.                                                     */
/* $rtn_hdr_end                                                 */
/****************************************************************/
 int g2_i2c_set_speed(int speed_khz, int enbl)
{
	unsigned int reg_v;
	unsigned short per_clk_khz, prer;

	//debug("Set speed %d \n",speed_khz);
	
	if (!speed_khz) {
		debug("Invalid CLK value \n");
		return 0;
	}

	if (100 != speed_khz  && 400 != speed_khz) {
		debug("WRNG: I2C Speed set to %dKHz!\n", speed_khz) ;
	}

	per_clk_khz = G2_PER_CLOCK ;
	/*
	* The equation to compute the data to be programmed is
	* freq_bwc = 1/(5 * per_clk (prer + 1))
	* per_clk units specifed by Time Period
	* or,
	* Assumed units as Mhz
	* freq_bwc = (per_clk_khz * 1000)/5 * (speed_khz + 1);
	* where, per_clk is 100Mhz (10ns for Time period)
	* 100KHz, the prer to be programmed is 199
	*/
	prer = (per_clk_khz * 1000)/ (5 * (speed_khz + 1));
	reg_v = prer << BIW_CFG_PRER_OFF | (enbl ? BIW_CFG_CORE_EN : 0);


	g2_i2c_write_reg(G2_BIW_CFG, reg_v);
	
	//debug("Set speed ok \n");
	
	return (CS_OK) ;
}

unsigned int i2c_get_bus_speed(void)
{
    debug("i2c_get_bus_speed\n");
}


/*
 * Poll the i2c status register until the specified bit is set.
 * Returns 0 if timed out (100 msec)
 */
 short g2_i2c_poll_biw_done(void) {
	
	unsigned short retry_cnt = G2_I2C_RETRY_CNT;

	do {
		if (g2_i2c_read_reg(G2_BIW_CTRL) & BIW_CTRL_DONE)
			break;
	} while (retry_cnt--);

	if (!retry_cnt) {
		debug("%s: I2C Bus biw_done fail\n", __func__);
		return CS_ERROR;
	}

	return (CS_OK) ;

}

 void i2c_reset( void )
{
	debug("I2c reset\n");

}

int
i2c_probe(unsigned char chip)
{
/**
	#define CS4341_MCLK		0x00
	#define CS4341_MODE		0x01
	#define CS4341_TRANS	0x02
	#define CS4341_CHA_VOL	0x03
	#define CS4341_CHB_VOL	0x04
	cs4341_reg_i2c_write(CS4341_I2C_SLAVE_ADDR, CS4341_MODE, 0x11);
	cs4341_reg_i2c_write(CS4341_I2C_SLAVE_ADDR, CS4341_TRANS, 0x49)

	cs4341_reg_i2c_write(CS4341_I2C_SLAVE_ADDR, CS4341_CHA_VOL, 0x0);
	cs4341_reg_i2c_write(CS4341_I2C_SLAVE_ADDR, CS4341_CHB_VOL, 0x0);
	//cs4341_reg_i2c_write(CS4341_I2C_SLAVE_ADDR, CS4341_MODE, 0x10);
	cs4341_reg_i2c_write(CS4341_I2C_SLAVE_ADDR, CS4341_MODE, 0x20);
	d.s 0xf0000018 %l 0x04000001
	d.s 0xF0700020 %l 0x808DB92D
	Set GLOBAL_PIN_MUX (0xf0000018) pmux_frac_clk_en_gpio1_16 (bit26) to 1
	Write 0x808DB92D to 0xF0700020


**/

	debug("I2c probe\n");
	return 0;
}

 unsigned int g2_i2c_write(unsigned char data)
{
	unsigned int reg_v = data;


	g2_i2c_write_reg(G2_BIW_TXR, reg_v);

	return (CS_OK) ;
}

 unsigned int g2_i2c_read(unsigned char *p_data)
{
	

    /* Read the Data */
	*p_data = g2_i2c_read_reg(G2_BIW_RXR) & BIW_RXR_MASK;


	return (CS_OK) ;
}


 unsigned int g2_i2c_set_intf_ctrl(unsigned int ctrl_value)
{
	unsigned int reg_v = (BIW_CTRL_DONE | BIW_CTRL_ACK_IN | BIW_CTRL_WRITE |
		            BIW_CTRL_READ | BIW_CTRL_STOP | BIW_CTRL_START);

	if (!(ctrl_value & reg_v)) {
		return CS_ERROR;
	}

	ctrl_value &= reg_v;

	g2_i2c_write_reg(G2_BIW_CTRL, ctrl_value);

  	return (CS_OK) ;
}

 unsigned int g2_i2c_data_read(unsigned char slave_addr, unsigned char reg, unsigned char *value_p)
{
	int err_code;

	//printf("g2_i2c_data_read : slave_addr %x reg %x\n",slave_addr, reg);
	
	
	/* start and send slave address */
	if (g2_i2c_write((slave_addr << 1) | I2C_WRITE_CMD) != CS_OK) {
		debug("I2c Write fail\n");
		return CS_ERROR;
	}

	g2_i2c_set_intf_ctrl(BIW_CTRL_START | BIW_CTRL_WRITE);
	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);

	/* select register */
	if (g2_i2c_write(reg) != CS_OK) {
		debug("I2c Write fail\n");
		return CS_ERROR;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_WRITE);
	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);

	/* restart and send slave address */
	if (g2_i2c_write((slave_addr << 1) | I2C_READ_CMD) != CS_OK) {
		debug("I2c Write fail\n");
		return CS_ERROR;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_START | BIW_CTRL_WRITE);
	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);

	/* read register data */
	g2_i2c_set_intf_ctrl(BIW_CTRL_READ | BIW_CTRL_ACK_IN | BIW_CTRL_STOP);
//	g2_i2c_set_intf_ctrl(BIW_CTRL_READ | BIW_CTRL_STOP);

	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);
	if (g2_i2c_read(value_p) != CS_OK) {
		debug("I2c Read fail\n");
		return CS_ERROR;
	}

	return CS_OK;
}

 unsigned int g2_i2c_data_write(unsigned char slave_addr, unsigned char reg, unsigned char value)
{
	unsigned int err_code;

	//printf("g2_i2c_data_write : slave_addr %x, reg %x, value %x\n",slave_addr, reg, value);
	
	/* start and send slave address */
	if (g2_i2c_write((slave_addr << 1) | I2C_WRITE_CMD) != CS_OK) {
		debug("I2c Write fail\n");
		return CS_ERROR;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_START | BIW_CTRL_WRITE);
	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);

	/* select register */
	if (g2_i2c_write(reg) != CS_OK) {
		debug("I2c Write fail\n");
		return CS_ERROR;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_WRITE);
	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);

	/* write register data */
	if (g2_i2c_write(value) != CS_OK) {
		debug("I2c Write fail\n");
		return CS_ERROR;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_STOP | BIW_CTRL_WRITE);
	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);

	return CS_OK;
}

int
i2c_read (unsigned char chip, unsigned int addr, int alen,
	  unsigned char *buffer, int len)
{
	int i;
#if 0	
	if (alen > 1)
		return 1;
	if (alen == 1) {
		addr = addr & 0xff;
	}
	for(i=0;i<len;i++)
		g2_i2c_data_read(chip, addr, (buffer+i));
#else
	int err_code;
	
	//printf("g2_i2c_data_read : chip %x addr %x\n",chip, addr);
	
	
	/* start and send slave address */
	if (g2_i2c_write((chip << 1) | I2C_WRITE_CMD) != CS_OK) {
		debug("I2c Write fail\n");
		return CS_ERROR;
	}

	g2_i2c_set_intf_ctrl(BIW_CTRL_START | BIW_CTRL_WRITE);
	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);

	/* select register */
	for(i=0;i<alen;i++)
	{
		if (g2_i2c_write( ((addr >>(i*8))& 0xff) ) != CS_OK) {
			debug("I2c Write fail\n");
			return CS_ERROR;
		}
	
		g2_i2c_set_intf_ctrl(BIW_CTRL_WRITE);
		if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
			debug("I2C Bus biw_done fail\n");
			return err_code;
		}
		g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);
        }
        	
		/* restart and send slave address */
		if (g2_i2c_write((chip << 1) | I2C_READ_CMD) != CS_OK) {
			debug("I2c Write fail\n");
			return CS_ERROR;
		}
		g2_i2c_set_intf_ctrl(BIW_CTRL_START | BIW_CTRL_WRITE);
		if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
			debug("I2C Bus biw_done fail\n");
			return err_code;
		}
		g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);
        
        for(i=0;i<len;i++)
	{	
		/* read register data */
		if(i==(len-1))
			g2_i2c_set_intf_ctrl(BIW_CTRL_READ | BIW_CTRL_ACK_IN | BIW_CTRL_STOP);
		else
			g2_i2c_set_intf_ctrl(BIW_CTRL_READ | BIW_CTRL_ACK_IN);
//		g2_i2c_set_intf_ctrl(BIW_CTRL_READ | BIW_CTRL_STOP);
        	
		if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
			debug("I2C Bus biw_done fail\n");
			return err_code;
		}
		g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);
		if (g2_i2c_read((buffer+i)) != CS_OK) {
			debug("I2c Read fail\n");
			return CS_ERROR;
		}
		g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);
	}


#endif			
	return 0;
		
}

int
i2c_write(unsigned char chip, unsigned int addr, int alen,
	  unsigned char *buffer, int len)
{

	int i;
	//unsigned char *buf;
#if 0
	/* we only allow one address byte */
	if (alen > 1)
		return 1;
	if (alen == 1) {
		addr = addr & 0xff;
	}
	for(i=0;i<len;i++)
		g2_i2c_data_write(chip, addr, *(buffer+i));

#else
	unsigned int err_code;

	//printf("g2_i2c_data_write : chip %x, addr %x, *(buffer+i) %x\n",chip, addr, *(buffer+i));
	
	/* start and send slave address */
	if (g2_i2c_write((chip << 1) | I2C_WRITE_CMD) != CS_OK) {
		debug("I2c Write fail\n");
		return CS_ERROR;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_START | BIW_CTRL_WRITE);
	if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
		debug("I2C Bus biw_done fail\n");
		return err_code;
	}
	g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);

	for(i=0;i<alen;i++)
	{
		/* select register */
		if (g2_i2c_write( ((addr >>(i*8))& 0xff) ) != CS_OK) {
			debug("I2c Write fail\n");
			return CS_ERROR;
		}
		
		g2_i2c_set_intf_ctrl(BIW_CTRL_WRITE);
		if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
			debug("I2C Bus biw_done fail\n");
			return err_code;
		}
		g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);
        }
        
        for(i=0;i<len;i++)
	{	
		/* write register data */
		if (g2_i2c_write(*(buffer+i)) != CS_OK) {
			debug("I2c Write fail\n");
			return CS_ERROR;
		}
		if(i==(len-1))
			g2_i2c_set_intf_ctrl(BIW_CTRL_STOP | BIW_CTRL_WRITE);
		else
			g2_i2c_set_intf_ctrl(BIW_CTRL_WRITE);
			
		if ((err_code = g2_i2c_poll_biw_done()) != CS_OK) {
			debug("I2C Bus biw_done fail\n");
			return err_code;
		}
		g2_i2c_set_intf_ctrl(BIW_CTRL_DONE);
	}
#endif	
	return 0;
}

/*
 * Main initialization routine
 */
void
i2c_init(int speed, int slaveaddr)
{
	g2_i2c_set_speed(100, 1);

	//debug ("Found G2 i2c\n");
	return;
}

#endif /* CONFIG_HARD_I2C */
