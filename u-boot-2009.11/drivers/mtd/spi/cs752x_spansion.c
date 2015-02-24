
#include <common.h>
#include <malloc.h>
#include <spi_flash.h>

#include "spi_flash_internal.h"
#include <asm/arch/registers.h>

#define SF_AC_OPCODE                	0x0000
#define SF_AC_OPCODE_1_DATA           	0x0100
#define SF_AC_OPCODE_2_DATA           	0x0200
#define SF_AC_OPCODE_3_DATA           	0x0300
#define SF_AC_OPCODE_4_DATA           	0x0400
#define SF_AC_OPCODE_4_ADDR         	0x0500
#define SF_AC_OPCODE_4_ADDR_1_DATA    	0x0600
#define SF_AC_OPCODE_4_ADDR_2_DATA    	0x0700
#define SF_AC_OPCODE_4_ADDR_3_DATA    	0x0800
#define SF_AC_OPCODE_4_ADDR_4_DATA    	0x0900
#define SF_AC_OPCODE_4_ADDR_X_1_DATA   	0x0A00
#define SF_AC_OPCODE_4_ADDR_X_2_DATA   	0x0B00
#define SF_AC_OPCODE_4_ADDR_X_3_DATA   	0x0C00
#define SF_AC_OPCODE_4_ADDR_X_4_DATA   	0x0D00
#define SF_AC_OPCODE_4_ADDR_4X_1_DATA  	0x0E00

#define SF_START_BIT_ENABLE             0x0002 

#define SFLASH_FORCEBURST               0x2000 
#define SFLASH_FORCETERM                0x1000




#define g_chipen			0x0
#define SPANSION_FLASH_TYPE		0x0

#define SPANSION_WRITE_STATUS_OPCODE	( 0x01 | SF_AC_OPCODE_1_DATA  )
#define SPANSION_PROGRAM_OPCODE         ( 0x02 | SF_AC_OPCODE_4_ADDR_1_DATA )
#define SPANSION_READ_OPCODE            ( 0x03 | SF_AC_OPCODE_4_ADDR_1_DATA )
#define SPANSION_READ_STATUS_OPCODE	( 0x05 | SF_AC_OPCODE_2_DATA  )
#define SPANSION_WRITE_ENABLE_OPCODE	( 0x06 | SF_AC_OPCODE )
#define SPANSION_ERASE_OPCODE           ( 0xD8 | SF_AC_OPCODE_4_ADDR )

#define SPANSION_STATUS_WEL             0x02
#define SPANSION_STATUS_WIP             0x01
#define SPANSION_STATUS_BP		0x1C
#define SPANSION_STATUS_SRWD            0x80

#define SPANSION_WEL_TIMEOUT             400
#define SPANSION_PROGRAM_TIMEOUT         300
#define SPANSION_ERASE_TIMEOUT          4000
#define SPANSION_CMD_TIMEOUT		2000


#define S25FL128P_SECTOR_SIZE         0x00010000
#define S25FL128P_PAGE_SIZE           0x00000100
#define S25FL128P_SIZE		      0x01000000


static void write_flash_ctrl_reg(unsigned int ofs,unsigned int data)
{
	unsigned int *base;

	base = (unsigned int *)(ofs);
	*base = data;
}

static unsigned int read_flash_ctrl_reg(unsigned int ofs)
{
    unsigned int *base;

    base = (unsigned int *)( ofs);
    return (*base);
}

static int cs752x_spansion_cmd(u32 opcode, u32 addr, u32 data)
{
	unsigned long tmp;
	unsigned long timebase;

	write_flash_ctrl_reg(FLASH_SF_ACCESS, opcode | g_chipen);
	write_flash_ctrl_reg(FLASH_SF_ADDRESS, addr);
	write_flash_ctrl_reg(FLASH_SF_DATA, data);
	write_flash_ctrl_reg(FLASH_FLASH_ACCESS_START, SF_START_BIT_ENABLE);

	timebase = get_timer( 0 );
	do {
		tmp = read_flash_ctrl_reg(FLASH_FLASH_ACCESS_START);
		if ((tmp & SF_START_BIT_ENABLE) == 0) {
			return 0;
		}
	} while (get_timer(timebase) < SPANSION_CMD_TIMEOUT);

	return -1;
}

static int wait_flash_status(u32 flag, u32 value, u32 wait_msec)
{
	unsigned long status;
	unsigned long timebase;

	timebase = get_timer(0);
	do {
		if (cs752x_spansion_cmd(SPANSION_READ_STATUS_OPCODE, 0, 0)) {
			return -1;
		}

		status = read_flash_ctrl_reg(FLASH_SF_DATA);
		if ((status & flag) == value) {
			return 0;
		}

	} while (get_timer(timebase) < wait_msec);

	return -1;
}

static int spansion_read(struct spi_flash *flash,
			     u32 offset, size_t len, void *buf)
{
	unsigned long old_type;
	unsigned char *src;

	old_type = read_flash_ctrl_reg( FLASH_TYPE);
	write_flash_ctrl_reg( FLASH_TYPE, SPANSION_FLASH_TYPE );


	write_flash_ctrl_reg( FLASH_SF_ACCESS, g_chipen);
	src= (unsigned char *)((unsigned int )(CONFIG_SYS_FLASH_BASE) + offset);
	memcpy( buf,  src, len);

	write_flash_ctrl_reg( FLASH_TYPE, old_type);

	return 0;
}

static int spansion_write(struct spi_flash *flash,
			  u32 offset, size_t len, const void *buf)
{
	unsigned long old_type;
	unsigned long opcode;
	unsigned char *src, *dst;

	old_type = read_flash_ctrl_reg(FLASH_TYPE);
	write_flash_ctrl_reg(FLASH_TYPE, SPANSION_FLASH_TYPE);

	if ((offset % S25FL128P_PAGE_SIZE) || (len % S25FL128P_PAGE_SIZE)) {
		write_flash_ctrl_reg(FLASH_TYPE, old_type);
		return -1;
	}

	src = (unsigned char *)buf;
	dst = (unsigned char *)(CONFIG_SYS_FLASH_BASE + offset);
	while (len > 0) {

		if (cs752x_spansion_cmd(SPANSION_WRITE_ENABLE_OPCODE, 0, 0)) {
			break;
		}

		if (wait_flash_status (SPANSION_STATUS_WEL, SPANSION_STATUS_WEL, SPANSION_WEL_TIMEOUT)) {
			break;
		}

		opcode = SPANSION_PROGRAM_OPCODE | g_chipen | SFLASH_FORCEBURST;
		write_flash_ctrl_reg(FLASH_SF_ACCESS, opcode);


		read_flash_ctrl_reg(FLASH_SF_ACCESS);	/* dummy read */
		//wmb();
		memcpy( dst, src, S25FL128P_PAGE_SIZE);

		opcode = g_chipen | SFLASH_FORCETERM;
		write_flash_ctrl_reg(FLASH_SF_ACCESS, opcode);

		wait_flash_status(SPANSION_STATUS_WIP, 0, SPANSION_PROGRAM_OPCODE);


		len -= S25FL128P_PAGE_SIZE;
		dst += S25FL128P_PAGE_SIZE;
		src += S25FL128P_PAGE_SIZE;
		
	}

	write_flash_ctrl_reg(FLASH_TYPE, old_type);

	return len > 0 ? -1 : 0;
}

int spansion_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	unsigned long old_type;

	old_type = read_flash_ctrl_reg(FLASH_TYPE);
	write_flash_ctrl_reg(FLASH_TYPE, SPANSION_FLASH_TYPE);

	if ((offset % S25FL128P_SECTOR_SIZE) || (len % S25FL128P_SECTOR_SIZE)) {
		write_flash_ctrl_reg(FLASH_TYPE, old_type);
		return -1;
	}

	while (len > 0) {

		if (cs752x_spansion_cmd(SPANSION_WRITE_ENABLE_OPCODE, 0, 0)) {
			break;
		}

		if (wait_flash_status (SPANSION_STATUS_WEL, SPANSION_STATUS_WEL, SPANSION_WEL_TIMEOUT)) {
			break;
		}

		if (cs752x_spansion_cmd(SPANSION_ERASE_OPCODE, offset, 0)) {
			break;
		}

		if (wait_flash_status (SPANSION_STATUS_WIP, 0, SPANSION_ERASE_TIMEOUT)) {
			break;
		}

		len -= S25FL128P_SECTOR_SIZE;
		offset += S25FL128P_SECTOR_SIZE;
	}

	write_flash_ctrl_reg(FLASH_TYPE, old_type);
	return len > 0 ? -1 : 0;
}

struct spi_flash *spi_flash_probe(unsigned int bus, unsigned int cs,
				  unsigned int max_hz, unsigned int spi_mode)
{
	struct spi_flash *flash;

	flash = malloc(sizeof(struct spi_flash));
	if (!flash) {
		debug("SF: Failed to allocate memory\n");
		return NULL;
	}

	flash->spi = NULL;
	flash->name = "Spansion";
	flash->size = S25FL128P_SIZE;

	flash->write = spansion_write;
	flash->erase = spansion_erase;
	flash->read = spansion_read;

	return flash;
}

void spi_flash_free(struct spi_flash *flash)
{
	free(flash);
}
