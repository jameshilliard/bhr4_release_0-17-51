
#include <common.h>
#include <malloc.h>
#include <spi_flash.h>

#include "spi_flash_internal.h"
#include <asm/arch/registers.h>

#define CFI_MFR_AMD             0x0001
#define CFI_MFR_AMIC            0x0037
#define CFI_MFR_ATMEL           0x001F
#define CFI_MFR_EON             0x001C
#define CFI_MFR_FUJITSU         0x0004
#define CFI_MFR_HYUNDAI         0x00AD
#define CFI_MFR_INTEL           0x0089
#define CFI_MFR_MACRONIX        0x00C2
#define CFI_MFR_NEC             0x0010
#define CFI_MFR_PMC             0x009D
#define CFI_MFR_SAMSUNG         0x00EC
#define CFI_MFR_SHARP           0x00B0
#define CFI_MFR_SST             0x00BF
#define CFI_MFR_ST              0x0020 /* STMicroelectronics */
#define CFI_MFR_TOSHIBA         0x0098
#define CFI_MFR_WINBOND         0x00DA

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
//#define SFLASH_FLASH_TYPE		0x0
#define SFLASH_4BYTE_ADDR		0x400


/* Flash opcodes. */
#define SFLASH_WRITE_STATUS_OPCODE	( 0x01 | SF_AC_OPCODE_1_DATA  )
#define SFLASH_PROGRAM_OPCODE         	( 0x02 | SF_AC_OPCODE_4_ADDR_1_DATA )
#define SFLASH_READ_OPCODE            	( 0x03 | SF_AC_OPCODE_4_ADDR_1_DATA )
#define SFLASH_FASTREAD_OPCODE          ( 0x0B | SF_AC_OPCODE_4_ADDR_1_DATA )
#define SFLASH_READ_STATUS_OPCODE	( 0x05 | SF_AC_OPCODE_1_DATA  )
#define SFLASH_WRITE_ENABLE_OPCODE	( 0x06 | SF_AC_OPCODE )
#define SFLASH_BE_4K_OPCODE           	( 0x20 | SF_AC_OPCODE_4_ADDR )
#define SFLASH_BE_32K_OPCODE           	( 0x52 | SF_AC_OPCODE_4_ADDR )
#define SFLASH_CHIP_ERASE_OPCODE      	( 0xc7 | SF_AC_OPCODE_4_ADDR )
#define SFLASH_ERASE_OPCODE           	( 0xD8 | SF_AC_OPCODE_4_ADDR )
#define SFLASH_READID_OPCODE           	( 0x9F | SF_AC_OPCODE_4_DATA )


/* Used for SST flashes only. */
//#define OPCODE_BP               0x02    /* Byte program */
//#define OPCODE_WRDI             0x04    /* Write disable */
//#define OPCODE_AAI_WP           0xad    /* Auto address increment word program */


/* Used for Macronix flashes only. */
#define SFLASH_OPCODE_EN4B             ( 0xb7 | SF_AC_OPCODE  )    /* Enter 4-byte mode */
#define SFLASH_OPCODE_EX4B             ( 0xe9 | SF_AC_OPCODE  )    /* Exit 4-byte mode */

/* Used for Spansion flashes only. */
#define SFLASH_OPCODE_BRWR             ( 0x17 | SF_AC_OPCODE_1_DATA  )   /* Bank register write */

#define SFLASH_STATUS_WEL             	0x02
#define SFLASH_STATUS_WIP             	0x01
#define SFLASH_STATUS_BP		0x1C
#define SFLASH_STATUS_SRWD            	0x80

#define SFLASH_WEL_TIMEOUT             	400
#define SFLASH_PROGRAM_TIMEOUT         	300
#define SFLASH_ERASE_TIMEOUT          	4000
#define SFLASH_CMD_TIMEOUT		2000


//#define S25FL128P_SECTOR_SIZE         0x00040000	//0x00010000
//#define S25FL128P_PAGE_SIZE           0x00000100
//#define S25FL128P_SIZE		      0x01000000
static u32 sflash_sector_size = 0x00040000;	//0x00010000
static u32 sflash_page_size = 0x00000100;
static u32 sflash_size = 0x01000000;
static u32 sflash_type = 0x0;

/****************************************************************************/

/*
 * SPI device driver setup and teardown
 */
struct spi_device_id {
        char name[32];
        unsigned long driver_data      /* Data private to the driver */
                        __attribute__((aligned(sizeof(unsigned long))));
};


#define JEDEC_MFR(_jedec_id)    ((_jedec_id) >> 16)

struct flash_info {
        /* JEDEC id zero means "no ID" (most older chips); otherwise it has
         * a high byte of zero plus three data bytes: the manufacturer id,
         * then a two byte device id.
         */
        u32             jedec_id;
        u16             ext_id;

        /* The size listed here is what works with OPCODE_SE, which isn't
         * necessarily called a "sector" by the vendor.
         */
        unsigned        sector_size;
        u16             n_sectors;

        u16             page_size;
        u16             addr_width;

        u16             flags;
#define SECT_4K         0x01            /* OPCODE_BE_4K works uniformly */
#define M25P_NO_ERASE   0x02            /* No erase command needed */
};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)      \
        ((unsigned long)&(struct flash_info) {                         \
                .jedec_id = (_jedec_id),                                \
                .ext_id = (_ext_id),                                    \
                .sector_size = (_sector_size),                          \
                .n_sectors = (_n_sectors),                              \
                .page_size = 256,                                       \
                .flags = (_flags),                                      \
        })

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */
static const struct spi_device_id m25p_ids[] = {
        /* Atmel -- some are (confusingly) marketed as "DataFlash" */
        { "at25fs010",  INFO(0x1f6601, 0, 32 * 1024,   4, SECT_4K) },
        { "at25fs040",  INFO(0x1f6604, 0, 64 * 1024,   8, SECT_4K) },

        { "at25df041a", INFO(0x1f4401, 0, 64 * 1024,   8, SECT_4K) },
        { "at25df321a", INFO(0x1f4701, 0, 64 * 1024,  64, SECT_4K) },
        { "at25df641",  INFO(0x1f4800, 0, 64 * 1024, 128, SECT_4K) },

        { "at26f004",   INFO(0x1f0400, 0, 64 * 1024,  8, SECT_4K) },
        { "at26df081a", INFO(0x1f4501, 0, 64 * 1024, 16, SECT_4K) },
        { "at26df161a", INFO(0x1f4601, 0, 64 * 1024, 32, SECT_4K) },
        { "at26df321",  INFO(0x1f4700, 0, 64 * 1024, 64, SECT_4K) },

        /* EON -- en25xxx */
        { "en25f32", INFO(0x1c3116, 0, 64 * 1024,  64, SECT_4K) },
        { "en25p32", INFO(0x1c2016, 0, 64 * 1024,  64, 0) },
        { "en25q32b", INFO(0x1c3016, 0, 64 * 1024,  64, 0) },
        { "en25p64", INFO(0x1c2017, 0, 64 * 1024, 128, 0) },

        /* Intel/Numonyx -- xxxs33b */
        { "160s33b",  INFO(0x898911, 0, 64 * 1024,  32, 0) },
        { "320s33b",  INFO(0x898912, 0, 64 * 1024,  64, 0) },
        { "640s33b",  INFO(0x898913, 0, 64 * 1024, 128, 0) },

        /* Macronix */
        { "mx25l4005a",  INFO(0xc22013, 0, 64 * 1024,   8, SECT_4K) },
        { "mx25l8005",   INFO(0xc22014, 0, 64 * 1024,  16, 0) },
        { "mx25l1606e",  INFO(0xc22015, 0, 64 * 1024,  32, SECT_4K) },
        { "mx25l3205d",  INFO(0xc22016, 0, 64 * 1024,  64, 0) },
        { "mx25l6405d",  INFO(0xc22017, 0, 64 * 1024, 128, 0) },
	/*        { "mx25l12805d", INFO(0xc22018, 0, 64 * 1024, 256, 0) }, */
        { "mx25l12845e", INFO(0xc22018, 0, 64 * 1024, 256, 0) },
        { "mx25l12855e", INFO(0xc22618, 0, 64 * 1024, 256, 0) },
        /* 257 4 bytes address */
        { "mx25l25735e", INFO(0xc22019, 0, 64 * 1024, 512, 0) },
        /* 256 3 bytes address */
        { "mx25l25635e", INFO(0xc22019, 0, 64 * 1024, 512, 0) },
        { "mx25l25655e", INFO(0xc22619, 0, 64 * 1024, 512, 0) },
        { "mx66l51235f", INFO(0xc2201a, 0, 64 * 1024, 1024, 0) },

        /* Spansion -- single (large) sector size only, at least
         * for the chips listed here (without boot sectors).
         */
        { "s25sl004a",  INFO(0x010212,      0,  64 * 1024,   8, 0) },
        { "s25sl008a",  INFO(0x010213,      0,  64 * 1024,  16, 0) },
        { "s25sl016a",  INFO(0x010214,      0,  64 * 1024,  32, 0) },
        { "s25sl032a",  INFO(0x010215,      0,  64 * 1024,  64, 0) },
        { "s25sl032p",  INFO(0x010215, 0x4d00,  64 * 1024,  64, SECT_4K) },
        { "s25sl064a",  INFO(0x010216,      0,  64 * 1024, 128, 0) },
        { "s25fl256s0", INFO(0x010219, 0x4d00, 256 * 1024, 128, 0) },
        { "s25fl256s1", INFO(0x010219, 0x4d01,  64 * 1024, 512, 0) },
        { "s25fl512s",  INFO(0x010220, 0x4d00, 256 * 1024, 256, 0) },
        { "s70fl01gs",  INFO(0x010221, 0x4d00, 256 * 1024, 256, 0) },
        { "s25sl12800", INFO(0x012018, 0x0300, 256 * 1024,  64, 0) },
        /* { "s25sl12801", INFO(0x012018, 0x0301,  64 * 1024, 256, 0) }, */
        { "s25fl129p0", INFO(0x012018, 0x4d00, 256 * 1024,  64, 0) },
        { "s25fl129p1", INFO(0x012018, 0x4d01,  64 * 1024, 256, 0) },
        { "s25fl016k",  INFO(0xef4015,      0,  64 * 1024,  32, SECT_4K) },
        { "s25fl064k",  INFO(0xef4017,      0,  64 * 1024, 128, SECT_4K) },

        /* SST -- large erase sizes are "overlays", "sectors" are 4K */
        { "sst25vf040b", INFO(0xbf258d, 0, 64 * 1024,  8, SECT_4K) },
        { "sst25vf080b", INFO(0xbf258e, 0, 64 * 1024, 16, SECT_4K) },
        { "sst25vf016b", INFO(0xbf2541, 0, 64 * 1024, 32, SECT_4K) },
        { "sst25vf032b", INFO(0xbf254a, 0, 64 * 1024, 64, SECT_4K) },
        { "sst25wf512",  INFO(0xbf2501, 0, 64 * 1024,  1, SECT_4K) },
        { "sst25wf010",  INFO(0xbf2502, 0, 64 * 1024,  2, SECT_4K) },
        { "sst25wf020",  INFO(0xbf2503, 0, 64 * 1024,  4, SECT_4K) },
        { "sst25wf040",  INFO(0xbf2504, 0, 64 * 1024,  8, SECT_4K) },

        /* ST Microelectronics -- newer production may have feature updates */
        { "m25p05",  INFO(0x202010,  0,  32 * 1024,   2, 0) },
        { "m25p10",  INFO(0x202011,  0,  32 * 1024,   4, 0) },
        { "m25p20",  INFO(0x202012,  0,  64 * 1024,   4, 0) },
        { "m25p40",  INFO(0x202013,  0,  64 * 1024,   8, 0) },
        { "m25p80",  INFO(0x202014,  0,  64 * 1024,  16, 0) },
        { "m25p16",  INFO(0x202015,  0,  64 * 1024,  32, 0) },
        { "m25p32",  INFO(0x202016,  0,  64 * 1024,  64, 0) },
        { "m25p64",  INFO(0x202017,  0,  64 * 1024, 128, 0) },
        { "m25p128", INFO(0x202018,  0, 256 * 1024,  64, 0) },

        { "m25p05-nonjedec",  INFO(0, 0,  32 * 1024,   2, 0) },
        { "m25p10-nonjedec",  INFO(0, 0,  32 * 1024,   4, 0) },
        { "m25p20-nonjedec",  INFO(0, 0,  64 * 1024,   4, 0) },
        { "m25p40-nonjedec",  INFO(0, 0,  64 * 1024,   8, 0) },
        { "m25p80-nonjedec",  INFO(0, 0,  64 * 1024,  16, 0) },
        { "m25p16-nonjedec",  INFO(0, 0,  64 * 1024,  32, 0) },
        { "m25p32-nonjedec",  INFO(0, 0,  64 * 1024,  64, 0) },
        { "m25p64-nonjedec",  INFO(0, 0,  64 * 1024, 128, 0) },
        { "m25p128-nonjedec", INFO(0, 0, 256 * 1024,  64, 0) },

        { "m45pe10", INFO(0x204011,  0, 64 * 1024,    2, 0) },
        { "m45pe80", INFO(0x204014,  0, 64 * 1024,   16, 0) },
        { "m45pe16", INFO(0x204015,  0, 64 * 1024,   32, 0) },

        { "m25pe80", INFO(0x208014,  0, 64 * 1024, 16,       0) },
        { "m25pe16", INFO(0x208015,  0, 64 * 1024, 32, SECT_4K) },

        { "m25px32",    INFO(0x207116,  0, 64 * 1024, 64, SECT_4K) },
        { "m25px32-s0", INFO(0x207316,  0, 64 * 1024, 64, SECT_4K) },
        { "m25px32-s1", INFO(0x206316,  0, 64 * 1024, 64, SECT_4K) },
        { "m25px64",    INFO(0x207117,  0, 64 * 1024, 128, 0) },

        /* Winbond -- w25x "blocks" are 64K, "sectors" are 4KiB */
        { "w25x10", INFO(0xef3011, 0, 64 * 1024,  2,  SECT_4K) },
        { "w25x20", INFO(0xef3012, 0, 64 * 1024,  4,  SECT_4K) },
        { "w25x40", INFO(0xef3013, 0, 64 * 1024,  8,  SECT_4K) },
        { "w25x80", INFO(0xef3014, 0, 64 * 1024,  16, SECT_4K) },
        { "w25x16", INFO(0xef3015, 0, 64 * 1024,  32, SECT_4K) },
        { "w25x32", INFO(0xef3016, 0, 64 * 1024,  64, SECT_4K) },
        { "w25q32", INFO(0xef4016, 0, 64 * 1024,  64, SECT_4K) },
        { "w25x64", INFO(0xef3017, 0, 64 * 1024, 128, SECT_4K) },
        { "w25q64", INFO(0xef4017, 0, 64 * 1024, 128, SECT_4K) },

        /* Catalyst / On Semiconductor -- non-JEDEC */
        { },
};

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
	} while (get_timer(timebase) < SFLASH_CMD_TIMEOUT);

	return -1;
}

static int wait_flash_status(u32 flag, u32 value, u32 wait_msec)
{
	unsigned long status;
	unsigned long timebase;

	timebase = get_timer(0);
	do {
		if (cs752x_spansion_cmd(SFLASH_READ_STATUS_OPCODE, 0, 0)) {
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
	write_flash_ctrl_reg( FLASH_TYPE, sflash_type );


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
	unsigned long opcode,size;
	unsigned char *src, *dst;

	old_type = read_flash_ctrl_reg(FLASH_TYPE);
	write_flash_ctrl_reg(FLASH_TYPE, sflash_type);

	//if ((offset % sflash_page_size) || (len % sflash_page_size)) {
	//	write_flash_ctrl_reg(FLASH_TYPE, old_type);
	//	return -1;
	//}

	src = (unsigned char *)buf;
	dst = (unsigned char *)(CONFIG_SYS_FLASH_BASE + offset);
	while (len > 0) {

		size = sflash_page_size - ((u32)dst % sflash_page_size);
		if (size > len)
			size = len;

		if (cs752x_spansion_cmd(SFLASH_WRITE_ENABLE_OPCODE, 0, 0)) {
			break;
		}

		if (wait_flash_status (SFLASH_STATUS_WEL, SFLASH_STATUS_WEL, SFLASH_WEL_TIMEOUT)) {
			break;
		}

		opcode = SFLASH_PROGRAM_OPCODE | g_chipen | SFLASH_FORCEBURST;
		write_flash_ctrl_reg(FLASH_SF_ACCESS, opcode);


		read_flash_ctrl_reg(FLASH_SF_ACCESS);	/* dummy read */
		//wmb();
		memcpy( dst, src, size);

		opcode = g_chipen | SFLASH_FORCETERM;
		write_flash_ctrl_reg(FLASH_SF_ACCESS, opcode);

		wait_flash_status(SFLASH_STATUS_WIP, 0, SFLASH_PROGRAM_OPCODE);


		len -= size;
		dst += size;
		src += size;

	}

	write_flash_ctrl_reg(FLASH_TYPE, old_type);

	return len > 0 ? -1 : 0;
}

int spansion_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	unsigned long old_type;

	old_type = read_flash_ctrl_reg(FLASH_TYPE);
	write_flash_ctrl_reg(FLASH_TYPE, sflash_type);

	if ((offset % sflash_sector_size) || (len % sflash_sector_size)) {
		write_flash_ctrl_reg(FLASH_TYPE, old_type);
		debug("SF: Erase length not sector alignment.\n");
		return -1;
	}

	while (len > 0) {

		if (cs752x_spansion_cmd(SFLASH_WRITE_ENABLE_OPCODE, 0, 0)) {
			break;
		}

		if (wait_flash_status (SFLASH_STATUS_WEL, SFLASH_STATUS_WEL, SFLASH_WEL_TIMEOUT)) {
			break;
		}

		if (cs752x_spansion_cmd(SFLASH_ERASE_OPCODE, offset, 0)) {
			break;
		}

		if (wait_flash_status (SFLASH_STATUS_WIP, 0, SFLASH_ERASE_TIMEOUT)) {
			break;
		}

		len -= sflash_sector_size;
		offset += sflash_sector_size;
	}

	write_flash_ctrl_reg(FLASH_TYPE, old_type);
	return len > 0 ? -1 : 0;
}

/*
 * Enable/disable 4-byte addressing mode.
 */
static inline int cs752x_set_4byte(u32 jedec_id, int enable)
{

	u32 tmp;

        switch (JEDEC_MFR(jedec_id)) {
        case CFI_MFR_MACRONIX:
               // flash->command[0] = enable ? OPCODE_EN4B : OPCODE_EX4B;
               // return spi_write(flash->spi, flash->command, 1);

               	if (cs752x_spansion_cmd(enable ? SFLASH_OPCODE_EN4B : SFLASH_OPCODE_EX4B, 0, 0)) {
			debug("SF: MACRONIX set 4 bytes error.\n");
		}
		break;
	case CFI_MFR_ST:
		/* Before issue EN4B or EX4B the WRITE ENABLE command must issued first. */
		if (cs752x_spansion_cmd(SFLASH_WRITE_ENABLE_OPCODE, 0, 0)) {
			break;
		}

		if (wait_flash_status (SFLASH_STATUS_WEL, SFLASH_STATUS_WEL, SFLASH_WEL_TIMEOUT)) {
			break;
		}

		if (cs752x_spansion_cmd(enable ? SFLASH_OPCODE_EN4B : SFLASH_OPCODE_EX4B, 0, 0)) {
			debug("SF: Micron flash set 4 bytes error.\n");
		}

		break;
        default:
                /* Spansion style */
                //flash->command[0] = OPCODE_BRWR;
                //flash->command[1] = enable << 7;
                //return spi_write(flash->spi, flash->command, 2);
                if (cs752x_spansion_cmd(SFLASH_OPCODE_BRWR, 0, enable << 7)) {
			debug("SF: SFlash set 4 bytes error.\n");
		}
        }

        tmp = read_flash_ctrl_reg(FLASH_TYPE);

        if(enable)
        	sflash_type = (tmp | SFLASH_4BYTE_ADDR);
	else
		sflash_type = (tmp & ~SFLASH_4BYTE_ADDR);

	write_flash_ctrl_reg(FLASH_TYPE, sflash_type);

	return 0;
}

struct spi_device_id *cs752x_jedec_probe(void)
{
        u32                     jedec, tmp;
        u16                     ext_jedec;
        struct flash_info       *info;
	unsigned long status;

	if (cs752x_spansion_cmd(SFLASH_READID_OPCODE, 0, 0)) {
		return NULL;
	}

	status = read_flash_ctrl_reg(FLASH_SF_DATA);

	jedec = ((status&0xff)<<16) | (status&0xff00) | ((status&0xff0000)>>16);

	//For G2 just can read 4 bytesdata only, So without ext_jedec
        //ext_jedec = id[3] << 8 | id[4];

        for (tmp = 0; tmp < ARRAY_SIZE(m25p_ids) - 1; tmp++) {

                info = (void *)m25p_ids[tmp].driver_data;

                if (info->jedec_id == jedec) {
                        //if (info->ext_id != 0 && info->ext_id != ext_jedec)
                        //        continue;
                        return &m25p_ids[tmp];
                }
        }

        debug("SF: unrecognized JEDEC id %06x\n", jedec);

        return NULL;
}

struct spi_flash *spi_flash_probe(unsigned int bus, unsigned int cs,
				  unsigned int max_hz, unsigned int spi_mode)
{
	struct spi_flash *flash;
	const struct spi_device_id *jid;
	struct flash_info               *info;
	u32 tmp;



	jid = cs752x_jedec_probe();


	if(!jid)
		return NULL;

	info = (void *)jid->driver_data;

	flash = malloc(sizeof(struct spi_flash));
	if (!flash) {
		debug("SF: Failed to allocate memory\n");
		return NULL;
	}

	/*
         * Atmel, SST and Intel/Numonyx serial flash tend to power
         * up with the software protection bits set
         */

        if (JEDEC_MFR(info->jedec_id) == CFI_MFR_ATMEL ||
            JEDEC_MFR(info->jedec_id) == CFI_MFR_INTEL ||
            JEDEC_MFR(info->jedec_id) == CFI_MFR_SST) {
              //  write_enable(flash);
              //  write_sr(flash, 0);
              debug("SF: Atmel, Intel or SST SFlash.\n");
        }

	flash->spi = NULL;
	flash->name = jid->name;

	sflash_sector_size = info->sector_size;
	/* For S25-FL512SAIFG1 page size is 512B, 64MB with RESET# pin*/
	if(info->jedec_id == 0x010220)
		info->page_size = 512;
	sflash_page_size = info->page_size;
	sflash_size = info->sector_size * info->n_sectors;
	/* prefer "small sector" erase if possible */
        if (info->flags & SECT_4K) {
        	debug("SF: Check OPCODE_BE_4K.\n");
                //flash->erase_opcode = OPCODE_BE_4K;
                //flash->mtd.erasesize = 4096;
        }

	flash->size = sflash_size;

	flash->write = spansion_write;
	flash->erase = spansion_erase;
	flash->read = spansion_read;

	/* enable 4-byte addressing if the device exceeds 16MiB */
        if (flash->size > 0x1000000) {
                debug("SF: 4 Bytes mode flash.\n");
		tmp = read_flash_ctrl_reg(FLASH_TYPE);
		write_flash_ctrl_reg(FLASH_TYPE, tmp | SFLASH_4BYTE_ADDR);
		sflash_type = (tmp | SFLASH_4BYTE_ADDR);

                cs752x_set_4byte(info->jedec_id, 1);
        }
        else
        {	/* use strapping setting */
		sflash_type = read_flash_ctrl_reg(FLASH_TYPE);
                //debug("SF: 3 Bytes mode flash.\n");
        }

	return flash;
}

void spi_flash_free(struct spi_flash *flash)
{
	free(flash);
}
