#include <config.h>
#include <common.h>
#include <command.h>
#include <malloc.h>

#include <asm/arch/registers.h>

#define OTP_ADDR 0xF5008000
#define OTP_SIZE 1024
#define OTP_TLV_MAGIC 0x3C
#define OTP_MAGIC_FIRST 0x5A
#define OTP_MAGIC_LAST 0xA5

#define T_VALID_START 1
#define T_VALID_END 31

/**
 *  Borrowed from common/cmd_nand.c
 */
static inline int str2long(char *p, ulong * num)
{
	char *endptr;

	*num = simple_strtoul(p, &endptr, 16);
	return (*p != '\0' && *endptr == '\0') ? 1 : 0;
}

/**
 * @return On success, the value of the offset of OTP is returned. On error, -1 is returned.
 */
static int read_one_byte(uint32_t offset)
{
	if (offset > OTP_SIZE)
		return -1;

	unsigned char *p = (unsigned char *)OTP_ADDR;
	return p[offset];
}

/**
 * @return On success, zero is returned. On error, non-zero value is returned.
 */
static int program_register(uint8_t sel, uint32_t addr, uint32_t max_addr)
{
	OTP_CTL_t otp_ctrl_inst;
	OTP_PROG_t otp_prog_inst;
	uint32_t *otpprog_reg, *otpctrl_reg;
	int try_cnt;

	memset(&otp_ctrl_inst, 0, sizeof(otp_ctrl_inst));
	memset(&otp_prog_inst, 0, sizeof(otp_prog_inst));

	otp_ctrl_inst.bf.CLKCNT = 0xd5;
	otp_ctrl_inst.bf.TZPCDECPROT_SPARE0 = 0;
	otp_ctrl_inst.bf.TZPCDECPROT_SPARE1 = 0;
	otp_ctrl_inst.bf.TZPCDECPROT_QMPB = 1;
	otp_ctrl_inst.bf.TZPCDECPROT_RTC = 0;
	otp_ctrl_inst.bf.AXIPREVENT = 0;
	otp_ctrl_inst.bf.AXIOPEN = 1;
	otp_ctrl_inst.bf.NP1 = 4;
	otp_ctrl_inst.bf.NP2 = 0;

	otp_prog_inst.bf.__ACCESS = 1;
	otp_prog_inst.bf.__SEL = sel;
	otp_prog_inst.bf.__LOCK = 0;
	otp_prog_inst.bf.__FAIL = 0;
	otp_prog_inst.bf.__DATA = 1;
	otp_prog_inst.bf.__MAXADDR = max_addr;
	otp_prog_inst.bf.__ADDR = addr;

	otpctrl_reg = (uint32_t *) OTP_CTL;
	*otpctrl_reg = otp_ctrl_inst.wrd;

	otpprog_reg = (uint32_t *) OTP_PROG;
	*otpprog_reg = otp_prog_inst.wrd;

	try_cnt = 0;
	do {
		udelay(1);
		otp_prog_inst.wrd = *otpprog_reg;
	} while (otp_prog_inst.bf.__ACCESS && ++try_cnt < 50);

	if (try_cnt >= 50)
		return 1;

	otp_prog_inst.wrd = *otpprog_reg;
	return otp_prog_inst.bf.__FAIL;
}

/**
 * @return On success, zero is returned. On error, non-zero value is returned.
 */

static int write_one_byte(uint32_t offset, uint32_t value)
{
	uint32_t i;
	uint8_t *otpImage;

	if (offset > 1024 || value > 0xff)
		return 1;

	otpImage = (uint8_t *) (OTP_ADDR);

	for (i = 0; i < 8; ++i) {
		if ((value & (0x01 << i)) == 0) {
			if (otpImage[offset] & (0x01 << i))
				return 0;
			continue;
		} else if (otpImage[offset] & (0x01 << i))
			continue;

		if (program_register(1, offset * 8 + i, 0) != 0) {
			return 0;
		}
	}

	if (read_one_byte(offset) == value)
		return 0;

	return 1;
}

/**
 * @return On success, the first writable address is returned; On error, zero is returned
 */
static uint32_t find_write_pos(void)
{
	uint32_t write_pos;
	unsigned char *otpImage;

	write_pos = OTP_SIZE - 2;
	otpImage = (unsigned char *)OTP_ADDR;

	for (; write_pos > 0; --write_pos) {
		if (otpImage[write_pos] != 0) {
			break;
		}
	}

	if ( write_pos >= OTP_SIZE - 2)
		return 0;
	return ++write_pos;
}

/**
 * @return On success, zero is returned. On error, the written bytes is returned;
 */
static int write_one_tlv(uint32_t offset, uint32_t addr, uint32_t len)
{
	uint32_t i, last_index;
	char *p;

	p = (char *)(addr + len - 1);
	last_index = OTP_SIZE - 1;

	for (i = 0; i < len && offset < last_index; ++offset, --p, ++i) {
		if (write_one_byte(offset, *p) != 0)
			return i + 1;
	}

	if (i == len)
		return 0;

	return i;
}

/**
 * @return On success, zero is returned. On error, non-zero value is returned;
 */
static int write_magic(void)
{
	if (read_one_byte(OTP_SIZE - 1) == OTP_MAGIC_LAST)
		return 0;

	if (write_one_byte(OTP_SIZE - 1, OTP_MAGIC_LAST) == 0)
		return 0;

	if (read_one_byte(0) == OTP_MAGIC_FIRST)
		return 0;

	if (write_one_byte(0, OTP_MAGIC_FIRST) == 0)
		return 0;

	return 1;
}

/**
 * @return On success, zero is returned. On error, non-zero value is returned;
 */
static int write_tlv(uint32_t addr, uint32_t len)
{
	unsigned char *otp;
	uint32_t write_pos, write_bytes, skip_len;
	char skip_tlv[4];

	write_pos = find_write_pos();
	if (write_pos == 0 || write_pos + len > (OTP_SIZE - 1))
		return 1;

	do {
		write_bytes = write_one_tlv(write_pos, addr, len);

		if (write_bytes == 0)
			break;

		skip_len = 0;
		do {
			/* Write a *SKIP* TLV to skip the errors.  */
			skip_len += write_bytes;
			write_pos += write_bytes;

			if (write_pos + skip_len + 4 > (OTP_SIZE - 1))
				return 1;

			skip_tlv[0] = OTP_TLV_MAGIC;
			skip_tlv[1] = 0x08;
			skip_tlv[2] = skip_len >> 8;
			skip_tlv[3] = skip_len & 0xFF;

			write_bytes = write_one_tlv(write_pos, &skip_tlv, 4);

			if (write_bytes == 0) {
				write_pos += 4;
				break;
			}
		} while (1);
	} while (1);

	return write_magic();
}

/*
 * @return 0
 */
static int read_tlv(uint32_t tlv_type)
{
	uint8_t i, j, type;
	uint8_t *ptrotp, *end;
	uint16_t len;

	ptrotp = (uint8_t *) (OTP_ADDR + OTP_SIZE - 1);
	end = (uint8_t *) (OTP_ADDR + 1);

	for (;;) {
		/*
		 * stop processing when not
		 * enough space is left to hold
		 * a TLV (4 bytes minimum)
		 */
		if (ptrotp - end < 4)
			break;

		if (ptrotp[0] != OTP_TLV_MAGIC) {
			ptrotp--;	/* skip to next byte and try again */
			continue;
		}

		type = ptrotp[-1];
		if (type < T_VALID_START || type > T_VALID_END) {
			ptrotp--;
			continue;
		}

		len = ptrotp[-2] | (ptrotp[-3] << 8);
		if (len > ptrotp - end) {
			ptrotp--;
			continue;
		}

		if (type != tlv_type) {
			ptrotp -= 4 + len;
			continue;
		}

		/* TLV header is OK */
		printf("type 0x%2x : 0x%02x 0x%02x 0x%02x 0x%02x \n",
		       tlv_type, ptrotp[0], ptrotp[-1], ptrotp[-2], ptrotp[-3]);

		ptrotp -= 4;
		for (i = 0; i < len;) {
			printf("            ");
			for (j = 0; j < 16 && i < len; ++i, ++j, --ptrotp) {
				printf("0x%02x ", *ptrotp);
			}
			printf("\n");
		}
	}

	return 0;
}

static int do_g2_otp_read(int argc, char *argv[])
{

	uint32_t offset;

	if (!str2long(argv[0], &offset))
		return 1;

	if (offset >= OTP_SIZE)
		return 1;

	printf("0x%x : 0x%x\n", offset, read_one_byte(offset));

	return 0;
}

/**
 * @return On success, zero is returned. On error, non-zero value is returned;
 */
static int do_g2_otp_write(int argc, char *argv[])
{
	uint32_t offset, value;

	if (!str2long(argv[0], &offset) || !str2long(argv[1], &value))
		return 1;

	if (offset >= OTP_SIZE)
		return 1;

	printf("Before: 0x%x\n", read_one_byte(offset));
	write_one_byte(offset, value);
	printf("After : 0x%x\n", read_one_byte(offset));

	return 0;
}

/*
 * @return On success, zero is returned. On error, non-zero value is returned;
 */
static int check_otp_valid(void)
{
	uint8_t *otp;

	otp = (uint8_t *) (OTP_ADDR);

	if (otp[OTP_SIZE - 1] == OTP_MAGIC_LAST)
		return 0;

	if (otp[0] == OTP_MAGIC_FIRST)
		return 0;

	if (otp[OTP_SIZE - 1] == 0 || otp[0] == 0)
		return 0;

	return 1;
}

static int do_g2_otp_read_tlv(int argc, char *argv[])
{
	uint32_t tlv_type;
	if (!str2long(argv[0], &tlv_type))
		return 1;

	if (tlv_type < T_VALID_START || tlv_type > T_VALID_END)
		return 1;

	read_tlv(tlv_type);

	return 0;
}

static int do_g2_otp_write_tlv(int argc, char *argv[])
{
	uint32_t addr, len;

	if (!str2long(argv[0], &addr) || !str2long(argv[1], &len))
		return 1;

	return write_tlv(addr, len);
}

static int do_g2_otp_lock(void)
{
	return program_register(6, 0, 0);
}

struct otp_cmd {
	const char *cmd;
	int argc;
	int (*cmd_process) (int argc, char *argv[]);
};

static int do_g2_otp(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{

	struct otp_cmd cmds[] = {
		{"read", 3, do_g2_otp_read},
		{"write", 4, do_g2_otp_write},
		{"readtlv", 3, do_g2_otp_read_tlv},
		{"writetlv", 4, do_g2_otp_write_tlv},
		{"lock", 2, do_g2_otp_lock},
	};

	struct otp_cmd *first, *last;

	first = cmds;
	last = cmds + ARRAY_SIZE(cmds);

	for (; first != last; ++first) {
		if (strcmp(first->cmd, argv[1]) == 0 && argc == first->argc)
			return first->cmd_process(argc - 2, argv + 2);
	}

	cmd_usage(cmdtp);
	return 1;
}

U_BOOT_CMD(otp, 4, 2, do_g2_otp,
	   "One-Time-Programmable sub-system",
	   "otp read offset \n"
	   " - read the value of 'offset' byte from the OTP beginning\n"
	   "otp write offset value\n"
	   " - write 'offset' byte with the value from the OTP beginning\n"
	   "otp writetlv address len\n"
	   " - write the OTP acccording to the contnet of address\n"
	   "otp readtlv tlv_type\n"
	   " - read tlvs in the OTP and its tlv type is equal to tlv_type\n"
	   "otp lock\n"
	   " - lock the OTP. The OTP is readonly\n"
	  );
