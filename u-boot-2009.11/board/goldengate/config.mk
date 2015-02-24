ifeq ($(CS_PLATFORM_TYPE),FPGA)
  TEXT_BASE = 0xE4000000
else
  TEXT_BASE = 0x04000000
endif

LDSCRIPT := $(SRCTREE)/board/goldengate/u-boot.lds

