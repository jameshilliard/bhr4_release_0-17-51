################################################################################
#
# Cortina CS752X Linux driver
# Copyright (c) 2010 by Cortina Systems Incorporated.  
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#
# The full GNU General Public License is included in this distribution in
# the file called "COPYING".
#
# Contact Information:
# Mailing List <eric.wang@cortina-systems.com>
# Cortina Systems, Inc., 1376 Bordeaux Drive Sunnyvale, CA 94089
# http://www.cortina-systems.com/
#
################################################################################
EXTRA_CFLAGS += -I$(srctree)/drivers/net/ethernet/cs752x/src/include
EXTRA_CFLAGS += -I$(srctree)/drivers/net/ethernet/cs752x/src/fe/include
EXTRA_CFLAGS += -I$(srctree)/drivers/net/ethernet/cs752x/src/tm
EXTRA_CFLAGS += -I$(srctree)/drivers/net/ethernet/cs752x/src/qm

obj-$(CONFIG_CS752X) += cs752x_voq_cntr.o
obj-$(CONFIG_CS752X_PROC) += cs752x_proc.o cs752x_proc_qos.o
obj-$(CONFIG_CS75XX_HW_ACCEL_IPLIP) += cs752x_proc_iplip.o
