/*
 * Copyright (c) Cortina-Systems Limited 2010.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __CS_MCAST_H__
#define __CS_MCAST_H__

#include <mach/cs_types.h>

#define CS_E_MCAST_ADDR_EXISTS		1
#define CS_E_NONE			CS_OK	/* Operation completed successfully */
#define CS_E_INIT			-1	/* Module is not initialized */
#define CS_E_MCAST_ADDR_ADD_FAIL	-2
#define CS_E_MCAST_ADDR_DELETE_FAIL	-3

typedef enum {
	CS_MCAST_PORT_REPLICATION = 1,
	CS_MCAST_ARBITRARY_REPLICATION = 2,
} cs_mcast_mode;

typedef enum {
	CS_VLAN_TRANSPARENT,
	CS_VLAN_POP,
	CS_VLAN_SWAP,
} cs_vlan_cmd_t;

typedef enum {
	CS_MCAST_EXCLUDE = 0,
	CS_MCAST_INCLUDE,
} cs_mcast_filter_mode;

typedef enum {
	CS_IPV4 = 0,
	CS_IPV6 = 1
} cs_ip_afi_t;

typedef struct cs_ip_address {
	cs_ip_afi_t		afi;
	union {
		cs_uint32	ipv4_addr;
		cs_uint32	ipv6_addr[4];
	};
	cs_uint8		addr_len;	/* length in bits */
} cs_ip_address_t;

typedef struct cs_mcast_vlan_cfg {
	/* On LynxE named as mcast_vlan_id.
	 * vlan tag received from the network on the PON port.
	 */
	cs_uint16		in_vlan_id;

	cs_vlan_cmd_t		cs_vlan_cmd;

	/* On LynxE named as uni_vlan_id.
	 * fixed vlan_id used for the SWAP command when sending a
	 * packet to the UNI port.
	 */
	cs_uint16		out_vlan_id;
} cs_mcast_vlan_cfg_t;

typedef struct cs_mcast_entry_s {
	cs_port_id_t		sub_port;
	cs_uint16		mcast_vlan;
	cs_uint8		mac_addr[6];
} cs_mcast_entry_t;

#define CS_MCAST_MAX_ADDRESS	32
typedef struct cs_mcast_address_s {
	cs_ip_afi_t		afi;
	cs_uint16		sub_port;
	cs_uint16		mcast_vlan;

	/* if IPv4, only first four bytes are used */
	cs_ip_address_t		grp_addr;

	/* if CS_MCAST_EXCLUDE and src_num = 0, equal to IGMPv2 entry.
	 * if CS_MCAST_INCLUDE and src_num = 0, no entry will be set to HW.
	 */
	cs_mcast_filter_mode	mode;

	/* Unicast MAC address of joining host. Used by HW in arbitrary replication. */
	cs_uint8		dest_mac[6];

	cs_uint16		src_num;
	cs_ip_address_t		src_list[CS_MCAST_MAX_ADDRESS];
} cs_mcast_address_t;


#ifndef CONFIG_CS75XX_DATA_PLANE_MULTICAST

#define cs_l2_mcast_address_add(d, p, e)		({ CS_OK; })
#define cs_l2_mcast_address_delete(d, p, e)		({ CS_OK; })
#define cs_l2_mcast_port_address_num_get(d, p, n)	({ CS_OK; })
#define cs_l2_mcast_port_address_entry_get(d, p, n, e)	({ CS_OK; })
#define cs_l2_mcast_port_address_clear(d, p)		({ CS_OK; })
#define cs_l2_mcast_address_all_clear(d)		({ CS_OK; })
#define cs_l2_mcast_mode_set(d, m)			({ CS_OK; })
#define cs_12_mcast_wan_port_id_set(d, p)		({ CS_OK; })
#define cs_12_mcast_wan_port_id_get(d, p)		({ CS_OK; })

#else /* CONFIG_CS75XX_DATA_PLANE_MULTICAST */


/* Add IP multicast forward entry to HW, it can support IGMPv3/MLDv2.
 * To add multiple ports the function must be called multiple times.
 */
cs_status cs_l2_mcast_address_add(cs_dev_id_t dev_id, cs_port_id_t port_id, cs_mcast_address_t *entry);

/* Delete one IP multicast entry on the port.
 * To delete multiple ports the function must be called multiple times.
 */
cs_status cs_l2_mcast_address_delete(cs_dev_id_t dev_id, cs_port_id_t port_id, cs_mcast_address_t *entry);

/* Retrieve all IP multicast entries on one port. */
cs_status cs_l2_mcast_port_address_get(cs_dev_id_t dev_id, cs_port_id_t port_id, cs_uint16 *num, cs_mcast_address_t **entry_pp);

/* Delete all multicast entries on one port. */
cs_status cs_l2_mcast_port_address_clear(cs_dev_id_t dev_id, cs_port_id_t port_id);

/* Clear all the HW accelerated multicast flows. */
cs_status cs_l2_mcast_address_all_clear(cs_dev_id_t dev_id);

/* Set CS_MCAST_PORT_REPLICATION or CS_MCAST_ARBITRARY_REPLICATION.
 * Default is CS_MCAST_PORT_REPLICATION.
 *
 * When mode changes, the application must explicitly delete all the HW
 * accelerated multicast flows from the previous mode.
 */
cs_status cs_l2_mcast_mode_set(cs_dev_id_t dev_id, cs_mcast_mode mode);

cs_status cs_12_mcast_wan_port_id_set(cs_dev_id_t dev_id, cs_port_id_t port_id);

cs_port_id_t cs_12_mcast_wan_port_id_get(cs_dev_id_t dev_id);

int cs_mcast_init(void);

#endif /* CONFIG_CS75XX_DATA_PLANE_MULTICAST */

#endif /* __CS_MCAST_H__ */

