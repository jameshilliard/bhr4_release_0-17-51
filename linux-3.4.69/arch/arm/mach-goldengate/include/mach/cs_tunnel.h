/*
 * Copyright (c) Cortina-Systems Limited 2013.  All rights reserved.
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
/*
 * cs_iplip.h
 *
 *
 * This header file defines the data structures and APIs for CS Tunnel 
 * Acceleration.
 * Currently supported:
 * 	IPv6 over PPP over L2TP over IPv4 over PPPoE (IPLIP) Kernel Module.
 */
#ifndef __CS_TUNNEL_H__
#define __CS_TUNNEL_H__

#include "cs_types.h"

//typedef unsigned short cs_uint16;
//typedef unsigned short cs_dev_id_t;
//typedef unsigned short cs_port_id_t;
typedef unsigned int cs_tunnel_id_t;
typedef unsigned short cs_session_id_t;

typedef struct cs_pppoe_hdr_s {
	unsigned char 	type:4; /* Type = 0x1 */
	unsigned char 	ver:4;  /* Version = 0x1 */
	unsigned char 	code;  	/* Code = 0x00 */
	unsigned short 	sid;  	/* Session ID */
	unsigned short	len;	/* Size of data in bytes */
}  __attribute__((packed)) cs_pppoe_hdr_t;

typedef unsigned short cs_ppp2_pro_t;

typedef struct cs_ppp_hdr_s {
	unsigned char 	addr;  /* Address = 0xff */
	unsigned char 	ctrl;  /* Control = 0x03 */
	unsigned short 	pro;  /* Protocol = 0x0057 */
}  __attribute__((packed)) cs_ppp_hdr_t;

typedef enum {
	CS_L2TP_TYPE_1 = 1,	/* L=0, O=0 */
	CS_L2TP_TYPE_2 = 2,	/* L=1, O=0 */
	CS_L2TP_TYPE_3 = 3,	/* L=0, O=1, Offset=0 */
	CS_L2TP_TYPE_4 = 4	/* L=1, O=1, Offset=0 */
} cs_l2tp_type_t; 

typedef union {
	unsigned short  u16;
	struct cs_l2tp_ver_s {
		unsigned short version : 4; /* bits 3:0 */
		unsigned short rsvd1   : 4; /* bits 7:4 */
		unsigned short p       : 1; /* bit 8 */
		unsigned short o       : 1; /* bit 9 */
		unsigned short rsvd2   : 1; /* bit 10 */
		unsigned short s       : 1; /* bit 11 */
		unsigned short rsvd3   : 2; /* bits 13:12 */
		unsigned short l       : 1; /* bit 14 */
		unsigned short t       : 1; /* bit 15 */
	} bits;
}  __attribute__((packed)) cs_l2tp_ver_t;

typedef struct cs_l2tp_hdr1_s {
	unsigned short  ver;  /* Version and flags */
	unsigned short 	tid;  /* Tunnel ID */
	unsigned short 	sid;  /* Session ID */
}  __attribute__((packed)) cs_l2tp_hdr1_t;

typedef struct cs_l2tp_hdr2_s {
	unsigned short  ver;  /* Version and flags */
	unsigned short 	len;  /* Optional length */
	unsigned short 	tid;  /* Tunnel ID */
	unsigned short 	sid;  /* Session ID */
}  __attribute__((packed)) cs_l2tp_hdr2_t;

typedef struct cs_l2tp_hdr3_s {
	unsigned short  ver;  /* Version and flags */
	unsigned short 	tid;  /* Tunnel ID */
	unsigned short 	sid;  /* Session ID */
	unsigned short 	offset;  /* Optional offset */
}  __attribute__((packed)) cs_l2tp_hdr3_t;

typedef struct cs_l2tp_hdr4_s {
	unsigned short  ver;  /* Version and flags */
	unsigned short 	len;  /* Optional length */
	unsigned short 	tid;  /* Tunnel ID */
	unsigned short 	sid;  /* Session ID */
	unsigned short 	offset;  /* Optional offset */
}  __attribute__((packed)) cs_l2tp_hdr4_t;

typedef cs_l2tp_hdr2_t cs_l2tp_hdr_t;


typedef enum {
	CS_IPV4 = 0,
	CS_IPV6 = 1
} cs_ip_afi; /* (to be defined newly if doesn't exist) */

typedef enum {
	CS_L2TP = 1,
	CS_GRE = 2,
	CS_GRE_IPSEC = 3
} cs_tunnel_type_t; 

typedef struct cs_vlan {
	unsigned short tpid;
	unsigned short vid:12;
	unsigned short dei:1;
	unsigned short cos:3;
} cs_vlan_t; 

typedef struct cs_ip_address_s {
	//unsigned char  is_wlan;    /* 0: the IP belongs to LAN interface
	//			   	1: it belongs to WLAN interface */
	//cs_vlan_t vlan;
	cs_ip_afi afi; /* address family identifier */
	union {
		unsigned int ipv4_addr;
		unsigned int ipv6_addr[4];
	};
	unsigned char addr_len; /* length in bits */
} cs_ip_address_t; 

typedef struct cs_pppoe_port_cfg_s {
	cs_uint8_t src_mac[6];
	cs_uint8_t dest_mac[6];
	cs_uint16_t pppoe_session_id;
	cs_uint32_t vlan_tag; /* 0: untagged, 1~4094: normal tag */
	cs_port_id_t tx_phy_port; /* Port number corresponding to 
                             		WAN Port on which PPPoE 
                             		session is established */
} cs_pppoe_port_cfg_t;

typedef struct cs_l2tp_tunnel_cfg_s {
	unsigned short  ver;  /* Version and flags */
	unsigned short 	len;  /* Optional Length */ /* need caculate */
	unsigned short 	tid;  /* Tunnel ID */
	unsigned short	ipv4_id;  /* ID of IPv4 hdr */ /* increasement */
	unsigned short	dest_port; /* UDP port of L2TP hdr */
	unsigned short	src_port; /* UDP port of L2TP hdr */
} cs_l2tp_tunnel_cfg_t;

typedef struct cs_tunnel_cfg_s {
	cs_tunnel_type_t type;	/* tunnel type */
	cs_ip_address_t dest_addr; /* tunnel destination IP */
	cs_ip_address_t src_addr; /* tunnel src IP */
	cs_port_id_t tx_port;    
	/* out bound port - 
	  can be physical port - such as WAN port
	  or logical ports such as PPPoE ports created by data   
	  plane and returned back to the caller 
	*/
	union {
		cs_l2tp_tunnel_cfg_t l2tp;
	};
} cs_tunnel_cfg_t;


/* exported APIs */
cs_status
cs_pppoe_port_add(
		cs_dev_id_t		device_id,
 		cs_port_id_t		port_id,
		cs_port_id_t		pppoe_port_id
		);

cs_status
cs_pppoe_port_delete(
		cs_dev_id_t		device_id,
		cs_port_id_t		pppoe_port_id
		);

cs_status
cs_pppoe_port_encap_set(
		cs_dev_id_t		device_id,
		cs_port_id_t		pppoe_port_id,
		cs_pppoe_port_cfg_t	*p_cfg
		);

cs_status
cs_pppoe_port_config_get(
		cs_dev_id_t		device_id,
		cs_port_id_t		pppoe_port_id,
		cs_pppoe_port_cfg_t	*p_cfg
		);

cs_status 
cs_tunnel_add(
		cs_dev_id_t		device_id,
		cs_tunnel_cfg_t		*p_tunnel_cfg,
		cs_tunnel_id_t		*p_tunnel_id
		);

cs_status 
cs_tunnel_delete(
		cs_dev_id_t		device_id,
		cs_tunnel_cfg_t		*p_tunnel_cfg
		);

cs_status 
cs_tunnel_delete_by_idx(
		cs_dev_id_t		device_id,
		cs_tunnel_id_t		tunnel_id
		);

cs_status 
cs_tunnel_get(
		cs_dev_id_t		device_id,
		cs_tunnel_id_t		tunnel_id,
		cs_tunnel_cfg_t		*p_tunnel_cfg
		);

cs_status 
cs_l2tp_session_add(
		cs_dev_id_t		device_id,
		cs_tunnel_id_t		tunnel_id,
		cs_session_id_t		session_id
		);

cs_status 
cs_l2tp_session_delete(
		cs_dev_id_t		device_id,
		cs_tunnel_id_t		tunnel_id,
		cs_session_id_t		session_id
		);

cs_status 
cs_l2tp_session_get(
		cs_dev_id_t		device_id,
		cs_tunnel_id_t		tunnel_id,
		cs_session_id_t		session_id,
		cs_boolean_t		*is_present
		);

cs_status 
cs_ipv6_over_l2tp_add(
		cs_dev_id_t		device_id,
		cs_tunnel_id_t		tunnel_id,
		cs_session_id_t		session_id,
		cs_ip_address_t		*ipv6_prefix
		);

cs_status 
cs_ipv6_over_l2tp_delete(
		cs_dev_id_t		device_id,
   		cs_tunnel_id_t		tunnel_id,
		cs_session_id_t		session_id,
		cs_ip_address_t		*ipv6_prefix
		);

cs_status 
cs_ipv6_over_l2tp_getnext(
		cs_dev_id_t		device_id,
   		cs_tunnel_id_t		tunnel_id,
		cs_session_id_t		session_id,
		cs_ip_address_t		*ipv6_prefix
		);
typedef struct {
        unsigned int sub_cmd; /* refer to cs_tunnel_iplip_ioctl_sub_cmd_e */
        /* parameters for commands */
        union {
                /* cs_pppoe_port_add (cs_dev_id_t device_id, cs_port_id_t port_id, cs_port_id_t pppoe_port_id) */
                /* cs_pppoe_port_delete (cs_dev_id_t device_id, cs_port_id_t pppoe_port_id) */
                struct cs_tunnel_iplip_pppoe_port_add_delete_param {
                        unsigned short device_id;
                        unsigned short port_id;
                        unsigned short pppoe_port_id;
                } pppoe_port_add_delete_param;

                /*cs_pppoe_port_encap_set (cs_dev_id_t device_id, cs_port_id_t pppoe_port_id, cs_pppoe_port_cfg_t *p_cfg)
                  cs_pppoe_port_encap_get (cs_dev_id_t device_id, cs_port_id_t pppoe_port_id, cs_pppoe_port_cfg_t *p_cfg)*/
                struct cs_tunnel_iplip_pppoe_port_encap_set_get_param {
                        unsigned short device_id;
                        unsigned short pppoe_port_id;
                        cs_pppoe_port_cfg_t pppoe_port_cfg;
                } pppoe_port_encap_param;

                /*cs_tunnel_add(cs_dev_id_t device_id, cs_tunnel_cfg_t *p_tunnel_cfg, cs_tunnel_id_t *p_tunnel_id)
                  cs_tunnel_delete(cs_dev_id_t device_id, cs_tunnel_cfg_t *p_tunnel_cfg)
                  cs_tunnel_delete_by_idx (cs_dev_id_t device_id, cs_tunnel_id_t tunnel_id)
                  cs_tunnel_get(cs_dev_id_t device_id, cs_tunnel_id_t tunnel_id, cs_tunnel_cfg_t *p_tunnel_cfg) */
                 struct cs_tunnel_iplip_tunnel_param {
                        unsigned short device_id;
                        cs_tunnel_id_t tunnel_id;
                        cs_tunnel_cfg_t tunnel_cfg;
                } tunnel_param;

                /*cs_l2tp_session_add(cs_dev_id_t device_id, cs_tunnel_id_t tunnel_id, uint16 session_id)
                  cs_l2tp_session_delete(cs_dev_id_t device_id, cs_tunnel_id_t tunnel_id, uint16 session_id)
                  cs_l2tp_session_get (cs_dev_id_t device_id, cs_tunnel_id_t tunnel_id, uint16 session_id, u8 *is_present)*/
                struct cs_l2tp_session_param {
                        unsigned short device_id;
                        cs_tunnel_id_t tunnel_id;
                        cs_session_id_t session_id;
                        unsigned char is_present;
                } l2tp_session_param;

                /*cs_ipv6_over_l2tp_add(cs_dev_id_t device_id, cs_tunnel_id_t tunnel_id, cs_session_id_t session_id, cs_ip_address *ipv6_prefix)
                  cs_ipv6_over_l2tp_delete (cs_dev_id_t device_id, cs_tunnel_id_t tunnel_id, cs_session_id_t session_id, cs_ip_address *ipv6_prefix)
                  cs_ipv6_over_l2tp_getnext (cs_dev_id_t device_id, cs_tunnel_id_t tunnel_id, cs_session_id_t session_id, cs_ip_address *ipv6_prefix)*/
                struct cs_ipv6_over_l2tp_param {
                        unsigned short device_id;
                        cs_tunnel_id_t tunnel_id;
                        cs_session_id_t session_id;
                        cs_ip_address_t ipv6_prefix;
                } ipv6_over_l2tp_param;
        };

        /* return value from TUNNEL IPLIP APIs */
        int ret;
} cs_tunnel_iplip_api_entry_t;


#endif /* __CS_TUNNEL_H__ */
