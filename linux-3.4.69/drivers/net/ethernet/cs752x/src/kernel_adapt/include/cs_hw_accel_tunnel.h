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
#ifndef __CS_HW_ACCEL_TUNNEL_H__
#define __CS_HW_ACCEL_TUNNEL_H__

#include <mach/cs_tunnel.h>

#define CS_IPLIP_TBL_SIZE	64
#define CS_TUNNEL_TBL_SIZE	64
#define CS_INVALID_TUNNEL_ID	0xFFFFFFFF

struct cs_iplip_hdr1_s {
	struct ethhdr		ethh;
	cs_pppoe_hdr_t		pppoeh;
	cs_ppp2_pro_t		ppp2h;
	struct iphdr		iph;
	struct udphdr		udph;
	cs_l2tp_hdr1_t		l2tph;
	cs_ppp_hdr_t		ppph;
} __attribute__((packed));

struct cs_iplip_hdr2_s {
	struct ethhdr		ethh;
	cs_pppoe_hdr_t		pppoeh;
	cs_ppp2_pro_t		ppp2h;
	struct iphdr		iph;
	struct udphdr		udph;
	cs_l2tp_hdr2_t		l2tph;
	cs_ppp_hdr_t		ppph;
} __attribute__((packed));

struct cs_iplip_hdr3_s {
	struct ethhdr		ethh;
	cs_pppoe_hdr_t		pppoeh;
	cs_ppp2_pro_t		ppp2h;
	struct iphdr		iph;
	struct udphdr		udph;
	cs_l2tp_hdr3_t		l2tph;
	cs_ppp_hdr_t		ppph;
} __attribute__((packed));

struct cs_iplip_hdr4_s {
	struct ethhdr		ethh;
	cs_pppoe_hdr_t		pppoeh;
	cs_ppp2_pro_t		ppp2h;
	struct iphdr		iph;
	struct udphdr		udph;
	cs_l2tp_hdr4_t		l2tph;
	cs_ppp_hdr_t		ppph;
} __attribute__((packed));

typedef struct iplip_entry1_s {
	unsigned int		crc32;		/* crc32 of iplip_hdr */
	union {
		struct cs_iplip_hdr1_s iplip_hdr;
		unsigned char	      iplip_octet[sizeof(struct cs_iplip_hdr1_s)];
	};
	unsigned char		valid;	/*0: invalid 1: valid*/
	unsigned char		dir;	/* 0: unknown, 1: upstream, 2: downstream */
} __attribute__((packed)) cs_iplip_entry1_t;

typedef struct iplip_entry2_s {
	unsigned int		crc32;		/* crc32 of iplip_hdr */
	union {
		struct cs_iplip_hdr2_s iplip_hdr;
		unsigned char	      iplip_octet[sizeof(struct cs_iplip_hdr2_s)];
	};
	unsigned char		valid;	/*0: invalid 1: valid*/
	unsigned char		dir;	/* 0: unknown, 1: upstream, 2: downstream */
} __attribute__((packed)) cs_iplip_entry2_t;

typedef struct iplip_entry3_s {
	unsigned int		crc32;		/* crc32 of iplip_hdr */
	union {
		struct cs_iplip_hdr3_s iplip_hdr;
		unsigned char	      iplip_octet[sizeof(struct cs_iplip_hdr3_s)];
	};
	unsigned char		valid;	/*0: invalid 1: valid*/
	unsigned char		dir;	/* 0: unknown, 1: upstream, 2: downstream */
} __attribute__((packed)) cs_iplip_entry3_t;

typedef struct iplip_entry4_s {
	unsigned int		crc32;		/* crc32 of iplip_hdr */
	union {
		struct cs_iplip_hdr4_s iplip_hdr;
		unsigned char	      iplip_octet[sizeof(struct cs_iplip_hdr4_s)];
	};
	unsigned char		valid;	/*0: invalid 1: valid*/
	unsigned char		dir;	/* 0: unknown, 1: upstream, 2: downstream */
} __attribute__((packed)) cs_iplip_entry4_t;

typedef cs_iplip_entry2_t cs_iplip_entry_t;

/* structures for IPC */
#define CS_IPLIP_IPC_CLNT_ID		0x5

/* CPU send to PE */
#define CS_IPLIP_IPC_PE_RESET		0x01
#define CS_IPLIP_IPC_PE_STOP		0x02
#define CS_IPLIP_IPC_PE_SET_ENTRY	0x03
#define CS_IPLIP_IPC_PE_DEL_ENTRY	0x04
#define CS_IPLIP_IPC_PE_DUMP_TBL	0x05
#define CS_IPLIP_IPC_PE_ECHO		0x06
#define CS_IPLIP_IPC_PE_MIB_EN		0x07

#define CS_IPLIP_IPC_PE_MAX		CS_IPLIP_IPC_PE_MIB_EN

/* PE send to CPU */
#define CS_IPLIP_IPC_PE_RESET_ACK	0x11
#define CS_IPLIP_IPC_PE_STOP_ACK	0x12
#define CS_IPLIP_IPC_PE_SET_ACK		0x13
#define CS_IPLIP_IPC_PE_DEL_ACK		0x14
#define CS_IPLIP_IPC_PE_ECHO_ACK	0x15

#define CS_IPLIP_IPC_PE_MAX_ACK		CS_IPLIP_IPC_PE_ECHO_ACK

/* CS_IPLIP_IPC_PE_INIT */
/* CS_IPLIP_IPC_PE_STOP */
/* CS_IPLIP_IPC_PE_DUMP_TBL */
/* No data */

/* CS_IPLIP_IPC_PE_SET_ENTRY */
typedef struct cs_iplip_ipc_msg_set1_s {
	unsigned char		idx;
	unsigned char		l2tp_type; /* cs_l2tp_type_t */
	unsigned char		resvd[2];
	cs_iplip_entry1_t	iplip_entry;
} __attribute__ ((__packed__)) cs_iplip_ipc_msg_set1_t;

typedef struct cs_iplip_ipc_msg_set2_s {
	unsigned char		idx;
	unsigned char		l2tp_type; /* cs_l2tp_type_t */
	unsigned char		resvd[2];
	cs_iplip_entry2_t	iplip_entry;
} __attribute__ ((__packed__)) cs_iplip_ipc_msg_set2_t;

typedef struct cs_iplip_ipc_msg_set3_s {
	unsigned char		idx;
	unsigned char		l2tp_type; /* cs_l2tp_type_t */
	unsigned char		resvd[2];
	cs_iplip_entry3_t	iplip_entry;
} __attribute__ ((__packed__)) cs_iplip_ipc_msg_set3_t;

typedef struct cs_iplip_ipc_msg_set4_s {
	unsigned char		idx;
	unsigned char		l2tp_type; /* cs_l2tp_type_t */
	unsigned char		resvd[2];
	cs_iplip_entry4_t	iplip_entry;
} __attribute__ ((__packed__)) cs_iplip_ipc_msg_set4_t;


/* CS_IPLIP_IPC_PE_DEL_ENTRY */
typedef struct cs_iplip_ipc_msg_del_s {
	unsigned char		idx;
	unsigned char		resvd[3];
} __attribute__ ((__packed__)) cs_iplip_ipc_msg_del_t;

/* CS_IPLIP_IPC_PE_MIB_EN_ENTRY */
typedef struct cs_iplip_ipc_msg_en_s {
	unsigned char		enbl;	/* 0: disable, 1: enable */
	unsigned char		resvd[3];
} __attribute__ ((__packed__)) cs_iplip_ipc_msg_en_t;


#define CS_TUNNEL_TID(pppoe_port_id, index) ((pppoe_port_id << 16) | index & 0xFFFF)
#define CS_TUNNEL_GET_PID_FROM_TID(tunnel_id) (tunnel_id >> 16)
#define CS_TUNNEL_GET_IDX_FROM_TID(tunnel_id) (tunnel_id & 0xFFFF)

/* internal structure */
typedef struct cs_ip_address_entry_s {
	cs_ip_address_t		ip;
	struct cs_ip_address_entry_s	*prev;
	struct cs_ip_address_entry_s	*next;
	struct cs_l2tp_session_entry_s	*session;	/* parent */

	/* LAN --> PE hash information */
	unsigned int voq_pol_idx; 
	unsigned int fwd_rslt_idx; 
	unsigned short fwd_hash_idx; 
} cs_ip_address_entry_t;

typedef struct cs_l2tp_session_entry_s {
	cs_session_id_t		session_id;
	cs_ip_address_entry_t	*ip_h;	/* ip list head */
	unsigned int		ip_cnt;	/* ip entry counter */
	unsigned char		iplip_idx;	/* index in IPLIP table */
	struct cs_l2tp_session_entry_s	*prev;
	struct cs_l2tp_session_entry_s	*next;
	struct cs_tunnel_entry_s	*tunnel;	/* parent */
} cs_l2tp_session_entry_t;


typedef struct cs_tunnel_entry_s {
	cs_tunnel_id_t		tunnel_id;
	cs_tunnel_cfg_t 	tunnel_cfg;
	cs_l2tp_session_entry_t	*se_h;	/* session list head */
	unsigned char		se_cnt;	/* session counter */
	unsigned char		dir;	/* 0: unknown, 1: upstream, 2: downstream */
	unsigned int		ti_cnt;	/* total IPv6 prefix counter */
	struct cs_tunnel_entry_s	*prev;
	struct cs_tunnel_entry_s	*next;
	struct cs_pppoe_port_entry_s	*pppoe_port;	/* parent */

	/* hash information */
	/* 0: PE --> WAN
	   1: WAN --> PE
	   2: PE1 --> CPU */
	unsigned int voq_pol_idx[3];
	unsigned int vlan_rslt_idx[3];
	unsigned int fwd_rslt_idx[3];
	unsigned short fwd_hash_idx[3];
} cs_tunnel_entry_t;

typedef struct cs_pppoe_port_entry_s {
	cs_dev_id_t	device_id;
	cs_port_id_t	port_id;
	cs_port_id_t	pppoe_port_id;
	int		pppoe_ifindex;	/* identify tunnel direction */
	int		ppp_ifindex;	/* identify IPv6 routing destination */
	cs_pppoe_port_cfg_t	p_cfg;
	cs_tunnel_entry_t	*tu_h;	/* tunnel list head */
	unsigned char		tu_cnt;		/* tunnel counter */
	//unsigned short		tu_idx;		/* increasing index to generate unique tunnel_id */
	struct cs_pppoe_port_entry_s *next;
} cs_pppoe_port_entry_t;

typedef struct cs_pppoe_port_list_s {
	cs_pppoe_port_entry_t *port_list_hdr;
	unsigned int	pppoe_port_cnt;		/* pppoe_port counter */
	unsigned char	tt_cnt;			/* total tunnel counter */
	unsigned char	ts_cnt;			/* total session counter */
	cs_tunnel_entry_t *tu_list[CS_TUNNEL_TBL_SIZE]; /* tunnel list */
} cs_pppoe_port_list_t;

typedef struct cs_iplip_tbl_s {
	cs_iplip_entry_t entry[CS_IPLIP_TBL_SIZE];
	cs_l2tp_session_entry_t	*session_ptr[CS_IPLIP_TBL_SIZE];
} cs_iplip_tbl_t;

typedef union {
	unsigned char octet[ETH_ALEN];
	struct {
		unsigned char index;
		unsigned char rsvd;
		unsigned int crc32;
	} __attribute__ ((__packed__));
} cs_iplip_mac_t;

/* exported APIs */
int cs_is_ppp_tunnel_traffic(struct sk_buff *skb);
void cs_lan2pe_hash_add(struct sk_buff *skb);
int cs_iplip_ppp_ifindex_set(int pppoe_port, int ppp_ifindex);
int cs_iplip_pppoe_ifindex_set(int pppoe_port, int pppoe_ifindex);




/* IPC handlers */

int
cs_iplip_ipc_send_reset(void);

int
cs_iplip_ipc_send_stop(void);

int
cs_iplip_ipc_send_set_entry(
		unsigned char		idx,
		cs_iplip_entry_t	*iplip_entry
		);

int
cs_iplip_ipc_send_del_entry(
		unsigned char		idx
		);

int
cs_iplip_ipc_send_dump(void);

int
cs_iplip_ipc_send_echo(void);

int
cs_iplip_ipc_send_mib_en(
		unsigned char		enbl
		);


/* module API */
void cs_hw_accel_tunnel_init(void);
void cs_hw_accel_tunnel_exit(void);


#endif /* __CS_HW_ACCEL_TUNNEL_H__ */
