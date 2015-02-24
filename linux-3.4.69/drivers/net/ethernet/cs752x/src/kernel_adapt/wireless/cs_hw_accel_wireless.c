#include <linux/list.h>		/* list_head structure */
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/jiffies.h>

#include <mach/cs_types.h>
#include "cs_core_logic.h"
#include "cs_hw_accel_manager.h"
#include "cs_fe_mc.h"
#include "cs_core_hmu.h"
#include "cs_core_vtable.h"
#include "cs752x_eth.h"
#include "cs_wfo_csme.h"
#include <mach/cs75xx_pni.h>
#include <mach/cs75xx_ipc_wfo.h>


#ifdef CONFIG_CS752X_PROC
#include "cs752x_proc.h"
extern u32 cs_acp_enable;
extern u32 cs_adapt_debug;
extern u32 cs_ni_fastbridge;
#define DBG(x)	if (cs_adapt_debug & CS752X_ADAPT_WIRELESS) x

#else
#define DBG(x)	{}
#endif /* CONFIG_CS752X_PROC */

enum CS_WIRELESS_MEM_STATUS {
	CS_WIRELESS_FREE		= 0,
	CS_WIRELESS_ALLOCATED
};

#define CS_HW_ACCEL_WIRELESS_INTERFACE_MAX 4
#define CS_WIRELESS_MAX_PRIO 4



typedef struct cs_wireless_interface_s {	
	u16  status;
	struct net_device *dev;
	int (*cb_start_xmit)(struct sk_buff *skb, struct net_device *dev);
	int (*cb_prio_2_ac)(int prio);
	int (*cb_ac_2_prio)(int ac);
	u16  tx_voq_base;
	u16  egress_port_id;
	uint16_t rx_default_hash_idx[CS_WIRELESS_MAX_PRIO];
} cs_wireless_interfac_t;



static cs_wireless_interfac_t cs_wireless_list[CS_HW_ACCEL_WIRELESS_INTERFACE_MAX];

static spinlock_t cs_wireless_lock;

static int cs_hw_accel_create_rx_default_fwd_hash(int vap_idx)
{
	fe_fwd_result_entry_t fwdrslt_entry;
	fe_voq_pol_entry_t voqpol_entry;
	fe_sw_hash_t key;
	u64 fwd_hm_flag;
	u32 crc32;
	u16 hash_index, crc16;
	unsigned int fwdrslt_idx, voqpol_idx;
	int  i, j;

	memset(&fwdrslt_entry, 0x0, sizeof(fwdrslt_entry));
	memset(&voqpol_entry, 0x0, sizeof(voqpol_entry));
	memset(&key, 0x0, sizeof(key));

	//for (i = 0; i < 2; i++) {
		for (j = 0; j < CS_WIRELESS_MAX_PRIO; j++) {
			if (cs_core_vtable_get_hashmask_flag_from_apptype(
				CORE_FWD_APP_TYPE_PE_RECIDX, &fwd_hm_flag))
				continue;

			if (cs_core_vtable_get_hashmask_index_from_apptype(
				CORE_FWD_APP_TYPE_PE_RECIDX, &key.mask_ptr_0_7))
				continue;

			key.lspid = CPU_PORT;
			key.recirc_idx = vap_idx * CS_WIRELESS_MAX_PRIO + j + 1;
			if (cs_fe_hash_calc_crc(&key, &crc32, &crc16, CRC16_CCITT))
				continue;

			voqpol_entry.voq_base = CPU_PORT5_VOQ_BASE + vap_idx;			
			
			if (cs_fe_table_add_entry(FE_TABLE_VOQ_POLICER, &voqpol_entry,
				&voqpol_idx))
				continue;

			fwdrslt_entry.dest.voq_pol_table_index = voqpol_idx;
			fwdrslt_entry.l3.ip_da_replace_en = 0;
			fwdrslt_entry.l3.ip_sa_replace_en = 0;
			fwdrslt_entry.l3.ip_sa_index = (u16) j;
			fwdrslt_entry.l3.ip_da_index = 0;
			
			if (cs_fe_table_add_entry(FE_TABLE_FWDRSLT, &fwdrslt_entry,
				&fwdrslt_idx)) {
				cs_fe_table_del_entry_by_idx(FE_TABLE_VOQ_POLICER,
					voqpol_idx, false);
				continue;
			}

			if (cs_fe_hash_add_hash(crc32, crc16, key.mask_ptr_0_7,
				(u16)fwdrslt_idx, &cs_wireless_list[vap_idx].rx_default_hash_idx[j])) {
				cs_fe_fwdrslt_del_by_idx(fwdrslt_idx);
				continue;
			}

		}
	//}
	return 1;
}

static int cs_hw_accel_delete_rx_default_fwd_hash(int vap_idx)
{
	int  i;	
	for (i = 0; i < CS_WIRELESS_MAX_PRIO; i++) {
		if (cs_wireless_list[vap_idx].rx_default_hash_idx[i] != 0) {
			cs_fe_hash_del_hash(cs_wireless_list[vap_idx].rx_default_hash_idx[i]);
			cs_wireless_list[vap_idx].rx_default_hash_idx[i] = 0;
		}
	}
}

int cs_hw_accel_wirelss_clean_fwd_path_by_mac(char * mac)
{
	cs_core_hmu_value_t hmu_value;
	int i;
	DBG(printk("%s delete mac=%pM\n", __func__, mac));

	hmu_value.type = CS_CORE_HMU_WATCH_IN_MAC_SA;
	for (i = 0; i < 6; i++)
		hmu_value.value.mac_addr[i] = mac[5 - i];
	hmu_value.mask = 0;

	cs_core_hmu_delete_hash(CS_CORE_HMU_WATCH_IN_MAC_SA, &hmu_value);


	hmu_value.type = CS_CORE_HMU_WATCH_OUT_MAC_DA;
	for (i = 0; i < 6; i++)
		hmu_value.value.mac_addr[i] = mac[5 - i];

	cs_core_hmu_delete_hash(CS_CORE_HMU_WATCH_OUT_MAC_DA, &hmu_value);
	
	return 0;
}

int cs_hw_accel_wireless_clean_all_hash(void)
{
	int i, j;
	for (i = 0; i < CS_HW_ACCEL_WIRELESS_INTERFACE_MAX; i++) {
		if (cs_wireless_list[i].status == CS_WIRELESS_ALLOCATED) {
			if (cs_wireless_list[i].dev)
				cs_hw_accel_wirelss_clean_fwd_path_by_mac(cs_wireless_list[i].dev->dev_addr);	
		}
	}
	
}

void cs_wireless_callback_hma(unsigned long notify_event,
		unsigned long value)
{
	DBG(printk("%s() cs hw accel wireless event%ld\n", __func__,
			notify_event));
	switch (notify_event) {
		case CS_HAM_ACTION_MODULE_DISABLE:
		case CS_HAM_ACTION_CLEAN_HASH_ENTRY:
			cs_hw_accel_wireless_clean_all_hash();
			break;
		case CS_HAM_ACTION_MODULE_REMOVE:			
			break;
		case CS_HAM_ACTION_MODULE_INSERT:			
			break;
	}

}

int cs_hw_accel_wireless_enable(void)
{
	return cs_accel_kernel_module_enable(CS752X_ADAPT_ENABLE_WIRELESS);
}

int cs_hw_accel_wireless_init(void)
{
	int i, j;
	for (i = 0; i < CS_HW_ACCEL_WIRELESS_INTERFACE_MAX; i++) {
		cs_wireless_list[i].status = CS_WIRELESS_FREE;				
		switch (i) {
			case 0:	cs_wireless_list[i].tx_voq_base = CPU_PORT3_VOQ_BASE;
					break;
			case 1:	cs_wireless_list[i].tx_voq_base = CPU_PORT3_VOQ_BASE + 4;
					break;
			case 2:	cs_wireless_list[i].tx_voq_base = CPU_PORT4_VOQ_BASE;
					break;
			case 3:	cs_wireless_list[i].tx_voq_base = CPU_PORT4_VOQ_BASE + 4;
					break;
		}
		for (j = 0; j < CS_WIRELESS_MAX_PRIO; j++) {
			cs_wireless_list[i].rx_default_hash_idx[j] = 0;
		}
	}
	cs_hw_accel_mgr_register_proc_callback(CS752X_ADAPT_ENABLE_WIRELESS,
					       cs_wireless_callback_hma);
}

int cs_hw_accel_wireless_exit(void)
{
	cs_hw_accel_mgr_register_proc_callback(CS752X_ADAPT_ENABLE_WIRELESS, NULL);
	return 0;
}

extern int cs_wfo_mc_mac_table_del_entry(unsigned char* mac_da);
extern int cs_wfo_mc_mac_table_add_entry(unsigned char* mac_da, unsigned char* p_egress_port_id);

int cs_hw_accel_wireless_register(struct net_device *dev, int (*cb_start_xmit)(struct sk_buff *, struct net_device *), 
	int (*cb_prio_2_ac)(int prio), 	int (*cb_ac_2_prio)(int ac))
{
	int i;
	int rc;
	u8 egress_port_id;
	printk("%s %pM \n", __func__, dev->dev_addr);
	for (i = 0; i < CS_HW_ACCEL_WIRELESS_INTERFACE_MAX; i++) {
		if (cs_wireless_list[i].status == CS_WIRELESS_FREE) {
			cs_wireless_list[i].status = CS_WIRELESS_ALLOCATED;
			cs_wireless_list[i].cb_start_xmit = cb_start_xmit;
			cs_wireless_list[i].cb_prio_2_ac = cb_prio_2_ac;
			cs_wireless_list[i].cb_ac_2_prio = cb_ac_2_prio;
			cs_wireless_list[i].dev = dev;
			rc = cs_wfo_mc_mac_table_add_entry(dev->dev_addr, &cs_wireless_list[i].egress_port_id);			
            if ( rc != CS_MC_MAC_ENTRY_STATUS_SUCCESS) {
            	printk("%s cannot allocate a free Multicast MAC entry \n", __func__);
            	return -1;
            }
            cs_hw_accel_create_rx_default_fwd_hash(i);            
            printk("\t allocate idx=%d mc_mac_idx=%d\n", i, cs_wireless_list[i].egress_port_id);
			return i;
		}
	}
	return -1;
}

int cs_hw_accel_wireless_unregister(struct net_device *dev)
{
	int i;
	printk("%s %pM \n", __func__, dev->dev_addr);
	for (i = 0; i < CS_HW_ACCEL_WIRELESS_INTERFACE_MAX; i++) {
		if (cs_wireless_list[i].dev == dev) {
			cs_hw_accel_delete_rx_default_fwd_hash(i);  
			cs_wfo_mc_mac_table_del_entry(dev->dev_addr);
			cs_hw_accel_wirelss_clean_fwd_path_by_mac(dev->dev_addr);			
			cs_wireless_list[i].status = CS_WIRELESS_FREE;			
			cs_wireless_list[i].dev = NULL;
			return i;
		}
	}
	return -1;
}

/*							TX 				RX  
 * wireless_device[0]  		72~75			88
 * wireless_device[1]  		76~79			89
 * wireless_device[2]  		80~83			90
 * wireless_device[3]  		84~87			91
 */
extern int cs752x_fast_bridging(struct sk_buff *skb); 
int cs_hw_accel_wireless_handle(int voq, struct sk_buff *skb, unsigned int sw_action)
{
	int i;
	int dev_id;
	int direction;
	struct net_device *dev;
	cs_kernel_accel_cb_t *cs_cb = CS_KERNEL_SKB_CB(skb);
	
	if (voq >= CPU_PORT5_VOQ_BASE) {
		dev_id = voq - CPU_PORT5_VOQ_BASE;
		direction = 1;
	} else {
		dev_id = (voq - CPU_PORT3_VOQ_BASE) / 4;
		direction = 0;
	} 		
	if (dev_id > CS_HW_ACCEL_WIRELESS_INTERFACE_MAX)
		return 1;
		
	if (cs_wireless_list[dev_id].status != CS_WIRELESS_ALLOCATED)
		return 1;
		
	DBG(printk("%s dev_id=%d direction=%s voq=%d sw_action=%d\n", __func__, dev_id, (direction == 0)?"TX":"RX", voq, sw_action));
	if (direction == 0) {
		/*For wifi TX, hit the hash and directly send to wifi driver*/
		if (cs_cb != NULL) {
			cs_cb->common.sw_only = CS_SWONLY_STATE;
		}		
		cs_wireless_list[dev_id].cb_start_xmit(skb, cs_wireless_list[dev_id].dev);
		
	} else {
		/*For wifi RX, doesn't hit the hash*/
		if (cs_cb != NULL) {
			cs_cb->common.module_mask |= CS_MOD_MASK_WIRELESS;
			if (cs_hw_accel_wireless_enable()) {
				cs_cb->common.ingress_port_id = SW_WIRELESS_PORT;
				cs_core_logic_input_set_cb(skb);
			} else {
				cs_cb->common.sw_only = CS_SWONLY_STATE;
			}
		}
		skb->dev = cs_wireless_list[dev_id].dev;	
#ifdef CONFIG_CS752X_PROC
		skb->mac_header = skb->data;
		skb->data = skb->data + ETH_HLEN;
		skb->len -= ETH_HLEN;
		if ((cs_ni_fastbridge != 0) && (cs752x_fast_bridging(skb) == 0))
			goto SKB_HANDLED;

		skb->data = skb->data - ETH_HLEN;
		skb->len += ETH_HLEN;
#endif		
		skb->protocol = eth_type_trans(skb, skb->dev);
		skb->priority = cs_wireless_list[dev_id].cb_ac_2_prio(sw_action);				
		netif_receive_skb(skb);
	}	
SKB_HANDLED:	
	return 0;
}

int cs_hw_accel_wireless_tx(struct sk_buff *skb)
{
	int i;
	int dev_id = -1;
	struct net_device *dev = skb->dev;
	int voq;
	cs_kernel_accel_cb_t *cs_cb = CS_KERNEL_SKB_CB(skb);
	
	if (cs_cb == NULL) 
		return 0;
	
	if (!cs_hw_accel_wireless_enable())
		return 0;
		
	if ((cs_cb->common.tag != CS_CB_TAG) ||
		(cs_cb->common.sw_only & (CS_SWONLY_DONTCARE |
						  CS_SWONLY_STATE)))
		return 0;		
		
	for (i = 0; i < CS_HW_ACCEL_WIRELESS_INTERFACE_MAX; i++) {
		if (cs_wireless_list[i].dev == dev) {
			dev_id = i;	
			break;
		}
	}
	
	if (dev_id == -1)
		return -1;
		
	if (cs_wireless_list[dev_id].status != CS_WIRELESS_ALLOCATED)
		return -1;
	cs_cb->common.module_mask |= CS_MOD_MASK_WIRELESS;
	cs_cb->action.voq_pol.d_voq_id = cs_wireless_list[dev_id].tx_voq_base + 
			cs_wireless_list[dev_id].cb_prio_2_ac(skb->priority);
	cs_cb->action.voq_pol.cos_nop = 1;
	cs_cb->common.egress_port_id = cs_wireless_list[dev_id].egress_port_id;
	cs_core_logic_add_connections(skb);
	DBG(printk("%s dev_id=%d \n", __func__, dev_id));
	return 0;
}

extern int ni_special_start_xmit_none_bypass_ne(u16 recirc_idx, u32 buf0, int len0, u32 buf1, int len1, struct sk_buff *skb);

int cs_hw_accel_wireless_rx(struct sk_buff *skb)
{
	int i;
	int dev_id = -1;
	struct net_device *dev = skb->dev;
	int voq;
	cs_kernel_accel_cb_t *cs_cb;
	u16 recirc_idx;
	char * data_ptr;
	
	if (!cs_hw_accel_wireless_enable())
		return -1;
		
	for (i = 0; i < CS_HW_ACCEL_WIRELESS_INTERFACE_MAX; i++) {
		if (cs_wireless_list[i].dev == dev) {
			dev_id = i;	
			break;
		}
	}
	
	if (dev_id == -1)
		return -1;
		
	if (cs_wireless_list[dev_id].status != CS_WIRELESS_ALLOCATED)
		return -1;
	
	if (skb_shinfo(skb)->nr_frags)
		return -1;
	

	/*xmit skb to NE*/					
	skb_push(skb, ETH_HLEN);
	recirc_idx = dev_id * CS_WIRELESS_MAX_PRIO + cs_wireless_list[dev_id].cb_prio_2_ac(skb->priority) + 1;
	DBG(printk("%s dev_id=%d recirc_idx=%d \n", __func__, dev_id, recirc_idx));

#ifdef CONFIG_CS752X_PROC
	if(cs_acp_enable & CS75XX_ACP_ENABLE_NI){
		data_ptr = virt_to_phys(skb->data)|GOLDENGATE_ACP_BASE;
	}
	else
#endif
#ifdef CONFIG_CS75XX_NI_EXPERIMENTAL_SW_CACHE_MANAGEMENT
			data_ptr = dma_map_single(NULL, (void *)skb->data,
					skb->dirty_buffer ? skb->len : SMP_CACHE_BYTES,
					DMA_TO_DEVICE);
#else /* CONFIG_CS75XX_NI_EXPERIMENTAL_SW_CACHE_MANAGEMENT */
			data_ptr = dma_map_single(NULL, (void *)skb->data, skb->len,
						DMA_TO_DEVICE);		
#endif /* CONFIG_CS75XX_NI_EXPERIMENTAL_SW_CACHE_MANAGEMENT */
	ni_special_start_xmit_none_bypass_ne(recirc_idx, data_ptr, skb->len, 0, 0, skb) ;

	return 0;
}

EXPORT_SYMBOL(cs_hw_accel_wireless_init);
EXPORT_SYMBOL(cs_hw_accel_wireless_exit);
EXPORT_SYMBOL(cs_hw_accel_wireless_unregister);
EXPORT_SYMBOL(cs_hw_accel_wireless_register);
EXPORT_SYMBOL(cs_hw_accel_wireless_handle);
EXPORT_SYMBOL(cs_hw_accel_wireless_tx);
EXPORT_SYMBOL(cs_hw_accel_wireless_rx);
EXPORT_SYMBOL(cs_hw_accel_wirelss_clean_fwd_path_by_mac);