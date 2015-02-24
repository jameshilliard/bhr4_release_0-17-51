
#include <linux/export.h>
#include <net/neighbour.h>
#include <net/arp.h>
#include <linux/jiffies.h>
#include "cs_hw_accel_arp.h"
#include "cs_core_hmu.h"

#define PFX     "CS_HW_ARP_SYNC"
#define PRINT(format, args...) printk(KERN_WARNING PFX \
	":%s:%d: " format, __func__, __LINE__ , ## args)
#ifdef CONFIG_CS752X_PROC
#include "cs752x_proc.h"
extern u32 cs_adapt_debug;
#define DBG(x) {if (cs_adapt_debug & CS752X_ADAPT_ARP) x;}
#else
#define DBG(x) { }
#endif

static cs_core_hmu_value_t *construct_core_hmu_value_for_mac(u8 *mac_addr)
{
	cs_core_hmu_value_t *new_core_value;
	int i;

	new_core_value = kzalloc(sizeof(cs_core_hmu_value_t), GFP_ATOMIC);
	if (new_core_value == NULL)
		return NULL;

	DBG(PRINT("creating core hmu value for mac addr = 0x%02x.%02x."
				"%02x.%02x.%02x.%02x\n", mac_addr[5],
				mac_addr[4], mac_addr[3], mac_addr[2],
				mac_addr[1], mac_addr[0]));

	for (i = 0; i < 6; i++)
		new_core_value->value.mac_addr[i] = mac_addr[5 - i];
	new_core_value->mask = 0;

	return new_core_value;
}

static int cs_arp_get_last_use(u8 *mac_addr, unsigned long *last_use)
{
	cs_core_hmu_value_t *core_value;
	unsigned long curr_last_use;
	bool is_found = false;
	int ret;

	core_value = construct_core_hmu_value_for_mac(mac_addr);
	if (core_value == NULL)
		return -1;

	core_value->type = CS_CORE_HMU_WATCH_IN_MAC_SA;
	ret = cs_core_hmu_get_last_use(CS_CORE_HMU_WATCH_IN_MAC_SA, core_value,
			&curr_last_use);
	if (ret == 0) {
		DBG(PRINT("get last use = %ld\n", curr_last_use));
		is_found = true;
		*last_use = curr_last_use;
	} else {
		DBG(PRINT("unable to get last use, ret = %d\n", ret));
	}

	core_value->type = CS_CORE_HMU_WATCH_OUT_MAC_DA;
	ret = cs_core_hmu_get_last_use(CS_CORE_HMU_WATCH_OUT_MAC_DA, core_value,
			&curr_last_use);
	if (ret == 0) {
		DBG(PRINT("get last use = %ld\n", curr_last_use));
		if (is_found == true) {
			if (time_before(*last_use, curr_last_use))
				*last_use = curr_last_use;
		} else {
			is_found = true;
			*last_use = curr_last_use;
		}
	} else {
		DBG(PRINT("unable to get last use, ret = %d\n", ret));
	}

	if (is_found == false)
		ret = -1;
	else
		ret = 0;

	kfree(core_value);
	return ret;
}

void cs_arp_delete(void *data)
{
	struct neighbour *neigh = (struct neighbour *)data;
	cs_core_hmu_value_t *core_value;
	int i = 0;

	if (neigh == NULL)
		return;

	if (neigh->tbl != &arp_tbl)
		return;

	DBG(PRINT("0x%x:got here w/nud_state %d!\n", (u32)neigh,
				neigh->nud_state));

	DBG(printk("ha = "));
	for (i = 0; i < neigh->dev->addr_len; i++)
		DBG(printk("%02x.", (neigh->ha[i])));
	DBG(printk("\n\n"));

	core_value = construct_core_hmu_value_for_mac((u8 *)neigh->ha);
	if (core_value == NULL)
		return;

	core_value->type = CS_CORE_HMU_WATCH_IN_MAC_SA;
	cs_core_hmu_delete_hash(CS_CORE_HMU_WATCH_IN_MAC_SA, core_value);
	core_value->type = CS_CORE_HMU_WATCH_OUT_MAC_DA;
	cs_core_hmu_delete_hash(CS_CORE_HMU_WATCH_OUT_MAC_DA, core_value);
	kfree(core_value);

	return;
} /* cs_arp_delete */
EXPORT_SYMBOL(cs_arp_delete);

void cs_arp_update_used(void *data)
{
	struct neighbour *neigh = (struct neighbour *)data;
	unsigned long use_jiffies = 0;
	int ret;

	if (neigh == NULL)
		return;

	if (neigh->tbl != &arp_tbl)
		return;

	DBG(PRINT("0x%x:got here w/nud_state %d @%lu!\n", (u32)neigh,
				neigh->nud_state, jiffies));

	/*
	 * find the garbage collector of this neighbour entry.
	 * update neighbour's last used with the jiffies that's
	 * kept on garbage collector.
	 */
	ret = cs_arp_get_last_use((u8 *)neigh->ha, &use_jiffies);
	if (ret != 0)
		return;

	DBG(PRINT("0x%x:got here w/nud_state %d!\n", (u32)neigh,
				neigh->nud_state));
	DBG(PRINT("jiffies neigh(%lu) vs gc(%lu)\n", neigh->used,
				use_jiffies));

	if (time_before(neigh->used, use_jiffies))
		neigh->used = use_jiffies;

	return;
} /* cs_arp_update_used */
EXPORT_SYMBOL(cs_arp_update_used);

static int cs_arp_core_hmu_callback(u32 watch_bitmask,
		cs_core_hmu_value_t * value, u32 status)
{
	// well we don't really need callback here!!
	DBG(PRINT("watch_bitmask = %x, status = %x\n", watch_bitmask, status));
	return 0;
}

static cs_core_hmu_t arp_hmu_in_mac_sa = {
	.watch_bitmask = CS_CORE_HMU_WATCH_IN_MAC_SA,
	.value_mask = NULL,
	.callback = cs_arp_core_hmu_callback,
};

static cs_core_hmu_t arp_hmu_out_mac_da = {
	.watch_bitmask = CS_CORE_HMU_WATCH_OUT_MAC_DA,
	.value_mask = NULL,
	.callback = cs_arp_core_hmu_callback,
};

void cs_hw_arp_init(void)
{
	int ret = 0;

	ret |= cs_core_hmu_register_watch(&arp_hmu_in_mac_sa);
	ret |= cs_core_hmu_register_watch(&arp_hmu_out_mac_da);
	if (ret != 0) {
		PRINT("unable to register HMU for ARP!\n");
		return;
	}

	return;
} /* cs_hw_arp_init */

void cs_hw_arp_exit(void)
{
	int ret = 0;

	ret |= cs_core_hmu_unregister_watch(&arp_hmu_in_mac_sa);
	ret |= cs_core_hmu_unregister_watch(&arp_hmu_out_mac_da);
	if (ret != 0) {
		printk("%s:%d:unable to unregister HMU for ARP!\n", __func__,
				__LINE__);
		return;
	}
} /* cs_hw_arp_exit */

