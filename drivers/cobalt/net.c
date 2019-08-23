/* 
 * cobalt net wrappers
 * Copyright (c) 2000, Cobalt Networks, Inc.
 * Copyright (c) 2001, Sun Microsystems, Inc.
 * $Id: net.c,v 1.11 2001/10/27 00:40:24 thockin Exp $
 * author: thockin@sun.com
 *
 * This should be SMP safe.  The only critical data is the list of devices.
 * The LED handler runs at timer-interrupt, so we must use the IRQ safe forms
 * of the locks. --TPH
 */

#include <stdarg.h>
#include <stddef.h>
#include <linux/init.h>
#include <linux/config.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <asm/io.h>

#include <cobalt/cobalt.h>
#include <cobalt/net.h>
#include <cobalt/led.h>

#define MAX_COBT_NETDEVS	2
static struct net_device *netdevs[MAX_COBT_NETDEVS];
static int n_netdevs;
static spinlock_t cobaltnet_lock = SPIN_LOCK_UNLOCKED;

#if defined(CONFIG_COBALT_LED)
static unsigned int
net_led_handler(void *data)
{
	int i;
	unsigned int leds = 0;
	static int txrxmap[MAX_COBT_NETDEVS] = {LED_ETH0_TXRX, LED_ETH1_TXRX};
	static int linkmap[MAX_COBT_NETDEVS] = {LED_ETH0_LINK, LED_ETH1_LINK};
	unsigned long flags;
	static unsigned long net_old[MAX_COBT_NETDEVS];

	spin_lock_irqsave(&cobaltnet_lock, flags);

	for (i = 0; i < n_netdevs; i++) {
		unsigned long txrxstate;
		struct net_device *dev = netdevs[i];
		if (!dev) {
			continue;
		}
		/* check for link */
		if (netif_running(dev) && netif_carrier_ok(dev)) {
			leds |= linkmap[i];
		}
		/* check for tx/rx */
		txrxstate = dev->trans_start ^ dev->last_rx;
		if (txrxstate != net_old[i]) {
			leds |= txrxmap[i];
			net_old[i] = txrxstate;
		}
	}

	spin_unlock_irqrestore(&cobaltnet_lock, flags);

	return leds;
}
#endif

/* 
 * We try to be VERY explicit here.  Fine for now, may eventually break down.
 */
void 
cobalt_net_register(struct net_device *ndev)
{
	unsigned long flags;
	int i;
	
	if (!ndev) {
		return;
	}

	/* we'll track the first MAX_COBT_NETDEVS NICs */
	if (n_netdevs >= MAX_COBT_NETDEVS) {
		return;
	}

	spin_lock_irqsave(&cobaltnet_lock, flags);

	/* find a free slot */
	for (i = 0; i < MAX_COBT_NETDEVS; i++) {
		if (!netdevs[i]) {
			netdevs[i] = ndev;
			n_netdevs++;
			break;
		}
	}

	spin_unlock_irqrestore(&cobaltnet_lock, flags);
}

void 
cobalt_net_unregister(struct net_device *ndev)
{
	int i;
	unsigned long flags;
	
	if (!ndev) {
		return;
	}

	spin_lock_irqsave(&cobaltnet_lock, flags);

	/* try to remove it from the list */
	for (i = 0; i < MAX_COBT_NETDEVS; i++) {
		if (netdevs[i] == ndev) {
			netdevs[i] = NULL;
			n_netdevs--;
			break;
		}
	}

	spin_unlock_irqrestore(&cobaltnet_lock, flags);
}

int __init
cobalt_net_init(void)
{
#if defined(CONFIG_COBALT_LED)
	/* register an LED handler */
	cobalt_fpled_register(net_led_handler, NULL);
#endif

	return 0;
}
