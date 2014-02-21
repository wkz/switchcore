/* \\/ Westermo - Switchcore driver
 *
 * Copyright (C) 2014  Westermo Teleindustri AB
 *
 * Author: Tobias Waldekranz <tobias.waldekranz@westermo.se>
 *
 */

#include <linux/etherdevice.h>
#include <linux/gpio.h>
#include <linux/if_vlan.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_irq.h>

#include "sc_private.h"

static inline void __sc_update_proto(struct sk_buff *skb)
{
	struct ethhdr *eth = eth_hdr(skb);
	
	if (ntohs(eth->h_proto) >= ETH_P_802_3_MIN)
		skb->protocol = eth->h_proto;
	else
		skb->protocol = htons(ETH_P_802_2);
}

static inline uint32_t __sc_get_tag(uint8_t *buf)
{
	uint32_t tag;

	tag  = buf[0] << 24;
	tag |= buf[1] << 16;
	tag |= buf[2] <<  8;
	tag |= buf[3];
	return tag;
}

rx_handler_result_t sc_chan_rx(struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	struct sc *sc = skb->dev->rx_handler_data;
	struct sc_dev *dev;
	struct sc_port *port;
	uint32_t tag, addr, idx;

	tag = __sc_get_tag(skb->data - ETH_PLEN);

	if (unlikely(((tag & DSA_TYPE) != DSA_FWD) &&
		     ((tag & DSA_TYPE) != DSA_TOCPU)))
		goto drop;

	addr = (tag & DSA_DEV)  >> DSA_DEV_SH;
	idx  = (tag & DSA_PORT) >> DSA_PORT_SH;

	dev = sc->dev_by_addr[addr];
	if (unlikely(!dev || idx >= SC_PORTS_MAX))
		goto drop;

	port = &dev->port[idx];
	if (unlikely(!port->ndev))
		goto drop;

	skb_pull_rcsum(skb, DSA_HLEN);
	memmove(skb->data - ETH_HLEN,
		skb->data - ETH_HLEN - DSA_HLEN,
		2 * ETH_ALEN);

	skb->mac_header += DSA_HLEN;
	skb->network_header += DSA_HLEN;
	__sc_update_proto(skb);

	if (tag & DSA_1QTAGGED) {
		uint16_t dot1q;

		dot1q  = (tag & DSA_1QMASK);
		dot1q |= ((tag & DSA_1QTCI) >> DSA_1QTCI_MV);

		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), dot1q);
	}

	skb->dev = port->ndev;
	skb->pkt_type = PACKET_HOST;
	skb->dev->stats.rx_packets++;
	skb->dev->stats.rx_bytes += skb->len;
	return RX_HANDLER_ANOTHER;

drop:
	dev_kfree_skb(skb);
	return RX_HANDLER_CONSUMED;
}

int sc_irq_find(const struct sc_irq *irqc)
{
	int irq = -ESRCH;
	struct device_node *sc = NULL;

	switch (irqc->type) {
	case SC_IRQ_SRC_OF:
		sc = of_find_compatible_node(NULL, NULL, "switchcore");
		if (!sc)
			return -ENODEV;

		irq = irq_of_parse_and_map (sc, irqc->src.of);
		break;

	case SC_IRQ_SRC_GPIO:
		irq = gpio_to_irq(irqc->src.gpio);
		break;
	}

	return irq;
}

int sc_chan_add(struct sc *sc, const struct sc_chan_config *chanc)
{
	struct net_device *chandev;
	struct sc_chan *chan;
	int err = 0, id = chanc->id;

	mutex_lock(&sc->lock);

	chandev = dev_get_by_name(dev_net(sc->ndev), chanc->ifname);
	if (!chandev) {
		err = -ENODEV;
		goto out;
	}

	if (chanc->id >= SC_CHANS_MAX) {
		err = -EINVAL;
		goto out;
	}

	chan = &sc->chan[chanc->id];
	chan->id = chanc->id;
	chan->ndev = chandev;
	
	netdev_info(sc->ndev, "chan%d: iface:%s\n", chan->id,
		    netdev_name(chandev));
	sc->chan[id].flags |= SC_F_ACTIVE;
out:
	mutex_unlock(&sc->lock);
	return err;
}

static int sc_chan_attach(struct sc_chan *chan)
{
	struct sc *sc = sc_from_chan(chan);
	int err;

	err = netdev_rx_handler_register(chan->ndev, sc_chan_rx, sc);
	if (err) {
		netdev_err(sc->ndev, "error: attaching to %s: %d\n",
			   netdev_name(chan->ndev), err);
		goto err_handler;
	}

	err = dev_set_promiscuity(chan->ndev, 1);
	if (err) {
		netdev_err(sc->ndev, "error: %s insisting on chastity\n",
			   netdev_name(chan->ndev));
		goto err_promiscuity;
	}

	netdev_info(sc->ndev, "%s: attached\n", netdev_name(chan->ndev));
	return 0;

err_promiscuity:
	netdev_rx_handler_unregister(chan->ndev);
err_handler:
	return err;
}

static int sc_chan_detach(struct sc_chan *chan)
{
	struct sc *sc = sc_from_chan(chan);

	dev_set_promiscuity(chan->ndev, -1);
	netdev_rx_handler_unregister(chan->ndev);		

	netdev_info(sc->ndev, "%s: detached\n", netdev_name(chan->ndev));
	return 0;
}

static netdev_tx_t sc_ndev_start_xmit(struct sk_buff *skb,
				      struct net_device *ndev)
{
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int sc_ndev_open(struct net_device *ndev)
{
	struct sc *sc = netdev_priv(ndev);
	struct sc_chan *chan;
	int i;

	for (i = 0, chan = sc->chan; i < SC_CHANS_MAX; i++, chan++) {
		if (!(chan->flags & SC_F_ACTIVE))
			continue;
		
		sc_chan_attach(chan);
	}

	return 0;
}

static int sc_ndev_stop(struct net_device *ndev)
{
	struct sc *sc = netdev_priv(ndev);
	struct sc_chan *chan;
	int i;

	for (i = 0, chan = sc->chan; i < SC_CHANS_MAX; i++, chan++) {
		if (!(chan->flags & SC_F_ACTIVE))
			continue;
		
		sc_chan_detach(chan);
	}

	return 0;
}


static int sc_ndev_init(struct net_device *ndev)
{
	return 0;
}

static const struct net_device_ops sc_ndev_ops = {
	.ndo_start_xmit	= sc_ndev_start_xmit,
	.ndo_open	= sc_ndev_open,
	.ndo_stop	= sc_ndev_stop,
	.ndo_init	= sc_ndev_init,
};

void sc_ndev_setup(struct net_device *ndev)
{
	struct sc *sc = netdev_priv (ndev);
	int err;

	ether_setup(ndev);
	ndev->netdev_ops = &sc_ndev_ops;

	mutex_init(&sc->lock);
	sc->ndev = ndev;
	sc->wq = create_singlethread_workqueue("sc");
	if (!sc->wq)
		netdev_err(sc->ndev, "error: could not create workqueue\n");

	err = sc_cdev_init(sc);
	if (err)
		netdev_err(sc->ndev, "error: could not register char device\n");

	return;
}

static int __init sc_module_init(void)
{
	return sc_netlink_init();
}

static void __exit sc_module_exit(void)
{
	sc_netlink_exit();
}

module_init(sc_module_init);
module_exit(sc_module_exit);
MODULE_LICENSE("GPL");
