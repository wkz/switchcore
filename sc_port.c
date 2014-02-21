/* \\/ Westermo - Switchcore driver
 *
 * Copyright (C) 2014  Westermo Teleindustri AB
 *
 * Author: Tobias Waldekranz <tobias.waldekranz@westermo.se>
 *
 */

#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>

#include "sc_private.h"
#include "sc_regs.h"

struct sc_port_priv {
	struct sc_port *port;

	const uint32_t tag_fwd;
	const uint32_t tag_fromcpu;
};

static inline void __sc_put_tag(uint8_t *buf, uint32_t tag)
{
	buf[0] = (tag >> 24);
	buf[1] = (tag >> 16) & 0xff;
	buf[2] = (tag >>  8) & 0xff;
	buf[3] = (tag >>  0) & 0xff;
}

static netdev_tx_t sc_port_ndev_start_xmit(struct sk_buff *skb,
					   struct net_device *ndev)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;
	uint32_t tag = 0;

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	if (unlikely(skb_cow_head(skb, DSA_HLEN)))
		goto drop;

	tag |= skb->sc.dot1q & DSA_1QMASK;
	tag |= (skb->sc.dot1q & ~DSA_1QMASK) << DSA_1QTCI_MV;

	if (vlan_tx_tag_present(skb)) {
		tag |= vlan_tx_tag_get(skb);
		tag |= DSA_1QTAGGED;

		/* to lower layers: this is not the tag you're looking for */
		skb->vlan_tci = 0;
	}

	if (likely(skb->sc.bridged))
		tag |= priv->tag_fwd;
	else
		tag |= priv->tag_fromcpu;

	skb_push(skb, DSA_HLEN);
	memmove(skb->data, skb->data + DSA_HLEN, 2 * ETH_ALEN);
	__sc_put_tag(skb->data + 2 * ETH_ALEN, tag);
	skb->mac_header -= DSA_HLEN;

	skb->dev = sc_from_dev(sc_dev_from_port(port))->chan[skb->sc.chan].ndev;
	dev_queue_xmit(skb);
	return NETDEV_TX_OK;

drop:
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int sc_port_ndev_open(struct net_device *ndev)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;

	if (!port->phy || port->phy->link)
		return 0;

	genphy_resume(port->phy);
	return 0;
}

static int sc_port_ndev_stop(struct net_device *ndev)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;

	if (!port->phy)
		return 0;

	genphy_suspend(port->phy);
	return 0;
}

static int sc_port_ndev_init(struct net_device *ndev)
{
	return 0;
}

static void sc_port_ndev_change_rx_flags(struct net_device *ndev, int change)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;
	struct net_device *chandev = sc_from_dev(sc_dev_from_port(port))->ndev;

	if (change & IFF_ALLMULTI)
		dev_set_allmulti(chandev, ndev->flags & IFF_ALLMULTI ? 1 : -1);
	if (change & IFF_PROMISC)
		dev_set_promiscuity(chandev, ndev->flags & IFF_PROMISC ? 1 : -1);
}

static void sc_port_ndev_set_rx_mode(struct net_device *ndev)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;
	struct net_device *chandev = sc_from_dev(sc_dev_from_port(port))->ndev;

	dev_mc_sync(chandev, ndev);
	dev_uc_sync(chandev, ndev);
}

static int sc_port_ndev_set_mac_address(struct net_device *ndev, void *a)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;
	struct net_device *chandev = sc_from_dev(sc_dev_from_port(port))->ndev;
	struct sockaddr *addr = a;
	int err;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	if (!(ndev->flags & IFF_UP))
		goto out;

	if (compare_ether_addr(addr->sa_data, chandev->dev_addr)) {
		err = dev_uc_add(chandev, addr->sa_data);
		if (err < 0)
			return err;
	}

	if (compare_ether_addr(ndev->dev_addr, chandev->dev_addr))
		dev_uc_del(chandev, ndev->dev_addr);

out:
	memcpy(ndev->dev_addr, addr->sa_data, ETH_ALEN);
	return 0;
}

static int sc_port_ndev_ioctl(struct net_device *ndev,
			      struct ifreq *ifr, int cmd)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;

	if (!port->phy)
		return -EOPNOTSUPP;

	if ((cmd == SIOCSSOFTCARRIER) &&
	    (port->type == SC_PORT_SOFT_PHY)) {
		port->soft_status.link = ifr->ifr_ifru.ifru_ivalue;
		phy_start_aneg(port->phy);
		return 0;
	}

	return phy_mii_ioctl(port->phy, ifr, cmd);
}

static const struct net_device_ops sc_port_ndev_ops = {
	.ndo_init               = sc_port_ndev_init,
	.ndo_open               = sc_port_ndev_open,
	.ndo_stop               = sc_port_ndev_stop,
	.ndo_start_xmit         = sc_port_ndev_start_xmit,
	.ndo_change_rx_flags    = sc_port_ndev_change_rx_flags,
	.ndo_set_rx_mode        = sc_port_ndev_set_rx_mode,
	.ndo_set_mac_address    = sc_port_ndev_set_mac_address,
	.ndo_do_ioctl           = sc_port_ndev_ioctl,
};


static int sc_port_get_settings(struct net_device *ndev,
				struct ethtool_cmd *cmd)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;
	int err;

	err = -EOPNOTSUPP;
	if (port->phy != NULL) {
		err = phy_read_status(port->phy);
		if (!err)
			err = phy_ethtool_gset(port->phy, cmd);
	}

	return err;
}

static int sc_port_set_settings(struct net_device *ndev,
				struct ethtool_cmd *cmd)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;

	if (!port->phy)
		return -EOPNOTSUPP;

	return phy_ethtool_sset(port->phy, cmd);
}

static void sc_port_get_drvinfo(struct net_device *ndev,
				struct ethtool_drvinfo *drvinfo)
{
	strlcpy(drvinfo->driver, "switchcore", sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, "2.0", sizeof(drvinfo->version));
	strlcpy(drvinfo->fw_version, "N/A", sizeof(drvinfo->fw_version));
	strlcpy(drvinfo->bus_info, "N/A", sizeof(drvinfo->bus_info));
}

static int sc_port_nway_reset(struct net_device *ndev)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;

	if (port->phy != NULL)
		return genphy_restart_aneg(port->phy);

	return -EOPNOTSUPP;
}

static u32 sc_port_get_link(struct net_device *ndev)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;

	if (port->phy != NULL) {
		genphy_update_link(port->phy);
		return port->phy->link;
	}

	return -EOPNOTSUPP;
}

static const struct ethtool_ops sc_port_ethtool_ops = {
	.get_settings           = sc_port_get_settings,
	.set_settings           = sc_port_set_settings,
	.get_drvinfo            = sc_port_get_drvinfo,
	.nway_reset             = sc_port_nway_reset,
	.get_link               = sc_port_get_link,
	/* .get_strings            = sc_port_ndev_get_strings, */
	/* .get_ethtool_stats      = sc_port_ndev_get_ethtool_stats, */
	/* .get_sset_count         = sc_port_ndev_get_sset_count, */
};

void sc_port_ndev_adjust_link(struct net_device *ndev)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;
	struct sc_dev *dev = sc_dev_from_port(port);
	int pcs = 0;

	if ((port->type != SC_PORT_EXT_PHY) &&
	    (port->type != SC_PORT_SOFT_PHY))
		return;

	netdev_dbg(ndev, "adjust %s %d-%s\n",
		   port->phy->link ? "UP" : "DOWN",
		   port->phy->speed,
		   port->phy->duplex ? "FULL" : "HALF");

	pcs = mdiobus_read(dev->smi, SC_PORT(port->addr), 0x1);
	pcs &= ~(0x3F);

	if (port->phy->link)
		pcs |= SC_PCS_LINKVALUE | SC_PCS_FORCELINK;
	if (port->phy->duplex)
		pcs |= SC_PCS_DPX | SC_PCS_FORCEDPX;

	if (port->phy->speed == 1000)
		pcs |= SC_PCS_SPEED_1000;
	else if (port->phy->speed == 100)
		pcs |= SC_PCS_SPEED_100;

	mdiobus_write (dev->smi, SC_PORT(port->addr), 0x1, pcs);
}

int sc_port_ndev_soft_update(struct net_device *ndev,
			     struct fixed_phy_status *status)
{
	struct sc_port_priv *priv = netdev_priv(ndev);
	struct sc_port *port = priv->port;

	if (status->link == port->soft_status.link)
		return 0;

	*status = port->soft_status;
	return 0;
}

int sc_port_poll_phy (struct sc_port *port)
{
	/* TODO: fast_link_active */
	struct sc_dev *dev = sc_dev_from_port(port);
	uint16_t irq_stat;

	irq_stat = mdiobus_read(dev->smi, SC_PHY(port->addr), SC_PHY_IRQSTAT);
	if (irq_stat == 0xffff) {
		netdev_err(port->ndev, "error: phy access, hw broken?\n");
		return -EIO;
	}

	if (!(irq_stat & SC_PHY_IRQSTAT_LINK) &&
	    !(irq_stat & SC_PHY_IRQSTAT_ANEG))
		return -ENOENT;

	genphy_read_status(port->phy);
	if (port->phy->link)
		netif_carrier_on(port->ndev);
	else
		netif_carrier_off(port->ndev);

	return 0;
}

static int is_fixed_0(struct device *dev, const void *arg)
{
	return !strcmp(to_mii_bus(dev)->id, "fixed-0");
}

static int sc_port_setup_soft_phy(struct sc_port *port)
{
	struct device *fixed;
	int err;

	fixed = class_find_device(&mdio_bus_class, NULL, NULL, is_fixed_0);
	if (!fixed)
		return -ENODEV;

	port->soft_status.speed = 100;
	port->soft_status.duplex = 1;
	err = fixed_phy_add (0, port->addr, &port->soft_status);
	if (err)
		return err;

	port->phy = mdiobus_scan(to_mii_bus(fixed), port->addr);
	if (IS_ERR_OR_NULL(port->phy))
		return -ENODEV;

	fixed_phy_set_link_update(port->phy, sc_port_ndev_soft_update);

	phy_connect_direct(port->ndev, port->phy, sc_port_ndev_adjust_link,
			   PHY_INTERFACE_MODE_SGMII);
	return 0;
}

static int sc_port_setup_ext_phy(struct sc_port *port,
				 const struct sc_port_config *portc)
{
	struct sc_dev *dev = sc_dev_from_port(port);
	uint16_t page, r16, r17;
	
	page = mdiobus_read(dev->smi_phy, SC_PHY(port->addr), 0x16);

	/* configure MV112's INIT pin to be used as IRQ */
	mdiobus_write(dev->smi_phy, SC_PHY(port->addr), 0x16, 3);
	r16  = mdiobus_read(dev->smi_phy, SC_PHY(port->addr), 16);
	r16 &= ~(0xf << 8);
	r16 |=  (0xe << 8);
	mdiobus_write(dev->smi_phy, SC_PHY(port->addr), 16, r16);

	/* configure INIT's polarity, active low */
	r17  = mdiobus_read(dev->smi_phy, SC_PHY(port->addr), 17);
	r17 &= ~(0x3 << 4);
	mdiobus_write(dev->smi_phy, SC_PHY(port->addr), 17, r17);

	mdiobus_write(dev->smi_phy, SC_PHY(port->addr), 0x16, page);

	port->phy->irq = sc_irq_find(&portc->irq_phy);
	if (port->phy->irq < 0)
		return port->phy->irq;

	return 0;
}

static int sc_port_setup_phy(struct sc_port *port,
			     const struct sc_port_config *portc)
{
	struct sc_dev *dev = sc_dev_from_port(port);
	int err;

	if (port->type == SC_PORT_SOFT_PHY)
		return sc_port_setup_soft_phy(port);

	port->phy = mdiobus_scan(dev->smi_phy, port->addr);
	if (IS_ERR_OR_NULL(port->phy))
		return -ENODEV;

	if ((port->type == SC_PORT_EXT_PHY) &&
	    (err = sc_port_setup_ext_phy(port, portc)))
	    return err;

	phy_connect_direct(port->ndev, port->phy, sc_port_ndev_adjust_link,
			   PHY_INTERFACE_MODE_SGMII);
        port->phy->autoneg = AUTONEG_ENABLE;
	port->phy->speed = 0;
	port->phy->duplex = 0;
	port->phy->advertising = port->phy->supported | ADVERTISED_Autoneg;
	phy_start_aneg(port->phy);

	return 0;
}

int sc_port_add(struct sc_dev *dev, const struct sc_port_config *portc)
{
	struct sc *sc = sc_from_dev(dev);
	struct sc_port *port = &dev->port[portc->addr];
	struct sc_port_priv *priv;
	struct net_device *ndev;
	int err;

	port->addr = portc->addr;
	port->type = portc->type;

	ndev = alloc_netdev(sizeof(struct sc_port_priv), portc->name, ether_setup);
	if (!ndev)
		return -ENOMEM;

	ndev->priv_flags |= IFF_SC_PORT;
	ndev->hw_features = NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX;
	ndev->features = ndev->hw_features;
	ndev->tx_queue_len = 0;
	memcpy(ndev->dev_addr, portc->mac, ETH_ALEN);
	ndev->netdev_ops = &sc_port_ndev_ops;
	SET_ETHTOOL_OPS(ndev, &sc_port_ethtool_ops);

	priv = netdev_priv(ndev);
	priv->port = port;
	*((uint32_t *)&priv->tag_fwd) = DSA_FWD |
		(dev->addr << DSA_DEV_SH) |
		(port->addr << DSA_PORT_SH);

	*((uint32_t *)&priv->tag_fromcpu) = DSA_FROMCPU |
		(dev->addr << DSA_DEV_SH) |
		(port->addr << DSA_PORT_SH);

	netif_carrier_off(ndev);

	err = register_netdevice(ndev);
	if (err) {
		netdev_err(sc->ndev, "error: unable to add port %s: %d\n",
			   portc->name, err);
		goto out_free;
	}
	
	SET_NETDEV_DEV(ndev, &sc->ndev->dev);
	port->ndev = ndev;

	err = sc_port_setup_phy(port, portc);
	if (err) {
		netdev_err(ndev, "error: unable to attach to PHY: %d\n", err);
		goto out_unregister;		
	}

	netdev_info(sc->ndev, "dev%d-%d: name:%s phy:%s (%s)\n", dev->id, port->addr,
		    netdev_name(ndev), dev_name(&port->phy->dev), port->phy->drv->name);
	return 0;

out_unregister:
	unregister_netdevice(ndev);
out_free:
	free_netdev(ndev);
	return err;
}

int sc_ports_add(struct sc_dev *dev, const struct sc_port_config *portc)
{
	int i, err;

	for (i = 0; i < SC_PORTS_MAX; i++) {
		if (portc[i].type == SC_PORT_UNUSED)
			continue;

		if (portc[i].addr >= SC_PORTS_MAX)
			return -EINVAL;

		err = sc_port_add(dev, &portc[i]);
		if (err)
			return err;
	}
	return 0;
}
