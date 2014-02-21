/* \\/ Westermo - Switchcore driver
 *
 * Copyright (C) 2014  Westermo Teleindustri AB
 *
 * Author: Tobias Waldekranz <tobias.waldekranz@westermo.se>
 *
 */

#ifndef _SC_PRIVATE_H
#define _SC_PRIVATE_H

#include <linux/cdev.h>
#include <linux/netdevice.h>
#include <linux/phy_fixed.h>
#include <uapi/linux/westermo/sc_netlink.h>

#define SC_CHANS_MAX 3
#define SC_DEVS_MAX  3

#define ETH_PLEN 2
#define DSA_HLEN 4

#define DSA_TYPE     0xc0000000
#define DSA_FWD      0xc0000000
#define DSA_TOCPU    0x00000000
#define DSA_FROMCPU  0x40000000

#define DSA_DEV      0x1f000000
#define DSA_DEV_SH   24

#define DSA_PORT     0x00f80000
#define DSA_PORT_SH  19

#define DSA_1QTAGGED 0x20000000

#define DSA_1QMASK   0x0000efff

#define DSA_1QTCI    0x00010000
#define DSA_1QTCI_MV 4


enum sc_flag {
	SC_FNR_ACTIVE,
#define SC_F_ACTIVE (1 << SC_FNR_ACTIVE)
};

struct sc_dev;

struct sc_port {
	uint8_t            addr;
	enum sc_port_type  type;

	struct net_device *ndev;
	struct phy_device *phy;
	struct fixed_phy_status soft_status;
};

struct sc_dev {
	uint32_t id;

	uint32_t        flags;
	uint16_t        ver;

	int                irq;
	int                irq_err;
	struct work_struct work;

	struct mutex    lock;
	uint8_t         addr;
	struct mii_bus *mii;
	struct mii_bus *smi;
	struct mii_bus *smi_phy;

	struct sc_port port[SC_PORTS_MAX];
};

struct sc_chan {
	uint32_t id;

	uint32_t           flags;
	struct net_device *ndev;
};

struct sc {
	struct mutex lock;

	struct net_device       *ndev;
	struct workqueue_struct *wq;
	struct {
		struct cdev       cdev;
		struct class     *class;
		struct device    *dev;
	} cdev;
	
	struct sc_dev  dev[SC_DEVS_MAX];
	struct sc_dev *dev_by_addr[32];

	struct sc_chan chan[SC_CHANS_MAX];
};

static inline struct sc_dev *sc_dev_from_port(struct sc_port *port)
{
	port -= port->addr;
	return container_of(port, struct sc_dev, port[0]);
}

static inline struct sc *sc_from_dev(struct sc_dev *dev)
{
	dev -= dev->id;
	return container_of(dev, struct sc, dev[0]);
}

static inline struct sc *sc_from_chan(struct sc_chan *chan)
{
	chan -= chan->id;
	return container_of(chan, struct sc, chan[0]);
}


/* sc_main.c */
int sc_irq_find(const struct sc_irq *irqc);
int sc_chan_add(struct sc *sc, const struct sc_chan_config *conf);

/* sc_cdev.c */
int sc_cdev_init(struct sc *sc);

/* sc_dev.c */
int sc_dev_add_ports(struct sc *sc, const struct sc_ports_config *portsc);
int sc_dev_add (struct sc *sc, const struct sc_dev_config  *devc);

void sc_ndev_setup   (struct net_device *dev);

/* sc_port.c */
int sc_port_poll_phy(struct sc_port *port);
int sc_ports_add(struct sc_dev *dev, const struct sc_port_config *port);

/* sc_netlink.c */
int  __init sc_netlink_init(void);
void __exit sc_netlink_exit(void);

/* sc_smi.c */
enum smi_phy_mode {
	SMI_PHY_DIRECT,
	SMI_PHY_INDIRECT,
};

int smi_read(struct mii_bus *bus, int addr, int reg);
int smi_write(struct mii_bus *bus, int addr, int reg, uint16_t val);

struct mii_bus *smi_create    (struct mii_bus *mii, uint8_t addr);
struct mii_bus *smi_phy_create(struct mii_bus *smi_mii, enum smi_phy_mode mode);

/* static inline void sc_dump_skb(struct sk_buff *skb) */
/* { */
/* 	pr_info("head:%p l2:%p l3:%p data:%p\n", */
/* 		skb->head, skb->mac_header, skb->network_header, skb->data); */
/* 	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 16, 1, skb->head, 0x80, 0); */
/* 	pr_info("\n\n"); */
/* } */

#endif	/* _SC_PRIVATE_H */
