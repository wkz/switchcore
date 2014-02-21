/* \\/ Westermo - Switchcore driver
 *
 * Copyright (C) 2014  Westermo Teleindustri AB
 *
 * Author: Tobias Waldekranz <tobias.waldekranz@westermo.se>
 *
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>

#include "sc_private.h"
#include "sc_regs.h"

struct sc_dev_model {
	uint16_t model;
	const char *name;
};

struct sc_dev_model sc_6046 = 	{ .model = 0x048, .name  = "88E6046" };
struct sc_dev_model sc_6095 = 	{ .model = 0x095, .name  = "88E6095" };
struct sc_dev_model sc_6097 = 	{ .model = 0x099, .name  = "88E6097" };
struct sc_dev_model sc_6185 = 	{ .model = 0x1a7, .name  = "88E6185" };
struct sc_dev_model sc_6352 = 	{ .model = 0x352, .name  = "88E6352" };

static struct sc_dev_model *supported_models[] = {
	&sc_6046,
	&sc_6095,
	&sc_6097,
	&sc_6185,
	&sc_6352,
	NULL,
};

static int sc_model_ok(uint16_t ver)
{
	struct sc_dev_model **m;
	uint16_t model = ver >> 4;

	for (m = supported_models; *m; m++)
		if ((*m)->model == model)
			return 1;

	return 0;
}

static const char *sc_model_verstr(uint16_t ver)
{
	static char buf[32];
	struct sc_dev_model **m;
	const char *name = NULL;
	uint16_t model = ver >> 4;

	for (m = supported_models; *m; m++) {
		if ((*m)->model == model) {
			name = (*m)->name;
			break;
		}
	}

	if (!name)
		snprintf(buf, 32, "UNKNOWN(%#.4x)", ver);
	else
		snprintf(buf, 32, "%s rev %d", name, ver & 0xf);

	return buf;
}

static inline int sc_dev_is(struct sc_dev *dev, struct sc_dev_model *m)
{
	return dev->ver >> 4 == m->model;
}

static inline int sc_dev_has_ext_phys(struct sc_dev *dev)
{
	int i;

	/* the 6185 only has external PHYs */
	if (sc_dev_is(dev, &sc_6185))
		return 1;

	for (i = 0; i < SC_PORTS_MAX; i++)
		if (dev->port[i].type == SC_PORT_EXT_PHY)
			return 1;

	return 0;
}

static int sc_dev_poll_phys(struct sc_dev *dev)
{
	struct sc_port *port;
	uint32_t irq_mask;
	int i, any_ok = 1;

	if      (sc_dev_is(dev, &sc_6185))
		irq_mask = 0xff;
	else if (sc_dev_is(dev, &sc_6352))
		irq_mask = 0x0f;
	else
		irq_mask = mdiobus_read(dev->smi, SC_PHY(0), SC_PHY_IRQSUM);
	
	for (i = 0, port = dev->port; i < SC_PORTS_MAX; i++, port++) {
		if (!(irq_mask & (1 << port->addr)) || !(port->phy))
			continue;

		any_ok &= sc_port_poll_phy(port);
	}		

	return any_ok;
}

static int sc_dev_poll_links(struct sc_dev *dev)
{
	struct sc *sc = sc_from_dev(dev);
	uint16_t gs, gc, is = 0, im = 0;
	int ret = 1;

	mutex_lock(&dev->lock);

	if (!(dev->flags & SC_F_ACTIVE))
		goto out;

	gs = mdiobus_read(dev->smi, SC_GLOBAL1, SC_GS);
	gc = mdiobus_read(dev->smi, SC_GLOBAL1, SC_GC);

	if (sc_dev_is(dev, &sc_6352)) {
		is = mdiobus_read(dev->smi, SC_GLOBAL2, SC_IS);
		im = mdiobus_read(dev->smi, SC_GLOBAL2, SC_IM);
	}

	netdev_dbg(sc->ndev, "dev%d-irq: gs:%#.4x gc:%#.4x is:%#.4x im:%#.4x\n",
		   dev->id, gs, gc, is, im);

	if ((gc & SC_GC_ATUPROB) && (gs & SC_GS_ATUPROB)) {
		mdiobus_write(dev->smi, SC_GLOBAL1, SC_GC, gc & ~SC_GC_ATUPROB);
		/* TODO: send message to macd */
		ret = 0;
		goto out;
	}

	if (sc_dev_is(dev, &sc_6352)) {
		ret = 1;

		if (is & 0xf) 	/* TODO: read the 6352's datasheet and define this */
			ret = sc_dev_poll_phys(dev);

		goto out;
	}

	if (gs & SC_GS_PHYINT)
		ret = sc_dev_poll_phys(dev);

out:
	mutex_unlock(&dev->lock);
	return ret;
}

static void sc_dev_irq_work(struct work_struct *work)
{
	struct sc_dev *dev = container_of(work, struct sc_dev, work);
	struct sc *sc = sc_from_dev(dev);
	int i, unhandled = 1;

	/* irqs can be shared between #s */
	for (i = 0; i < SC_DEVS_MAX; i++) {
		if (sc->dev[i].irq == dev->irq)
			unhandled &= sc_dev_poll_links(&sc->dev[i]);
	}

	if (unhandled) {
		/* TODO: implement this */
		/* unhandled &= sc_dev_setup_ext_phys(dev); */
	}

	/* detect interrupt storms, usually stemming from faulty
	 * SFPs. keep interrupt disabled until the next poll cycle */
	dev->irq_err = unhandled ? dev->irq_err + 1 : 0;
	if (dev->irq_err > 10) {
		/* TODO: trigger hw alarm here */
		netdev_err(sc->ndev, "error: unhandled irq%d, disabling",
			   dev->irq);
		dev->irq_err = 0;
		return;
	} 

	enable_irq(dev->irq);
	return;
}

static irqreturn_t sc_dev_irq(int irq, void *data)
{
	struct sc_dev *dev = data;
	struct sc *sc = sc_from_dev(dev);

	/* let the workqueue enable the irq later */
	disable_irq_nosync(irq);
	queue_work(sc->wq, &dev->work);

	return IRQ_HANDLED;
}

static int sc_dev_setup_irq(struct sc_dev *dev)
{
	struct sc *sc = sc_from_dev(dev);
	int i;

	/* multiple #'s can share an irq, don't request it twice */
	for (i = 0; i < SC_DEVS_MAX; i++)
		if ((sc->dev[i].flags & SC_F_ACTIVE) &&
		    (sc->dev[i].irq == dev->irq))
			return 0;

	INIT_WORK(&dev->work, sc_dev_irq_work);
	return request_irq(dev->irq, sc_dev_irq, IRQF_TRIGGER_LOW, "sc", dev);
}

static int mdio_has_parent(struct device *dev, const void *parent)
{
	return to_mii_bus(dev)->parent == parent;
}

static int mdio_has_id(struct device *dev, const void *name)
{
	return !strncmp(to_mii_bus(dev)->id, (const char *)name,
			MII_BUS_ID_SIZE);
}

static int sc_dev_setup_mii(struct sc_dev *dev,
			    const struct sc_dev_config *devc)
{
	struct sc *sc = sc_from_dev(dev);
	struct device *miidev;
	int (*search)(struct device *, const void *) = NULL;
	const void *arg = NULL;

	switch (devc->mdio_src_type) {
	case SC_MDIO_SRC_CHAN:
		if (!(sc->chan[devc->mdio_src.chan].flags & SC_F_ACTIVE))
			break;
		
		search = mdio_has_parent;
		arg = sc->chan[devc->mdio_src.chan].ndev->dev.parent;
		break;

	case SC_MDIO_SRC_NAMED:
		search = mdio_has_id;
		arg = devc->mdio_src.name;
		break;
	}

	if (!(search && arg))
		return -EINVAL;

	miidev = class_find_device(&mdio_bus_class, NULL, arg, search);
	if (!miidev)
		return -ENODEV;

	dev->mii = to_mii_bus(miidev);
	return 0;
}

static int sc_dev_setup_smi_phy(struct sc_dev *dev)
{
	enum smi_phy_mode mode = SMI_PHY_DIRECT;

	if (sc_dev_is(dev, &sc_6352))
		mode = SMI_PHY_INDIRECT;

	dev->smi_phy = smi_phy_create(dev->smi, mode);
	if (!dev->smi_phy)
		return -ENOMEM;

	return 0;
}

static int sc_dev_probe(struct sc_dev *dev)
{
	struct sc *sc = sc_from_dev(dev);

	dev->ver = mdiobus_read(dev->smi, SC_PORT(0), 0x3);

	netdev_info(sc->ndev, "dev%d: %s\n", dev->id,
		    sc_model_verstr(dev->ver));

	if (!sc_model_ok(dev->ver))
		return -ESRCH;

	return 0;
}

int sc_dev_add_ports(struct sc *sc, const struct sc_ports_config *portsc)
{
	struct sc_dev *dev;
	int err;

	if (portsc->dev >= SC_DEVS_MAX)
		return -EINVAL;

	dev = &sc->dev[portsc->dev];

	mutex_lock(&sc->lock);
	mutex_lock(&dev->lock);

	err = sc_ports_add(dev, portsc->port);
	if (err)
		goto out;

	err = sc_dev_setup_irq(dev);
	if (err)
		goto out;

	dev->flags |= SC_F_ACTIVE;

out:
	mutex_unlock(&dev->lock);
	mutex_unlock(&sc->lock);
	return err;
}

int sc_dev_add(struct sc *sc, const struct sc_dev_config *devc)
{
	struct sc_dev *dev;
	int err;
	
	mutex_lock(&sc->lock);

	if (devc->id >= SC_DEVS_MAX)
		return -EINVAL;

	dev = &sc->dev[devc->id];

	mutex_init(&dev->lock);
	mutex_lock(&dev->lock);
	dev->id = devc->id;

	err = sc_dev_setup_mii(dev, devc);
	if (err)
		goto out;

	dev->smi = smi_create(dev->mii, devc->addr);
	if (!dev->smi) {
		err = -ENOMEM;
		goto out;
	}

	dev->addr = devc->addr;
	sc->dev_by_addr[dev->addr] = dev;
	err = sc_dev_probe(dev);
	if (err)
		goto out;

	err = sc_dev_setup_smi_phy(dev);
	if (err)
		goto out;

	dev->irq = sc_irq_find(&devc->irq);
	if (dev->irq < 0) {
		err = dev->irq;
		goto out;
	}

out:
	mutex_unlock(&dev->lock);
	mutex_unlock(&sc->lock);
	return err;
}
