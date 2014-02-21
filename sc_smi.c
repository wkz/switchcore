/* \\/ Westermo - Switchcore driver
 *
 * Copyright (C) 2014  Westermo Teleindustri AB
 *
 * Author: Tobias Waldekranz <tobias.waldekranz@westermo.se>
 *
 * Handles SMI addressing by creating a virtual MDIO bus to each
 * switchcore.
 *
 * NOTES:
 *
 * 1. The SMI bus will share locking with the parent MDIO bus. This
 *    ensures that the SMI transaction will be atomic.
 *
 * 2. As a consequence of (1), the standard mdiobus_* family of
 *    functions can not be used since the bus has already been
 *    locked. Instead the parent bus's private methods are used
 *    directly, i.e. (struct mii_bus *)bus->read/write/reset.
 */

#include <linux/phy.h>

#include "sc_private.h"
#include "sc_regs.h"

struct smi_bus {
	struct mii_bus *mii;
	uint8_t addr;
};

struct smi_phy_bus {
	struct mii_bus *smi;
	enum smi_phy_mode mode;
};

static int smi_wait_ready(struct mii_bus *bus)
{
	struct smi_bus *smi = bus->priv;
	int ret, i;

	for (i = 0; i < 16; i++) {
		ret = smi->mii->read(smi->mii, smi->addr, 0);
		if (ret < 0)
			return ret;

		if ((ret & 0x8000) == 0)
			return 0;
	}

	return -ETIMEDOUT;
}

int smi_read(struct mii_bus *bus, int addr, int reg)
{
	struct smi_bus *smi = bus->priv;
	int ret;

	mutex_lock(&smi->mii->mdio_lock);

	ret = smi_wait_ready(bus);
	if (ret < 0)
		goto out;

	ret = smi->mii->write(smi->mii, smi->addr,
			      0, 0x9800 | (addr << 5) | reg);
	if (ret < 0)
		goto out;

	ret = smi_wait_ready(bus);
	if (ret < 0)
		goto out;

	ret = smi->mii->read(smi->mii, smi->addr, 1);

out:
	mutex_unlock(&smi->mii->mdio_lock);
	/* pr_info("%s read(%#x, %#x): %#x\n", bus->id, addr, reg, ret); */
	return ret;
}

int smi_write(struct mii_bus *bus, int addr, int reg, uint16_t val)
{
	struct smi_bus *smi = bus->priv;
	int ret;

	/* pr_info("%s write(%#x, %#x, %#x)\n", bus->id, addr, reg, val); */

	mutex_lock(&smi->mii->mdio_lock);

	ret = smi_wait_ready(bus);
	if (ret < 0)
		goto out;

	ret = smi->mii->write(smi->mii, smi->addr, 1, val);
	if (ret < 0)
		goto out;

	ret = smi->mii->write(smi->mii, smi->addr,
			      0, 0x9400 | (addr << 5) | reg);
	if (ret < 0)
		goto out;

	ret = smi_wait_ready(bus);

out:
	mutex_unlock(&smi->mii->mdio_lock);
	return ret;
}

static int smi_reset(struct mii_bus *bus)
{
	return 0;
}

static inline int smi_ppu_save_disable(struct mii_bus *bus)
{
	uint16_t gc;

	gc = smi_read(bus, SC_GLOBAL1, SC_GC);
	if (gc & SC_GC_PPU)
	{
		smi_write(bus, SC_GLOBAL1, SC_GC, gc & ~SC_GC_PPU);

                while ((smi_read(bus, SC_GLOBAL1, SC_GS) & 0xc000) == 0xc000)
			msleep(1);
	}
	return gc;
}

static inline void smi_ppu_restore(struct mii_bus *bus, uint16_t gc)
{
	if (gc & SC_GC_PPU)
        {
		smi_write(bus, SC_GLOBAL1, SC_GC, gc);

                while ((smi_read(bus, SC_GLOBAL1, SC_GS) & 0xc000) != 0xc000)
			msleep(1);

        }
}

int smi_phy_direct_read(struct mii_bus *bus, int addr, int reg)
{
	struct smi_phy_bus *smi_phy = bus->priv;
	uint16_t gc;
	int ret;

	mutex_lock(&smi_phy->smi->mdio_lock);
	gc = smi_ppu_save_disable(smi_phy->smi);

	ret = smi_read(smi_phy->smi, addr, reg);

	smi_ppu_restore(smi_phy->smi, gc);
	mutex_unlock(&smi_phy->smi->mdio_lock);
	return ret;
}

int smi_phy_direct_write(struct mii_bus *bus, int addr, int reg, uint16_t val)
{
	struct smi_phy_bus *smi_phy = bus->priv;
	uint16_t gc;
	int ret;

	mutex_lock(&smi_phy->smi->mdio_lock);
	gc = smi_ppu_save_disable(smi_phy->smi);

	ret = smi_write(smi_phy->smi, addr, reg, val);

	smi_ppu_restore(smi_phy->smi, gc);
	mutex_unlock(&smi_phy->smi->mdio_lock);
	return ret;
}

static int smi_phy_wait_ready(struct mii_bus *smi_bus)
{
	int retries, sc;

	for (retries = 100; retries; retries--) {
		sc = smi_read(smi_bus, SC_GLOBAL2, SC_SC);
		if (!(sc & SC_SC_BUSY))
			return 0;
	}
	return 1;
}

int smi_phy_indirect_read(struct mii_bus *bus, int addr, int reg)
{
	struct smi_phy_bus *smi_phy = bus->priv;
	int ret = -EIO;

	mutex_lock(&smi_phy->smi->mdio_lock);

	if (smi_phy_wait_ready(smi_phy->smi))
		goto out;

	if (smi_write(smi_phy->smi, SC_GLOBAL2,
		      SC_SC, 0x9800 | (addr << 5) | reg))
		goto out;

	if (smi_phy_wait_ready(smi_phy->smi))
		goto out;

	ret = smi_read(smi_phy->smi, SC_GLOBAL2, SC_SD);
out:
	mutex_unlock(&smi_phy->smi->mdio_lock);
	return ret;
}

int smi_phy_indirect_write(struct mii_bus *bus, int addr, int reg, uint16_t val)
{
	struct smi_phy_bus *smi_phy = bus->priv;
	int ret = -EIO;

	mutex_lock(&smi_phy->smi->mdio_lock);

	if (smi_phy_wait_ready(smi_phy->smi))
		goto out;

	if (smi_write(smi_phy->smi, SC_GLOBAL2, SC_SD, val))
		goto out;

	if (smi_write(smi_phy->smi, SC_GLOBAL2,
		      SC_SC, 0x9400 | (addr << 5) | reg))
		goto out;

	if (smi_phy_wait_ready(smi_phy->smi))
		goto out;
	    
	ret = 0;
out:
	mutex_unlock(&smi_phy->smi->mdio_lock);
	return ret;
}


struct mii_bus *smi_create(struct mii_bus *mii, uint8_t addr)
{
	struct mii_bus *bus;
	struct smi_bus *smi;
	int ret;

	bus = mdiobus_alloc_size(sizeof(*smi));
	if (!bus)
		return NULL;

	snprintf(bus->id, MII_BUS_ID_SIZE, "smi-%#.2x", addr);
	bus->name     = bus->id;
	bus->read     = smi_read;
	bus->write    = smi_write;
	bus->reset    = smi_reset;
	bus->parent   = &mii->dev;
	bus->phy_mask = 0xffffffff;

	smi = bus->priv;
	smi->mii  = mii;
	smi->addr = addr;

	ret = mdiobus_register(bus);
	if (ret) {
		mdiobus_free(bus);
		return NULL;
	}

	return bus;
}

struct mii_bus *smi_phy_create(struct mii_bus *smi_mii, enum smi_phy_mode mode)
{
	struct mii_bus *bus;
	struct smi_phy_bus *smi_phy;
	int ret;

	bus = mdiobus_alloc_size(sizeof(*smi_phy));
	if (!bus)
		return NULL;

	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-phy", smi_mii->id);
	bus->name     = bus->id;

	switch (mode) {
	case SMI_PHY_DIRECT:
		bus->read  = smi_phy_direct_read;
		bus->write = smi_phy_direct_write;
		break;

	case SMI_PHY_INDIRECT:
		bus->read  = smi_phy_indirect_read;
		bus->write = smi_phy_indirect_write;
		break;
	}

	bus->reset    = smi_reset;
	bus->parent   = &smi_mii->dev;
	bus->phy_mask = 0xffffffff;

	smi_phy = bus->priv;
	smi_phy->smi  = smi_mii;
	smi_phy->mode = mode;

	ret = mdiobus_register(bus);
	if (ret) {
		mdiobus_free(bus);
		return NULL;
	}

	return bus;
}
/**
 * Local Variables:
 *  version-control: t
 *  c-file-style: "linux"
 * End:
 */
