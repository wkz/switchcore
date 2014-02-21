/* \\/ Westermo - Switchcore driver
 *
 * Copyright (C) 2014  Westermo Teleindustri AB
 *
 * Author: Tobias Waldekranz <tobias.waldekranz@westermo.se>
 *
 */

#include <linux/kernel.h>
#include <net/rtnetlink.h>

#include "sc_private.h"

struct rtnl_link_ops sc_link_ops;

const struct nla_policy ifla_sc_policy[IFLA_SC_MAX+1] = {
	[IFLA_SC_PORTS] = { .len = sizeof(struct sc_ports_config) },
	[IFLA_SC_DEV]   = { .len = sizeof(struct sc_dev_config) },
	[IFLA_SC_CHAN]  = { .len = sizeof(struct sc_chan_config) },
};

static int sc_rtnl_changelink(struct net_device *dev,
			      struct nlattr *tb[],
			      struct nlattr *data[])
{
	struct sc *sc = netdev_priv(dev);
	int ret = 0;

	if (data[IFLA_SC_PORTS])
		ret |= sc_dev_add_ports(sc, nla_data(data[IFLA_SC_PORTS]));

	if (data[IFLA_SC_DEV])
		ret |= sc_dev_add(sc, nla_data(data[IFLA_SC_DEV]));

	if (data[IFLA_SC_CHAN])
		ret |= sc_chan_add(sc, nla_data(data[IFLA_SC_CHAN]));

	return ret;
}

static int sc_rtnl_newlink(struct net *src_net,
			   struct net_device *dev,
			   struct nlattr *tb[],
			   struct nlattr *data[])
{
	dev->rtnl_link_ops = &sc_link_ops;
	return register_netdevice(dev);
}

struct rtnl_link_ops sc_link_ops __read_mostly = {
	.kind		= "switchcore",
	.priv_size	= sizeof(struct sc),
	.maxtype        = IFLA_SC_MAX,
	.policy         = ifla_sc_policy,

	.setup          = sc_ndev_setup,
	.newlink	= sc_rtnl_newlink,
	.changelink     = sc_rtnl_changelink,
	/* .validate	= sc_validate, */
};

int __init sc_netlink_init(void)
{
	return rtnl_link_register(&sc_link_ops);
}

void __exit sc_netlink_exit(void)
{
	rtnl_link_unregister(&sc_link_ops);
}
/**
 * Local Variables:
 *  version-control: t
 *  c-file-style: "linux"
 * End:
 */
