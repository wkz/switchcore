/* \\/ Westermo - Switchcore driver
 *
 * Copyright (C) 2014  Westermo Teleindustri AB
 *
 * Author: Tobias Waldekranz <tobias.waldekranz@westermo.se>
 *
 */

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "msApi.h"
#include "msIoctl.h"

#include "sc_private.h"

#define MAX_IOCTL_BUFSIZE 512

static long sc_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct sc *sc = filp->private_data;
	struct sc_dev *dev;
	char buffer[MAX_IOCTL_BUFSIZE];
	char result[4];
	int rc = -EINVAL;
	unsigned int result_len = 0;
	QD_IOCTL ioctl_cmd;
	PGT_IOCTL_PARAM params;
	PGT_IOCTL_PARAM results;

	if (cmd != QD_IOCTL_CMD)
		return -EINVAL;

	if (copy_from_user(&ioctl_cmd, (void __user *)arg, sizeof(QD_IOCTL)))
		goto out;

	if (ioctl_cmd.insize >= MAX_IOCTL_BUFSIZE) {
		rc = -EINVAL;
		goto out;
	}

	if (copy_from_user(buffer, ioctl_cmd.inbuf, ioctl_cmd.insize))
		goto out;

	params  = (PGT_IOCTL_PARAM)buffer;
	results = (PGT_IOCTL_PARAM)result;

	switch (ioctl_cmd.cmd) {
	case IOCTL_gsysReadMiiReg:
		dev = &sc->dev[params->devIdx];

		if (!dev->smi) {
			netdev_warn(sc->ndev, "warning: read without smi\n");
			break;
		}

		mutex_lock(&dev->lock);
		rc = smi_read(dev->smi,
			      params->FirstParam.u32Data,
			      params->SecondParam.u32Data);

		mutex_unlock(&dev->lock);

		if (rc < 0)
			break;

		results->FirstParam.u32Data = rc;
		rc = 0;
		result_len = sizeof(int);
		break;

	case IOCTL_gsysWriteMiiReg:
		dev = &sc->dev[params->devIdx];

		if (!dev->smi) {
			netdev_warn(sc->ndev, "warning: write without smi\n");
			break;
		}

		mutex_lock(&dev->lock);
		rc = smi_write(sc->dev[params->devIdx].smi,
			       params->FirstParam.u32Data,
			       params->SecondParam.u32Data,
			       params->ThirdParam.u16Data);
		mutex_unlock(&dev->lock);

		result_len = 0;
		break;

	case IOCTL_UNM_INIT:
		/* make libarmwell happy */
		rc = 0;
		break;

	default:
		netdev_warn(sc->ndev, "warning: invalid ioctl 0x%x",
			    ioctl_cmd.cmd);
		break;
	}

	if (rc != 0) {
		ioctl_cmd.outsize = -1;
		netdev_err(sc->ndev, "error: ioctl 0x%x(size %d): %d",
			   ioctl_cmd.cmd, ioctl_cmd.insize, rc);
		rc = -EINVAL;
		goto out;
	}

	if (result_len > 0 && ioctl_cmd.outbuf && result_len <= ioctl_cmd.outsize)
		if (copy_to_user(ioctl_cmd.outbuf, result, result_len))
			goto out;
	ioctl_cmd.outsize = result_len;

	if (copy_to_user((void __user *)arg, (const void *)&ioctl_cmd, sizeof(QD_IOCTL)))
		goto out;

	rc = 0;
out:

	return rc;
}

static int sc_cdev_open(struct inode *inode, struct file *filp)
{
	struct sc *sc = container_of(inode->i_cdev, struct sc, cdev.cdev);

	filp->private_data = sc;
	return 0;
}

static int sc_cdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations sc_cdev_fops = {
	.open           = sc_cdev_open,
	.release        = sc_cdev_release,
	.unlocked_ioctl = sc_cdev_ioctl,
};

int sc_cdev_init(struct sc *sc)
{
	dev_t devnum;
	int err;

	err = alloc_chrdev_region(&devnum, 0, 1, "sc");
	if (err)
		return err;

	cdev_init(&sc->cdev.cdev, &sc_cdev_fops);
	sc->cdev.cdev.owner = THIS_MODULE;
	err = cdev_add(&sc->cdev.cdev, devnum, 1);
	if (err)
		return err;

	sc->cdev.class = class_create(sc->cdev.cdev.owner, "sc");
	if (IS_ERR(sc->cdev.class))
		return PTR_ERR(sc->cdev.class);

	sc->cdev.dev = device_create(sc->cdev.class, NULL, devnum, NULL, "sc0");
	if (!sc->cdev.dev)
		return -ENODEV;
		
	return 0;
}
/**
 * Local Variables:
 *  version-control: t
 *  c-file-style: "linux"
 * End:
 */
