// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/fs.h>
#include <linux/miscdevice.h>

#include "fbiob-ioctl.h"
#include "fboss-mdio.h"

#define MDIO_PHY_ADDR_MASK	0x1F
#define MDIO_DEV_ADDR_MASK	0x1F
#define MDIO_C45_REG_MASK	0xFFFF

#define MDIO_PHY_ADDR(_a)	((_a) & MDIO_PHY_ADDR_MASK)
#define MDIO_DEV_ADDR(_a)	((_a) & MDIO_DEV_ADDR_MASK)
#define MDIO_C45_REG(_r)	((_r) & MDIO_C45_REG_MASK)

static long fb_mdio_cdev_ioctl(struct file *file,
			       unsigned int cmd,
			       unsigned long param)
{
	int val;
	int ret = 0;
	struct fb_mdio_xfer xfer;
	void __user *userp = (void __user *)param;
	struct fb_mdio_desc *mdesc = container_of(file->private_data,
					struct fb_mdio_desc, miscdev);
	struct mii_bus *bus = mdesc->bus;

	if (copy_from_user(&xfer, userp, sizeof(xfer)))
		return -EFAULT;

	switch (cmd) {
	case FBIOC_MDIO_C45_READ:
		val = mdiobus_c45_read(bus, MDIO_PHY_ADDR(xfer.phy_addr),
					MDIO_DEV_ADDR(xfer.dev_addr),
					MDIO_C45_REG(xfer.reg_addr));
		if (val < 0) {
			ret = val;
		} else {
			xfer.reg_data = MDIO_C45_REG(val);
			if (copy_to_user(userp, &xfer, sizeof(xfer)))
				ret = -EFAULT;
		}
		break;

	case FBIOC_MDIO_C45_WRITE:
		ret = mdiobus_c45_write(bus, MDIO_PHY_ADDR(xfer.phy_addr),
					MDIO_DEV_ADDR(xfer.dev_addr),
					MDIO_C45_REG(xfer.reg_addr),
					MDIO_C45_REG(xfer.reg_data));
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations fb_mdio_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fb_mdio_cdev_ioctl,
};

void fb_mdio_cdev_destroy(struct fb_mdio_desc *mdesc)
{
	misc_deregister(&mdesc->miscdev);
}

int fb_mdio_cdev_init(struct fb_mdio_desc *mdesc, struct mii_bus *bus, u32 id)
{
	snprintf(mdesc->name, sizeof(mdesc->name), "fb-mdio-%u", id);
	mdesc->miscdev.minor = MISC_DYNAMIC_MINOR;
	mdesc->miscdev.name = mdesc->name;
	mdesc->miscdev.fops = &fb_mdio_fops;
	mdesc->miscdev.parent = bus->parent;
	mdesc->bus = bus;

	return misc_register(&mdesc->miscdev);
}
