// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 Nexthop Systems Inc.
//
// FPGA I2C controller driver for the auxiliary devices the fboss_iob_pci
// parent creates. The parent maps nothing, so the driver maps the registers it
// needs itself; interrupts and the reference clock come from nh_irq.

#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/auxiliary_bus.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/property.h>
#include <linux/sysfs.h>
#include <linux/io.h>

#include "fbiob-auxdev.h"
#include "nh_fpga_i2c.h"
#include "nh_fpga_i2c_mux.h"
#include "nh_irq.h"
#include "nh_fpga_i2c_masters.h"
#include "platform/nh_platform.h"

#define DRIVER_NAME "nh_fpga_i2c"
#define DRIVER_VERSION "1.0"

/* Monotonic id for the global xiic-i2c platform-device namespace. */
static atomic_t nh_fpga_xiic_pdev_id = ATOMIC_INIT(0);

/* Platform-device names of the interrupt-driven and polled xiic drivers. */
#define NH_XIIC_IRQ_DRV "xiic-i2c"
#define NH_XIIC_POLL_DRV "nh-xiic-i2c"

/*
 * I2C master controllers occupy a contiguous block of the FPGA CSR space,
 * starting at NH_FPGA_I2C_BLOCK_BASE with one controller every
 * NH_FPGA_I2C_MASTER_STRIDE bytes. A controller's logical master index is its
 * position in that block, which is what mux_config.logical_master encodes.
 * These values are consistent across all supported SKUs.
 */
#define NH_FPGA_I2C_BLOCK_BASE 0x40000
#define NH_FPGA_I2C_MASTER_STRIDE 0x200

/* Configure mux support based on platform data and CSR offset */
static int configure_mux_support(struct nh_fpga_i2c *i2c,
				 struct fbiob_aux_adapter *adap,
				 struct auxiliary_device *aux_dev,
				 int master_id)
{
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	u16 device_id = pdev->device;
	u32 csr_offset = nh_csr_rel(pdev, adap->data.csr_offset);
	int logical_master;
	const struct nh_platform_cfg *platform_cfg;
	int j;

	dev_dbg(&aux_dev->dev, "FPGA device ID: 0x%04x, Master ID: %d\n",
		device_id, master_id);

	/* Initialize mux support fields */
	i2c->num_mux_channels = 0;
	i2c->mux_reg = NULL;
	i2c->mux_client = NULL;

	/*
	 * Derive the logical master index from the controller's position in the
	 * FPGA CSR block, not from master_id. mux_config.logical_master is that
	 * positional index. The previous "master_id % 10" proxy aliased
	 * transceiver (OSFP) masters whose id shared a last digit with a
	 * mux-capable master onto the mux table, creating phantom muxes at 0x71
	 * and corrupting the i2c topology.
	 */
	if (csr_offset < NH_FPGA_I2C_BLOCK_BASE) {
		dev_dbg(&aux_dev->dev,
			"csr_offset 0x%x below i2c block base 0x%x — no mux\n",
			csr_offset, NH_FPGA_I2C_BLOCK_BASE);
		return 0;
	}
	if ((csr_offset - NH_FPGA_I2C_BLOCK_BASE) % NH_FPGA_I2C_MASTER_STRIDE) {
		dev_warn(
			&aux_dev->dev,
			"csr_offset 0x%x not aligned to master stride 0x%x — no mux\n",
			csr_offset, NH_FPGA_I2C_MASTER_STRIDE);
		return 0;
	}
	logical_master = (csr_offset - NH_FPGA_I2C_BLOCK_BASE) /
			 NH_FPGA_I2C_MASTER_STRIDE;

	platform_cfg = nh_get_platform_dev(pdev);
	if (!platform_cfg) {
		dev_warn(&aux_dev->dev,
			 "Unknown device 0x%04x — no mux support\n", device_id);
		return 0;
	}

	if (!platform_cfg->mux_cfg) {
		dev_dbg(&aux_dev->dev, "%s: no mux support\n",
			platform_cfg->name);
		return 0;
	}

	/* Known platform with mux — search for this logical master */
	for (j = 0; j < platform_cfg->mux_count; j++) {
		if (platform_cfg->mux_cfg[j].logical_master == logical_master) {
			/* Mux select register: shared FPGA CSR space, so not requested. */
			i2c->mux_reg = devm_ioremap(
				&aux_dev->dev,
				pci_resource_start(pdev, 0) +
					platform_cfg->mux_cfg[j].reg_offset,
				sizeof(u32));
			if (!i2c->mux_reg)
				return -ENOMEM;
			i2c->num_mux_channels =
				platform_cfg->mux_cfg[j].num_channels;
			i2c->mux_pdata.mux_reg = i2c->mux_reg;
			i2c->mux_pdata.num_channels = i2c->num_mux_channels;
			/* Lets sibling drivers address a channel without
			 * parsing the adapter name, whose "#N" is a
			 * dynamically assigned auxiliary device id. */
			i2c->mux_pdata.pdev = pdev;
			i2c->mux_pdata.logical_master = logical_master;
			dev_dbg(&aux_dev->dev,
				"%s master %d: %s, %d channels at offset 0x%04x\n",
				platform_cfg->name, logical_master,
				platform_cfg->mux_cfg[j].description,
				platform_cfg->mux_cfg[j].num_channels,
				platform_cfg->mux_cfg[j].reg_offset);
			return 0;
		}
	}

	dev_dbg(&aux_dev->dev, "%s master %d: no mux\n", platform_cfg->name,
		logical_master);
	return 0;
}

/* Auxiliary device probe function */
int nh_fpga_i2c_probe(struct auxiliary_device *aux_dev,
		      const struct auxiliary_device_id *id)
{
	struct fbiob_aux_adapter *adap;
	struct pci_dev *pdev = to_pci_dev(aux_dev->dev.parent);
	const struct nh_platform_cfg *platform_cfg;
	struct nh_fpga_i2c *i2c;
	struct clk_hw *clk_hw;
	resource_size_t mem_start, mem_size;
	const char *xiic_drv;
	int ret, master_id, pdev_id, virq;

	adap = container_of(aux_dev, struct fbiob_aux_adapter, auxdev);

	platform_cfg = nh_get_platform_dev(pdev);
	if (!platform_cfg || !platform_cfg->irq_cfg)
		return -ENODEV;

	i2c = devm_kzalloc(&aux_dev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->aux_dev = aux_dev;
	i2c->master_bus_num = -1;
	master_id = adap->data.id.id;

	dev_info(&aux_dev->dev,
		 "Initializing I2C Master %d at bus addr 0x%x\n",
		 master_id, adap->data.csr_offset);

	/* Resolve mux topology before the bus notifier fires (during pdev add). */
	ret = configure_mux_support(i2c, adap, aux_dev, master_id);
	if (ret) {
		dev_err(&aux_dev->dev, "Failed to configure mux support: %d\n",
			ret);
		return ret;
	}

	/* nh_irq sets the FPGA up on the first call, inside the creation ioctl,
	 * before PlatformManager scans for the new bus. */
	ret = nh_irq_resolve(pdev, nh_csr_rel(pdev, adap->data.csr_offset),
			     &virq, &clk_hw);
	if (ret) {
		dev_err(&aux_dev->dev,
			"Failed to resolve IRQ and clock for Master %d: %d\n",
			master_id, ret);
		return ret;
	}

	/* Unique xiic-i2c pdev id from a plain counter: master_id collides
	 * across FPGAs, and a csr-offset-derived index aliases the M4062 ext
	 * masters onto existing ids (-EEXIST). */
	pdev_id = atomic_inc_return(&nh_fpga_xiic_pdev_id);
	xiic_drv = platform_cfg->i2c_poll ? NH_XIIC_POLL_DRV : NH_XIIC_IRQ_DRV;

	/* clkdev alias on the child pdev name so the xiic driver's clk_get finds it. */
	i2c->cl = clkdev_hw_create(clk_hw, NULL, "%s.%d", xiic_drv, pdev_id);
	if (!i2c->cl) {
		dev_err(&aux_dev->dev,
			"Failed to create clk_lookup for Master %d\n",
			master_id);
		return -ENOMEM;
	}

	/* csr_offset is already the bus address. */
	mem_start = adap->data.csr_offset;
	mem_size = platform_cfg->irq_cfg->i2c_csr_channel_size;

	struct resource res[] = {
		DEFINE_RES_MEM(mem_start, mem_size),
		DEFINE_RES_IRQ(virq),
	};

	/* alloc+add split so drvdata is set before the probe/notifier runs. */
	i2c->pdev = platform_device_alloc(xiic_drv, pdev_id);
	if (!i2c->pdev) {
		dev_err(&aux_dev->dev, "Failed to alloc %s.%d\n", xiic_drv,
			pdev_id);
		ret = -ENOMEM;
		goto err_drop_clk;
	}
	i2c->pdev->dev.parent = &aux_dev->dev;

	ret = platform_device_add_resources(i2c->pdev, res, ARRAY_SIZE(res));
	if (ret) {
		dev_err(&aux_dev->dev,
			"Failed to add resources to %s.%d: %d\n", xiic_drv,
			pdev_id, ret);
		goto err_put_pdev;
	}

	/* clock-frequency for i2c-xiic via software node (it defaults to 100kHz). */
	if (adap->data.i2c_data.bus_freq_hz) {
		struct property_entry props[] = {
			PROPERTY_ENTRY_U32("clock-frequency",
					   adap->data.i2c_data.bus_freq_hz),
			{}
		};

		ret = device_create_managed_software_node(&i2c->pdev->dev,
							  props, NULL);
		if (ret) {
			dev_err(&aux_dev->dev,
				"Failed to attach clock-frequency property: %d\n",
				ret);
			goto err_put_pdev;
		}
	}

	auxiliary_set_drvdata(aux_dev, i2c);

	ret = platform_device_add(i2c->pdev);
	if (ret) {
		dev_err(&aux_dev->dev, "Failed to add %s.%d: %d\n", xiic_drv,
			pdev_id, ret);
		auxiliary_set_drvdata(aux_dev, NULL);
		goto err_put_pdev;
	}

	dev_info(
		&aux_dev->dev,
		"Registered %s.%d for master %d (mem=0x%llx+0x%llx, virq=%d)\n",
		xiic_drv, pdev_id, master_id, (u64)mem_start, (u64)mem_size,
		virq);

	return 0;

err_put_pdev:
	platform_device_put(i2c->pdev);
	i2c->pdev = NULL;
err_drop_clk:
	clkdev_drop(i2c->cl);
	i2c->cl = NULL;
	return ret;
}

/* Auxiliary device remove function */
void nh_fpga_i2c_remove(struct auxiliary_device *aux_dev)
{
	struct nh_fpga_i2c *i2c = auxiliary_get_drvdata(aux_dev);

	if (!i2c) {
		dev_warn(&aux_dev->dev, "I2C remove: controller is NULL\n");
		return;
	}

	/* Drop the compat symlinks before their targets disappear. */
	for (int i = 0; i < i2c->num_mux_channels; i++) {
		char name[16];

		snprintf(name, sizeof(name), "channel-%d", i);
		sysfs_remove_link(&aux_dev->dev.kobj, name);
	}
	if (i2c->master_bus_num >= 0) {
		char name[16];

		snprintf(name, sizeof(name), "i2c-%d", i2c->master_bus_num);
		sysfs_remove_link(&aux_dev->dev.kobj, name);
		i2c->master_bus_num = -1;
	}

	/* Remove the mux client before the xiic pdev: its reference on the master
	 * adapter would otherwise wedge i2c_del_adapter() and hang rmmod. */
	if (i2c->mux_client) {
		i2c_unregister_device(i2c->mux_client);
		i2c->mux_client = NULL;
	}

	if (i2c->pdev) {
		platform_device_unregister(i2c->pdev);
		i2c->pdev = NULL;
	}

	if (i2c->cl) {
		clkdev_drop(i2c->cl);
		i2c->cl = NULL;
	}

	auxiliary_set_drvdata(aux_dev, NULL);

	dev_info(&aux_dev->dev, "FPGA I2C controller removed\n");
}

/* nh_ prefix keeps the match apart from fboss_iob_i2c's i2c_master. */
static const struct auxiliary_device_id nh_fpga_i2c_aux_id_table[] = {
	{
		.name = FBOSS_IOB_PCI_DRIVER ".nh_i2c_master",
	},
	{
		.name = FBOSS_IOB_PCI_DRIVER ".i2c_master_ext",
	},
	{},
};
MODULE_DEVICE_TABLE(auxiliary, nh_fpga_i2c_aux_id_table);

/* Auxiliary driver structure */
static struct auxiliary_driver nh_fpga_i2c_aux_driver = {
	.name = DRIVER_NAME,
	.probe = nh_fpga_i2c_probe,
	.remove = nh_fpga_i2c_remove,
	.id_table = nh_fpga_i2c_aux_id_table,
};

/* Walk up the device tree looking for a device bound to our aux driver.
 * Returns the matching device, or NULL if none found within max_depth hops. */
static struct device *nh_fpga_i2c_find_owning_aux(struct device *dev,
						  int max_depth)
{
	for (int i = 0; i < max_depth && dev; i++) {
		if (dev->driver == &nh_fpga_i2c_aux_driver.driver)
			return dev;
		dev = dev->parent;
	}
	return NULL;
}

/* Create i2c-N / channel-N compatibility symlinks under the aux device kobj
 * so PlatformManager can discover the buses at the expected location */
static void nh_fpga_i2c_add_compat_link(struct device *aux_kdev,
					struct i2c_adapter *adap,
					const char *link_name)
{
	int ret =
		sysfs_create_link(&aux_kdev->kobj, &adap->dev.kobj, link_name);
	if (ret && ret != -EEXIST)
		dev_warn(aux_kdev, "Failed to create %s symlink: %d\n",
			 link_name, ret);
}

/* i2c_bus_type notifier: renames our xiic master adapters, creates the compat
 * symlinks, and instantiates the fpga-mux client. Handles both the master
 * adapter and the mux channel adapters. */
static int nh_fpga_i2c_bus_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct device *dev = data;
	struct i2c_adapter *i2c_adap;
	struct device *aux_kdev;
	struct auxiliary_device *aux_dev;
	struct fbiob_aux_adapter *adap;
	struct nh_fpga_i2c *i2c;
	char link_name[16];

	if (action != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	i2c_adap = i2c_verify_adapter(dev);
	if (!i2c_adap || !dev->parent)
		return NOTIFY_DONE;

	/* Case 1: master adapter (parent xiic-i2c pdev, grandparent aux dev). */
	aux_kdev = nh_fpga_i2c_find_owning_aux(dev->parent->parent, 1);
	if (aux_kdev) {
		struct pci_dev *pdev = to_pci_dev(aux_kdev->parent);

		aux_dev = container_of(aux_kdev, struct auxiliary_device, dev);
		adap = container_of(aux_dev, struct fbiob_aux_adapter, auxdev);
		i2c = auxiliary_get_drvdata(aux_dev);

		if (!i2c || i2c->pdev != to_platform_device(dev->parent))
			return NOTIFY_DONE;

		snprintf(i2c_adap->name, sizeof(i2c_adap->name),
			 "FPGA %s I2C Adapter #%d", pci_name(pdev),
			 adap->data.id.id);

		snprintf(link_name, sizeof(link_name), "i2c-%d", i2c_adap->nr);
		nh_fpga_i2c_add_compat_link(aux_kdev, i2c_adap, link_name);
		i2c->master_bus_num = i2c_adap->nr;

		/* Instantiate the fpga-mux client at 0x71 (a register-mapped
		 * pseudo-address) when this master has a mux. */
		if (i2c->num_mux_channels > 0 && !i2c->mux_client) {
			struct i2c_board_info info = {
				I2C_BOARD_INFO("fpga-mux", 0x71),
				.platform_data = &i2c->mux_pdata,
			};

			i2c->mux_client = i2c_new_client_device(i2c_adap, &info);
			if (IS_ERR(i2c->mux_client)) {
				dev_warn(
					&aux_dev->dev,
					"Failed to instantiate FPGA mux: %ld\n",
					PTR_ERR(i2c->mux_client));
				i2c->mux_client = NULL;
			}
		}

		return NOTIFY_OK;
	}

	/* Case 2: mux channel adapter (walk up to our aux device). */
	aux_kdev = nh_fpga_i2c_find_owning_aux(dev->parent, 4);
	if (aux_kdev) {
		int chan_id;

		aux_dev = container_of(aux_kdev, struct auxiliary_device, dev);
		i2c = auxiliary_get_drvdata(aux_dev);
		if (!i2c)
			return NOTIFY_DONE;

		/* Parse chan_id from the i2c-mux core's "i2c-M-mux (chan_id K)" name. */
		if (sscanf(i2c_adap->name, "i2c-%*d-mux (chan_id %d)",
			   &chan_id) != 1)
			return NOTIFY_DONE;

		snprintf(link_name, sizeof(link_name), "channel-%d", chan_id);
		nh_fpga_i2c_add_compat_link(aux_kdev, i2c_adap, link_name);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static struct notifier_block nh_fpga_i2c_nb = {
	.notifier_call = nh_fpga_i2c_bus_notify,
};

/* Module initialization */
static int __init nh_fpga_i2c_init_module(void)
{
	int ret;

	pr_info("NH FPGA I2C driver v%s loading\n", DRIVER_VERSION);

	ret = bus_register_notifier(&i2c_bus_type, &nh_fpga_i2c_nb);
	if (ret) {
		pr_err("Failed to register i2c bus notifier: %d\n", ret);
		return ret;
	}

	ret = auxiliary_driver_register(&nh_fpga_i2c_aux_driver);
	if (ret) {
		pr_err("Failed to register auxiliary driver: %d\n", ret);
		bus_unregister_notifier(&i2c_bus_type, &nh_fpga_i2c_nb);
		return ret;
	}

	pr_info("NH FPGA I2C driver loaded successfully\n");
	return 0;
}

/* Module cleanup */
static void __exit nh_fpga_i2c_exit_module(void)
{
	pr_info("NH FPGA I2C driver unloading\n");
	auxiliary_driver_unregister(&nh_fpga_i2c_aux_driver);
	bus_unregister_notifier(&i2c_bus_type, &nh_fpga_i2c_nb);
	pr_info("NH FPGA I2C driver unloaded\n");
}

module_init(nh_fpga_i2c_init_module);
module_exit(nh_fpga_i2c_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Arif Mohammad <marif@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA I2C Controller Driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_SOFTDEP("pre: i2c-xiic nh_xiic nh_fpga_mux nh_irq");
