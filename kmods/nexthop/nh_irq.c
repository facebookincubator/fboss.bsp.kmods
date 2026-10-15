// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2026 Nexthop Systems Inc.
//
// MSI vectors and reference clock for the Nexthop FPGAs under the
// fboss_iob_pci parent, set up per FPGA on first use by nh_fpga_i2c and the
// LED drivers.

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "nh_fpga_irq.h"
#include "nh_irq.h"
#include "platform/nh_platform.h"

#define DRIVER_NAME "nh_irq"
#define DRIVER_VERSION "1.0"

/* One regmap_irq per status-register bit; unused bits never get unmasked. */
#define NH_FPGA_IRQS_PER_DOMAIN 32

static const struct regmap_config nh_irq_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

/* Per-FPGA context, keyed by the parent pci_dev. Created on first use and
 * destroyed by the parent-unbind devres action or at module exit;
 * nh_irq_ctxs_lock covers creation, lookup and destruction. */
struct nh_irq_ctx {
	struct list_head node;
	struct pci_dev *pdev; /* we hold a reference */
	bool unbind_hook_armed;
	const struct nh_fpga_irq_cfg *irq_cfg; /* NULL: no interrupt-driven I2C */
	void __iomem *mmio_base; /* BAR0, shared with the LED and xcvr drivers */
	struct regmap *regmap;
	struct regmap_irq_chip *irq_chips; /* one per MSI domain */
	struct regmap_irq_chip_data **irq_chip_data;
	int num_chips_added;
	struct clk_hw *ref_clk_hw; /* AXI IIC reference clock */
	bool msi_allocated;
	bool bus_master_set;
	struct mutex led_reg_lock; /* LED register RMW, shared by the LED drivers */
};

static LIST_HEAD(nh_irq_ctxs);
static DEFINE_MUTEX(nh_irq_ctxs_lock);

static void nh_irq_parent_unbind_action(void *data);

static struct nh_irq_ctx *nh_irq_find_ctx(struct pci_dev *pdev)
{
	struct nh_irq_ctx *ctx;

	lockdep_assert_held(&nh_irq_ctxs_lock);
	list_for_each_entry(ctx, &nh_irq_ctxs, node) {
		if (ctx->pdev == pdev)
			return ctx;
	}
	return NULL;
}

static int nh_irq_init_one_chip(struct nh_irq_ctx *ctx, int idx)
{
	struct device *dev = &ctx->pdev->dev;
	const struct nh_fpga_msi_domain_cfg *dom = &ctx->irq_cfg->domains[idx];
	struct regmap_irq_chip *chip = &ctx->irq_chips[idx];
	struct regmap_irq *irqs;
	int linux_irq, i, ret;

	irqs = kcalloc(NH_FPGA_IRQS_PER_DOMAIN, sizeof(*irqs), GFP_KERNEL);
	if (!irqs)
		return -ENOMEM;

	for (i = 0; i < NH_FPGA_IRQS_PER_DOMAIN; i++) {
		irqs[i].reg_offset = 0;
		irqs[i].mask = BIT(i);
	}

	chip->name = dom->chip_name;
	chip->status_base = dom->isr_offset;
	chip->unmask_base = dom->ier_offset;
	chip->ack_base = dom->isr_offset; /* write 1 to clear via status reg */
	/*
	 * Bits latched before the driver probes (e.g. across a host reboot
	 * while the FPGA stays powered) are never acked otherwise, which wedges
	 * the corresponding I2C master forever.
	 */
	chip->init_ack_masked = true;
	chip->num_regs = 1;
	chip->irqs = irqs;
	chip->num_irqs = NH_FPGA_IRQS_PER_DOMAIN;

	linux_irq = pci_irq_vector(ctx->pdev, idx);
	if (linux_irq < 0) {
		dev_err(dev, "Failed to get Linux IRQ for MSI domain %d: %d\n",
			idx, linux_irq);
		kfree(irqs);
		return linux_irq;
	}

	ret = regmap_add_irq_chip(ctx->regmap, linux_irq, 0, 0, chip,
				  &ctx->irq_chip_data[idx]);
	if (ret) {
		dev_err(dev,
			"Failed to register regmap_irq_chip for domain %d: %d\n",
			idx, ret);
		kfree(irqs);
		return ret;
	}
	ctx->num_chips_added++;

	dev_info(dev,
		 "Registered regmap_irq_chip '%s' for MSI domain %d on Linux IRQ %d\n",
		 dom->chip_name, idx, linux_irq);
	return 0;
}

static void nh_irq_release_irq_resources(struct nh_irq_ctx *ctx)
{
	int i;

	for (i = ctx->num_chips_added - 1; i >= 0; i--) {
		int linux_irq = pci_irq_vector(ctx->pdev, i);

		regmap_del_irq_chip(linux_irq, ctx->irq_chip_data[i]);
		kfree(ctx->irq_chips[i].irqs);
	}
	ctx->num_chips_added = 0;
	kfree(ctx->irq_chip_data);
	ctx->irq_chip_data = NULL;
	kfree(ctx->irq_chips);
	ctx->irq_chips = NULL;
	if (ctx->regmap) {
		regmap_exit(ctx->regmap);
		ctx->regmap = NULL;
	}
	if (ctx->ref_clk_hw) {
		clk_hw_unregister_fixed_rate(ctx->ref_clk_hw);
		ctx->ref_clk_hw = NULL;
	}
	if (ctx->msi_allocated) {
		u16 cmd;

		/* pci_free_irq_vectors() re-enables INTx through pci_intx(),
		 * which on a pcim-managed device adds a devres entry; added
		 * from the parent's devres release it outlives the unbind and
		 * the next probe fails with -EBUSY. Clear INTX_DISABLE first
		 * so there is nothing to change. */
		pci_read_config_word(ctx->pdev, PCI_COMMAND, &cmd);
		if (cmd & PCI_COMMAND_INTX_DISABLE)
			pci_write_config_word(ctx->pdev, PCI_COMMAND,
					      cmd & ~PCI_COMMAND_INTX_DISABLE);
		pci_free_irq_vectors(ctx->pdev);
		ctx->msi_allocated = false;
	}
	if (ctx->bus_master_set) {
		pci_clear_master(ctx->pdev);
		ctx->bus_master_set = false;
	}
	if (ctx->mmio_base) {
		iounmap(ctx->mmio_base);
		ctx->mmio_base = NULL;
	}
	ctx->irq_cfg = NULL;
}

/* Caller holds nh_irq_ctxs_lock. */
static void nh_irq_destroy_ctx(struct nh_irq_ctx *ctx)
{
	list_del(&ctx->node);
	if (ctx->unbind_hook_armed)
		devm_remove_action(&ctx->pdev->dev, nh_irq_parent_unbind_action,
				   ctx->pdev);
	nh_irq_release_irq_resources(ctx);
	pci_dev_put(ctx->pdev);
	kfree(ctx);
}

/* Runs at the parent's unbind, after remove() destroyed the auxiliary
 * devices and before the PCI core frees the MSI vectors. */
static void nh_irq_parent_unbind_action(void *data)
{
	struct pci_dev *pdev = data;
	struct nh_irq_ctx *ctx;

	mutex_lock(&nh_irq_ctxs_lock);
	ctx = nh_irq_find_ctx(pdev);
	if (ctx) {
		ctx->unbind_hook_armed = false;
		dev_info(&pdev->dev,
			 "parent driver unbinding; releasing interrupt resources\n");
		nh_irq_destroy_ctx(ctx);
	}
	mutex_unlock(&nh_irq_ctxs_lock);
}

/* Caller holds nh_irq_ctxs_lock. */
static struct nh_irq_ctx *nh_irq_create_ctx(struct pci_dev *pdev)
{
	const struct nh_platform_cfg *platform_cfg;
	const struct nh_fpga_irq_cfg *irq_cfg;
	struct nh_irq_ctx *ctx;
	char *clk_name;
	u32 fpga_base_fn, min_base_fn;
	int ret, i;

	platform_cfg = nh_get_platform_dev(pdev);
	if (!platform_cfg)
		return ERR_PTR(-ENODEV);
	irq_cfg = platform_cfg->irq_cfg;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);
	ctx->pdev = pci_dev_get(pdev);
	mutex_init(&ctx->led_reg_lock);

	/* Shared with the LED and transceiver drivers; not requested, the xiic
	 * children claim sub-ranges of it. */
	ctx->mmio_base = ioremap(pci_resource_start(pdev, 0),
				 pci_resource_len(pdev, 0));
	if (!ctx->mmio_base) {
		ret = -ENOMEM;
		goto err;
	}

	if (!irq_cfg)
		goto register_ctx;

	min_base_fn = irq_cfg->min_fpga_version & NH_FPGA_VERSION_MASK;
	fpga_base_fn = ioread32(ctx->mmio_base + NH_FPGA_VERSION_REG_OFFSET) &
		       NH_FPGA_VERSION_MASK;
	if (min_base_fn && fpga_base_fn < min_base_fn) {
		dev_info(&pdev->dev,
			 "FPGA firmware 0x%04x older than the 0x%04x required for interrupt-driven I2C; I2C disabled\n",
			 fpga_base_fn, min_base_fn);
		goto register_ctx;
	}

	/* MSI needs bus mastering; the parent does not enable it. */
	pci_set_master(pdev);
	ctx->bus_master_set = true;

	ret = pci_alloc_irq_vectors(pdev, irq_cfg->num_domains,
				    irq_cfg->num_domains, PCI_IRQ_MSI);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to allocate %d MSI vectors: %d\n",
			irq_cfg->num_domains, ret);
		goto err;
	}
	ctx->msi_allocated = true;
	dev_info(&pdev->dev, "Allocated %d MSI vectors\n", ret);

	/* regmap_add_irq_chip() takes the IRQ domain's fwnode from this device. */
	ctx->regmap = regmap_init_mmio(&pdev->dev, ctx->mmio_base,
				       &nh_irq_regmap_config);
	if (IS_ERR(ctx->regmap)) {
		ret = PTR_ERR(ctx->regmap);
		ctx->regmap = NULL;
		dev_err(&pdev->dev, "Failed to initialize regmap: %d\n", ret);
		goto err;
	}

	ctx->irq_cfg = irq_cfg;
	ctx->irq_chips = kcalloc(irq_cfg->num_domains, sizeof(*ctx->irq_chips),
				 GFP_KERNEL);
	ctx->irq_chip_data = kcalloc(irq_cfg->num_domains,
				     sizeof(*ctx->irq_chip_data), GFP_KERNEL);
	if (!ctx->irq_chips || !ctx->irq_chip_data) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < irq_cfg->num_domains; i++) {
		ret = nh_irq_init_one_chip(ctx, i);
		if (ret)
			goto err;
	}

	clk_name = kasprintf(GFP_KERNEL, "%s-refclk", pci_name(pdev));
	if (!clk_name) {
		ret = -ENOMEM;
		goto err;
	}
	ctx->ref_clk_hw = clk_hw_register_fixed_rate(NULL, clk_name, NULL, 0,
						     irq_cfg->ref_clk_hz);
	kfree(clk_name);
	if (IS_ERR(ctx->ref_clk_hw)) {
		ret = PTR_ERR(ctx->ref_clk_hw);
		ctx->ref_clk_hw = NULL;
		dev_err(&pdev->dev, "Failed to register reference clock: %d\n",
			ret);
		goto err;
	}
	dev_info(&pdev->dev, "Registered reference clock at %u Hz\n",
		 irq_cfg->ref_clk_hz);

register_ctx:
	/* Newest devres action on the parent, so it runs first at unbind. */
	ret = devm_add_action(&pdev->dev, nh_irq_parent_unbind_action, pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register parent-unbind hook: %d\n",
			ret);
		goto err;
	}
	ctx->unbind_hook_armed = true;

	list_add(&ctx->node, &nh_irq_ctxs);
	dev_info(&pdev->dev, "nh_irq companion ready for %s (%s)\n",
		 pci_name(pdev),
		 ctx->irq_cfg ? "interrupt-driven I2C" : "no interrupts");
	return ctx;

err:
	nh_irq_release_irq_resources(ctx);
	pci_dev_put(ctx->pdev);
	kfree(ctx);
	return ERR_PTR(ret);
}

/* Caller holds nh_irq_ctxs_lock. */
static struct nh_irq_ctx *nh_irq_get_ctx(struct pci_dev *pdev)
{
	struct nh_irq_ctx *ctx = nh_irq_find_ctx(pdev);

	return ctx ? ctx : nh_irq_create_ctx(pdev);
}

int nh_irq_resolve(struct pci_dev *pdev, u32 csr_offset_rel, int *virq_out,
		   struct clk_hw **clk_hw_out)
{
	const struct nh_fpga_irq_cfg *cfg;
	struct nh_irq_ctx *ctx;
	int virq, master_index, i, ret = 0;
	u8 msi_domain = 0, hw_irq = 0;

	if (!pdev || !virq_out || !clk_hw_out)
		return -EINVAL;

	mutex_lock(&nh_irq_ctxs_lock);
	ctx = nh_irq_get_ctx(pdev);
	if (IS_ERR(ctx)) {
		ret = PTR_ERR(ctx);
		goto out;
	}

	cfg = ctx->irq_cfg;
	if (!cfg) {
		ret = -ENOENT;
		goto out;
	}

	/* Same master lookup as nh_fpga_resolve_master_irq_clk(). */
	if (csr_offset_rel < cfg->i2c_csr_base || !cfg->i2c_csr_channel_size) {
		ret = -ENOENT;
		goto out;
	}
	master_index =
		(csr_offset_rel - cfg->i2c_csr_base) / cfg->i2c_csr_channel_size;

	for (i = 0; i < cfg->num_domains; i++) {
		const struct nh_fpga_msi_domain_cfg *dom = &cfg->domains[i];

		if (master_index >= dom->first_master &&
		    master_index <= dom->last_master) {
			msi_domain = i;
			hw_irq = dom->first_hw_irq +
				 (master_index - dom->first_master);
			break;
		}
	}
	if (i == cfg->num_domains) {
		ret = -ENOENT;
		goto out;
	}

	if (!ctx->irq_chip_data[msi_domain]) {
		ret = -ENODEV;
		goto out;
	}

	virq = regmap_irq_get_virq(ctx->irq_chip_data[msi_domain], hw_irq);
	if (virq < 0) {
		ret = virq;
		goto out;
	}
	if (virq == 0) {
		ret = -ENODEV;
		goto out;
	}

	*virq_out = virq;
	*clk_hw_out = ctx->ref_clk_hw;
out:
	mutex_unlock(&nh_irq_ctxs_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(nh_irq_resolve);

struct mutex *nh_irq_led_reg_lock(struct pci_dev *pdev)
{
	struct nh_irq_ctx *ctx;

	mutex_lock(&nh_irq_ctxs_lock);
	ctx = nh_irq_get_ctx(pdev);
	mutex_unlock(&nh_irq_ctxs_lock);
	return IS_ERR(ctx) ? NULL : &ctx->led_reg_lock;
}
EXPORT_SYMBOL_GPL(nh_irq_led_reg_lock);

void __iomem *nh_irq_mmio_base(struct pci_dev *pdev)
{
	struct nh_irq_ctx *ctx;

	mutex_lock(&nh_irq_ctxs_lock);
	ctx = nh_irq_get_ctx(pdev);
	mutex_unlock(&nh_irq_ctxs_lock);
	return IS_ERR(ctx) ? NULL : ctx->mmio_base;
}
EXPORT_SYMBOL_GPL(nh_irq_mmio_base);

static void nh_irq_teardown_all(void)
{
	struct nh_irq_ctx *ctx, *tmp;

	mutex_lock(&nh_irq_ctxs_lock);
	list_for_each_entry_safe(ctx, tmp, &nh_irq_ctxs, node)
		nh_irq_destroy_ctx(ctx);
	mutex_unlock(&nh_irq_ctxs_lock);
}

static int __init nh_irq_init(void)
{
	pr_info("nh_irq companion loaded\n");
	return 0;
}

static void __exit nh_irq_exit(void)
{
	nh_irq_teardown_all();
	pr_info("nh_irq companion unloaded\n");
}

module_init(nh_irq_init);
module_exit(nh_irq_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Spandan Chowdhury <spandan@nexthop.ai>");
MODULE_DESCRIPTION("Nexthop FPGA IRQ/clock driver");
MODULE_VERSION(DRIVER_VERSION);
