// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mdio.h>
#include <linux/of_mdio.h>
#include <linux/module.h>

#include "fboss_iob_pci.h"
#include "fbiob-auxdev.h"
#include "fboss_iob_mmio.h"
#include "fboss-mdio.h"

#define DRIVER_NAME "fboss_iob_mdio"

/* Define the delay time */
#define FBIOB_MDIO_POLL_TIME_US     20
/* Define MDIO max timeout */
#define FBOSS_MDIO_MAX_TIMEOUT_US   1000
/* Define the waiting time for controller reset*/
#define FBOSS_MDIO_RESET_TIME_MS    50

/*
 *  MDIO controller blocks and offsets.
 */
#define FBIOB_MDIO_BLK_START_DOMOFF       0x200
/* FBIOB_MDIO_BLK_SIZE 0x20 */
#define FBIOB_MDIO_EXT_BLK_START_DOMOFF   0xC0
#define FBIOB_MDIO_EXT_BLK_SIZE           0x4

/*
 *  MDIO registers and offsets.
 */
#define FBIOB_MDIO_REG_CFG      0x0
#define FBIOB_MDIO_REG_CMD      0x4
#define FBIOB_MDIO_REG_WR_DAT   0x8
#define FBIOB_MDIO_REG_RD_DAT   0xC
#define FBIOB_MDIO_REG_STS      0x10
#define FBIOB_MDIO_REG_INTR_MSK 0x14
#define FBIOB_MDIO_REG_SRC      0x18
#define FBIOB_MDIO_REG_RST      0x1C

#define FBIOB_MDIO_RTM_REG_RST  0x0

enum fbiob_mdio_opcode {
	BUS_WRITE,
	BUS_READ
};

enum fbiob_mdio_state {
	STATE_INIT,
	STATE_READY,
	STATE_BUSY
};

struct fbiob_mdio_bus {
	struct auxiliary_device *auxdev;
	struct fb_mdio_desc mdesc;
	__u32 csr_bus_addr;
	void __iomem *mmio_csr;
	__u32 ext_csr_bus_addr;
	void __iomem *mmio_ext_csr;
	enum fbiob_mdio_state state;
};

union fbiob_mdio_cfg {
	u32 u;
	struct fbiob_mdio_cfg_s {
		u32 mdc_div_0_7:8;
		u32 mode_8:1;
		u32 reserved_9_31:23;
	} s;
};

union fbiob_mdio_cmd {
	u32 u;
	struct fbiob_mdio_cmd_s {
		u32 devad_0_4:5;
		u32 reserved_5_6:2;
		u32 rw_7:1;
		u32 phyad_8_12:5;
		u32 reserved_13_15:3;
		u32 reg_16_31:16;
	} s;
};

union fbiob_mdio_wr_dat {
	u32 u;
	struct fbiob_mdio_wr_dat_s {
		u32 wr_data_0_15:16;
		u32 debug_16_31:16;
	} s;
};

union fbiob_mdio_rd_dat {
	u32 u;
	struct fbiob_mdio_rd_dat_s {
		u32 rd_data_0_15:16;
		u32 reserved_16_31:16;
	} s;
};

union fbiob_mdio_sts {
	u32 u;
	struct fbiob_mdio_sts_s {
		u32 done_0:1;
		u32 err_1:1;
		u32 unexpected_2:1;
		u32 reserved_3_7:5;
		u32 debug_8_15:8;
		u32 reserved_16_31:16;
	} s;
};

union fbiob_mdio_intr_msk {
	u32 u;
	struct fbiob_mdio_intr_msk_s {
		u32 done_msk_0:1;
		u32 err_msk_1:1;
		u32 reserved_2_31:30;
	} s;
};

union fbiob_mdio_rst {
	u32 u;
	struct fbiob_mdio_rst_s {
		u32 reset_0:1;
		u32 reserved_1_31:31;
	} s;
};

union fbiob_mdio_rtm_rst {
	u32 u;
	struct fbiob_mdio_rtm_rst_s {
		u32 reset_0:1;
		u32 refclk_1:1;
		u32 reserved_2_4:3;
		u32 intr_stat_5:1;
		u32 reserved_6_8:3;
		u32 intr_stat_change_9:1;
		u32 reserved_10_31:22;
	} s;
};

/*
 * Besides the main MDIO controller block region, there is an extended block
 * region which is used to control devices on buses by IOs. To get the IO block
 * address for devices under a specific MDIO controller/bus, firstly we calculate
 * the main block index by dividing the main block offset with its block size,
 * then using the same block index and ext-region base address, we can finally
 * get the corresponding block bus address.
 */
static u32 get_ext_csr_bus_addr(struct device *dev, u32 csr_bus_addr)
{
	struct fbiob_priv *priv;
	struct pci_dev *pdev;
	u32 base_bus_addr;
	int bus_off, dom_start_off, blk_dom_off;
	int blk_idx;

	pdev = to_pci_dev(dev);
	priv = pci_get_drvdata(pdev);
	base_bus_addr = priv->bar0_bus_addr;

	bus_off = csr_bus_addr - base_bus_addr;

	if (bus_off >= FBDOM2_GLOBAL_REG_START)
		dom_start_off = FBDOM2_GLOBAL_REG_START;
	else
		dom_start_off = FBDOM1_GLOBAL_REG_START;

	blk_dom_off = bus_off - dom_start_off -
						FBIOB_MDIO_BLK_START_DOMOFF;
	blk_idx =  blk_dom_off / FBIOB_MDIO_BLK_SIZE;

	return base_bus_addr + dom_start_off +
			FBIOB_MDIO_EXT_BLK_START_DOMOFF +
			blk_idx * FBIOB_MDIO_EXT_BLK_SIZE;
}

static void fbiob_mdio_clean_status(struct fbiob_mdio_bus *bus)
{
	union fbiob_mdio_sts sts;

	sts.u = readl(bus->mmio_csr + FBIOB_MDIO_REG_STS);
	sts.s.done_0 = 1;
	sts.s.err_1 = 1;
	writel(sts.u, bus->mmio_csr + FBIOB_MDIO_REG_STS);
}

static int wait_bus_ready(struct fbiob_mdio_bus *bus)
{
	union fbiob_mdio_sts sts;
	int retry = FBOSS_MDIO_MAX_TIMEOUT_US / FBIOB_MDIO_POLL_TIME_US;

	do {
		if (bus->state == STATE_READY)
			return 0;

		sts.u = readl(bus->mmio_csr + FBIOB_MDIO_REG_STS);
		if (unlikely(sts.s.err_1))
			return -EIO;
		if (likely(sts.s.done_0))
			return 0;

		fsleep(FBIOB_MDIO_POLL_TIME_US);
	} while (retry--);

	return -EBUSY;
}

static int fbiob_mdio_reset(struct fbiob_mdio_bus *bus)
{
	union fbiob_mdio_rst rst;
	union fbiob_mdio_intr_msk msk;

	rst.u = readl(bus->mmio_csr + FBIOB_MDIO_REG_RST);
	rst.s.reset_0 = 1;
	writel(rst.u, bus->mmio_csr + FBIOB_MDIO_REG_RST);

	fsleep(FBOSS_MDIO_RESET_TIME_MS * 1000);

	rst.u = readl(bus->mmio_csr + FBIOB_MDIO_REG_RST);
	if (unlikely(rst.s.reset_0))
		return -ETIMEDOUT;

	msk.u = readl(bus->mmio_csr + FBIOB_MDIO_REG_INTR_MSK);
	msk.s.done_msk_0 = 0;
	msk.s.err_msk_1 = 0;
	writel(msk.u, bus->mmio_csr + FBIOB_MDIO_REG_INTR_MSK);

	bus->state = STATE_READY;
	return 0;
}

static int fbiob_mdio_rtm_reset(struct fbiob_mdio_bus *bus, int rtm_addr)
{
	union fbiob_mdio_rtm_rst rst;

	if (rtm_addr != 0)
		return -EIO;

	rst.u = readl(bus->mmio_ext_csr + FBIOB_MDIO_RTM_REG_RST);
	rst.s.reset_0 = 0;
	writel(rst.u, bus->mmio_ext_csr + FBIOB_MDIO_RTM_REG_RST);

	fsleep(FBOSS_MDIO_RESET_TIME_MS * 1000);

	rst.u = readl(bus->mmio_ext_csr + FBIOB_MDIO_RTM_REG_RST);
	rst.s.reset_0 = 1;
	writel(rst.u, bus->mmio_ext_csr + FBIOB_MDIO_RTM_REG_RST);

	return 0;
}

static int fbiob_mdio_read_c45(struct mii_bus *mdio, int addr, int devad,
			    int regnum)
{
	int ret;
	union fbiob_mdio_rd_dat rd_data;
	union fbiob_mdio_cmd  cmd;
	struct fbiob_mdio_bus *bus = (struct fbiob_mdio_bus *)(mdio->priv);

	ret = wait_bus_ready(bus);
	/*
	 * restore the state machine of the controller to end last instruction
	 * and prepare for next operation
	 */
	fbiob_mdio_clean_status(bus);
	if (unlikely(ret)) {
		bus->state = STATE_READY;
		return ret;
	}
	bus->state = STATE_BUSY;

	cmd.u = readl(bus->mmio_csr + FBIOB_MDIO_REG_CMD);
	cmd.s.phyad_8_12 = addr;
	cmd.s.devad_0_4 = devad;
	cmd.s.reg_16_31 = regnum;
	cmd.s.rw_7 = BUS_READ;
	writel(cmd.u, bus->mmio_csr + FBIOB_MDIO_REG_CMD);

	ret = wait_bus_ready(bus);
	fbiob_mdio_clean_status(bus);
	bus->state = STATE_READY;
	if (unlikely(ret))
		return ret;

	rd_data.u = readl(bus->mmio_csr + FBIOB_MDIO_REG_RD_DAT);

	return rd_data.s.rd_data_0_15;
}

static int fbiob_mdio_write_c45(struct mii_bus *mdio, int addr, int devad,
			     int regnum, u16 val)
{
	int ret;
	union fbiob_mdio_wr_dat wr_data;
	union fbiob_mdio_cmd cmd;
	struct fbiob_mdio_bus *bus = (struct fbiob_mdio_bus *)(mdio->priv);

	ret = wait_bus_ready(bus);
	fbiob_mdio_clean_status(bus);
	if (unlikely(ret)) {
		bus->state = STATE_READY;
		return ret;
	}

	bus->state = STATE_BUSY;

	wr_data.s.wr_data_0_15 = val;
	writel(wr_data.u, bus->mmio_csr + FBIOB_MDIO_REG_WR_DAT);

	cmd.u = readl(bus->mmio_csr + FBIOB_MDIO_REG_CMD);
	cmd.s.phyad_8_12 = addr;
	cmd.s.devad_0_4 = devad;
	cmd.s.reg_16_31 = regnum;
	cmd.s.rw_7 = BUS_WRITE;
	writel(cmd.u, bus->mmio_csr + FBIOB_MDIO_REG_CMD);

	return 0;
}

static ssize_t reset_bus_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;
	struct mii_bus *mdio = dev_get_drvdata(dev->parent);
	struct fbiob_mdio_bus *bus = mdio->priv;

	ret = fbiob_mdio_reset(bus);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_WO(reset_bus);

static ssize_t reset_rtm_0_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;
	struct mii_bus *mdio = dev_get_drvdata(dev->parent);
	struct fbiob_mdio_bus *bus = mdio->priv;

	ret = fbiob_mdio_rtm_reset(bus, 0);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_WO(reset_rtm_0);

static void fbiob_mdio_remove(struct auxiliary_device *auxdev)
{
	struct mii_bus *mdio = dev_get_drvdata(&auxdev->dev);
	struct fbiob_mdio_bus *bus =  mdio->priv;

	fb_mdio_cdev_destroy(&bus->mdesc);
}

static int fbiob_mdio_probe(struct auxiliary_device *auxdev,
			    const struct auxiliary_device_id *id)
{
	int ret;
	struct resource *res;
	struct fbiob_mdio_bus *bus;
	struct mii_bus *mdio;
	struct fbiob_aux_adapter *aux_adap;

	aux_adap = (struct fbiob_aux_adapter *)container_of(auxdev,
					struct fbiob_aux_adapter, auxdev);

	mdio = devm_mdiobus_alloc_size(&auxdev->dev, sizeof(*bus));
	if (unlikely(!mdio))
		return -ENOMEM;

	dev_set_drvdata(&auxdev->dev, mdio);

	bus = mdio->priv;

	bus->csr_bus_addr = aux_adap->data.csr_offset;

	res = devm_request_mem_region(&auxdev->dev, bus->csr_bus_addr,
				FBIOB_MDIO_BLK_SIZE, auxdev->name);
	if (unlikely(!res))
		return -EBUSY;

	bus->mmio_csr = devm_ioremap(&auxdev->dev, bus->csr_bus_addr,
					FBIOB_MDIO_BLK_SIZE);
	if (unlikely(!bus->mmio_csr))
		return -ENOMEM;

	/* Request address space for the extended retimer control block */

	if (aux_adap->data.iobuf_offset != FBIOB_INVALID_OFFSET)
		/* "iobuf_offset" is pointed to the extended control block */
		bus->ext_csr_bus_addr = aux_adap->data.iobuf_offset;
	else
		/* calculate the address to the extended control block */
		bus->ext_csr_bus_addr = get_ext_csr_bus_addr(auxdev->dev.parent,
								bus->csr_bus_addr);

	res = devm_request_mem_region(&auxdev->dev, bus->ext_csr_bus_addr,
				FBIOB_MDIO_EXT_BLK_SIZE, auxdev->name);
	if (unlikely(!res))
		return -EBUSY;

	bus->mmio_ext_csr = devm_ioremap(&auxdev->dev, bus->ext_csr_bus_addr,
						FBIOB_MDIO_EXT_BLK_SIZE);
	if (unlikely(!bus->mmio_ext_csr))
		return -ENOMEM;

	bus->auxdev = auxdev;

	ret = fbiob_mdio_reset(bus);
	if (unlikely(ret))
		return ret;

	mdio->name = DRIVER_NAME;
	snprintf(mdio->id, MII_BUS_ID_SIZE, "%s.%d", auxdev->name, auxdev->id);
	mdio->parent = &auxdev->dev;
	mdio->read_c45 = fbiob_mdio_read_c45;
	mdio->write_c45 = fbiob_mdio_write_c45;

	/* Mask out all PHYs from auto probing. */
	mdio->phy_mask = ~0;
	ret = devm_mdiobus_register(&auxdev->dev, mdio);
	if (unlikely(ret))
		return ret;

	ret = device_create_file(&mdio->dev, &dev_attr_reset_bus);
	if (unlikely(ret))
		return ret;

	ret = device_create_file(&mdio->dev, &dev_attr_reset_rtm_0);
	if (unlikely(ret))
		return ret;

	ret = fb_mdio_cdev_init(&bus->mdesc, mdio, auxdev->id);
	if (unlikely(ret))
		return ret;

	dev_info(&auxdev->dev, "mdio controller registered (csr=0x%x)",
		  bus->csr_bus_addr);
	return 0;
}

static const struct auxiliary_device_id fboss_iob_mdio_ids[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER".mdio_controller" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, fboss_iob_mdio_ids);

static struct auxiliary_driver fboss_iob_mdio_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = fbiob_mdio_probe,
	.remove = fbiob_mdio_remove,
	.id_table = fboss_iob_mdio_ids,
};
module_auxiliary_driver(fboss_iob_mdio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yves Wang <yves.wang@celestica.com>");
MODULE_DESCRIPTION("Meta FBOSS IOB_FPGA MDIO Controller Driver");
MODULE_VERSION(BSP_VERSION);
