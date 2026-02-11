// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/mtd/spi-nor.h>

#include "fbiob-auxdev.h"
#include "fboss_iob_mmio.h"

#define DRIVER_NAME		"fboss_iob_spi"

/*
 * SPI Timing Profile bitmap.
 */
#define FBIOB_SPI_MODES		(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH)
#define FBIOB_SPI_CS_NUM	1

/*
 * FBIOB SPI registers and offsets.
 */
#define FBIOB_SPI_REG_TIMING	0
#define FBIOB_SPI_REG_FUNC_CTL	0x4
#define FBIOB_SPI_REG_DESC_CMD	0x8
#define FBIOB_SPI_REG_DESC_STS	0xC

/*
 * FBIOB SPI register Timing Profile bitmap.
 */
#define FBIOB_SPI_CTL_REVISION_MASK	GENMASK(31, 24)
#define FBIOB_SPI_CTL_CPOL		BIT(15)
#define FBIOB_SPI_CTL_SAMPLE_DELAY_MASK GENMASK(14, 12)
#define FBIOB_SPI_CTL_CPHA		BIT(11)
#define FBIOB_SPI_CTL_CLK_DIV_MASK	GENMASK(7, 0)

/*
 * FBIOB SPI register Controller Reset bitmap.
 */
#define FBIOB_SPI_CTL_RST		BIT(0)
#define FBIOB_SPI_MANUAL_CS_EN		BIT(1)

/*
 * FBIOB SPI register Descriptor Command bitmap.
 */
#define FBIOB_SPI_DESC_BIT_VALID	BIT(31)
#define FBIOB_SPI_DESC_BIT_CS_MAN_ACT	BIT(30)
#define FBIOB_SPI_DESC_BIT_CSN_MODE	BIT(29)
#define FBIOB_SPI_DESC_DATA_LEN_MASK	GENMASK(16, 8)
#define FBIOB_SPI_DESC_BIT_ERR_INT	BIT(1)
#define FBIOB_SPI_DESC_BIT_DONE_INT	BIT(0)

/*
 * FBIOB SPI register Descriptor status bitmap.
 */
#define FBIOB_SPI_DESC_BUSY_STS		BIT(2)
#define FBIOB_SPI_DESC_ERR_INT_STS	BIT(1)
#define FBIOB_SPI_DESC_DONE_INT_STS	BIT(0)

#define FBIOB_SPI_MAX_XFER_LEN		512
#define FBIOB_SPI_MISO_BUF_OFFSET	0x200

#define FBIOB_SPI_XFER_POLL_TIME_US	10

/*
 * More flash read commands from flashrom spi.h.
 */

/* Some Atmel AT25F* models have bit 3 as don't care bit in commands */
#define AT25F_RDID		0x15	/* 0x15 or 0x1d */

/* Read Electronic Manufacturer Signature */
#define JEDEC_REMS		0x90

/* Read Electronic Signature */
#define JEDEC_RES		0xab

/* Some ST M95X model */
#define ST_M95_RDID		0x83

/* Read flag status register */
#define SPINOR_OP_RDFSR		0x70
/* Read Extended Address Register */
#define SPINOR_OP_RDEAR		0xc8

struct fbiob_spi_bus {
	struct device *dev;
	struct auxiliary_device *auxdev;

	u32 csr_bus_addr;
	u32 iobuf_bus_addr;
	void __iomem *mmio_csr;
	void __iomem *mmio_mosi;
	void __iomem *mmio_miso;

	struct spi_controller *controller;
	unsigned int num_spidevs;
	struct spi_device *spidevs[FBIOB_SPIDEV_MAX];
};

static u32 spi_csr_read(struct fbiob_spi_bus *bus, u32 offset)
{
	return readl(bus->mmio_csr + offset);
}

static void spi_csr_write(struct fbiob_spi_bus *bus, u32 offset, u32 val)
{
	writel(val, bus->mmio_csr + offset);
}

static void spi_iobuf_read(struct fbiob_spi_bus *bus,
			   void *data,
			   size_t size)
{
	fbiob_bulk_read(bus->mmio_miso, data, size);
}

static void spi_iobuf_write(struct fbiob_spi_bus *bus,
			    const void *data,
			    size_t size)
{
	fbiob_bulk_write(bus->mmio_mosi, data, size);
}

static int fbiob_spi_init_hw(struct fbiob_spi_bus *bus)
{
	u32 val;

	/* reset the controller */
	spi_csr_write(bus, FBIOB_SPI_REG_FUNC_CTL, FBIOB_SPI_CTL_RST);

	val = spi_csr_read(bus, FBIOB_SPI_REG_FUNC_CTL);

	/* enable SPI CS manual mode */
	val |= FBIOB_SPI_MANUAL_CS_EN;
	spi_csr_write(bus, FBIOB_SPI_REG_FUNC_CTL, val);

	dev_info(bus->dev, "manual chip-select mode enabled");
	return 0;
}

static int fbiob_spi_setup_xfer(struct spi_device *spidev)
{
	u32 val;
	struct fbiob_spi_bus *bus = spi_controller_get_devdata(spidev->controller);

	val = spi_csr_read(bus, FBIOB_SPI_REG_TIMING);

	if (spidev->mode & SPI_CPOL)
		val |= FBIOB_SPI_CTL_CPOL;
	else
		val &= ~FBIOB_SPI_CTL_CPOL;

	if (spidev->mode & SPI_CPHA)
		val |= FBIOB_SPI_CTL_CPHA;
	else
		val &= ~FBIOB_SPI_CTL_CPHA;

	/* controller config sck 12.5M */
	val |= (0x1 & FBIOB_SPI_CTL_CLK_DIV_MASK);
	val |= ((0x2 << 12) & FBIOB_SPI_CTL_SAMPLE_DELAY_MASK);

	spi_csr_write(bus, FBIOB_SPI_REG_TIMING, val);

	return 0;
}

static void fbiob_spi_set_cs(struct spi_device *slave, bool level)
{
	u32 val;
	struct fbiob_spi_bus *bus = spi_controller_get_devdata(slave->controller);

	val = spi_csr_read(bus, FBIOB_SPI_REG_DESC_CMD);

	if (level)
		val &= ~FBIOB_SPI_DESC_BIT_CS_MAN_ACT;
	else
		val |= FBIOB_SPI_DESC_BIT_CS_MAN_ACT;

	spi_csr_write(bus, FBIOB_SPI_REG_DESC_CMD, val);
}

static int wait_till_xfer_done(struct fbiob_spi_bus *bus)
{
	int retry = 30;		//12.5M transmit 512 bytes about 328 ms
	u32 val;

	do {
		usleep_range(FBIOB_SPI_XFER_POLL_TIME_US,
					FBIOB_SPI_XFER_POLL_TIME_US * 2);
		val = spi_csr_read(bus, FBIOB_SPI_REG_DESC_STS);

		if (val & FBIOB_SPI_DESC_ERR_INT_STS)
			return -EIO;
		if (val & FBIOB_SPI_DESC_DONE_INT_STS)
			return 0;
	} while (retry--);

	return -ETIMEDOUT;
}

static int fbiob_spi_send_cmd(struct fbiob_spi_bus *bus, int xfer_len)
{
	u32 val;

	val = spi_csr_read(bus, FBIOB_SPI_REG_DESC_CMD);

	/* Trigger spi controller and enable intrrupt */
	val |= FBIOB_SPI_DESC_BIT_VALID;
	val &= ~(FBIOB_SPI_DESC_BIT_ERR_INT | FBIOB_SPI_DESC_BIT_DONE_INT);

	/* setting command register, setup transmit data length */
	val &= (~FBIOB_SPI_DESC_DATA_LEN_MASK);
	if (xfer_len != FBIOB_SPI_MAX_XFER_LEN)
		val |= ((xfer_len << 8) & FBIOB_SPI_DESC_DATA_LEN_MASK);

	spi_csr_write(bus, FBIOB_SPI_REG_DESC_CMD, val);

	return 0;
}

static int fbiob_spi_xfer_tx(struct fbiob_spi_bus *bus,
			    struct spi_transfer *xfer)
{
	int ret, nbytes;
	u32 val;
	int xfer_len = xfer->len;
	const u8 *txbuf = xfer->tx_buf;

	while (xfer_len > 0) {
		/* Clear all the status bits before starting new transaction */
		val =  FBIOB_SPI_DESC_ERR_INT_STS | FBIOB_SPI_DESC_DONE_INT_STS;
		spi_csr_write(bus, FBIOB_SPI_REG_DESC_STS, val);

		nbytes = (xfer_len < FBIOB_SPI_MAX_XFER_LEN) ?
			xfer_len : FBIOB_SPI_MAX_XFER_LEN;

		/*
		 * Step 1: upload command/data to mosi buffer.
		 */
		spi_iobuf_write(bus, txbuf, nbytes);

		/*
		 * Step 2: update control register to start transaction.
		 */
		ret = fbiob_spi_send_cmd(bus, nbytes);
		if (ret < 0)
			return ret;

		ret = wait_till_xfer_done(bus);
		if (ret < 0)
			return ret;

		xfer_len -= nbytes;
		txbuf += nbytes;
	} /* while */

	return 0;
}

static int fbiob_spi_xfer_single_rx(struct fbiob_spi_bus *bus,
				   struct spi_transfer *xfer,
				   unsigned int rx_offset)
{
	int ret;
	u8 *rx_buf = xfer->rx_buf;
	unsigned int nleft, nrequest;

	nleft = xfer->len - rx_offset;
	nrequest = (nleft < FBIOB_SPI_MAX_XFER_LEN ?
		    nleft : FBIOB_SPI_MAX_XFER_LEN);

	ret = fbiob_spi_send_cmd(bus, nrequest);
	if (ret < 0)
		return ret;

	ret = wait_till_xfer_done(bus);
	if (ret < 0)
		return ret;

	spi_iobuf_read(bus, &rx_buf[rx_offset], nrequest);

	return nrequest;
}

static int fbiob_spi_xfer_rx(struct fbiob_spi_bus *bus,
			    struct spi_transfer *xfer)
{
	int ret;
	unsigned int ndata = 0;
	u32 val;

	while (ndata < xfer->len) {
		val =  FBIOB_SPI_DESC_ERR_INT_STS | FBIOB_SPI_DESC_DONE_INT_STS;
		spi_csr_write(bus, FBIOB_SPI_REG_DESC_STS, val);

		ret = fbiob_spi_xfer_single_rx(bus, xfer, ndata);
		if (ret < 0)
			return ret;

		ndata += ret;
	}

	return 0;
}

static int fbiob_spi_xfer_one(struct spi_controller *controller,
			      struct spi_device *spidev,
			      struct spi_transfer *xfer)
{
	int ret;
	struct fbiob_spi_bus *bus = spi_controller_get_devdata(controller);

	if (xfer->tx_buf != NULL)
		ret = fbiob_spi_xfer_tx(bus, xfer);

	if (xfer->rx_buf != NULL)
		ret = fbiob_spi_xfer_rx(bus, xfer);

	return ret;
}

static void fbiob_spi_destroy_clients(struct fbiob_spi_bus *bus)
{
	unsigned int i;

	for (i = 0; i < bus->num_spidevs; i++) {
		if (bus->spidevs[i] != NULL) {
			spi_unregister_device(bus->spidevs[i]);
			bus->spidevs[i] = NULL;
		}
	}
}

static int fbiob_spi_init_clients(struct fbiob_spi_bus *bus,
				  const struct fbiob_spi_data *pdata)
{
	unsigned int i;
	struct spi_board_info board_info;
	struct spi_controller *controller = bus->controller;

	bus->num_spidevs = pdata->num_spidevs;
	if (bus->num_spidevs > FBIOB_SPIDEV_MAX) {
		dev_warn(bus->dev, "num_spidevs out of range: %u > %u",
			 bus->num_spidevs, FBIOB_SPIDEV_MAX);
		bus->num_spidevs = FBIOB_SPIDEV_MAX;
	}

	for (i = 0; i < bus->num_spidevs; i++) {
		const struct spi_dev_info *dev_info = &pdata->spidevs[i];

		memset(&board_info, 0, sizeof(board_info));
		board_info.bus_num = controller->bus_num;
		board_info.chip_select = dev_info->chip_select;
		board_info.max_speed_hz = dev_info->max_speed_hz;
		snprintf(board_info.modalias, sizeof(board_info.modalias),
			 "%s", dev_info->modalias);

		bus->spidevs[i] = spi_new_device(controller, &board_info);
		if (bus->spidevs[i] == NULL) {
			dev_err(bus->dev, "failed to create spidev, cs %u",
				board_info.chip_select);
			goto error;
		}
	}

	return 0;

error:
	fbiob_spi_destroy_clients(bus);
	return -ENXIO;
}

static int fbiob_spi_init_mmio(struct fbiob_spi_bus *bus)
{
	struct resource *res;
	struct device *dev = bus->dev;
	struct auxiliary_device *auxdev = bus->auxdev;

	res = devm_request_mem_region(dev, bus->csr_bus_addr,
				FBIOB_SPI_CSR_BLK_SIZE, auxdev->name);
	if (!res)
		return -EBUSY;

	bus->mmio_csr = devm_ioremap(dev, bus->csr_bus_addr,
					FBIOB_SPI_CSR_BLK_SIZE);
	if (!bus->mmio_csr)
		return -ENOMEM;

	res = devm_request_mem_region(dev, bus->iobuf_bus_addr,
				FBIOB_SPI_IOBUF_SIZE, auxdev->name);
	if (!res)
		return -EBUSY;

	bus->mmio_mosi = devm_ioremap(dev, bus->iobuf_bus_addr,
					FBIOB_SPI_IOBUF_SIZE);
	if (!bus->mmio_mosi)
		return -ENOMEM;
	bus->mmio_miso = bus->mmio_mosi + FBIOB_SPI_MISO_BUF_OFFSET;

	return 0;
}

static void fbiob_spi_remove(struct auxiliary_device *auxdev)
{
	struct fbiob_spi_bus *bus = dev_get_drvdata(&auxdev->dev);

	fbiob_spi_destroy_clients(bus);
}

static int fbiob_spi_probe(struct auxiliary_device *auxdev,
			   const struct auxiliary_device_id *id)
{
	int ret;
	struct spi_controller *controller;
	struct fbiob_spi_bus *bus;
	struct device *dev = &auxdev->dev;
	struct fbiob_aux_adapter *aux_adap =
			(struct fbiob_aux_adapter *)container_of(auxdev,
					struct fbiob_aux_adapter, auxdev);
	struct fbiob_aux_data *pdata = &aux_adap->data;

	if ((pdata->csr_offset == FBIOB_INVALID_OFFSET) ||
	    (pdata->iobuf_offset == FBIOB_INVALID_OFFSET))
		return -EINVAL;

	if (pdata->spi_data.num_spidevs == 0)
		return -EINVAL;

	controller = devm_spi_alloc_host(dev, sizeof(*bus));
	if (controller == NULL)
		return -ENOMEM;

	controller->mode_bits = FBIOB_SPI_MODES;
	controller->num_chipselect = FBIOB_SPI_CS_NUM;
	controller->setup = fbiob_spi_setup_xfer;
	controller->set_cs = fbiob_spi_set_cs;
	controller->transfer_one = fbiob_spi_xfer_one;

	bus = spi_controller_get_devdata(controller);
	bus->dev = dev;
	bus->auxdev = auxdev;
	bus->controller = controller;
	bus->csr_bus_addr = pdata->csr_offset;
	bus->iobuf_bus_addr = pdata->iobuf_offset;
	dev_set_drvdata(dev, bus);

	ret = fbiob_spi_init_mmio(bus);
	if (ret < 0)
		return ret;

	ret = fbiob_spi_init_hw(bus);
	if (ret) {
		dev_err(dev, "failed to initialize hardware, error=%d", ret);
		return ret;
	}

	ret = devm_spi_register_controller(dev, controller);
	if (ret)
		return ret;

	ret = fbiob_spi_init_clients(bus, &pdata->spi_data);
	if (ret)
		return ret;

	dev_info(dev, "spi bus %u registered (controller at 0x%x)",
		 controller->bus_num, bus->csr_bus_addr);
	return 0;
}

static const struct auxiliary_device_id fboss_iob_spi_ids[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER".spi_master" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, fboss_iob_spi_ids);

static struct auxiliary_driver fboss_iob_spi_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.remove = fbiob_spi_remove,
	.probe = fbiob_spi_probe,
	.id_table = fboss_iob_spi_ids,
};
module_auxiliary_driver(fboss_iob_spi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tao Ren <taoren@meta.com>");
MODULE_DESCRIPTION("Meta FBOSS IOB_FPGA SPI Controller Driver");
MODULE_VERSION(BSP_VERSION);
