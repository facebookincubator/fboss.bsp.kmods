// SPDX-License-Identifier: GPL-2.0+
// Copyright (c) Meta Platforms, Inc. and affiliates.

#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/module.h>

#include "fbiob-auxdev.h"
#include "fboss_iob_mmio.h"

#define DRIVER_NAME	"fboss_iob_i2c"

/*
 * FBIOB I2C registers and offsets.
 */
#define FBIOB_I2C_REG_FUNC_CTRL		0
#define FBIOB_I2C_REG_TIMING_CFG	0x4
#define FBIOB_I2C_REG_INTR_STS		0x10
#define FBIOB_I2C_REG_CMD_STS		0x14
#define FBIOB_I2C_REG_BUF_CTRL		0x1C
#define FBIOB_I2C_IOBUF_OFFSET		0x80

/*
 * There are 2 separate data buffers for I2C transactions, and both are
 * 128 bytes.
 * The maximum payload size is 256 bytes in READ transfers (when both
 * data buffers are used).
 * The maximum payload size is 255 bytes for WRITE transfers (when both
 * data buffers are used), because 1 byte is reserved for I2C address.
 */
#define FBIOB_I2C_IOBUF_SIZE		128
#define FBIOB_I2C_RX_PAYLOAD_MAX	256
#define FBIOB_I2C_TX_PAYLOAD_MAX	255

/*
 * I2C Device Function Control Function register bitmap
 */
#define FBIOB_I2C_EN_MASTER_FUNC	BIT(0)

/*
 * I2C Deivce Interrupt Control Register.
 */
#define FBIOB_I2C_INTR_XFER_TIMEOUT_STS		BIT(3)
#define FBIOB_I2C_INTR_RX_DONE_STS		BIT(2)
#define FBIOB_I2C_INTR_TX_NACK_STS		BIT(1)
#define FBIOB_I2C_INTR_TX_DONE_STS		BIT(0)

/*
 * I2C Device Command and Status Register
 */
#define FBIOB_I2C_BUS_BUSY_STS          BIT(16)
#define FBIOB_I2C_BUS_RCV_CMD_LAST	    BIT(4)
#define FBIOB_I2C_BUS_STOP_CMD		    BIT(3)
#define FBIOB_I2C_BUS_RCV_CMD		    BIT(2)
#define FBIOB_I2C_BUS_TRANSMIT_CMD	    BIT(1)
#define FBIOB_I2C_BUS_START_CMD		    BIT(0)

/*
 * I2C Device Buffer Controller Register
 */
#define FBIOB_I2C_BUF_MEM_PAGE_SEL		BIT(6)
#define FBIOB_I2C_TX_BUF_SIZE_SHIFT			8
#define FBIOB_I2C_RX_BUF_SIZE_SHIFT			16
#define FBIOB_I2C_ACT_RX_BUF_SIZE_SHIFT	    24
#define FBIOB_I2C_BUF_SIZE_CTRL_REG_LEN     32
#define FBIOB_I2C_ACT_BUF_SIZE_MASK	GENMASK(	\
		FBIOB_I2C_BUF_SIZE_CTRL_REG_LEN,		\
		FBIOB_I2C_ACT_RX_BUF_SIZE_SHIFT)
#define FBIOB_I2C_RX_BUF_SIZE_MASK	GENMASK(	\
		FBIOB_I2C_ACT_RX_BUF_SIZE_SHIFT-1,		\
		FBIOB_I2C_RX_BUF_SIZE_SHIFT)
#define FBIOB_I2C_TX_BUF_SIZE_MASK	GENMASK(	\
		FBIOB_I2C_RX_BUF_SIZE_SHIFT-1,			\
		FBIOB_I2C_TX_BUF_SIZE_SHIFT)

/*
 * Define the delay time is 100us
 */
#define FBIOB_I2C_XFER_POLL_TIME_US	    100

/* I2C protocol smbus max timeout 35 ms */
#define FBOSS_I2C_SMBUS_MAX_TIMEOUT_US	35000

enum fbiob_i2c_opcode {
	BUS_READ,
	BUS_WRITE
};

enum fbiob_i2c_state {
	STATE_DONE,
	STATE_ERROR,
	STATE_START,
	STATE_STOP
};

struct fbiob_i2c_bus {
	struct device *dev;
	struct auxiliary_device *auxdev;
	u32 csr_bus_addr;
	void __iomem *mmio_csr;
	void __iomem *mmio_iobuf;
	struct i2c_adapter adap;
	struct i2c_msg *msg;
	enum fbiob_i2c_state state;
};

static u32 i2c_csr_read(struct fbiob_i2c_bus *bus, unsigned int offset)
{
	return readl(bus->mmio_csr + offset);
}

static void i2c_csr_write(struct fbiob_i2c_bus *bus, unsigned int offset,
			 u32 val)
{
	writel(val, bus->mmio_csr + offset);
}

static void i2c_iobuf_read(struct fbiob_i2c_bus *bus,
			   void *buf,
			   size_t size)
{
	fbiob_bulk_read(bus->mmio_iobuf, buf, size);
}

static void i2c_iobuf_write(struct fbiob_i2c_bus *bus,
			    const void *buf,
			    size_t size)
{
	fbiob_bulk_write(bus->mmio_iobuf, buf, size);
}

static u32 fbiob_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static int check_msg_params(struct i2c_msg *msgs, int num)
{
	int i;

	if (!num)
		return -EINVAL;

	if (unlikely(msgs[0].addr > 0x7f))
		return -EINVAL;

	for (i = 0; i < num; ++i) {
		if (unlikely(!msgs[i].buf))
			return -EINVAL;

		if (unlikely(msgs[0].addr != msgs[i].addr))
			return -EINVAL;
	}

	return 0;
}

/*
 * Reset I2C controller
 */
static int fbiob_i2c_reset(struct fbiob_i2c_bus *bus)
{
	u32 val;

	val = i2c_csr_read(bus, FBIOB_I2C_REG_FUNC_CTRL);

	// Clean the controller enabled bit to reset controller.
	val &= ~FBIOB_I2C_EN_MASTER_FUNC;
	i2c_csr_write(bus, FBIOB_I2C_REG_FUNC_CTRL, val);

	// Enable the I2C controller again after reset.
	val |= FBIOB_I2C_EN_MASTER_FUNC;
	i2c_csr_write(bus, FBIOB_I2C_REG_FUNC_CTRL, val);

	return 0;
}

/*
 * clean interrupt status.
 */
static void fbiob_i2c_clean_intr_status(struct fbiob_i2c_bus *bus)
{
	u32 cmd_data;

	cmd_data = FBIOB_I2C_INTR_TX_DONE_STS
	| FBIOB_I2C_INTR_TX_NACK_STS
	| FBIOB_I2C_INTR_RX_DONE_STS
	| FBIOB_I2C_INTR_XFER_TIMEOUT_STS;

	i2c_csr_write(bus, FBIOB_I2C_REG_INTR_STS, cmd_data);
}

static int fbiob_i2c_check_busy(struct fbiob_i2c_bus *bus)
{
	u32 val;

	val = i2c_csr_read(bus, FBIOB_I2C_REG_CMD_STS);

	if (val & FBIOB_I2C_BUS_BUSY_STS)
		return -EBUSY;

	return 0;
}

static int wait_bus_for_free(struct fbiob_i2c_bus *bus)
{
	int retry = FBOSS_I2C_SMBUS_MAX_TIMEOUT_US /
		    FBIOB_I2C_XFER_POLL_TIME_US;

	do {
		if (!fbiob_i2c_check_busy(bus))
			return 0;

		udelay(FBIOB_I2C_XFER_POLL_TIME_US);
	} while (retry--);

	return -EBUSY;
}

static void fbiob_i2c_stop(struct fbiob_i2c_bus *bus)
{
	u32 val;

	/* Get the current state of the controller */
	val = i2c_csr_read(bus, FBIOB_I2C_REG_CMD_STS);

	/* The I2C controller executes the stop action */
	val |= FBIOB_I2C_BUS_STOP_CMD;
	i2c_csr_write(bus, FBIOB_I2C_REG_CMD_STS, val);
}

/*
 * Polling I2C controller transaction status.
 */
static int wait_till_xfer_done(struct fbiob_i2c_bus *bus,
			int xfer_len, enum fbiob_i2c_opcode opcode)
{
	int retry;
	u32 val;

	/* Evaluate the number of retries based on 35ms */
	retry = FBOSS_I2C_SMBUS_MAX_TIMEOUT_US / FBIOB_I2C_XFER_POLL_TIME_US;

	do {
		udelay(FBIOB_I2C_XFER_POLL_TIME_US);

		val = i2c_csr_read(bus, FBIOB_I2C_REG_INTR_STS);

		if (val & FBIOB_I2C_INTR_TX_NACK_STS) {
			bus->state = STATE_ERROR;
			/* Execute the stop action after receiving the nack status */
			fbiob_i2c_stop(bus);
			return -EIO;
		}

		if (opcode == BUS_READ) {
			if (val & FBIOB_I2C_INTR_RX_DONE_STS) {
				bus->state = STATE_DONE;
				return 0;
			}
		} else if (opcode == BUS_WRITE) {
			if (val & FBIOB_I2C_INTR_TX_DONE_STS) {
				bus->state = STATE_DONE;
				return 0;
			}
		}
	} while (retry--);

	bus->state = STATE_STOP;
	return -ETIMEDOUT;
}

/*
 * I2C controller receive data from devices operation.
 */
static int fbiob_i2c_read(struct fbiob_i2c_bus *bus, struct i2c_msg *msg)
{
	int ret;
	unsigned int rx_len, xfer_len;
	u32 cmd_data, addr, regval;
	u8 *rx_buf = msg->buf;

	if (!rx_buf)
		return -EINVAL;

	rx_len = msg->len;
	/* check msg flags is smbus block read and reset the transmit length */
	if (msg->flags & I2C_M_RECV_LEN)
		rx_len = I2C_SMBUS_BLOCK_MAX;

	/* set slave device address to data buffer */
	addr = (msg->addr << 1) | I2C_SMBUS_READ;
	i2c_csr_write(bus, FBIOB_I2C_IOBUF_OFFSET, addr);

	/* Receive data operation process */
	while (rx_len) {
		char recv_buf[FBIOB_I2C_IOBUF_SIZE];
		int remaining_size;

		xfer_len = min_t(int, FBIOB_I2C_RX_PAYLOAD_MAX, rx_len);

		/* setup the receive data length to I2C controller */
		cmd_data = (((xfer_len - 1) << FBIOB_I2C_RX_BUF_SIZE_SHIFT)
				& FBIOB_I2C_RX_BUF_SIZE_MASK);

		/* default use low page data buffer */
		cmd_data &= ~FBIOB_I2C_BUF_MEM_PAGE_SEL;

		/* set receive buffer controller command */
		i2c_csr_write(bus, FBIOB_I2C_REG_BUF_CTRL, cmd_data);

		/* set receive command */
		cmd_data = FBIOB_I2C_BUS_START_CMD
		| FBIOB_I2C_BUS_TRANSMIT_CMD
		| FBIOB_I2C_BUS_RCV_CMD;

		if (xfer_len == rx_len)
			cmd_data |= FBIOB_I2C_BUS_RCV_CMD_LAST
					 | FBIOB_I2C_BUS_STOP_CMD;

		i2c_csr_write(bus, FBIOB_I2C_REG_CMD_STS, cmd_data);

		/* Poll command and status will be done */
		ret = wait_till_xfer_done(bus, xfer_len, BUS_READ);
		if (ret)
			return ret;

		rx_len -= xfer_len;
		/* adapt 128 and 256 bytes buffer */
		if (xfer_len > FBIOB_I2C_IOBUF_SIZE) {
			/* read low page buffer data */
			i2c_iobuf_read(bus, recv_buf, FBIOB_I2C_IOBUF_SIZE);
			memcpy(rx_buf, recv_buf, FBIOB_I2C_IOBUF_SIZE);
			rx_buf += FBIOB_I2C_IOBUF_SIZE;
			remaining_size = xfer_len - FBIOB_I2C_IOBUF_SIZE;

			/* set rx buffer controller command to switch high page data buffer */
			regval = i2c_csr_read(bus, FBIOB_I2C_REG_BUF_CTRL);
			regval |= FBIOB_I2C_BUF_MEM_PAGE_SEL;
			i2c_csr_write(bus, FBIOB_I2C_REG_BUF_CTRL, regval);

			/* read remaining data from high page buffer */
			i2c_iobuf_read(bus, recv_buf, remaining_size);
			memcpy(rx_buf, recv_buf, remaining_size);

			if (rx_buf && rx_len)
				rx_buf += remaining_size;
		} else {
			i2c_iobuf_read(bus, recv_buf, xfer_len);

			/*
			 * Check msg flags smbus block read and resetting the transmit length.
			 * The byte[0] value which one is actual return data length.
			 */
			if (msg->flags & I2C_M_RECV_LEN) {
				if (unlikely(recv_buf[0] > I2C_SMBUS_BLOCK_MAX))
					return -EINVAL;

				msg->len = recv_buf[0] +
						((msg->flags & I2C_CLIENT_PEC) ? 2 : 1);
				memcpy(rx_buf, recv_buf, msg->len);
			} else
				memcpy(rx_buf, recv_buf, xfer_len);

			if (rx_buf && rx_len)
				rx_buf += xfer_len;
		}
	}

	return 0;
}

static int fbiob_i2c_fill_tx_buf(struct fbiob_i2c_bus *bus,
				 u8 i2c_addr,
				 const u8 *payload,
				 size_t payload_size)
{
	u32 reg_csr;
	int nleft, processed;
	u8 io_buf[FBIOB_I2C_IOBUF_SIZE];

	if (payload_size > FBIOB_I2C_TX_PAYLOAD_MAX)
		return -EINVAL;

	/* Set transfer size and select low_page data buffer. */
	reg_csr = (payload_size << FBIOB_I2C_TX_BUF_SIZE_SHIFT) &
		  FBIOB_I2C_TX_BUF_SIZE_MASK;
	reg_csr &= ~FBIOB_I2C_BUF_MEM_PAGE_SEL;
	i2c_csr_write(bus, FBIOB_I2C_REG_BUF_CTRL, reg_csr);

	/* Copy I2C address and payload to the low_page data buffer. */
	io_buf[0] = i2c_addr;
	processed = min_t(size_t, payload_size, (FBIOB_I2C_IOBUF_SIZE - 1));
	memcpy(&io_buf[1], payload, processed);
	i2c_iobuf_write(bus, io_buf, processed + 1);

	/* Copy payload to high_page data buffer if needed. */
	nleft = payload_size - processed;
	if (nleft > 0) {
		reg_csr |= FBIOB_I2C_BUF_MEM_PAGE_SEL;
		i2c_csr_write(bus, FBIOB_I2C_REG_BUF_CTRL, reg_csr);

		memcpy(io_buf, payload + processed, nleft);
		i2c_iobuf_write(bus, io_buf, nleft);
	}

	return 0;
}

/*
 * I2C controller transmit data to devices operation.
 */
static int fbiob_i2c_write(struct fbiob_i2c_bus *bus,
			   struct i2c_msg *msg,
			   bool is_last_msg)
{
	u32 cmd_data;
	int ret, payload_len;
	int nleft = msg->len;
	const u8 *tx_buf = msg->buf;
	u8 i2c_addr = (u8)((msg->addr << 1) | I2C_SMBUS_WRITE);

	if (!tx_buf)
		return -EINVAL;

	do {
		payload_len = min_t(int, FBIOB_I2C_TX_PAYLOAD_MAX, nleft);

		ret = fbiob_i2c_fill_tx_buf(bus, i2c_addr, tx_buf, payload_len);
		if (ret < 0)
			return ret;

		tx_buf += payload_len;
		nleft -= payload_len;

		/* set start and transmit command */
		cmd_data = FBIOB_I2C_BUS_START_CMD | FBIOB_I2C_BUS_TRANSMIT_CMD;
		if (is_last_msg)
			cmd_data |= FBIOB_I2C_BUS_STOP_CMD;

		i2c_csr_write(bus, FBIOB_I2C_REG_CMD_STS, cmd_data);

		/* Poll command and status will be done */
		ret = wait_till_xfer_done(bus, (payload_len + 1), BUS_WRITE);
		if (ret)
			return ret;
	} while (nleft > 0);

	return 0;
}

static int fbiob_i2c_master_xfer(struct i2c_adapter *adap,
				 struct i2c_msg *msgs, int num)
{
	struct fbiob_i2c_bus *bus = i2c_get_adapdata(adap);
	int i, ret;
	bool is_last;

	ret = check_msg_params(msgs, num);
	if (ret)
		return ret;

	/* Check bus ready state */
	if (wait_bus_for_free(bus)) {
		dev_err(bus->dev, "I2C bus is busy, try to recover.\n");
		/* if i2c bus is busy, try to reset I2C */
		ret = fbiob_i2c_reset(bus);
		if (ret)
			return ret;

		/* after reset I2C, check bus status */
		ret = fbiob_i2c_check_busy(bus);
		if (ret)
			return ret;
	}

	bus->state = STATE_START;

	for (i = 0; i < num; i++) {
		is_last = ((i + 1) == num);

		fbiob_i2c_clean_intr_status(bus);

		if (msgs[i].flags & I2C_M_RD)
			ret = fbiob_i2c_read(bus, &msgs[i]);
		else
			ret = fbiob_i2c_write(bus, &msgs[i], is_last);

		if (ret < 0)
			break;
	}

	return ret < 0 ? ret : num;
}

static const struct i2c_algorithm fbiob_i2c_algorithm = {
	.master_xfer = fbiob_i2c_master_xfer,
	.functionality = fbiob_i2c_func,
};

static void fbiob_i2c_remove(struct auxiliary_device *auxdev)
{
	struct fbiob_i2c_bus *bus = dev_get_drvdata(&auxdev->dev);

	i2c_del_adapter(&bus->adap);
}

static int fbiob_i2c_probe(struct auxiliary_device *auxdev,
			   const struct auxiliary_device_id *id)
{
	int ret;
	struct resource *res;
	struct fbiob_i2c_bus *bus;
	struct device *dev = &auxdev->dev;
	struct fbiob_aux_adapter *aux_adap =
			(struct fbiob_aux_adapter *)container_of(auxdev,
					struct fbiob_aux_adapter, auxdev);

	bus = devm_kzalloc(dev, sizeof(*bus), GFP_KERNEL);
	if (bus == NULL)
		return -ENOMEM;
	dev_set_drvdata(dev, bus);

	bus->csr_bus_addr = aux_adap->data.csr_offset;
	res = devm_request_mem_region(dev, bus->csr_bus_addr,
				FBIOB_I2C_BLK_SIZE, auxdev->name);
	if (!res)
		return -EBUSY;

	bus->mmio_csr = devm_ioremap(dev, bus->csr_bus_addr,
					FBIOB_I2C_BLK_SIZE);
	if (!bus->mmio_csr)
		return -ENOMEM;
	bus->mmio_iobuf = bus->mmio_csr + FBIOB_I2C_IOBUF_OFFSET;

	bus->dev = dev;
	bus->auxdev = auxdev;
	bus->adap.owner = THIS_MODULE;
	bus->adap.algo = &fbiob_i2c_algorithm;
	bus->adap.dev.parent = dev;
	snprintf(bus->adap.name, sizeof(bus->adap.name), "fbiob %s.%u at 0x%x",
		 auxdev->name, auxdev->id, bus->csr_bus_addr);
	i2c_set_adapdata(&bus->adap, bus);

	ret = fbiob_i2c_reset(bus);
	if (ret)
		return ret;

	ret = i2c_add_adapter(&bus->adap);
	if (ret)
		return ret;

	dev_info(dev, "i2c bus %d registered (master at 0x%x)",
		 bus->adap.nr, bus->csr_bus_addr);
	return 0;
}

static const struct auxiliary_device_id fboss_iob_i2c_ids[] = {
	{ .name = FBOSS_IOB_PCI_DRIVER".i2c_master" },
	{ .name = FBOSS_IOB_PCI_DRIVER".iob_i2c_master" },
	{ .name = FBOSS_IOB_PCI_DRIVER".dom1_i2c_master" },
	{ .name = FBOSS_IOB_PCI_DRIVER".dom2_i2c_master" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, fboss_iob_i2c_ids);

static struct auxiliary_driver fboss_iob_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = fbiob_i2c_probe,
	.remove = fbiob_i2c_remove,
	.id_table = fboss_iob_i2c_ids,
};
module_auxiliary_driver(fboss_iob_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tao Ren <taoren@meta.com>");
MODULE_DESCRIPTION("Meta FBOSS IOB_FPGA I2C Controller Driver");
MODULE_VERSION(BSP_VERSION);
