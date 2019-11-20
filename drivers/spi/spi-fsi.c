// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) IBM Corporation 2019
 */

#include <linux/bits.h>
#include <linux/fsi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#define FSI_ENGID_SPI			0x23

#define FSI2SPI_DATA0			0x00
#define FSI2SPI_DATA1			0x04
#define FSI2SPI_CMD			0x08
#define  FSI2SPI_CMD_WRITE		 BIT(31)
#define FSI2SPI_RESET			0x18
#define FSI2SPI_STATUS			0x1c
#define  FSI2SPI_STATUS_ANY_ERROR	 BIT(31)
#define FSI2SPI_IRQ			0x20

#define SPI_FSI_ERROR			0x00
#define SPI_FSI_COUNTER_CFG		0x08
#define  SPI_FSI_COUNTER_CFG_LOOPS(x)	 (((u64)(x) & 0xffULL) << 32)
#define SPI_FSI_CFG1			0x10
#define SPI_FSI_CLOCK_CFG		0x18
#define  SPI_FSI_CLOCK_CFG_MM_ENABLE	 BIT_ULL(32)
#define  SPI_FSI_CLOCK_CFG_ECC_DISABLE	 GENMASK_ULL(35, 33)
#define  SPI_FSI_CLOCK_CFG_RESET1	 (BIT_ULL(36) | BIT_ULL(38))
#define  SPI_FSI_CLOCK_CFG_RESET2	 (BIT_ULL(37) | BIT_ULL(39))
#define SPI_FSI_MMAP			0x20
#define SPI_FSI_DATA_TX			0x28
#define SPI_FSI_DATA_RX			0x30
#define SPI_FSI_SEQUENCE		0x38
#define  SPI_FSI_SEQUENCE_STOP		 0x00
#define  SPI_FSI_SEQUENCE_SEL_SLAVE(x)	 (0x10 | ((x) & 0xf))
#define  SPI_FSI_SEQUENCE_SHIFT_OUT(x)	 (0x30 | ((x) & 0xf))
#define  SPI_FSI_SEQUENCE_SHIFT_IN(x)	 (0x40 | ((x) & 0xf))
#define  SPI_FSI_SEQUENCE_COPY_DATA_TX	 0xc0
#define  SPI_FSI_SEQUENCE_BRANCH(x)	 (0xe0 | ((x) & 0xf))
#define SPI_FSI_STATUS			0x40
#define  SPI_FSI_STATUS_ERROR		 \
	(GENMASK_ULL(31, 21) | GENMASK_ULL(15, 12))
#define  SPI_FSI_STATUS_IDLE		 BIT_ULL(48)
#define  SPI_FSI_STATUS_TDR_UNDERRUN	 BIT_ULL(57)
#define  SPI_FSI_STATUS_TDR_OVERRUN	 BIT_ULL(58)
#define  SPI_FSI_STATUS_TDR_FULL	 BIT_ULL(59)
#define  SPI_FSI_STATUS_RDR_UNDERRUN	 BIT_ULL(61)
#define  SPI_FSI_STATUS_RDR_OVERRUN	 BIT_ULL(62)
#define  SPI_FSI_STATUS_RDR_FULL	 BIT_ULL(63)
#define  SPI_FSI_STATUS_ANY_ERROR	 \
	(SPI_FSI_STATUS_ERROR | SPI_FSI_STATUS_TDR_UNDERRUN | \
	 SPI_FSI_STATUS_TDR_OVERRUN | SPI_FSI_STATUS_TDR_FULL | \
	 SPI_FSI_STATUS_RDR_UNDERRUN | SPI_FSI_STATUS_RDR_OVERRUN | \
	 SPI_FSI_STATUS_RDR_FULL)
#define SPI_FSI_PORT_CTRL		0x48

struct fsi_spi {
	struct device *dev;
	struct fsi_device *fsi;
};

struct fsi_spi_sequence {
	int bit;
	u64 data;
};

static int fsi_spi_check_status(struct fsi_spi *ctx)
{
	int rc;
	u32 sts;
	__be32 sts_be;

	rc = fsi_device_read(ctx->fsi, FSI2SPI_STATUS, &sts_be,
			     sizeof(sts_be));
	if (rc)
		return rc;

	sts = be32_to_cpu(sts_be);
	if (sts & FSI2SPI_STATUS_ANY_ERROR) {
		dev_err(ctx->dev, "Error with FSI2SPI interface: %08x\n", sts);
		return -EIO;
	}

	return 0;
}

static int fsi_spi_read_reg(struct fsi_spi *ctx, u32 offset, u64 *value)
{
	int rc;
	__be32 cmd_be;
	__be32 data_be;

	*value = 0ULL;

	cmd_be = cpu_to_be32(offset);
	rc = fsi_device_write(ctx->fsi, FSI2SPI_CMD, &cmd_be, sizeof(cmd_be));
	if (rc)
		return rc;

	rc = fsi_spi_check_status(ctx);
	if (rc)
		return rc;

	rc = fsi_device_read(ctx->fsi, FSI2SPI_DATA0, &data_be,
			     sizeof(data_be));
	if (rc)
		return rc;

	*value |= (u64)be32_to_cpu(data_be) << 32;

	rc = fsi_device_read(ctx->fsi, FSI2SPI_DATA1, &data_be,
			     sizeof(data_be));
	if (rc)
		return rc;

	*value |= (u64)be32_to_cpu(data_be);
	dev_dbg(ctx->dev, "Register read %02x[%016llx]\n", offset, *value);

	return 0;
}

static int fsi_spi_write_reg(struct fsi_spi *ctx, u32 offset, u64 value)
{
	int rc;
	__be32 cmd_be;
	__be32 data_be;

	dev_dbg(ctx->dev, "Register write %02x[%016llx].\n", offset, value);

	data_be = cpu_to_be32((value >> 32) & 0xFFFFFFFF);
	rc = fsi_device_write(ctx->fsi, FSI2SPI_DATA0, &data_be,
			      sizeof(data_be));
	if (rc)
		return rc;

	data_be = cpu_to_be32(value & 0xFFFFFFFF);
	rc = fsi_device_write(ctx->fsi, FSI2SPI_DATA1, &data_be,
			      sizeof(data_be));
	if (rc)
		return rc;

	cmd_be = cpu_to_be32(offset | FSI2SPI_CMD_WRITE);
	rc = fsi_device_write(ctx->fsi, FSI2SPI_CMD, &cmd_be, sizeof(cmd_be));
	if (rc)
		return rc;

	return fsi_spi_check_status(ctx);
}

static int fsi_spi_data_in(u64 in, u8 *rx, int len)
{
	int i;
	int num_bytes = len > 8 ? 8 : len;

	for (i = 0; i < num_bytes; ++i)
		rx[i] = (u8)((in >> (8 * i)) & 0xffULL);

	return num_bytes;
}

static int fsi_spi_data_out(u64 *out, const u8 *tx, int len)
{
	int i;
	int num_bytes = len > 8 ? 8 : len;

	*out = 0ULL;

	for (i = 0; i < num_bytes; ++i)
		*out |= (u64)tx[i] << (8 * (8 - (i + 1)));

	return num_bytes;
}

static int fsi_spi_reset(struct fsi_spi *ctx)
{
	int rc;

	dev_info(ctx->dev, "Resetting SPI controller.\n");

	rc = fsi_spi_write_reg(ctx, SPI_FSI_CLOCK_CFG,
			       SPI_FSI_CLOCK_CFG_RESET1);
	if (rc)
		return rc;

	rc = fsi_spi_write_reg(ctx, SPI_FSI_CLOCK_CFG,
			       SPI_FSI_CLOCK_CFG_RESET2);
	return rc;
}

static int fsi_spi_sequence_add(struct fsi_spi_sequence *seq, u8 val)
{
	seq->data |= (u64)val << seq->bit;
	seq->bit -= 8;

	return ((64 - seq->bit) / 8) - 2;
}

static void fsi_spi_sequence_init(struct fsi_spi_sequence *seq)
{
	seq->bit = 56;
	seq->data = 0ULL;
}

static int fsi_spi_sequence_transfer(struct fsi_spi *ctx,
				     struct fsi_spi_sequence *seq,
				     struct spi_transfer *transfer)
{
	int loops = 1;
	int idx = 0;
	int rc;
	u8 len;
	u8 rem = 0;

	if (transfer->len > 8) {
		loops = transfer->len / 8;
		rem = transfer->len - (loops * 8);
		len = 8;
	} else {
		len = transfer->len;
	}

	if (transfer->tx_buf) {
		idx = fsi_spi_sequence_add(seq,
					   SPI_FSI_SEQUENCE_SHIFT_OUT(len));
		if (rem)
			rem = SPI_FSI_SEQUENCE_SHIFT_OUT(rem);
	} else if (transfer->rx_buf) {
		idx = fsi_spi_sequence_add(seq,
					   SPI_FSI_SEQUENCE_SHIFT_IN(len));
		if (rem)
			rem = SPI_FSI_SEQUENCE_SHIFT_IN(rem);
	} else {
		return -EINVAL;
	}

	if (loops > 1) {
		fsi_spi_sequence_add(seq, SPI_FSI_SEQUENCE_BRANCH(idx));

		if (rem)
			fsi_spi_sequence_add(seq, rem);

		rc = fsi_spi_write_reg(ctx, SPI_FSI_COUNTER_CFG,
				       SPI_FSI_COUNTER_CFG_LOOPS(loops - 1));
		if (rc) {
			/* Ensure error returns < 0 in this case. */
			if (rc > 0)
				rc = -rc;

			return rc;
		}

		return loops;
	}

	return 0;
}

static int fsi_spi_transfer_data(struct fsi_spi *ctx,
				 struct spi_transfer *transfer)
{
	int rc = 0;
	u64 status = 0ULL;

	if (transfer->tx_buf) {
		int nb;
		int sent = 0;
		u64 out = 0ULL;
		const u8 *tx = transfer->tx_buf;

		while (transfer->len > sent) {
			nb = fsi_spi_data_out(&out, &tx[sent],
					      (int)transfer->len - sent);

			rc = fsi_spi_write_reg(ctx, SPI_FSI_DATA_TX, out);
			if (rc)
				return rc;

			do {
				rc = fsi_spi_read_reg(ctx, SPI_FSI_STATUS,
						      &status);
				if (rc)
					return rc;

				if (status & SPI_FSI_STATUS_ANY_ERROR) {
					rc = fsi_spi_reset(ctx);
					if (rc)
						return rc;

					return -EREMOTEIO;
				}
			} while (status & SPI_FSI_STATUS_TDR_FULL);

			sent += nb;
		}
	} else if (transfer->rx_buf) {
		int recv = 0;
		u64 in = 0ULL;
		u8 *rx = transfer->rx_buf;

		while (transfer->len > recv) {
			do {
				rc = fsi_spi_read_reg(ctx, SPI_FSI_STATUS,
						     &status);
				if (rc)
					return rc;

				if (status & SPI_FSI_STATUS_ANY_ERROR) {
					rc = fsi_spi_reset(ctx);
					if (rc)
						return rc;

					return -EREMOTEIO;
				}
			} while (!(status & SPI_FSI_STATUS_RDR_FULL));

			rc = fsi_spi_read_reg(ctx, SPI_FSI_DATA_RX, &in);
			if (rc)
				return rc;

			recv += fsi_spi_data_in(in, rx,
						(int)transfer->len - recv);
		}
	}

	return 0;
}

static int fsi_spi_transfer_init(struct fsi_spi *ctx)
{
	int rc;
	u64 clock_cfg = 0ULL;
	u64 status = 0ULL;

	do {
		rc = fsi_spi_read_reg(ctx, SPI_FSI_STATUS, &status);
		if (rc)
			return rc;

		if (status & SPI_FSI_STATUS_ANY_ERROR) {
			rc = fsi_spi_reset(ctx);
			if (rc)
				return rc;
		}
	} while (!(status & SPI_FSI_STATUS_IDLE));

	rc = fsi_spi_read_reg(ctx, SPI_FSI_CLOCK_CFG, &clock_cfg);
	if (rc)
		return rc;

	if ((clock_cfg & (SPI_FSI_CLOCK_CFG_ECC_DISABLE |
			  SPI_FSI_CLOCK_CFG_MM_ENABLE)) !=
	    SPI_FSI_CLOCK_CFG_ECC_DISABLE)
		rc = fsi_spi_write_reg(ctx, SPI_FSI_CLOCK_CFG,
				       SPI_FSI_CLOCK_CFG_ECC_DISABLE);

	return rc;
}

static int fsi_spi_transfer_one_message(struct spi_controller *ctlr,
					struct spi_message *mesg)
{
	int rc = 0;
	u8 seq_sel_slave = SPI_FSI_SEQUENCE_SEL_SLAVE(mesg->spi->chip_select);
	struct spi_transfer *transfer;
	struct fsi_spi *ctx = spi_controller_get_devdata(ctlr);

	list_for_each_entry(transfer, &mesg->transfers, transfer_list) {
		struct fsi_spi_sequence seq;
		struct spi_transfer *next = NULL;

		dev_dbg(ctx->dev, "Start %s of %d bytes.\n",
			transfer->tx_buf ? "tx" : "rx", transfer->len);

		fsi_spi_transfer_init(ctx);

		fsi_spi_sequence_init(&seq);
		fsi_spi_sequence_add(&seq, seq_sel_slave);

		rc = fsi_spi_sequence_transfer(ctx, &seq, transfer);
		if (rc < 0)
			goto error;

		if (list_is_last(&transfer->transfer_list, &mesg->transfers)) {
			seq_sel_slave = SPI_FSI_SEQUENCE_SEL_SLAVE(0);
		} else if (!rc) {
			next = list_next_entry(transfer, transfer_list);

			dev_dbg(ctx->dev, "Sequence start %s of %d bytes.\n",
				transfer->tx_buf ? "tx" : "rx", transfer->len);

			rc = fsi_spi_sequence_transfer(ctx, &seq, next);
			if (rc < 0)
				goto error;
		}

		fsi_spi_sequence_add(&seq, seq_sel_slave);

		rc = fsi_spi_write_reg(ctx, SPI_FSI_SEQUENCE, seq.data);
		if (rc)
			goto error;

		rc = fsi_spi_transfer_data(ctx, transfer);
		if (rc)
			goto error;

		if (next) {
			rc = fsi_spi_transfer_data(ctx, next);
			if (rc)
				goto error;

			transfer = next;
		}
	}

error:
	mesg->status = rc;
	spi_finalize_current_message(ctlr);

	return rc;
}

static size_t fsi_spi_max_transfer_size(struct spi_device *spi)
{
	return 2048;
}

static int fsi_spi_probe(struct device *dev)
{
	int rc;
	struct spi_controller *ctlr;
	struct fsi_spi *ctx;

	ctlr = spi_alloc_master(dev, sizeof(*ctx));
	if (!ctlr)
		return -ENOMEM;

	ctlr->dev.of_node = dev->of_node;
	ctlr->num_chipselect = 4;
	ctlr->flags = SPI_CONTROLLER_HALF_DUPLEX;
	ctlr->max_transfer_size = fsi_spi_max_transfer_size;
	ctlr->transfer_one_message = fsi_spi_transfer_one_message;

	ctx = spi_controller_get_devdata(ctlr);
	ctx->dev = &ctlr->dev;
	ctx->fsi = to_fsi_dev(dev);

	rc = devm_spi_register_controller(dev, ctlr);
	if (rc)
		spi_controller_put(ctlr);

	return rc;
}

static int fsi_spi_remove(struct device *dev)
{
	return 0;
}

static const struct fsi_device_id fsi_spi_ids[] = {
	{ FSI_ENGID_SPI, FSI_VERSION_ANY },
	{ }
};

static struct fsi_driver fsi_spi_driver = {
	.id_table = fsi_spi_ids,
	.drv = {
		.name = "spi-fsi",
		.bus = &fsi_bus_type,
		.probe = fsi_spi_probe,
		.remove = fsi_spi_remove,
	},
};

module_fsi_driver(fsi_spi_driver);

MODULE_AUTHOR("Eddie James <eajames@linux.ibm.com>");
MODULE_DESCRIPTION("FSI attached SPI controller");
MODULE_LICENSE("GPL");
