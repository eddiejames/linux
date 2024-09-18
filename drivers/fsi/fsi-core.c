// SPDX-License-Identifier: GPL-2.0-only
/*
 * FSI core driver
 *
 * Copyright (C) IBM Corporation 2016
 *
 * TODO:
 *  - Rework topology
 *  - s/chip_id/chip_loc
 *  - s/cfam/chip (cfam_id -> chip_id etc...)
 */

#include <linux/crc4.h>
#include <linux/device.h>
#include <linux/fsi.h>
#include <linux/idr.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "fsi-master.h"
#include "fsi-slave.h"

#define CREATE_TRACE_POINTS
#include <trace/events/fsi.h>

#define FSI_SLAVE_CONF_NEXT_MASK	GENMASK(31, 31)
#define FSI_SLAVE_CONF_SLOTS_MASK	GENMASK(23, 16)
#define FSI_SLAVE_CONF_SLOTS_SHIFT	16
#define FSI_SLAVE_CONF_VERSION_MASK	GENMASK(15, 12)
#define FSI_SLAVE_CONF_VERSION_SHIFT	12
#define FSI_SLAVE_CONF_TYPE_MASK	GENMASK(11, 4)
#define FSI_SLAVE_CONF_TYPE_SHIFT	4
#define FSI_SLAVE_CONF_CRC_SHIFT	4
#define FSI_SLAVE_CONF_CRC_MASK		GENMASK(3, 0)
#define FSI_SLAVE_CONF_DATA_BITS	28

#define FSI_PEEK_BASE			0x410

static const int engine_page_size = 0x400;

#define FSI_SLAVE_SIZE_23b		0x800000

static DEFINE_IDA(master_ida);

static const int slave_retries = 2;
static int discard_errors;

static dev_t fsi_base_dev;
static DEFINE_IDA(fsi_minor_ida);
#define FSI_CHAR_MAX_DEVICES	0x1000

/* Legacy /dev numbering: 4 devices per chip, 16 chips */
#define FSI_CHAR_LEGACY_TOP	64

static int fsi_master_read(struct fsi_master *master, int link,
		uint8_t slave_id, uint32_t addr, void *val, size_t size);
static int fsi_master_write(struct fsi_master *master, int link,
		uint8_t slave_id, uint32_t addr, const void *val, size_t size);
static int fsi_master_break(struct fsi_master *master, int link);

/*
 * fsi_device_read() / fsi_device_write() / fsi_device_peek()
 *
 * FSI endpoint-device support
 *
 * Read / write / peek accessors for a client
 *
 * Parameters:
 * dev:  Structure passed to FSI client device drivers on probe().
 * addr: FSI address of given device.  Client should pass in its base address
 *       plus desired offset to access its register space.
 * val:  For read/peek this is the value read at the specified address. For
 *       write this is value to write to the specified address.
 *       The data in val must be FSI bus endian (big endian).
 * size: Size in bytes of the operation.  Sizes supported are 1, 2 and 4 bytes.
 *       Addresses must be aligned on size boundaries or an error will result.
 */
int fsi_device_read(struct fsi_device *dev, uint32_t addr, void *val,
		size_t size)
{
	if (addr > dev->size || size > dev->size || addr > dev->size - size)
		return -EINVAL;

	return fsi_slave_read(dev->slave, dev->addr + addr, val, size);
}
EXPORT_SYMBOL_GPL(fsi_device_read);

int fsi_device_write(struct fsi_device *dev, uint32_t addr, const void *val,
		size_t size)
{
	if (addr > dev->size || size > dev->size || addr > dev->size - size)
		return -EINVAL;

	return fsi_slave_write(dev->slave, dev->addr + addr, val, size);
}
EXPORT_SYMBOL_GPL(fsi_device_write);

int fsi_device_peek(struct fsi_device *dev, void *val)
{
	uint32_t addr = FSI_PEEK_BASE + ((dev->unit - 2) * sizeof(uint32_t));

	return fsi_slave_read(dev->slave, addr, val, sizeof(uint32_t));
}

unsigned long fsi_device_local_bus_frequency(struct fsi_device *dev)
{
	return dev->slave->master->clock_frequency / dev->slave->clock_div;
}
EXPORT_SYMBOL_GPL(fsi_device_local_bus_frequency);

static void fsi_device_release(struct device *_device)
{
	struct fsi_device *device = to_fsi_dev(_device);

	of_node_put(device->dev.of_node);
	kfree(device);
}

static struct fsi_device *fsi_create_device(struct fsi_slave *slave)
{
	struct fsi_device *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	dev->dev.parent = &slave->dev;
	dev->dev.bus = &fsi_bus_type;
	dev->dev.release = fsi_device_release;

	return dev;
}

/* FSI slave support */
static int fsi_slave_calc_addr(struct fsi_slave *slave, uint32_t *addrp,
		uint8_t *idp)
{
	uint32_t addr = *addrp;
	uint8_t id = *idp;

	if (addr > slave->size)
		return -EINVAL;

	/* For 23 bit addressing, we encode the extra two bits in the slave
	 * id (and the slave's actual ID needs to be 0).
	 */
	if (addr > 0x1fffff) {
		if (slave->id != 0)
			return -EINVAL;
		id = (addr >> 21) & 0x3;
		addr &= 0x1fffff;
	}

	*addrp = addr;
	*idp = id;
	return 0;
}

static int fsi_slave_report_and_clear_errors(struct fsi_slave *slave)
{
	struct fsi_master *master = slave->master;
	__be32 irq, reset, stat;
	int rc, link;
	uint8_t id;

	link = slave->link;
	id = slave->id;

	rc = fsi_master_read(master, link, id, FSI_SLAVE_BASE + FSI_SISC,
			&irq, sizeof(irq));
	if (rc)
		return rc;

	rc =  fsi_master_read(master, link, id, FSI_SLAVE_BASE + FSI_SSTAT,
			&stat, sizeof(stat));
	if (rc)
		return rc;

	dev_dbg(&slave->dev, "status: 0x%08x, sisc: 0x%08x\n",
			be32_to_cpu(stat), be32_to_cpu(irq));
	trace_fsi_slave_error(slave, be32_to_cpu(irq), be32_to_cpu(stat));

	/* reset errors */
	reset = cpu_to_be32(FSI_SRES_ERRS);
	return fsi_master_write(master, link, id, FSI_SLAVE_BASE + FSI_SRES, &reset,
				sizeof(reset));
}

/* Encode slave local bus echo delay */
static inline uint32_t fsi_smode_echodly(int x)
{
	return (x & FSI_SMODE_ED_MASK) << FSI_SMODE_ED_SHIFT;
}

/* Encode slave local bus send delay */
static inline uint32_t fsi_smode_senddly(int x)
{
	return (x & FSI_SMODE_SD_MASK) << FSI_SMODE_SD_SHIFT;
}

/* Encode slave local bus clock rate ratio */
static inline uint32_t fsi_smode_lbcrr(int x)
{
	return (x & FSI_SMODE_LBCRR_MASK) << FSI_SMODE_LBCRR_SHIFT;
}

/* Encode slave ID */
static inline uint32_t fsi_smode_sid(int x)
{
	return (x & FSI_SMODE_SID_MASK) << FSI_SMODE_SID_SHIFT;
}

static uint32_t fsi_slave_smode(int id, int div, u8 t_senddly, u8 t_echodly)
{
	return FSI_SMODE_WSC | FSI_SMODE_ECRC
		| fsi_smode_sid(id)
		| fsi_smode_echodly(t_echodly - 1) | fsi_smode_senddly(t_senddly - 1)
		| fsi_smode_lbcrr(div - 1);
}

static int fsi_slave_set_smode(struct fsi_slave *slave, uint8_t id)
{
	uint32_t smode;
	__be32 data;

	/* set our smode register with the slave ID field to 0; this enables
	 * extended slave addressing
	 */
	smode = fsi_slave_smode(slave->id, slave->clock_div, slave->t_send_delay,
				slave->t_echo_delay);
	data = cpu_to_be32(smode);

	return fsi_master_write(slave->master, slave->link, id, FSI_SLAVE_BASE + FSI_SMODE,
				&data, sizeof(data));
}

static int fsi_slave_handle_error(struct fsi_slave *slave, bool write,
				  uint32_t addr, size_t size)
{
	struct fsi_master *master = slave->master;
	int rc, link;
	uint32_t reg;
	uint8_t id, send_delay, echo_delay;

	if (discard_errors)
		return -1;

	link = slave->link;
	id = slave->id;

	dev_dbg(&slave->dev, "handling error on %s to 0x%08x[%zd]",
			write ? "write" : "read", addr, size);

	/* try a simple clear of error conditions, which may fail if we've lost
	 * communication with the slave
	 */
	rc = fsi_slave_report_and_clear_errors(slave);
	if (!rc)
		return 0;

	/* send a TERM and retry */
	if (master->term) {
		rc = master->term(master, link, id);
		if (!rc) {
			rc = fsi_master_read(master, link, id, 0,
					&reg, sizeof(reg));
			if (!rc)
				rc = fsi_slave_report_and_clear_errors(slave);
			if (!rc)
				return 0;
		}
	}

	send_delay = slave->t_send_delay;
	echo_delay = slave->t_echo_delay;

	/* getting serious, reset the slave via BREAK */
	rc = fsi_master_break(master, link);
	if (rc)
		return rc;

	slave->t_send_delay = send_delay;
	slave->t_echo_delay = echo_delay;

	rc = fsi_slave_set_smode(slave, FSI_SMODE_SID_BREAK);
	if (rc)
		return rc;

	if (master->link_config)
		master->link_config(master, link,
				    slave->t_send_delay,
				    slave->t_echo_delay);

	return fsi_slave_report_and_clear_errors(slave);
}

int fsi_slave_read(struct fsi_slave *slave, uint32_t addr,
			void *val, size_t size)
{
	unsigned long flags;
	uint8_t id = slave->id;
	int rc, err_rc, i;

	rc = fsi_slave_calc_addr(slave, &addr, &id);
	if (rc)
		return rc;

	spin_lock_irqsave(&slave->lock, flags);
	for (i = 0; i < slave_retries; i++) {
		rc = fsi_master_read(slave->master, slave->link,
				id, addr, val, size);
		if (!rc)
			break;

		err_rc = fsi_slave_handle_error(slave, false, addr, size);
		if (err_rc)
			break;
	}
	spin_unlock_irqrestore(&slave->lock, flags);

	return rc;
}
EXPORT_SYMBOL_GPL(fsi_slave_read);

int fsi_slave_write(struct fsi_slave *slave, uint32_t addr,
			const void *val, size_t size)
{
	unsigned long flags;
	uint8_t id = slave->id;
	int rc, err_rc, i;

	rc = fsi_slave_calc_addr(slave, &addr, &id);
	if (rc)
		return rc;

	spin_lock_irqsave(&slave->lock, flags);
	for (i = 0; i < slave_retries; i++) {
		rc = fsi_master_write(slave->master, slave->link,
				id, addr, val, size);
		if (!rc)
			break;

		err_rc = fsi_slave_handle_error(slave, true, addr, size);
		if (err_rc)
			break;
	}
	spin_unlock_irqrestore(&slave->lock, flags);

	return rc;
}
EXPORT_SYMBOL_GPL(fsi_slave_write);

int fsi_slave_claim_range(struct fsi_slave *slave,
			  uint32_t addr, uint32_t size)
{
	if (addr + size < addr)
		return -EINVAL;

	if (addr + size > slave->size)
		return -EINVAL;

	/* todo: check for overlapping claims */
	return 0;
}
EXPORT_SYMBOL_GPL(fsi_slave_claim_range);

void fsi_slave_release_range(struct fsi_slave *slave,
			     uint32_t addr, uint32_t size)
{
}
EXPORT_SYMBOL_GPL(fsi_slave_release_range);

static bool fsi_device_node_matches(struct device *dev, struct device_node *np,
		uint32_t addr, uint32_t size)
{
	u64 paddr, psize;

	if (of_property_read_reg(np, 0, &paddr, &psize))
		return false;

	if (paddr != addr)
		return false;

	if (psize != size) {
		dev_warn(dev,
			"node %pOF matches probed address, but not size (got 0x%llx, expected 0x%x)",
			np, psize, size);
	}

	return true;
}

/* Find a matching node for the slave engine at @address, using @size bytes
 * of space. Returns NULL if not found, or a matching node with refcount
 * already incremented.
 */
static struct device_node *fsi_device_find_of_node(struct fsi_device *dev)
{
	struct device_node *parent, *np;

	parent = dev_of_node(&dev->slave->dev);
	if (!parent)
		return NULL;

	for_each_child_of_node(parent, np) {
		if (fsi_device_node_matches(&dev->dev, np,
					dev->addr, dev->size))
			return np;
	}

	return NULL;
}

static int fsi_slave_scan(struct fsi_slave *slave)
{
	uint32_t engine_addr;
	int rc, i;

	/*
	 * scan engines
	 *
	 * We keep the peek mode and slave engines for the core; so start
	 * at the third slot in the configuration table. We also need to
	 * skip the chip ID entry at the start of the address space.
	 */
	engine_addr = engine_page_size * 3;
	for (i = 2; i < engine_page_size / sizeof(uint32_t); i++) {
		uint8_t slots, version, type, crc;
		struct fsi_device *dev;
		uint32_t conf;
		__be32 data;

		rc = fsi_slave_read(slave, (i + 1) * sizeof(data),
				&data, sizeof(data));
		if (rc) {
			dev_warn(&slave->dev,
				"error reading slave registers\n");
			return -1;
		}
		conf = be32_to_cpu(data);

		crc = crc4(0, conf, 32);
		if (crc) {
			dev_warn(&slave->dev,
				"crc error in slave register at 0x%04x\n",
				i);
			return -1;
		}

		slots = (conf & FSI_SLAVE_CONF_SLOTS_MASK)
			>> FSI_SLAVE_CONF_SLOTS_SHIFT;
		version = (conf & FSI_SLAVE_CONF_VERSION_MASK)
			>> FSI_SLAVE_CONF_VERSION_SHIFT;
		type = (conf & FSI_SLAVE_CONF_TYPE_MASK)
			>> FSI_SLAVE_CONF_TYPE_SHIFT;

		/*
		 * Unused address areas are marked by a zero type value; this
		 * skips the defined address areas
		 */
		if (type != 0 && slots != 0) {

			/* create device */
			dev = fsi_create_device(slave);
			if (!dev)
				return -ENOMEM;

			dev->slave = slave;
			dev->engine_type = type;
			dev->version = version;
			dev->unit = i;
			dev->addr = engine_addr;
			dev->size = slots * engine_page_size;

			trace_fsi_dev_init(dev);

			dev_dbg(&slave->dev,
			"engine[%i]: type %x, version %x, addr %x size %x\n",
					dev->unit, dev->engine_type, version,
					dev->addr, dev->size);

			dev_set_name(&dev->dev, "%02x:%02x:%02x:%02x",
					slave->master->idx, slave->link,
					slave->id, i - 2);
			dev->dev.of_node = fsi_device_find_of_node(dev);

			rc = device_register(&dev->dev);
			if (rc) {
				dev_warn(&slave->dev, "add failed: %d\n", rc);
				put_device(&dev->dev);
			}
		}

		engine_addr += slots * engine_page_size;

		if (!(conf & FSI_SLAVE_CONF_NEXT_MASK))
			break;
	}

	return 0;
}

static unsigned long aligned_access_size(size_t offset, size_t count)
{
	unsigned long offset_unit, count_unit;

	/* Criteria:
	 *
	 * 1. Access size must be less than or equal to the maximum access
	 *    width or the highest power-of-two factor of offset
	 * 2. Access size must be less than or equal to the amount specified by
	 *    count
	 *
	 * The access width is optimal if we can calculate 1 to be strictly
	 * equal while still satisfying 2.
	 */

	/* Find 1 by the bottom bit of offset (with a 4 byte access cap) */
	offset_unit = BIT(__builtin_ctzl(offset | 4));

	/* Find 2 by the top bit of count */
	count_unit = BIT(8 * sizeof(unsigned long) - 1 - __builtin_clzl(count));

	/* Constrain the maximum access width to the minimum of both criteria */
	return BIT(__builtin_ctzl(offset_unit | count_unit));
}

static ssize_t fsi_slave_sysfs_raw_read(struct file *file,
		struct kobject *kobj, struct bin_attribute *attr, char *buf,
		loff_t off, size_t count)
{
	struct fsi_slave *slave = to_fsi_slave(kobj_to_dev(kobj));
	size_t total_len, read_len;
	int rc;

	if (off < 0)
		return -EINVAL;

	if (off > 0xffffffff || count > 0xffffffff || off + count > 0xffffffff)
		return -EINVAL;

	for (total_len = 0; total_len < count; total_len += read_len) {
		read_len = aligned_access_size(off, count - total_len);

		rc = fsi_slave_read(slave, off, buf + total_len, read_len);
		if (rc)
			return rc;

		off += read_len;
	}

	return count;
}

static ssize_t fsi_slave_sysfs_raw_write(struct file *file,
		struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct fsi_slave *slave = to_fsi_slave(kobj_to_dev(kobj));
	size_t total_len, write_len;
	int rc;

	if (off < 0)
		return -EINVAL;

	if (off > 0xffffffff || count > 0xffffffff || off + count > 0xffffffff)
		return -EINVAL;

	for (total_len = 0; total_len < count; total_len += write_len) {
		write_len = aligned_access_size(off, count - total_len);

		rc = fsi_slave_write(slave, off, buf + total_len, write_len);
		if (rc)
			return rc;

		off += write_len;
	}

	return count;
}

static const struct bin_attribute fsi_slave_raw_attr = {
	.attr = {
		.name = "raw",
		.mode = 0600,
	},
	.size = 0,
	.read = fsi_slave_sysfs_raw_read,
	.write = fsi_slave_sysfs_raw_write,
};

static void fsi_slave_release(struct device *dev)
{
	struct fsi_slave *slave = to_fsi_slave(dev);

	fsi_free_minor(slave->dev.devt);
	of_node_put(dev->of_node);
	kfree(slave);
}

static bool fsi_slave_node_matches(struct device_node *np,
		int link, uint8_t id)
{
	u64 addr;

	if (of_property_read_reg(np, 0, &addr, NULL))
		return false;

	return addr == (((u64)link << 32) | id);
}

/* Find a matching node for the slave at (link, id). Returns NULL if none
 * found, or a matching node with refcount already incremented.
 */
static struct device_node *fsi_slave_find_of_node(struct fsi_master *master,
		int link, uint8_t id)
{
	struct device_node *parent, *np;

	parent = dev_of_node(&master->dev);
	if (!parent)
		return NULL;

	for_each_child_of_node(parent, np) {
		if (fsi_slave_node_matches(np, link, id))
			return np;
	}

	return NULL;
}

static ssize_t cfam_read(struct file *filep, char __user *buf, size_t count,
			 loff_t *offset)
{
	struct fsi_slave *slave = filep->private_data;
	size_t total_len, read_len;
	loff_t off = *offset;
	ssize_t rc;

	if (off < 0)
		return -EINVAL;

	if (off > 0xffffffff || count > 0xffffffff || off + count > 0xffffffff)
		return -EINVAL;

	for (total_len = 0; total_len < count; total_len += read_len) {
		__be32 data;

		read_len = min_t(size_t, count, 4);
		read_len -= off & 0x3;

		rc = fsi_slave_read(slave, off, &data, read_len);
		if (rc)
			goto fail;
		rc = copy_to_user(buf + total_len, &data, read_len);
		if (rc) {
			rc = -EFAULT;
			goto fail;
		}
		off += read_len;
	}
	rc = count;
 fail:
	*offset = off;
	return rc;
}

static ssize_t cfam_write(struct file *filep, const char __user *buf,
			  size_t count, loff_t *offset)
{
	struct fsi_slave *slave = filep->private_data;
	size_t total_len, write_len;
	loff_t off = *offset;
	ssize_t rc;


	if (off < 0)
		return -EINVAL;

	if (off > 0xffffffff || count > 0xffffffff || off + count > 0xffffffff)
		return -EINVAL;

	for (total_len = 0; total_len < count; total_len += write_len) {
		__be32 data;

		write_len = min_t(size_t, count, 4);
		write_len -= off & 0x3;

		rc = copy_from_user(&data, buf + total_len, write_len);
		if (rc) {
			rc = -EFAULT;
			goto fail;
		}
		rc = fsi_slave_write(slave, off, &data, write_len);
		if (rc)
			goto fail;
		off += write_len;
	}
	rc = count;
 fail:
	*offset = off;
	return rc;
}

static loff_t cfam_llseek(struct file *file, loff_t offset, int whence)
{
	switch (whence) {
	case SEEK_CUR:
		break;
	case SEEK_SET:
		file->f_pos = offset;
		break;
	default:
		return -EINVAL;
	}

	return offset;
}

static int cfam_open(struct inode *inode, struct file *file)
{
	struct fsi_slave *slave = container_of(inode->i_cdev, struct fsi_slave, cdev);

	file->private_data = slave;

	return 0;
}

static const struct file_operations cfam_fops = {
	.owner		= THIS_MODULE,
	.open		= cfam_open,
	.llseek		= cfam_llseek,
	.read		= cfam_read,
	.write		= cfam_write,
};

static ssize_t send_term_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct fsi_slave *slave = to_fsi_slave(dev);
	struct fsi_master *master = slave->master;

	if (!master->term)
		return -ENODEV;

	master->term(master, slave->link, slave->id);
	return count;
}

static DEVICE_ATTR_WO(send_term);

static ssize_t slave_send_echo_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fsi_slave *slave = to_fsi_slave(dev);

	return sprintf(buf, "%u\n", slave->t_send_delay);
}

static ssize_t slave_send_echo_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fsi_slave *slave = to_fsi_slave(dev);
	struct fsi_master *master = slave->master;
	unsigned long val;
	int rc;

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	if (val < 1 || val > 16)
		return -EINVAL;

	if (!master->link_config)
		return -ENXIO;

	/* Current HW mandates that send and echo delay are identical */
	slave->t_send_delay = val;
	slave->t_echo_delay = val;

	rc = fsi_slave_set_smode(slave, slave->id);
	if (rc < 0)
		return rc;
	if (master->link_config)
		master->link_config(master, slave->link,
				    slave->t_send_delay,
				    slave->t_echo_delay);

	return count;
}

static DEVICE_ATTR(send_echo_delays, 0600,
		   slave_send_echo_show, slave_send_echo_store);

static ssize_t chip_id_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct fsi_slave *slave = to_fsi_slave(dev);

	return sprintf(buf, "%d\n", slave->chip_id);
}

static DEVICE_ATTR_RO(chip_id);

static ssize_t cfam_id_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct fsi_slave *slave = to_fsi_slave(dev);

	return sprintf(buf, "0x%x\n", slave->cfam_id);
}

static DEVICE_ATTR_RO(cfam_id);

static ssize_t config_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const unsigned int end = engine_page_size / sizeof(u32);
	struct fsi_slave *slave = to_fsi_slave(dev);
	__be32 data;
	int len = 0;
	u32 conf;
	int rc;

	for (unsigned int i = 0; i < end; ++i) {
		rc = fsi_slave_read(slave, i * sizeof(data), &data, sizeof(data));
		if (rc)
			return rc;

		conf = be32_to_cpu(data);
		if (crc4(0, conf, 32))
			return -EBADMSG;

		len += sysfs_emit_at(buf, len, "%08x\n", conf);
		if (!(conf & FSI_SLAVE_CONF_NEXT_MASK))
			break;
	}

	return len;
}

static DEVICE_ATTR_RO(config_table);

struct fsi_slave_attribute {
	struct device_attribute attr;
	int reg;
};

static ssize_t slave_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsi_slave_attribute *fattr = container_of(attr, struct fsi_slave_attribute, attr);
	struct fsi_slave *slave = to_fsi_slave(dev);
	__be32 data;
	int rc;

	rc = fsi_slave_read(slave, FSI_SLAVE_BASE + fattr->reg, &data, sizeof(data));
	if (rc)
		return rc;

	return sysfs_emit(buf, "%08x\n", be32_to_cpu(data));
}

static ssize_t slave_reg_8bpp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsi_slave_attribute *fattr = container_of(attr, struct fsi_slave_attribute, attr);
	struct fsi_slave *slave = to_fsi_slave(dev);
	__be32 data;
	int len = 0;
	int rc;
	int i;

	for (i = 0; i < 2; ++i) {
		rc = fsi_slave_read(slave, FSI_SLAVE_BASE + fattr->reg + (i * 4), &data,
				    sizeof(data));
		if (rc)
			return rc;

		len += sysfs_emit_at(buf, len, "%08x\n", be32_to_cpu(data));
	}

	return len;
}

#define FSI_SLAVE_ATTR(name, reg) struct fsi_slave_attribute dev_attr_##name = \
	{ __ATTR(name, 0444, slave_reg_show, NULL), reg }
#define FSI_SLAVE_ATTR_8BPP(name, reg) struct fsi_slave_attribute dev_attr_##name = \
	{ __ATTR(name, 0444, slave_reg_8bpp_show, NULL), reg }

static FSI_SLAVE_ATTR(smode, FSI_SMODE);
static FSI_SLAVE_ATTR(sdma, FSI_SDMA);
static FSI_SLAVE_ATTR(sisc, FSI_SISC);
static FSI_SLAVE_ATTR(sism, FSI_SISM);
static FSI_SLAVE_ATTR(siss, FSI_SISS);
static FSI_SLAVE_ATTR(sstat, FSI_SSTAT);
static FSI_SLAVE_ATTR(si1m, FSI_SI1M);
static FSI_SLAVE_ATTR(si1s, FSI_SI1S);
static FSI_SLAVE_ATTR(sic, FSI_SIC);
static FSI_SLAVE_ATTR(si2m, FSI_SI2M);
static FSI_SLAVE_ATTR(si2s, FSI_SI2S);
static FSI_SLAVE_ATTR(scmdt, FSI_SCMDT);
static FSI_SLAVE_ATTR(sdata, FSI_SDATA);
static FSI_SLAVE_ATTR(slastd, FSI_SLASTD);
static FSI_SLAVE_ATTR(smbl, FSI_SMBL);
static FSI_SLAVE_ATTR(soml, FSI_SOML);
static FSI_SLAVE_ATTR(snml, FSI_SNML);
static FSI_SLAVE_ATTR(smbr, FSI_SMBR);
static FSI_SLAVE_ATTR(somr, FSI_SOMR);
static FSI_SLAVE_ATTR(snmr, FSI_SNMR);
static FSI_SLAVE_ATTR_8BPP(scrsic, FSI_ScRSIC0);
static FSI_SLAVE_ATTR_8BPP(scrsim, FSI_ScRSIM0);
static FSI_SLAVE_ATTR_8BPP(scrsis, FSI_ScRSIS0);
static FSI_SLAVE_ATTR_8BPP(srsic, FSI_SRSIC0);
static FSI_SLAVE_ATTR_8BPP(srsim, FSI_SRSIM0);
static FSI_SLAVE_ATTR_8BPP(srsis, FSI_SRSIS0);
static FSI_SLAVE_ATTR(llmode, FSI_LLMODE);
static FSI_SLAVE_ATTR(llstat, FSI_LLSTAT);

static struct attribute *cfam_attrs[] = {
	&dev_attr_send_echo_delays.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_cfam_id.attr,
	&dev_attr_send_term.attr,
	&dev_attr_config_table.attr,
	&dev_attr_smode.attr.attr,
	&dev_attr_sdma.attr.attr,
	&dev_attr_sisc.attr.attr,
	&dev_attr_sism.attr.attr,
	&dev_attr_siss.attr.attr,
	&dev_attr_sstat.attr.attr,
	&dev_attr_si1m.attr.attr,
	&dev_attr_si1s.attr.attr,
	&dev_attr_sic.attr.attr,
	&dev_attr_si2m.attr.attr,
	&dev_attr_si2s.attr.attr,
	&dev_attr_scmdt.attr.attr,
	&dev_attr_sdata.attr.attr,
	&dev_attr_slastd.attr.attr,
	&dev_attr_smbl.attr.attr,
	&dev_attr_soml.attr.attr,
	&dev_attr_snml.attr.attr,
	&dev_attr_smbr.attr.attr,
	&dev_attr_somr.attr.attr,
	&dev_attr_snmr.attr.attr,
	&dev_attr_scrsic.attr.attr,
	&dev_attr_scrsim.attr.attr,
	&dev_attr_scrsis.attr.attr,
	&dev_attr_srsic.attr.attr,
	&dev_attr_srsim.attr.attr,
	&dev_attr_srsis.attr.attr,
	&dev_attr_llmode.attr.attr,
	&dev_attr_llstat.attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(cfam);

static char *cfam_devnode(const struct device *dev, umode_t *mode,
			  kuid_t *uid, kgid_t *gid)
{
	const struct fsi_slave *slave = to_fsi_slave(dev);

#ifdef CONFIG_FSI_NEW_DEV_NODE
	return kasprintf(GFP_KERNEL, "fsi/cfam%d", slave->cdev_idx);
#else
	return kasprintf(GFP_KERNEL, "cfam%d", slave->cdev_idx);
#endif
}

static const struct device_type cfam_type = {
	.name = "cfam",
	.devnode = cfam_devnode,
	.groups = cfam_groups
};

static char *fsi_cdev_devnode(const struct device *dev, umode_t *mode,
			      kuid_t *uid, kgid_t *gid)
{
#ifdef CONFIG_FSI_NEW_DEV_NODE
	return kasprintf(GFP_KERNEL, "fsi/%s", dev_name(dev));
#else
	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
#endif
}

const struct device_type fsi_cdev_type = {
	.name = "fsi-cdev",
	.devnode = fsi_cdev_devnode,
};
EXPORT_SYMBOL_GPL(fsi_cdev_type);

/* Backward compatible /dev/ numbering in "old style" mode */
static int fsi_adjust_index(int index)
{
#ifdef CONFIG_FSI_NEW_DEV_NODE
	return index;
#else
	return index + 1;
#endif
}

static const char *const fsi_dev_type_names[] = {
	"cfam",
	"sbefifo",
	"scom",
	"occ",
};

static int __fsi_get_new_minor(struct fsi_slave *slave, struct device_node *np,
			       enum fsi_dev_type type, dev_t *out_dev, int *out_index)
{
	int cid = slave->chip_id;
	int id;

	if (np && type < 4) {
		int aid = of_alias_get_id(np, fsi_dev_type_names[type]);

		if (aid >= 0) {
			/* Use the same scheme as the legacy numbers. */
			id = (aid << 2) | type;
			id = ida_alloc_range(&fsi_minor_ida, id, id, GFP_KERNEL);
			if (id >= 0) {
				*out_index = aid;
				*out_dev = fsi_base_dev + id;
				return 0;
			}

			if (id != -ENOSPC)
				return id;
		}
	}

	/* Check if we qualify for legacy numbering */
	if (cid >= 0 && cid < 16 && type < 4) {
		/*
		 * Try reserving the legacy number, which has 0 - 0x3f reserved
		 * in the ida range. cid goes up to 0xf and type contains two
		 * bits, so construct the id with the below two bit shift.
		 */
		id = (cid << 2) | type;
		id = ida_alloc_range(&fsi_minor_ida, id, id, GFP_KERNEL);
		if (id >= 0) {
			*out_index = fsi_adjust_index(cid);
			*out_dev = fsi_base_dev + id;
			return 0;
		}
		/* Other failure */
		if (id != -ENOSPC)
			return id;
		/* Fallback to non-legacy allocation */
	}
	id = ida_alloc_range(&fsi_minor_ida, FSI_CHAR_LEGACY_TOP,
			     FSI_CHAR_MAX_DEVICES - 1, GFP_KERNEL);
	if (id < 0)
		return id;
	*out_index = fsi_adjust_index(id);
	*out_dev = fsi_base_dev + id;
	return 0;
}

int fsi_get_new_minor(struct fsi_device *fdev, enum fsi_dev_type type,
		      dev_t *out_dev, int *out_index)
{
	return __fsi_get_new_minor(fdev->slave, fdev->dev.of_node, type, out_dev, out_index);
}
EXPORT_SYMBOL_GPL(fsi_get_new_minor);

void fsi_free_minor(dev_t dev)
{
	ida_free(&fsi_minor_ida, MINOR(dev));
}
EXPORT_SYMBOL_GPL(fsi_free_minor);

static int fsi_slave_init(struct fsi_master *master, int link, uint8_t id)
{
	const uint8_t break_id = (master->flags & FSI_MASTER_FLAG_NO_BREAK_SID) ? 0 :
		FSI_SMODE_SID_BREAK;
	uint32_t cfam_id;
	struct fsi_slave *slave;
	uint8_t crc;
	__be32 data, llmode, slbus;
	u32 clock;
	int rc;

	/* Currently, we only support single slaves on a link, and use the
	 * full 23-bit address range
	 */
	if (id != 0)
		return -EINVAL;

	rc = fsi_master_read(master, link, break_id, 0, &data, sizeof(data));
	if (rc) {
		dev_dbg(&master->dev, "can't read slave %02x:%02x %d\n",
				link, id, rc);
		return -ENODEV;
	}
	cfam_id = be32_to_cpu(data);

	crc = crc4(0, cfam_id, 32);
	if (crc) {
		trace_fsi_slave_invalid_cfam(master, link, cfam_id);
		dev_warn(&master->dev, "slave %02x:%02x invalid cfam id CRC!\n",
				link, id);
		return -EIO;
	}

	dev_dbg(&master->dev, "fsi: found chip %08x at %02x:%02x:%02x\n",
			cfam_id, master->idx, link, id);

	/* If we're behind a master that doesn't provide a self-running bus
	 * clock, put the slave into async mode
	 */
	if (master->flags & FSI_MASTER_FLAG_SWCLOCK) {
		llmode = cpu_to_be32(FSI_LLMODE_ASYNC);
		rc = fsi_master_write(master, link, break_id, FSI_SLAVE_BASE + FSI_LLMODE, &llmode,
				      sizeof(llmode));
		if (rc)
			dev_warn(&master->dev,
				"can't set llmode on slave:%02x:%02x %d\n",
				link, id, rc);
	}

	/* We can communicate with a slave; create the slave device and
	 * register.
	 */
	slave = kzalloc(sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return -ENOMEM;

	spin_lock_init(&slave->lock);
	dev_set_name(&slave->dev, "slave@%02x:%02x", link, id);
	slave->dev.type = &cfam_type;
	slave->dev.parent = &master->dev;
	slave->dev.of_node = fsi_slave_find_of_node(master, link, id);
	slave->dev.release = fsi_slave_release;
	device_initialize(&slave->dev);
	slave->clock_div = FSI_SMODE_LBCRR_DEFAULT;
	slave->cfam_id = cfam_id;
	slave->master = master;
	slave->link = link;
	slave->id = id;
	slave->size = FSI_SLAVE_SIZE_23b;
	slave->t_send_delay = FSI_SMODE_SD_DEFAULT;
	slave->t_echo_delay = FSI_SMODE_ED_DEFAULT;

	/* Get chip ID if any */
	slave->chip_id = -1;
	if (slave->dev.of_node) {
		uint32_t prop;
		if (!of_property_read_u32(slave->dev.of_node, "chip-id", &prop))
			slave->chip_id = prop;

	}

	if (master->clock_frequency && !device_property_read_u32(&slave->dev, "clock-frequency",
								 &clock) && clock)
		slave->clock_div = DIV_ROUND_UP(master->clock_frequency, clock);

	slbus = cpu_to_be32(FSI_SLBUS_FORCE);
	rc = fsi_master_write(master, link, id, FSI_SLAVE_BASE + FSI_SLBUS,
			      &slbus, sizeof(slbus));
	if (rc)
		dev_warn(&master->dev,
			 "can't set slbus on slave:%02x:%02x %d\n", link, id,
			 rc);

	rc = fsi_slave_set_smode(slave, break_id);
	if (rc) {
		dev_warn(&master->dev,
				"can't set smode on slave:%02x:%02x %d\n",
				link, id, rc);
		goto err_free;
	}

	/* Allocate a minor in the FSI space */
	rc = __fsi_get_new_minor(slave, slave->dev.of_node, fsi_dev_cfam, &slave->dev.devt,
				 &slave->cdev_idx);
	if (rc)
		goto err_free;

	trace_fsi_slave_init(slave);

	/* Create chardev for userspace access */
	cdev_init(&slave->cdev, &cfam_fops);
	rc = cdev_device_add(&slave->cdev, &slave->dev);
	if (rc) {
		dev_err(&slave->dev, "Error %d creating slave device\n", rc);
		goto err_free_ida;
	}

	/* Now that we have the cdev registered with the core, any fatal
	 * failures beyond this point will need to clean up through
	 * cdev_device_del(). Fortunately though, nothing past here is fatal.
	 */

	if (master->link_config)
		master->link_config(master, link,
				    slave->t_send_delay,
				    slave->t_echo_delay);

	/* Legacy raw file -> to be removed */
	rc = device_create_bin_file(&slave->dev, &fsi_slave_raw_attr);
	if (rc)
		dev_warn(&slave->dev, "failed to create raw attr: %d\n", rc);


	rc = fsi_slave_scan(slave);
	if (rc)
		dev_dbg(&master->dev, "failed during slave scan with: %d\n",
				rc);

	return 0;

err_free_ida:
	fsi_free_minor(slave->dev.devt);
err_free:
	of_node_put(slave->dev.of_node);
	kfree(slave);
	return rc;
}

/* FSI master support */
static int fsi_check_access(uint32_t addr, size_t size)
{
	if (size == 4) {
		if (addr & 0x3)
			return -EINVAL;
	} else if (size == 2) {
		if (addr & 0x1)
			return -EINVAL;
	} else if (size != 1)
		return -EINVAL;

	return 0;
}

static int fsi_master_read(struct fsi_master *master, int link,
		uint8_t slave_id, uint32_t addr, void *val, size_t size)
{
	int rc;

	rc = fsi_check_access(addr, size);
	if (!rc) {
		rc = master->read(master, link, slave_id, addr, val, size);
		if (rc)
			trace_fsi_master_error(master->idx, link, slave_id, addr, size, val, rc,
					       true);
		else
			trace_fsi_master_xfer(master->idx, link, slave_id, addr, size, val, true);
	}

	return rc;
}

static int fsi_master_write(struct fsi_master *master, int link,
		uint8_t slave_id, uint32_t addr, const void *val, size_t size)
{
	int rc;

	rc = fsi_check_access(addr, size);
	if (!rc) {
		rc = master->write(master, link, slave_id, addr, val, size);
		if (rc)
			trace_fsi_master_error(master->idx, link, slave_id, addr, size, val, rc,
					       false);
		else
			trace_fsi_master_xfer(master->idx, link, slave_id, addr, size, val, false);
	}

	return rc;
}

int fsi_master_link_enable(struct fsi_master *master, int link, bool enable)
{
	u32 msiep = 0x80000000 >> (4 * (link % 8));
	u32 menp = 0x80000000 >> (link % 32);
	int enable_idx = 4 * (link / 32);
	int irq_idx = 4 * (link / 8);
	int rc;

	if (enable) {
		rc = regmap_write(master->map, FSI_MSENP0 + enable_idx, menp);
		if (rc)
			return rc;

		mdelay(FSI_LINK_ENABLE_SETUP_TIME);

		rc = regmap_write(master->map, FSI_MSSIEP0 + irq_idx, msiep);
	} else {
		rc = regmap_write(master->map, FSI_MCSIEP0 + irq_idx, msiep);
		if (rc)
			return rc;

		rc = regmap_write(master->map, FSI_MCENP0 + enable_idx, menp);
	}

	return rc;
}
EXPORT_SYMBOL_GPL(fsi_master_link_enable);

static int fsi_master_link_disable(struct fsi_master *master, int link)
{
	if (master->link_enable)
		return master->link_enable(master, link, false);
	else if (master->map)
		return fsi_master_link_enable(master, link, false);

	return 0;
}

static int _fsi_master_link_enable(struct fsi_master *master, int link)
{
	if (master->link_enable)
		return master->link_enable(master, link, true);
	else if (master->map)
		return fsi_master_link_enable(master, link, true);

	return 0;
}

/*
 * Issue a break command on this link
 */
static int fsi_master_break(struct fsi_master *master, int link)
{
	int rc = 0;

	trace_fsi_master_break(master, link);

	if (master->send_break)
		rc = master->send_break(master, link);
	if (master->link_config)
		master->link_config(master, link, FSI_SMODE_SD_DEFAULT, FSI_SMODE_ED_DEFAULT);

	return rc;
}

static int fsi_master_scan(struct fsi_master *master)
{
	bool set_mmode = false;
	int link, rc;

	/* relative addressing is not allowed during slave ID initialization */
	if (master->map && (master->flags & FSI_MASTER_FLAG_RELA)) {
		u32 mmode = master->mmode & ~FSI_MMODE_RELA;

		rc = regmap_write(master->map, FSI_MMODE, mmode);
		if (rc)
			return rc;

		set_mmode = true;
	}

	trace_fsi_master_scan(master, true);
	for (link = 0; link < master->n_links; link++) {
		rc = _fsi_master_link_enable(master, link);
		if (rc) {
			dev_dbg(&master->dev,
				"enable link %d failed: %d\n", link, rc);
			continue;
		}
		rc = fsi_master_break(master, link);
		if (rc) {
			fsi_master_link_disable(master, link);
			dev_dbg(&master->dev,
				"break to link %d failed: %d\n", link, rc);
			continue;
		}

		rc = fsi_slave_init(master, link, 0);
		if (rc)
			fsi_master_link_disable(master, link);
	}

	if (set_mmode) {
		rc = regmap_write(master->map, FSI_MMODE, master->mmode);
		if (rc)
			return rc;
	}

	return 0;
}

static int fsi_slave_remove_device(struct device *dev, void *arg)
{
	device_unregister(dev);
	return 0;
}

static int fsi_master_remove_slave(struct device *dev, void *arg)
{
	struct fsi_slave *slave = to_fsi_slave(dev);

	device_for_each_child(dev, NULL, fsi_slave_remove_device);
	cdev_device_del(&slave->cdev, &slave->dev);
	put_device(dev);
	return 0;
}

static void fsi_master_unscan(struct fsi_master *master)
{
	trace_fsi_master_scan(master, false);
	device_for_each_child(&master->dev, NULL, fsi_master_remove_slave);
}

int fsi_master_rescan(struct fsi_master *master)
{
	int rc;

	mutex_lock(&master->scan_lock);
	fsi_master_unscan(master);
	rc = fsi_master_scan(master);
	mutex_unlock(&master->scan_lock);

	return rc;
}
EXPORT_SYMBOL_GPL(fsi_master_rescan);

static ssize_t master_rescan_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fsi_master *master = to_fsi_master(dev);
	int rc;

	rc = fsi_master_rescan(master);
	if (rc < 0)
		return rc;

	return count;
}

static DEVICE_ATTR(rescan, 0200, NULL, master_rescan_store);

static ssize_t master_break_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fsi_master *master = to_fsi_master(dev);

	fsi_master_break(master, 0);

	return count;
}

static DEVICE_ATTR(break, 0200, NULL, master_break_store);

struct fsi_master_attribute {
	struct device_attribute attr;
	int reg;
};

static ssize_t master_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsi_master_attribute *fattr = container_of(attr, struct fsi_master_attribute, attr);
	struct fsi_master *master = to_fsi_master(dev);
	unsigned int reg;
	int rc;

	rc = regmap_read(master->map, fattr->reg, &reg);
	if (rc)
		return rc;

	return sysfs_emit(buf, "%08x\n", reg);
}

static ssize_t master_reg_1bpp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsi_master_attribute *fattr = container_of(attr, struct fsi_master_attribute, attr);
	struct fsi_master *master = to_fsi_master(dev);
	unsigned int count = (master->n_links + 31) / 32;
	unsigned int reg;
	unsigned int i;
	int len = 0;
	int rc;

	for (i = 0; i < count; ++i) {
		rc = regmap_read(master->map, fattr->reg + (i * 4), &reg);
		if (rc)
			return rc;

		len += sysfs_emit_at(buf, len, "%08x\n", reg);
	}

	return len;
}

static ssize_t master_reg_4bpp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsi_master_attribute *fattr = container_of(attr, struct fsi_master_attribute, attr);
	struct fsi_master *master = to_fsi_master(dev);
	unsigned int count = (master->n_links + 7) / 8;
	unsigned int reg;
	unsigned int i;
	int len = 0;
	int rc;

	for (i = 0; i < count; ++i) {
		rc = regmap_read(master->map, fattr->reg + (i * 4), &reg);
		if (rc)
			return rc;

		len += sysfs_emit_at(buf, len, "%08x\n", reg);
	}

	return len;
}

static ssize_t master_reg_32bpp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fsi_master_attribute *fattr = container_of(attr, struct fsi_master_attribute, attr);
	struct fsi_master *master = to_fsi_master(dev);
	unsigned int reg;
	int len = 0;
	int rc;
	int i;

	for (i = 0; i < master->n_links; ++i) {
		rc = regmap_read(master->map, fattr->reg + (i * 4), &reg);
		if (rc)
			return rc;

		len += sysfs_emit_at(buf, len, "%08x\n", reg);
	}

	return len;
}

#define FSI_MASTER_ATTR(name, reg) struct fsi_master_attribute dev_attr_##name = \
	{ __ATTR(name, 0444, master_reg_show, NULL), reg }
#define FSI_MASTER_ATTR_1BPP(name, reg) struct fsi_master_attribute dev_attr_##name = \
	{ __ATTR(name, 0444, master_reg_1bpp_show, NULL), reg }
#define FSI_MASTER_ATTR_4BPP(name, reg) struct fsi_master_attribute dev_attr_##name = \
	{ __ATTR(name, 0444, master_reg_4bpp_show, NULL), reg }
#define FSI_MASTER_ATTR_32BPP(name, reg) struct fsi_master_attribute dev_attr_##name = \
	{ __ATTR(name, 0444, master_reg_32bpp_show, NULL), reg }

static FSI_MASTER_ATTR(mmode, FSI_MMODE);
static FSI_MASTER_ATTR(mdlyr, FSI_MDLYR);
static FSI_MASTER_ATTR_1BPP(mcrsp, FSI_MCRSP);
static FSI_MASTER_ATTR_1BPP(menp, FSI_MENP0);
static FSI_MASTER_ATTR_1BPP(mlevp, FSI_MLEVP0);
static FSI_MASTER_ATTR_1BPP(mrefp, FSI_MREFP0);
static FSI_MASTER_ATTR_1BPP(mhpmp, FSI_MHPMP0);
static FSI_MASTER_ATTR_4BPP(msiep, FSI_MSIEP0);
static FSI_MASTER_ATTR_1BPP(maesp, FSI_MAESP0);
static FSI_MASTER_ATTR(maeb, FSI_MAEB);
static FSI_MASTER_ATTR(mver, FSI_MVER);
static FSI_MASTER_ATTR_1BPP(mbsyp, FSI_MBSYP0);
static FSI_MASTER_ATTR_32BPP(mstap, FSI_MSTAP0);
static FSI_MASTER_ATTR(mesrb, FSI_MESRB0);
static FSI_MASTER_ATTR(mscsb, FSI_MSCSB0);
static FSI_MASTER_ATTR(matrb, FSI_MATRB0);
static FSI_MASTER_ATTR(mdtrb, FSI_MDTRB0);
static FSI_MASTER_ATTR(mectrl, FSI_MECTRL);

static struct attribute *master_mapped_attrs[] = {
	&dev_attr_mmode.attr.attr,
	&dev_attr_mdlyr.attr.attr,
	&dev_attr_mcrsp.attr.attr,
	&dev_attr_menp.attr.attr,
	&dev_attr_mlevp.attr.attr,
	&dev_attr_mrefp.attr.attr,
	&dev_attr_mhpmp.attr.attr,
	&dev_attr_msiep.attr.attr,
	&dev_attr_maesp.attr.attr,
	&dev_attr_maeb.attr.attr,
	&dev_attr_mver.attr.attr,
	&dev_attr_mbsyp.attr.attr,
	&dev_attr_mstap.attr.attr,
	&dev_attr_mesrb.attr.attr,
	&dev_attr_mscsb.attr.attr,
	&dev_attr_matrb.attr.attr,
	&dev_attr_mdtrb.attr.attr,
	&dev_attr_mectrl.attr.attr,
	NULL
};

static const struct attribute_group master_mapped_group = {
	.attrs = master_mapped_attrs,
};

static struct attribute *master_attrs[] = {
	&dev_attr_break.attr,
	&dev_attr_rescan.attr,
	NULL
};

ATTRIBUTE_GROUPS(master);

static struct class fsi_master_class = {
	.name = "fsi-master",
	.dev_groups = master_groups,
};

void fsi_master_error(struct fsi_master *master, int link)
{
	u32 bits = FSI_MMODE_EIP | FSI_MMODE_RELA;
	bool mmode = master->mmode & bits;

	if (trace_fsi_master_error_regs_enabled()) {
		unsigned int mesrb = 0xffffffff;
		unsigned int mstap = 0xffffffff;

		regmap_read(master->map, FSI_MESRB0, &mesrb);
		regmap_read(master->map, FSI_MSTAP0 + (link * 4), &mstap);

		trace_fsi_master_error_regs(master->idx, mesrb, mstap);
	}

	if (mmode)
		regmap_write(master->map, FSI_MMODE, master->mmode & ~bits);

	regmap_write(master->map, FSI_MRESP0, FSI_MRESP_RST_ALL_MASTER);

	if (mmode)
		regmap_write(master->map, FSI_MMODE, master->mmode);
}
EXPORT_SYMBOL_GPL(fsi_master_error);

static inline u32 fsi_mmode_crs0(u32 x)
{
	return (x & FSI_MMODE_CRS0MASK) << FSI_MMODE_CRS0SHFT;
}

static inline u32 fsi_mmode_crs1(u32 x)
{
	return (x & FSI_MMODE_CRS1MASK) << FSI_MMODE_CRS1SHFT;
}

int fsi_master_init(struct fsi_master *master, unsigned long parent_clock_frequency)
{
	unsigned int mlevp;
	unsigned int maeb;
	int div = 1;
	int rc;

	if (parent_clock_frequency) {
		u32 clock_frequency;

		if (device_property_read_u32(&master->dev, "bus-frequency", &clock_frequency) ||
		    !clock_frequency)
			clock_frequency = parent_clock_frequency;

		div = DIV_ROUND_UP(parent_clock_frequency, clock_frequency);
		master->clock_frequency = parent_clock_frequency / div;
	}

	rc = regmap_write(master->map, FSI_MRESP0, FSI_MRESP_RST_ALL_MASTER |
			  FSI_MRESP_RST_ALL_LINK | FSI_MRESP_RST_MCR | FSI_MRESP_RST_PYE);
	if (rc)
		return rc;

	rc = regmap_write(master->map, FSI_MECTRL, FSI_MECTRL_EOAE | FSI_MECTRL_P8_AUTO_TERM);
	if (rc)
		return rc;

	master->mmode = FSI_MMODE_ECRC | FSI_MMODE_EPC | fsi_mmode_crs0(div) |
		fsi_mmode_crs1(div) | FSI_MMODE_P8_TO_LSB;
	rc = regmap_write(master->map, FSI_MMODE, master->mmode);
	if (rc)
		return rc;

	rc = regmap_write(master->map, FSI_MDLYR, 0xffff0000);
	if (rc)
		return rc;

	rc = regmap_write(master->map, FSI_MSENP0, 0xffffffff);
	if (rc)
		return rc;

	mdelay(FSI_LINK_ENABLE_SETUP_TIME);

	rc = regmap_write(master->map, FSI_MCENP0, 0xffffffff);
	if (rc)
		return rc;

	rc = regmap_read(master->map, FSI_MAEB, &maeb);
	if (rc)
		return rc;

	rc = regmap_write(master->map, FSI_MRESP0, FSI_MRESP_RST_ALL_MASTER |
			  FSI_MRESP_RST_ALL_LINK);
	if (rc)
		return rc;

	rc = regmap_read(master->map, FSI_MLEVP0, &mlevp);
	if (rc)
		return rc;

	rc = regmap_write(master->map, FSI_MRESB0, FSI_MRESB_RST_GEN);
	if (rc)
		return rc;

	rc = regmap_write(master->map, FSI_MRESB0, FSI_MRESB_RST_ERR);
	if (rc)
		return rc;

	if (master->flags & FSI_MASTER_FLAG_INTERRUPT)
		master->mmode |= FSI_MMODE_EIP;
	if (master->flags & FSI_MASTER_FLAG_RELA)
		master->mmode |= FSI_MMODE_RELA;
	return regmap_write(master->map, FSI_MMODE, master->mmode);
}
EXPORT_SYMBOL_GPL(fsi_master_init);

void fsi_master_regmap_config(struct regmap_config *config)
{
	memset(config, 0, sizeof(*config));

	config->reg_bits = 32;
	config->val_bits = 32;
	config->disable_locking = true;	// master driver will lock
	config->fast_io = true;
	config->cache_type = REGCACHE_NONE;
	config->val_format_endian = REGMAP_ENDIAN_NATIVE;
	config->can_sleep = false;
}
EXPORT_SYMBOL_GPL(fsi_master_regmap_config);

int fsi_master_register(struct fsi_master *master)
{
	int rc;
	struct device_node *np;

	mutex_init(&master->scan_lock);

	/* Alloc the requested index if it's non-zero */
	if (master->idx) {
		master->idx = ida_alloc_range(&master_ida, master->idx,
					      master->idx, GFP_KERNEL);
	} else {
		master->idx = ida_alloc(&master_ida, GFP_KERNEL);
	}

	if (master->idx < 0)
		return master->idx;

	if (!dev_name(&master->dev))
		dev_set_name(&master->dev, "fsi%d", master->idx);

	if (master->flags & FSI_MASTER_FLAG_SWCLOCK)
		master->clock_frequency = 100000000; // POWER reference clock

	master->dev.class = &fsi_master_class;

	mutex_lock(&master->scan_lock);
	rc = device_register(&master->dev);
	if (rc) {
		ida_free(&master_ida, master->idx);
		goto out;
	}

	np = dev_of_node(&master->dev);
	if (!of_property_read_bool(np, "no-scan-on-init")) {
		fsi_master_scan(master);
	}
out:
	mutex_unlock(&master->scan_lock);

	if (!rc && master->map) {
		if (!sysfs_create_group(&master->dev.kobj, &master_mapped_group))
			master->groups = true;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(fsi_master_register);

void fsi_master_unregister(struct fsi_master *master)
{
	int idx = master->idx;

	trace_fsi_master_unregister(master);

	if (master->groups)
		sysfs_remove_group(&master->dev.kobj, &master_mapped_group);

	mutex_lock(&master->scan_lock);
	fsi_master_unscan(master);
	master->n_links = 0;
	mutex_unlock(&master->scan_lock);

	device_unregister(&master->dev);
	ida_free(&master_ida, idx);
}
EXPORT_SYMBOL_GPL(fsi_master_unregister);

/* FSI core & Linux bus type definitions */

static int fsi_bus_match(struct device *dev, const struct device_driver *drv)
{
	struct fsi_device *fsi_dev = to_fsi_dev(dev);
	const struct fsi_driver *fsi_drv = to_fsi_drv(drv);
	const struct fsi_device_id *id;

	if (!fsi_drv->id_table)
		return 0;

	for (id = fsi_drv->id_table; id->engine_type; id++) {
		if (id->engine_type != fsi_dev->engine_type)
			continue;
		if (id->version == FSI_VERSION_ANY ||
		    id->version == fsi_dev->version) {
			if (drv->of_match_table) {
				if (of_driver_match_device(dev, drv))
					return 1;
			} else {
				return 1;
			}
		}
	}

	return 0;
}

int fsi_driver_register(struct fsi_driver *fsi_drv)
{
	if (!fsi_drv)
		return -EINVAL;
	if (!fsi_drv->id_table)
		return -EINVAL;

	return driver_register(&fsi_drv->drv);
}
EXPORT_SYMBOL_GPL(fsi_driver_register);

void fsi_driver_unregister(struct fsi_driver *fsi_drv)
{
	driver_unregister(&fsi_drv->drv);
}
EXPORT_SYMBOL_GPL(fsi_driver_unregister);

struct bus_type fsi_bus_type = {
	.name		= "fsi",
	.match		= fsi_bus_match,
};
EXPORT_SYMBOL_GPL(fsi_bus_type);

static int __init fsi_init(void)
{
	int rc;

	rc = alloc_chrdev_region(&fsi_base_dev, 0, FSI_CHAR_MAX_DEVICES, "fsi");
	if (rc)
		return rc;
	rc = bus_register(&fsi_bus_type);
	if (rc)
		goto fail_bus;

	rc = class_register(&fsi_master_class);
	if (rc)
		goto fail_class;

	return 0;

 fail_class:
	bus_unregister(&fsi_bus_type);
 fail_bus:
	unregister_chrdev_region(fsi_base_dev, FSI_CHAR_MAX_DEVICES);
	return rc;
}
postcore_initcall(fsi_init);

static void fsi_exit(void)
{
	class_unregister(&fsi_master_class);
	bus_unregister(&fsi_bus_type);
	unregister_chrdev_region(fsi_base_dev, FSI_CHAR_MAX_DEVICES);
	ida_destroy(&fsi_minor_ida);
}
module_exit(fsi_exit);
module_param(discard_errors, int, 0664);
MODULE_DESCRIPTION("FSI core driver");
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(discard_errors, "Don't invoke error handling on bus accesses");
