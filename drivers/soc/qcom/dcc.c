/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <soc/qcom/memory_dump.h>

#define TIMEOUT_US		(100)

#define BM(lsb, msb)		((BIT(msb) - BIT(lsb)) + BIT(msb))
#define BMVAL(val, lsb, msb)	((val & BM(lsb, msb)) >> lsb)
#define BVAL(val, n)		((val & BIT(n)) >> n)

#define dcc_writel(drvdata, val, off)					\
	__raw_writel((val), drvdata->base + off)
#define dcc_readl(drvdata, off)						\
	__raw_readl(drvdata->base + off)

#define dcc_sram_writel(drvdata, val, off)				\
	__raw_writel((val), drvdata->ram_base + off)
#define dcc_sram_readl(drvdata, off)					\
	__raw_readl(drvdata->ram_base + off)

/* DCC registers */
#define DCC_HW_VERSION		(0x00)
#define DCC_HW_INFO		(0x04)
#define DCC_CGC_CFG		(0x10)
#define DCC_LL			(0x14)
#define DCC_RAM_CFG		(0x18)
#define DCC_CFG			(0x1C)
#define DCC_SW_CTL		(0x20)
#define DCC_STATUS		(0x24)
#define DCC_FETCH_ADDR		(0x28)
#define DCC_SRAM_ADDR		(0x2C)
#define DCC_INT_ENABLE		(0x30)
#define DCC_INT_STATUS		(0x34)
#define DCC_QSB_CFG		(0x38)

#define DCC_REG_DUMP_MAGIC_V2		(0x42445953)
#define DCC_REG_DUMP_VER		(1)

enum dcc_func_type {
	DCC_FUNC_TYPE_CAPTURE,
	DCC_FUNC_TYPE_CRC,
};

static const char * const str_dcc_func_type[] = {
	[DCC_FUNC_TYPE_CAPTURE]		= "cap",
	[DCC_FUNC_TYPE_CRC]		= "crc",
};

enum dcc_data_sink {
	DCC_DATA_SINK_ATB,
	DCC_DATA_SINK_SRAM
};

static const char * const str_dcc_data_sink[] = {
	[DCC_DATA_SINK_ATB]		= "atb",
	[DCC_DATA_SINK_SRAM]		= "sram",
};

struct dcc_config_entry {
	uint32_t		base;
	uint32_t		offset;
	uint32_t		len;
	uint32_t		index;
	struct list_head	list;
};

struct dcc_drvdata {
	void __iomem		*base;
	uint32_t		reg_size;
	struct device		*dev;
	struct mutex		mutex;
	void __iomem		*ram_base;
	uint32_t		ram_size;
	struct clk		*clk;
	enum dcc_data_sink	data_sink;
	enum dcc_func_type	func_type;
	uint32_t		ram_cfg;
	bool			enable;
	char			*sram_node;
	struct cdev		sram_dev;
	struct class		*sram_class;
	struct list_head	config_head;
	uint32_t		nr_config;
	void			*reg_buf;
	struct msm_dump_data	reg_data;
	bool			save_reg;
	void			*sram_buf;
	struct msm_dump_data	sram_data;
};

static bool dcc_ready(struct dcc_drvdata *drvdata)
{
	uint32_t val;

	/* poll until DCC ready */
	if (!readl_poll_timeout((drvdata->base + DCC_STATUS), val,
				(BVAL(val, 4) == 1), 1, TIMEOUT_US))
		return true;

	return false;
}

static int dcc_sw_trigger(struct dcc_drvdata *drvdata)
{
	int ret;

	ret = 0;
	mutex_lock(&drvdata->mutex);

	if (!drvdata->enable) {
		dev_err(drvdata->dev,
			"DCC is disabled. Can't send sw trigger.\n");
		ret = -EINVAL;
		goto err;
	}

	if (!dcc_ready(drvdata)) {
		dev_err(drvdata->dev, "DCC is not ready!\n");
		ret = -EBUSY;
		goto err;
	}

	dcc_writel(drvdata, 1, DCC_SW_CTL);

	if (!dcc_ready(drvdata)) {
		dev_err(drvdata->dev,
			"DCC is busy after receiving sw tigger.\n");
		ret = -EBUSY;
		goto err;
	}
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static int __dcc_ll_cfg(struct dcc_drvdata *drvdata)
{
	int ret = 0;
	uint32_t sram_offset = 0;
	uint32_t prev_addr, addr;
	uint32_t prev_off, off;
	uint32_t link;
	uint32_t pos;
	struct dcc_config_entry *entry;

	if (list_empty(&drvdata->config_head)) {
		dev_err(drvdata->dev,
			"No configuration is available to program in DCC SRAM!\n");
		return -EINVAL;
	}

	memset_io(drvdata->ram_base, 0, drvdata->ram_size);

	prev_addr = 0;
	link = 0;

	list_for_each_entry(entry, &drvdata->config_head, list) {
		/* Address type */
		addr = (entry->base >> 4) & BM(0, 27);
		addr |= BIT(31);
		off = entry->offset/4;

		if (!prev_addr || prev_addr != addr || prev_off > off) {
			/* Check if we need to write link of prev entry */
			if (link) {
				dcc_sram_writel(drvdata, link, sram_offset);
				sram_offset += 4;
			}

			/* Write address */
			dcc_sram_writel(drvdata, addr, sram_offset);
			sram_offset += 4;

			/* Reset link and prev_off */
			link = 0;
			prev_off = 0;
		}

		if ((off - prev_off) > 0xFF || entry->len > 0x7F) {
			dev_err(drvdata->dev,
				"DCC: Progamming error! Base: 0x%x, offset 0x%x.\n",
				entry->base, entry->offset);
			ret = -EINVAL;
			goto err;
		}

		if (link) {
			/*
			 * link already has one offset-length so new
			 * offset-length needs to be placed at bits [31:16]
			 */
			pos = 16;

			/* Clear bits [31:16] */
			link &= BM(0, 15);

		} else {
			/*
			 * link is empty, so new offset-length needs to be
			 * placed at bits [15:0]
			 */
			pos = 0;
			link = 1 << 16;
		}

		/* write new offset-length pair to correct position */
		link |= (((off-prev_off) & BM(0, 7)) |
			 ((entry->len << 8) & BM(8, 14))) << pos;

		if (pos) {
			dcc_sram_writel(drvdata, link, sram_offset);
			sram_offset += 4;
			link = 0;
		}

		prev_off  = off;
		prev_addr = addr;
	}

	if (link) {
		dcc_sram_writel(drvdata, link, sram_offset);
		sram_offset += 4;
	}

	/* Setting zero to indicate end of the list */
	dcc_sram_writel(drvdata, 0, sram_offset);
	sram_offset += 4;

	drvdata->ram_cfg = (sram_offset  / 4);

err:
	return ret;
}

static void __dcc_reg_dump(struct dcc_drvdata *drvdata)
{
	uint32_t *reg_buf;

	if (!drvdata->reg_buf)
		return;

	drvdata->reg_data.version = DCC_REG_DUMP_VER;

	reg_buf = drvdata->reg_buf;

	reg_buf[0] = dcc_readl(drvdata, DCC_HW_VERSION);
	reg_buf[1] = dcc_readl(drvdata, DCC_HW_INFO);
	reg_buf[2] = dcc_readl(drvdata, DCC_CGC_CFG);
	reg_buf[3] = dcc_readl(drvdata, DCC_LL);
	reg_buf[4] = dcc_readl(drvdata, DCC_RAM_CFG);
	reg_buf[5] = dcc_readl(drvdata, DCC_CFG);
	reg_buf[6] = dcc_readl(drvdata, DCC_SW_CTL);
	reg_buf[7] = dcc_readl(drvdata, DCC_STATUS);
	reg_buf[8] = dcc_readl(drvdata, DCC_FETCH_ADDR);
	reg_buf[9] = dcc_readl(drvdata, DCC_SRAM_ADDR);
	reg_buf[10] = dcc_readl(drvdata, DCC_INT_ENABLE);
	reg_buf[11] = dcc_readl(drvdata, DCC_INT_STATUS);
	reg_buf[12] = dcc_readl(drvdata, DCC_QSB_CFG);

	drvdata->reg_data.magic = DCC_REG_DUMP_MAGIC_V2;
}

static int dcc_enable(struct dcc_drvdata *drvdata)
{
	int ret = 0;

	mutex_lock(&drvdata->mutex);

	if (drvdata->enable) {
		dev_err(drvdata->dev, "DCC is already enabled!\n");
		mutex_unlock(&drvdata->mutex);
		return 0;
	}

	/* 1. Prepare and enable DCC clock */
	ret = clk_prepare_enable(drvdata->clk);
	if (ret)
		goto err;

	dcc_writel(drvdata, 0, DCC_LL);

	/* 2. Program linked-list in the SRAM */
	ret = __dcc_ll_cfg(drvdata);
	if (ret)
		goto err_prog_ll;

	/* 3. If in capture mode program DCC_RAM_CFG reg */
	if (drvdata->func_type == DCC_FUNC_TYPE_CAPTURE)
		dcc_writel(drvdata, drvdata->ram_cfg, DCC_RAM_CFG);

	/* 4. Configure data sink and function type */
	dcc_writel(drvdata, ((drvdata->data_sink << 4) | (drvdata->func_type)),
		   DCC_CFG);

	/* 5. Clears interrupt status register */
	dcc_writel(drvdata, 0, DCC_INT_STATUS);

	/* Make sure all config is written in sram */
	mb();

	/* 6. Set LL bit */
	dcc_writel(drvdata, 1, DCC_LL);
	drvdata->enable = 1;

	/* Save DCC registers */
	if (drvdata->save_reg)
		__dcc_reg_dump(drvdata);

err_prog_ll:
	if (!drvdata->enable)
		clk_disable_unprepare(drvdata->clk);
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static void dcc_disable(struct dcc_drvdata *drvdata)
{
	mutex_lock(&drvdata->mutex);
	if (!drvdata->enable) {
		mutex_unlock(&drvdata->mutex);
		return;
	}

	if (!dcc_ready(drvdata))
		dev_err(drvdata->dev, "DCC is not ready! Disabling DCC...\n");

	dcc_writel(drvdata, 0, DCC_LL);
	drvdata->enable = 0;

	/* Save DCC registers */
	if (drvdata->save_reg)
		__dcc_reg_dump(drvdata);

	clk_disable_unprepare(drvdata->clk);

	mutex_unlock(&drvdata->mutex);
}

static ssize_t dcc_show_func_type(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 str_dcc_func_type[drvdata->func_type]);
}

static ssize_t dcc_store_func_type(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	char str[10] = "";
	int ret;

	if (strlen(buf) >= 10)
		return -EINVAL;
	if (sscanf(buf, "%s", str) != 1)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	if (drvdata->enable) {
		ret = -EBUSY;
		goto out;
	}

	if (!strcmp(str, str_dcc_func_type[DCC_FUNC_TYPE_CAPTURE]))
		drvdata->func_type = DCC_FUNC_TYPE_CAPTURE;
	else if (!strcmp(str, str_dcc_func_type[DCC_FUNC_TYPE_CRC]))
		drvdata->func_type = DCC_FUNC_TYPE_CRC;
	else {
		ret = -EINVAL;
		goto out;
	}

	ret = size;
out:
	mutex_unlock(&drvdata->mutex);
	return ret;
}
static DEVICE_ATTR(func_type, S_IRUGO | S_IWUSR,
		   dcc_show_func_type, dcc_store_func_type);

static ssize_t dcc_show_data_sink(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 str_dcc_data_sink[drvdata->data_sink]);
}

static ssize_t dcc_store_data_sink(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	char str[10] = "";
	int ret;

	if (strlen(buf) >= 10)
		return -EINVAL;
	if (sscanf(buf, "%s", str) != 1)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);
	if (drvdata->enable) {
		ret = -EBUSY;
		goto out;
	}

	if (!strcmp(str, str_dcc_data_sink[DCC_DATA_SINK_SRAM]))
		drvdata->data_sink = DCC_DATA_SINK_SRAM;
	else if (!strcmp(str, str_dcc_data_sink[DCC_DATA_SINK_ATB]))
		drvdata->data_sink = DCC_DATA_SINK_ATB;
	else {
		ret = -EINVAL;
		goto out;
	}

	ret = size;
out:
	mutex_unlock(&drvdata->mutex);
	return ret;
}
static DEVICE_ATTR(data_sink, S_IRUGO | S_IWUSR,
		   dcc_show_data_sink, dcc_store_data_sink);

static ssize_t dcc_store_trigger(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int ret = 0;
	unsigned long val;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;
	if (val != 1)
		return -EINVAL;

	ret = dcc_sw_trigger(drvdata);
	if (ret)
		return ret;

	return size;
}
static DEVICE_ATTR(trigger, S_IWUSR, NULL, dcc_store_trigger);

static ssize_t dcc_show_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 (unsigned)drvdata->enable);
}

static ssize_t dcc_store_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret = 0;
	unsigned long val;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;

	if (val)
		ret = dcc_enable(drvdata);
	else
		dcc_disable(drvdata);

	if (ret)
		return ret;
	return size;

}
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, dcc_show_enable,
		   dcc_store_enable);

static ssize_t dcc_show_config(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);
	struct dcc_config_entry *entry;
	char local_buf[64];
	int len = 0, count = 0;

	buf[0] = '\0';

	mutex_lock(&drvdata->mutex);
	list_for_each_entry(entry, &drvdata->config_head, list) {
		len = snprintf(local_buf, 64,
			       "Index: 0x%x, Base: 0x%x, Offset: 0x%x, len: 0x%x\n",
			       entry->index, entry->base,
			       entry->offset, entry->len);

		if ((count + len) > PAGE_SIZE) {
			dev_err(dev, "DCC: Couldn't write complete config!\n");
			break;
		}

		strlcat(buf, local_buf, PAGE_SIZE);
		count += len;
	}

	mutex_unlock(&drvdata->mutex);

	return count;
}

static int dcc_config_add(struct dcc_drvdata *drvdata, unsigned base,
			  unsigned offset, unsigned len)
{
	int ret;
	struct dcc_config_entry *entry;

	mutex_lock(&drvdata->mutex);

	/* Validate data */
	if (!len || len > BM(0, 6)) {
		dev_err(drvdata->dev,
			"DCC: Invalid length! Expected range [1, %u] in words.\n",
			(unsigned int)BM(0, 6));
		ret = -EINVAL;
		goto err;
	}

	if (base & BM(0, 3)) {
		dev_err(drvdata->dev,
			"DCC: Invalid base! Expected [31:4] bits of actual address.\n");
		ret = -EINVAL;
		goto err;
	}

	entry = devm_kzalloc(drvdata->dev, sizeof(*entry), GFP_KERNEL);
	if (!entry) {
		ret = -ENOMEM;
		goto err;
	}

	entry->base = base;
	entry->offset = offset;
	entry->len = len;
	entry->index = drvdata->nr_config++;
	INIT_LIST_HEAD(&entry->list);
	list_add_tail(&entry->list, &drvdata->config_head);

	mutex_unlock(&drvdata->mutex);
	return 0;
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t dcc_store_config(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret;
	unsigned base, offset, len;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	if (sscanf(buf, "%x %x %u", &base, &offset, &len) != 3)
		return -EINVAL;

	ret = dcc_config_add(drvdata, base, offset, len);
	if (ret)
		return ret;

	return size;

}
static DEVICE_ATTR(config, S_IRUGO | S_IWUSR, dcc_show_config,
		   dcc_store_config);

static void dcc_config_reset(struct dcc_drvdata *drvdata)
{
	struct dcc_config_entry *entry, *temp;

	mutex_lock(&drvdata->mutex);

	list_for_each_entry_safe(entry, temp, &drvdata->config_head, list) {
		list_del(&entry->list);
		devm_kfree(drvdata->dev, entry);
		drvdata->nr_config--;
	}

	mutex_unlock(&drvdata->mutex);
}

static ssize_t dcc_store_config_reset(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	unsigned long val;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;

	if (val)
		dcc_config_reset(drvdata);

	return size;
}
static DEVICE_ATTR(config_reset, S_IWUSR, NULL, dcc_store_config_reset);

static ssize_t dcc_show_crc_error(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	mutex_lock(&drvdata->mutex);
	if (!drvdata->enable) {
		ret = -EINVAL;
		goto err;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%u\n",
			(unsigned)BVAL(dcc_readl(drvdata, DCC_STATUS), 0));
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}
static DEVICE_ATTR(crc_error, S_IRUGO, dcc_show_crc_error, NULL);

static ssize_t dcc_show_ready(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int ret;
	struct dcc_drvdata *drvdata = dev_get_drvdata(dev);

	mutex_lock(&drvdata->mutex);
	if (!drvdata->enable) {
		ret = -EINVAL;
		goto err;
	}

	ret = scnprintf(buf, PAGE_SIZE, "%u\n",
			(unsigned)BVAL(dcc_readl(drvdata, DCC_STATUS), 4));
err:
	mutex_unlock(&drvdata->mutex);
	return ret;
}
static DEVICE_ATTR(ready, S_IRUGO, dcc_show_ready, NULL);

static const struct device_attribute *dcc_attrs[] = {
	&dev_attr_func_type,
	&dev_attr_data_sink,
	&dev_attr_trigger,
	&dev_attr_enable,
	&dev_attr_config,
	&dev_attr_config_reset,
	&dev_attr_ready,
	&dev_attr_crc_error,
	NULL,
};

static int dcc_create_files(struct device *dev,
			    const struct device_attribute **attrs)
{
	int ret = 0, i;

	for (i = 0; attrs[i] != NULL; i++) {
		ret = device_create_file(dev, attrs[i]);
		if (ret) {
			dev_err(dev, "DCC: Couldn't create sysfs attribute: %s!\n",
				attrs[i]->attr.name);
			break;
		}
	}
	return ret;
}

static int dcc_sram_open(struct inode *inode, struct file *file)
{
	struct dcc_drvdata *drvdata = container_of(inode->i_cdev,
						   struct dcc_drvdata,
						   sram_dev);
	file->private_data = drvdata;
	return 0;
}

static ssize_t dcc_sram_read(struct file *file, char __user *data,
			     size_t len, loff_t *ppos)
{
	int ret;
	unsigned char *buf;
	struct dcc_drvdata *drvdata = file->private_data;

	/* EOF check */
	if (drvdata->ram_size <= *ppos)
		return 0;

	if ((*ppos + len) > drvdata->ram_size)
		len = (drvdata->ram_size - *ppos);

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf) {
		dev_err(drvdata->dev,
			"DCC: Couldn't allocate memory for sram read!\n");
		return -ENOMEM;
	}

	ret = clk_prepare_enable(drvdata->clk);
	if (ret) {
		kfree(buf);
		return ret;
	}

	memcpy_fromio(buf, (drvdata->ram_base + *ppos), len);

	clk_disable_unprepare(drvdata->clk);

	if (copy_to_user(data, buf, len)) {
		dev_err(drvdata->dev,
			"DCC: Couldn't copy all data to user!\n");
		kfree(buf);
		return -EFAULT;
	}

	*ppos += len;

	kfree(buf);

	return len;
}

static int dcc_sram_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations dcc_sram_fops = {
	.owner		= THIS_MODULE,
	.open		= dcc_sram_open,
	.read		= dcc_sram_read,
	.release	= dcc_sram_release,
	.llseek		= no_llseek,
};

static int dcc_sram_dev_register(struct dcc_drvdata *drvdata)
{
	int ret;
	struct device *device;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, 1, drvdata->sram_node);
	if (ret)
		goto err_alloc;

	cdev_init(&drvdata->sram_dev, &dcc_sram_fops);

	drvdata->sram_dev.owner = THIS_MODULE;
	ret = cdev_add(&drvdata->sram_dev, dev, 1);
	if (ret)
		goto err_cdev_add;

	drvdata->sram_class = class_create(THIS_MODULE,
					   drvdata->sram_node);
	if (IS_ERR(drvdata->sram_class)) {
		ret = PTR_ERR(drvdata->sram_class);
		goto err_class_create;
	}

	device = device_create(drvdata->sram_class, NULL,
			       drvdata->sram_dev.dev, drvdata,
			       drvdata->sram_node);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		goto err_dev_create;
	}

	return 0;
err_dev_create:
	class_destroy(drvdata->sram_class);
err_class_create:
	cdev_del(&drvdata->sram_dev);
err_cdev_add:
	unregister_chrdev_region(drvdata->sram_dev.dev, 1);
err_alloc:
	return ret;
}

static void dcc_sram_dev_deregister(struct dcc_drvdata *drvdata)
{
	device_destroy(drvdata->sram_class, drvdata->sram_dev.dev);
	class_destroy(drvdata->sram_class);
	cdev_del(&drvdata->sram_dev);
	unregister_chrdev_region(drvdata->sram_dev.dev, 1);
}

static int dcc_sram_dev_init(struct dcc_drvdata *drvdata)
{
	int ret = 0;
	size_t node_size;
	char *node_name = "dcc_sram";
	struct device *dev = drvdata->dev;
	node_size = strlen(node_name) + 1;

	drvdata->sram_node = devm_kzalloc(dev, node_size, GFP_KERNEL);
	if (!drvdata->sram_node) {
		dev_err(drvdata->dev, "DCC: sram node name allocation failed.\n");
		return -ENOMEM;
	}

	strlcpy(drvdata->sram_node, node_name, node_size);
	ret = dcc_sram_dev_register(drvdata);
	if (ret)
		dev_err(drvdata->dev, "DCC: sram node not registered.\n");

	return ret;
}

static void dcc_sram_dev_exit(struct dcc_drvdata *drvdata)
{
	dcc_sram_dev_deregister(drvdata);
}

static void dcc_allocate_dump_mem(struct dcc_drvdata *drvdata)
{
	int ret;
	struct device *dev = drvdata->dev;
	struct msm_dump_entry reg_dump_entry, sram_dump_entry;

	/* Allocate memory for dcc reg dump */
	drvdata->reg_buf = devm_kzalloc(dev, drvdata->reg_size, GFP_KERNEL);
	if (drvdata->reg_buf) {
		drvdata->reg_data.addr = virt_to_phys(drvdata->reg_buf);
		drvdata->reg_data.len = drvdata->reg_size;
		reg_dump_entry.id = MSM_DUMP_DATA_DCC_REG;
		reg_dump_entry.addr = virt_to_phys(&drvdata->reg_data);
		ret = msm_dump_data_register(MSM_DUMP_TABLE_APPS,
					     &reg_dump_entry);
		if (ret) {
			dev_err(dev, "DCC REG dump setup failed\n");
			devm_kfree(dev, drvdata->reg_buf);
		}
	} else {
		dev_err(dev, "DCC REG dump allocation failed\n");
	}

	/* Allocate memory for dcc sram dump */
	drvdata->sram_buf = devm_kzalloc(dev, drvdata->ram_size, GFP_KERNEL);
	if (drvdata->sram_buf) {
		drvdata->sram_data.addr = virt_to_phys(drvdata->sram_buf);
		drvdata->sram_data.len = drvdata->ram_size;
		sram_dump_entry.id = MSM_DUMP_DATA_DCC_SRAM;
		sram_dump_entry.addr = virt_to_phys(&drvdata->sram_data);
		ret = msm_dump_data_register(MSM_DUMP_TABLE_APPS,
					     &sram_dump_entry);
		if (ret) {
			dev_err(dev, "DCC SRAM dump setup failed\n");
			devm_kfree(dev, drvdata->sram_buf);
		}
	} else {
		dev_err(dev, "DCC SRAM dump allocation failed\n");
	}
}

static int dcc_probe(struct platform_device *pdev)
{
	int ret, i;
	struct device *dev = &pdev->dev;
	struct dcc_drvdata *drvdata;
	struct resource *res;
	const char *data_sink;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dcc-base");
	if (!res)
		return -ENODEV;

	drvdata->reg_size = resource_size(res);
	drvdata->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!drvdata->base)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "dcc-ram-base");
	if (!res)
		return -ENODEV;

	drvdata->ram_size = resource_size(res);
	drvdata->ram_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!drvdata->ram_base)
		return -ENOMEM;

	drvdata->clk = devm_clk_get(dev, "dcc_clk");
	if (IS_ERR(drvdata->clk)) {
		ret = PTR_ERR(drvdata->clk);
		goto err;
	}

	drvdata->save_reg = of_property_read_bool(pdev->dev.of_node,
						  "qcom,save-reg");

	mutex_init(&drvdata->mutex);

	INIT_LIST_HEAD(&drvdata->config_head);
	drvdata->nr_config = 0;

	ret = clk_prepare_enable(drvdata->clk);
	if (ret)
		goto err;

	memset_io(drvdata->ram_base, 0, drvdata->ram_size);

	clk_disable_unprepare(drvdata->clk);

	drvdata->data_sink = DCC_DATA_SINK_SRAM;
	ret = of_property_read_string(pdev->dev.of_node, "qcom,data-sink",
				      &data_sink);
	if (!ret) {
		for (i = 0; i < ARRAY_SIZE(str_dcc_data_sink); i++)
			if (!strcmp(data_sink, str_dcc_data_sink[i])) {
				drvdata->data_sink = i;
				break;
			}

		if (i == ARRAY_SIZE(str_dcc_data_sink)) {
			dev_err(dev, "Unknown sink type for DCC! Using '%s' as data sink\n",
				str_dcc_data_sink[drvdata->data_sink]);
		}
	}

	ret = dcc_sram_dev_init(drvdata);
	if (ret)
		goto err;

	ret = dcc_create_files(dev, dcc_attrs);
	if (ret)
		goto err;

	dcc_allocate_dump_mem(drvdata);

	return 0;
err:
	return ret;
}

static int dcc_remove(struct platform_device *pdev)
{
	struct dcc_drvdata *drvdata = platform_get_drvdata(pdev);

	dcc_sram_dev_exit(drvdata);

	dcc_config_reset(drvdata);

	return 0;
}

static const struct of_device_id msm_dcc_match[] = {
	{ .compatible = "qcom,dcc"},
	{}
};

static struct platform_driver dcc_driver = {
	.probe          = dcc_probe,
	.remove         = dcc_remove,
	.driver         = {
		.name   = "msm-dcc",
		.owner	= THIS_MODULE,
		.of_match_table	= msm_dcc_match,
	},
};

static int __init dcc_init(void)
{
	return platform_driver_register(&dcc_driver);
}
module_init(dcc_init);

static void __exit dcc_exit(void)
{
	platform_driver_unregister(&dcc_driver);
}
module_exit(dcc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM data capture and compare engine");
