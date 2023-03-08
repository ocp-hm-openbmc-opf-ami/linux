// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Intel Corporation
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define ROT_P_START 0x00000000
#define ROT_P_END 0x0000FFFF
#define ROT_S_START 0x04000000
#define ROT_S_END 0x0400FFFF
#define SEG_LOWER_BOUND(start) (((start) >> 12) & 0x0000ffff)
#define SEG_UPPER_BOUND(end) ((((end) >> 12) & 0x0000ffff) << 16)
#define SEG_WRITE_FILTER(start, end) (SEG_LOWER_BOUND(start) | SEG_UPPER_BOUND(end))

#define CMD_FLT_CTRL 0xA0
#define ADDR_FLT_CTRL 0xA4
#define ADDR_FLT_REG1 0xB0
#define ADDR_FLT_REG2 0xB4
#define FQCD07 0x11C
#define AQCD00 0x150
#define AQCD01 0x154
#define ENB_CMD_FLT 0x1
#define ENB_ADDR_FLT 0xF
#define CMD_READ 0x8000006B
#define CMD_PROG 0x80000202
#define CMD_ERASE 0x8000D8D8

enum spi_region {
	ROT_P = 0,
	ROT_S,
};

struct aspeed_spilock {
	struct device *dev;
	phys_addr_t addr;
	struct mutex lock;    /* FMC Registers protection */
};

static inline u32 aspeed_spilock_read(struct device *dev, u32 reg)
{
	u32 val;
	struct aspeed_spilock *spilock = dev_get_drvdata(dev);
	void __iomem *regp = ioremap(spilock->addr + reg, sizeof(u32));

	mutex_lock(&spilock->lock);
	val = readl(regp);
	mutex_unlock(&spilock->lock);
	iounmap(regp);
	return val;
}

static inline void aspeed_spilock_write(struct device *dev, u32 reg, u32 val)
{
	struct aspeed_spilock *spilock = dev_get_drvdata(dev);
	void __iomem *regp = ioremap(spilock->addr + reg, sizeof(u32));

	mutex_lock(&spilock->lock);
	writel(val, regp);
	mutex_unlock(&spilock->lock);
	iounmap(regp);
}

static void lock_spi_region(struct device *dev, enum spi_region region, bool lock)
{
	switch (region) {
	case ROT_P:
		if (lock) {
			aspeed_spilock_write(dev, ADDR_FLT_REG1,
					     SEG_WRITE_FILTER(ROT_P_START, ROT_P_END));
			dev_dbg(dev, "Locked ROT-P Region successfully\n");
		} else {
			aspeed_spilock_write(dev, ADDR_FLT_REG1, 0x0);
			dev_dbg(dev, "Unlocked ROT-P Region successfully\n");
		}
		break;
	case ROT_S:
		if (lock) {
			aspeed_spilock_write(dev, ADDR_FLT_REG2,
					     SEG_WRITE_FILTER(ROT_S_START, ROT_S_END));
			dev_dbg(dev, "Locked ROT-S Region successfully\n");
		} else {
			aspeed_spilock_write(dev, ADDR_FLT_REG2, 0x0);
			dev_dbg(dev, "Unlocked ROT-S Region successfully\n");
		}
		break;
	default:
		dev_err(dev, "Invalid SPI Region\n");
		return;
	}

	aspeed_spilock_write(dev, ADDR_FLT_CTRL, ENB_ADDR_FLT);
	aspeed_spilock_write(dev, FQCD07, CMD_READ);
	aspeed_spilock_write(dev, AQCD00, CMD_PROG);
	aspeed_spilock_write(dev, AQCD01, CMD_ERASE);
	aspeed_spilock_write(dev, CMD_FLT_CTRL, ENB_CMD_FLT);
}

static ssize_t lock_ROT_P_region_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int val = aspeed_spilock_read(dev, ADDR_FLT_REG1);

	return sprintf(buf, "%d", val ? 1 : 0);
}

static ssize_t lock_ROT_P_region_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int ret;
	bool lock;

	ret = kstrtobool(buf, &lock);
	if (ret)
		return ret;

	lock_spi_region(dev, ROT_P, lock);

	return count;
}

static ssize_t lock_ROT_S_region_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int val = aspeed_spilock_read(dev, ADDR_FLT_REG2);

	return sprintf(buf, "%d", val ? 1 : 0);
}

static ssize_t lock_ROT_S_region_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int ret;
	bool lock;

	ret = kstrtobool(buf, &lock);
	if (ret)
		return ret;

	lock_spi_region(dev, ROT_S, lock);

	return count;
}

static DEVICE_ATTR_ADMIN_RW(lock_ROT_P_region);
static DEVICE_ATTR_ADMIN_RW(lock_ROT_S_region);

static const struct of_device_id aspeed_spilock_of_matches[] = {
	{ .compatible = "aspeed,ast2600-spilock" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_spilock_of_matches);

static int aspeed_spilock_probe(struct platform_device *pdev)
{
	struct aspeed_spilock *spilock;
	struct resource *res;
	int ret;

	spilock = devm_kzalloc(&pdev->dev, sizeof(*spilock), GFP_KERNEL);
	if (!spilock)
		return -ENOMEM;

	spilock->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(spilock->dev, "cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}
	spilock->addr = res->start;

	mutex_init(&spilock->lock);

	ret = device_create_file(spilock->dev, &dev_attr_lock_ROT_P_region);
	if (ret)
		return ret;

	ret = device_create_file(spilock->dev, &dev_attr_lock_ROT_S_region);
	if (ret)
		goto err_rot_s_file;

	platform_set_drvdata(pdev, spilock);
	/* Lock ROT-P and ROT-S by default */
	lock_spi_region(spilock->dev, ROT_P, true);
	lock_spi_region(spilock->dev, ROT_S, true);

	return 0;

err_rot_s_file:
	device_remove_file(spilock->dev, &dev_attr_lock_ROT_P_region);
	return ret;
}

static int aspeed_spilock_remove(struct platform_device *pdev)
{
	struct aspeed_spilock *spilock = platform_get_drvdata(pdev);

	device_remove_file(spilock->dev, &dev_attr_lock_ROT_P_region);
	device_remove_file(spilock->dev, &dev_attr_lock_ROT_S_region);

	return 0;
}

static struct platform_driver aspeed_spilock_driver = {
	.probe = aspeed_spilock_probe,
	.remove = aspeed_spilock_remove,
	.driver = {
			.name = KBUILD_MODNAME,
			.of_match_table = aspeed_spilock_of_matches,
	},
};

module_platform_driver(aspeed_spilock_driver);

MODULE_AUTHOR("Meghan Saitwal <meghan.saitwal@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASPEED SPILOCK Driver");
