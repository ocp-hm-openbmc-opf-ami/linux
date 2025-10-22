// SPDX-License-Identifier: GPL-2.0+
/*
 * core-sysfs.c - ASPEED vHub sysfs interface for power save mode control
 *
 * Copyright (C) 2025 American Megatrends International, LLC
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "vhub.h"

#define DRIVER_NAME "usb"

struct vhub_sysfs_data {
	struct ast_vhub *vhub;
	bool force_disable;
	struct mutex lock;
};

static struct class *vhub_class;
static DEFINE_MUTEX(sysfs_lock);

/* Device tree compatible to sysfs node mapping */
static const struct {
	const char *compatible;
	int node_index;
} vhub_node_map[] = {
	{ "aspeed,ast2600-usb-vhub",   0 },
	{ "aspeed,ast2700-usb-vhuba0", 0 },
	{ "aspeed,ast2700-usb-vhuba1", 0 },
	{ "aspeed,ast2700-usb-vhubb0", 1 },
	{ "aspeed,ast2700-usb-vhubb1", 1 },
	{ "aspeed,ast2700-usb-vhubc", 2 },
	{ "aspeed,ast2700-usb-vhubd", 3 },
	{ NULL, -1 }
};

static int get_vhub_name(struct device *dev, char *buf, size_t size)
{
	struct device_node *np = dev->of_node;
	int i;

	if (!np || !buf || size == 0)
		return -EINVAL;

	for (i = 0; vhub_node_map[i].compatible; i++) {
		if (of_device_is_compatible(np, vhub_node_map[i].compatible)) {
			snprintf(buf, size, "vhub%d", vhub_node_map[i].node_index);
			return 0;
		}
	}

	/* No match found */
	return -ENODEV;
}

static int match_vhub_name(struct device *dev, const void *data)
{
	return sysfs_streq(dev_name(dev), data);
}

/* Sysfs: power_save_mode attribute */
static ssize_t power_save_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vhub_sysfs_data *data = dev_get_drvdata(dev);
	bool connected;

	if (!data || !data->vhub)
		return -EINVAL;

	mutex_lock(&data->lock);
	connected = ami_vhub_upstream_status(data->vhub);
	mutex_unlock(&data->lock);

	/* connected = 0 means power save mode is active (disconnected) */
	return sysfs_emit(buf, "%d\n", connected ? 0 : 1);
}

static ssize_t power_save_mode_store(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct vhub_sysfs_data *data = dev_get_drvdata(dev);
	int value;

	if (!data || !data->vhub)
		return -EINVAL;

	if (kstrtoint(buf, 10, &value) || value < 0 || value > 1)
		return -EINVAL;

	mutex_lock(&data->lock);

	if (data->force_disable && value == 1) {
		dev_err(dev, "Cannot enable power save mode while force_disable is active\n");
		mutex_unlock(&data->lock);
		return -EBUSY;
	}

	if (!data->force_disable) {
		if (value == 1) {
			dev_info(dev, "Enabling power save mode (disconnecting vHub)\n");
			(void)ami_vhub_upstream_disconnect(data->vhub);
		} else {
			dev_info(dev, "Disabling power save mode (connecting vHub)\n");
			(void)ami_vhub_upstream_connect(data->vhub);
		}
	}

	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR_RW(power_save_mode);

/* Sysfs: force_disable attribute */
static ssize_t force_disable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vhub_sysfs_data *data = dev_get_drvdata(dev);
	bool state;

	if (!data || !data->vhub)
		return -EINVAL;

	mutex_lock(&data->lock);
	state = data->force_disable;
	mutex_unlock(&data->lock);

	return sysfs_emit(buf, "%d\n", state ? 1 : 0);
}

static ssize_t force_disable_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct vhub_sysfs_data *data = dev_get_drvdata(dev);
	int value;

	if (!data || !data->vhub)
		return -EINVAL;

	if (kstrtoint(buf, 10, &value) || value < 0 || value > 1)
		return -EINVAL;

	mutex_lock(&data->lock);

	if (value == 1) {
		dev_info(dev, "Force disabling power save mode (vHub always connected)\n");
		(void)ami_vhub_upstream_connect(data->vhub);
		data->force_disable = true;
	} else {
		data->force_disable = false;
	}

	mutex_unlock(&data->lock);
	return count;
}
static DEVICE_ATTR_RW(force_disable);

static struct attribute *vhub_attrs[] = {
	&dev_attr_power_save_mode.attr,
	&dev_attr_force_disable.attr,
	NULL,
};

static const struct attribute_group vhub_group = {
	.attrs = vhub_attrs,
};

static const struct attribute_group *vhub_groups[] = {
	&vhub_group,
	NULL,
};

int ami_core_sysfs_register(struct device *dev, struct ast_vhub *vhub)
{
	char device_name[32];
	struct device *vhub_device;
	struct vhub_sysfs_data *data;
	int ret;

	ret = get_vhub_name(dev, device_name, sizeof(device_name));
	if (ret) {
		dev_warn(dev, "No sysfs mapping found, skipping registration\n");
		return -ENODEV;
	}

	/* Check if class is registered */
	if (!class_is_registered(vhub_class)) {
		dev_err(dev, "Sysfs class not initialized\n");
		return -ENODEV;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->vhub = vhub;
	data->force_disable = false;
	mutex_init(&data->lock);

	mutex_lock(&sysfs_lock);

	vhub_device = device_create_with_groups(vhub_class, dev, 0, data,
						vhub_groups, device_name);
	if (IS_ERR(vhub_device)) {
		dev_err(dev, "Failed to create sysfs device '%s': %ld\n",
			device_name, PTR_ERR(vhub_device));
		mutex_unlock(&sysfs_lock);
		kfree(data);
		return -ENODEV;
	}

	mutex_unlock(&sysfs_lock);

	dev_info(dev, "Registered sysfs device: %s\n", device_name);
	return 0;
}

void ami_core_sysfs_unregister(struct device *dev, struct ast_vhub *vhub)
{
	char device_name[32];
	struct device *vhub_device;
	struct vhub_sysfs_data *data;

	if (get_vhub_name(dev, device_name, sizeof(device_name)))
		return;

	mutex_lock(&sysfs_lock);

	vhub_device = class_find_device(vhub_class, NULL, device_name, match_vhub_name);
	if (!vhub_device) {
		mutex_unlock(&sysfs_lock);
		return;
	}

	data = dev_get_drvdata(vhub_device);
	device_unregister(vhub_device);
	put_device(vhub_device);

	mutex_unlock(&sysfs_lock);

	if (data) {
		mutex_destroy(&data->lock);
		kfree(data);
	}
	dev_info(dev, "Unregistered sysfs device: %s\n", device_name);
}

static int __init vhub_sysfs_init(void)
{
	vhub_class = class_create(DRIVER_NAME);
	if (IS_ERR(vhub_class)) {
		pr_err("Failed to create sysfs class\n");
		return PTR_ERR(vhub_class);
	}

	return 0;
}
subsys_initcall(vhub_sysfs_init);

static void __exit vhub_sysfs_exit(void)
{
	class_destroy(vhub_class);
}
module_exit(vhub_sysfs_exit);
