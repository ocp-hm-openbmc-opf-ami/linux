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
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

#include "vhub.h"

#define DRIVER_NAME "usb"

struct vhub_sysfs_data {
	struct ast_vhub *vhub;
	bool force_disable;
	struct mutex lock;
	struct dentry *debugfs_dir;
};

static struct class *vhub_class;
static DEFINE_MUTEX(sysfs_lock);

/* Reg base address to sysfs node mapping */
static const struct {
	resource_size_t base_addr;
	int node_index;
} vhub_node_map[] = {
	{ 0x1e6a0000, 0 },	/* ast2600 vhub */
	{ 0x12011000, 0 },	/* ast2700 vhuba1 */
	{ 0x12060000, 0 },	/* ast2700 vhuba0 */
	{ 0x12021000, 1 },	/* ast2700 vhubb1 */
	{ 0x12062000, 1 },	/* ast2700 vhubb0 */
	{ 0x14120000, 2 },	/* ast2700 vhubc */
	{ 0x14122000, 3 },	/* ast2700 vhubd */
};

static int get_vhub_name(struct device *dev, char *buf, size_t size)
{
	struct device_node *np = dev->of_node;
	struct resource res;
	int i;

	if (!np || !buf || size == 0)
		return -EINVAL;

	if (of_address_to_resource(np, 0, &res))
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(vhub_node_map); i++) {
		if (res.start == vhub_node_map[i].base_addr) {
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

/* Debugfs: force_disable attribute*/
static int force_disable_get(void *priv, u64 *value)
{
	struct vhub_sysfs_data *data = priv;

	if (!data || !data->vhub)
		return -EINVAL;

	mutex_lock(&data->lock);
	*value = data->force_disable;
	mutex_unlock(&data->lock);

	return 0;
}

static int force_disable_set(void *priv, u64 value)
{
	struct vhub_sysfs_data *data = priv;

	if (!data || !data->vhub)
		return -EINVAL;

	if (value > 1)
		return -EINVAL;

	mutex_lock(&data->lock);

	if (value == 1) {
		dev_info(&data->vhub->pdev->dev,
			"Force disabling power save mode (vHub always connected)\n");
		(void)ami_vhub_upstream_connect(data->vhub);
		data->force_disable = true;
	} else {
		data->force_disable = false;
	}

	mutex_unlock(&data->lock);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(force_disable_fops, force_disable_get,
			 force_disable_set, "%llu\n");

static struct attribute *vhub_attrs[] = {
	&dev_attr_power_save_mode.attr,
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

	data->debugfs_dir = debugfs_create_dir(device_name, usb_debug_root);
	debugfs_create_file("force_disable", 0644, data->debugfs_dir, data,
			    &force_disable_fops);

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
		debugfs_remove_recursive(data->debugfs_dir);
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
