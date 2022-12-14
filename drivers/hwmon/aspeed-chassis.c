// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 ASPEED Technology Inc.
 *
 * CHASSIS driver for the Aspeed SoC
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/mfd/syscon.h>

#define CHAI10 0x10
union chassis_ctrl_register {
	u32 value;
	struct {
		u32 intrusion_status_clear : 1; /*[0]*/
		u32 intrusion_int_enable : 1; /*[1]*/
		u32 intrusion_status : 1; /*[2]*/
		u32 battery_power_good : 1; /*[3]*/
		u32 chassis_raw_status : 1; /*[4]*/
		u32 reserved0 : 3; /*[5-7]*/
		u32 io_power_status_clear : 1; /*[8]*/
		u32 io_power_int_enable : 1; /*[9]*/
		u32 core_power_status : 1; /*[10]*/
		u32 reserved1 : 5; /*[11-15]*/
		u32 core_power_status_clear : 1; /*[16]*/
		u32 core_power_int_enable : 1; /*[17]*/
		u32 io_power_status : 1; /*[18]*/
		u32 reserved2 : 13; /*[19-31]*/
	} fields;
};

struct aspeed_chassis {
	struct device *dev;
	struct regmap *map;
	const struct attribute_group *groups[2];
};

static ssize_t intrusion_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	unsigned long val;
	struct aspeed_chassis *chassis = dev_get_drvdata(dev);
	union chassis_ctrl_register chassis_ctrl;
	int rc;

	if (kstrtoul(buf, 10, &val) < 0 || val != 0)
		return -EINVAL;

	rc = regmap_read(chassis->map, CHAI10, &chassis_ctrl.value);
	if (rc < 0)
		return rc;

	chassis_ctrl.fields.intrusion_status_clear = 1;
	rc = regmap_write(chassis->map, CHAI10, chassis_ctrl.value);
	if (rc < 0)
		return rc;

	chassis_ctrl.fields.intrusion_status_clear = 0;
	rc = regmap_write(chassis->map, CHAI10, chassis_ctrl.value);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t intrusion_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	struct aspeed_chassis *chassis = dev_get_drvdata(dev);
	union chassis_ctrl_register chassis_ctrl;
	u8 ret;
	int rc;

	rc = regmap_read(chassis->map, CHAI10, &chassis_ctrl.value);
	if (rc < 0)
		return rc;

	switch (sensor_attr->index) {
	case 0:
		ret = chassis_ctrl.fields.core_power_status;
		break;
	case 1:
		ret = chassis_ctrl.fields.io_power_status;
		break;
	case 2:
		ret = chassis_ctrl.fields.intrusion_status;
		break;
	default:
		ret = chassis_ctrl.fields.intrusion_status;
	}

	return sprintf(buf, "%d\n", ret);
}

static SENSOR_DEVICE_ATTR_RO(core_power, intrusion, 0);
static SENSOR_DEVICE_ATTR_RO(io_power, intrusion, 1);
static SENSOR_DEVICE_ATTR_RW(intrusion0_alarm, intrusion, 2);

static struct attribute *intrusion_dev_attrs[] = {
	&sensor_dev_attr_core_power.dev_attr.attr,
	&sensor_dev_attr_io_power.dev_attr.attr,
	&sensor_dev_attr_intrusion0_alarm.dev_attr.attr,
	NULL
};

static const struct attribute_group intrusion_dev_group = {
	.attrs = intrusion_dev_attrs,
	.is_visible = NULL,
};

static int aspeed_chassis_int_ctrl(struct aspeed_chassis *chassis, bool ctrl)
{
	union chassis_ctrl_register chassis_ctrl;
	u32 val = 0;
	int rc;

	rc = regmap_read(chassis->map, CHAI10, &val);
	if (rc < 0)
		return rc;

	chassis_ctrl.value = val;
	chassis_ctrl.fields.intrusion_int_enable = ctrl;
	chassis_ctrl.fields.io_power_int_enable = ctrl;
	chassis_ctrl.fields.core_power_int_enable = ctrl;
	rc = regmap_write(chassis->map, CHAI10, chassis_ctrl.value);
	if (rc < 0)
		return rc;

	return 0;
}

static const struct of_device_id aspeed_chassis_of_table[] = {
	{ .compatible = "aspeed,ast2600-chassis" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_chassis_of_table);

static int aspeed_chassis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct aspeed_chassis *priv;
	struct device *hwmon;
	int rc;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->map = syscon_node_to_regmap(pdev->dev.parent->of_node);
	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);

	rc = aspeed_chassis_int_ctrl(priv, false);
	if (rc < 0)
		return rc;

	priv->groups[0] = &intrusion_dev_group;
	priv->groups[1] = NULL;

	hwmon = devm_hwmon_device_register_with_groups(dev, "aspeed_chassis",
						       priv, priv->groups);

	return PTR_ERR_OR_ZERO(hwmon);
}

static struct platform_driver aspeed_chassis_driver = {
	.probe		= aspeed_chassis_probe,
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_chassis_of_table,
	},
};

module_platform_driver(aspeed_chassis_driver);

MODULE_AUTHOR("Billy Tsai<billy_tsai@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED CHASSIS Driver");
MODULE_LICENSE("GPL");
