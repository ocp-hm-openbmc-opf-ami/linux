// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for MPS Multi-phase Digital VR Controllers
 *
 * Copyright (C) 2022 Intel Corporation.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "pmbus.h"

#define MP2971_PAGE_NUM 2

static int mp2971_read_word_data(struct i2c_client *client, int page, int phase,
				 int reg)
{
	int ret;
	u32 mask;

	switch (reg) {
	case PMBUS_READ_VIN:
		mask = GENMASK(9, 0);
		break;
	case PMBUS_READ_VOUT:
		mask = GENMASK(11, 0);
		break;
	case PMBUS_READ_IOUT:
	case PMBUS_READ_IIN:
	case PMBUS_READ_POUT:
	case PMBUS_READ_PIN:
		mask = GENMASK(10, 0);
		break;
	case PMBUS_READ_TEMPERATURE_1:
		mask = GENMASK(7, 0);
		break;
	default:
		return -ENODATA;
	}
	ret = pmbus_read_word_data(client, page, phase, reg);
	if (ret < 0)
		return ret;

	return ret & mask;
}

static struct pmbus_driver_info mp2971_info = {
	.pages = MP2971_PAGE_NUM,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = direct,
	.format[PSC_TEMPERATURE] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = direct,
	.format[PSC_POWER] = direct,
	.m[PSC_VOLTAGE_OUT] = 1,
	.R[PSC_VOLTAGE_OUT] = 3,
	.m[PSC_CURRENT_OUT] = 1,
	.m[PSC_POWER] = 1,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		   PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		   PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP | PMBUS_HAVE_POUT |
		   PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT,
	.func[1] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		   PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
		   PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP | PMBUS_HAVE_POUT |
		   PMBUS_HAVE_PIN | PMBUS_HAVE_STATUS_INPUT,
	.read_word_data = mp2971_read_word_data,
};

static int mp2971_probe(struct i2c_client *client)
{
	struct pmbus_driver_info *info;
	u8 buf[I2C_SMBUS_BLOCK_MAX];
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_WORD_DATA |
				     I2C_FUNC_SMBUS_READ_BLOCK_DATA))
		return -ENODEV;

	ret = i2c_smbus_read_block_data(client, PMBUS_MFR_ID, buf);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read PMBUS_MFR_ID\n");
		return ret;
	}
	if (ret != 2 || strncmp(buf, "\x25", 1)) {
		dev_err(&client->dev, "MFR_ID unrecognized\n");
		return -ENODEV;
	}

	info = devm_kmemdup(&client->dev, &mp2971_info, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	return pmbus_do_probe(client, info);
}

static const struct i2c_device_id mp2971_id[] = {
	{ "mp2971", 0 },
	{ "mp2973", 0 },
	{} };

MODULE_DEVICE_TABLE(i2c, mp2971_id);

static const struct of_device_id __maybe_unused mp2971_of_match[] = {
	{ .compatible = "mps,mp2971" },
	{ .compatible = "mps,mp2973" },
	{}
};
MODULE_DEVICE_TABLE(of, mp2971_of_match);

static struct i2c_driver mp2971_driver = {
	.driver = {
		.name = "mp2971",
		.of_match_table = of_match_ptr(mp2971_of_match),
	},
	.probe_new = mp2971_probe,
	.id_table = mp2971_id,
};

module_i2c_driver(mp2971_driver);

MODULE_AUTHOR("Shamim Ali <shamimax.ali@intel.com>");
MODULE_DESCRIPTION("PMBus driver for MPS MP2971 device");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
