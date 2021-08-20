// SPDX-License-Identifier: GPL-2.0+
/*
 * RTC driver for PCF85053
 * Copyright (C) 2021 YADRO
 * Copyright (c) 2021 Intel Corporation
 */

#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/mc146818rtc.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#define NUM_TIME_REGS   (RTC_YEAR - RTC_SECONDS + 1)

struct pcf85053 {
	struct rtc_device *rtc;
	struct regmap *regmap;
};

static int pcf85053_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pcf85053 *pcf85053 = i2c_get_clientdata(client);
	unsigned char rtc_data[NUM_TIME_REGS];
	int rc;

	/*
	 * This will read a few unnecessary alarm registers, but better to read
	 * the whole sequence in one go so that carry/overflow conditions are
	 * avoided.
	 */
	rc = regmap_bulk_read(pcf85053->regmap, RTC_SECONDS, rtc_data, NUM_TIME_REGS);
	if (rc < 0) {
		dev_err(dev, "Fail to read time reg(%d)\n", rc);
		return rc;
	}

	tm->tm_sec	= bcd2bin(rtc_data[0]);
	tm->tm_min	= bcd2bin(rtc_data[2]);
	tm->tm_hour	= bcd2bin(rtc_data[4]);
	tm->tm_wday	= rtc_data[6];
	tm->tm_mday	= bcd2bin(rtc_data[7]);
	tm->tm_mon	= bcd2bin(rtc_data[8]) - 1;
	tm->tm_year	= bcd2bin(rtc_data[9]) + 100;

	return 0;
}

static const struct rtc_class_ops pcf85053_rtc_ops = {
	.read_time = pcf85053_rtc_read_time,
};

static const struct regmap_config pcf85053_rtc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int pcf85053_rtc_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct pcf85053 *pcf85053;
	int rc;

	pcf85053 = devm_kzalloc(&client->dev, sizeof(*pcf85053), GFP_KERNEL);
	if (!pcf85053)
		return -ENOMEM;

	pcf85053->regmap = devm_regmap_init_i2c(client, &pcf85053_rtc_regmap_config);
	if (IS_ERR(pcf85053->regmap)) {
		dev_err(&client->dev, "regmap_init failed\n");
		return PTR_ERR(pcf85053->regmap);
	}

	i2c_set_clientdata(client, pcf85053);

	pcf85053->rtc = devm_rtc_device_register(&client->dev, "pcf85053-rtc",
						 &pcf85053_rtc_ops, THIS_MODULE);
	if (IS_ERR(pcf85053->rtc))
		return PTR_ERR(pcf85053->rtc);

	return 0;
}

static const struct i2c_device_id pcf85053_rtc_id[] = {
	{ "rtc-pcf85053", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcf85053_rtc_id);

static const struct of_device_id pcf85053_rtc_of_match[] = {
	{ .compatible = "nxp,pcf85053", },
	{ }
};
MODULE_DEVICE_TABLE(of, pcf85053_rtc_of_match);

static struct i2c_driver pcf85053_rtc_driver = {
	.driver		= {
		.name	= "rtc-pcf85053",
		.of_match_table = pcf85053_rtc_of_match,
	},
	.probe		= pcf85053_rtc_probe,
	.id_table	= pcf85053_rtc_id,
};
module_i2c_driver(pcf85053_rtc_driver);

MODULE_DESCRIPTION("PCF85053 RTC driver");
MODULE_AUTHOR("Jonathan Doman <jonathan.doman@intel.com>");
MODULE_LICENSE("GPL");
