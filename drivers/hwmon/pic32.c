// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pic32.c - Part of fan sensors, Linux kernel modules for hardware
 *               monitoring
 * Copyright (C) 2023 Intel Corporation
 *
 * PIC32 MCU transfer control message between BMC and fan sensor
 * by I2C. It read rpm value of each fans and write a relevant
 * pwm value by phosphor-pid-control service.
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>

static const unsigned short normal_i2c[] = { 0x56, 0x5e, I2C_CLIENT_END };

#define PWM_MAX 255
#define PIC32_MAX 100
#define NR_CHANNEL 5

/* pic32 registers definition  */
#define PIC32_FAN_PRESENCE  0x00
#define PIC32_FAN_FAULT  0x01
#define PIC32_FAN_PWM(ch)  (0x03 + (ch))
#define PIC32_FAN_SPEED(ch)  (0x08 + (ch))

struct pic32_data {
	struct i2c_client *client;
	struct device *hwmon_dev;
	struct mutex update_lock; /* Lock for read/write */
};

static int pic32_read_pwm(struct device *dev, u32 attr, int channel, long *val)
{
	struct pic32_data *data = dev_get_drvdata(dev);
	int rv;

	switch (attr) {
	case hwmon_pwm_input:
		mutex_lock(&data->update_lock);
		rv = i2c_smbus_read_byte_data(data->client, PIC32_FAN_PWM(channel));
		if (rv < 0) {
			dev_err(dev, "failed to read pwm\n");
			mutex_unlock(&data->update_lock);
			return rv;
		}
		*val = (PWM_MAX * rv) / PIC32_MAX;
		mutex_unlock(&data->update_lock);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int pic32_write_pwm(struct device *dev, u32 attr, int channel, long val)
{
	struct pic32_data *data = dev_get_drvdata(dev);
	int err = 0;
	u8 wbuf[3];
	int pec;
	u16 msg;

	switch (attr) {
	case hwmon_pwm_input:
		val = ((val + 1) * PIC32_MAX) / PWM_MAX;
		mutex_lock(&data->update_lock);
		wbuf[0] = (data->client)->addr << 1; /* address */
		wbuf[1] = PIC32_FAN_PWM(channel); /* register */
		wbuf[2] = val; /* value */
		pec = i2c_smbus_pec(0, wbuf, 3); /* checksum */
		if (pec < 0)
			goto abort;

		msg = val | (pec << 8);
		dev_dbg(dev, "pwm write val = %ld, Reg = 0x%x\n", val, PIC32_FAN_PWM(channel));

		err = i2c_smbus_write_word_data(data->client, PIC32_FAN_PWM(channel), msg);
		if (err < 0)
			goto abort;

		mutex_unlock(&data->update_lock);
		return err;
	default:
		err = -EOPNOTSUPP;
		break;
	}

abort:
	dev_err(dev, "failed to write\n");
	mutex_unlock(&data->update_lock);
	return err;
}

static int pic32_read_fan(struct device *dev, u32 attr, int channel, long *val)
{
	struct pic32_data *data = dev_get_drvdata(dev);
	int rv;

	if (IS_ERR(data))
		return PTR_ERR(data);

	switch (attr) {
	case hwmon_fan_input:
		mutex_lock(&data->update_lock);
		rv = i2c_smbus_read_word_data(data->client, PIC32_FAN_SPEED(channel));
		if (rv < 0) {
			dev_warn(dev, "failed to read fan\n");
			mutex_unlock(&data->update_lock);
			return rv;
		}
		*val = rv;
		mutex_unlock(&data->update_lock);
		return 0;
	case hwmon_fan_fault:
		mutex_lock(&data->update_lock);
		rv = i2c_smbus_read_byte_data(data->client, PIC32_FAN_FAULT);
		if (rv < 0) {
			dev_warn(dev, "failed to read fault\n");
			mutex_unlock(&data->update_lock);
			return rv;
		}
		*val = (rv & BIT(channel)) ? 1 : 0;
		mutex_unlock(&data->update_lock);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int pic32_write_fan(struct device *dev, u32 attr, int channel, long val)
{
	struct pic32_data *data = dev_get_drvdata(dev);
	int err = 0;
	u8 wbuf[3];
	int pec, rv;
	u16 msg;

	switch (attr) {
	case hwmon_fan_fault:
		mutex_lock(&data->update_lock);
		rv = i2c_smbus_read_byte_data(data->client, PIC32_FAN_FAULT);
		if (rv < 0) {
			dev_err(dev, "failed to write fault\n");
			mutex_unlock(&data->update_lock);
			return rv;
		}
		val = val ? (rv | BIT(channel)) : (rv & ~BIT(channel));
		wbuf[0] = (data->client)->addr << 1; /* address */
		wbuf[1] = PIC32_FAN_FAULT; /* register */
		wbuf[2] = val; /* value */
		pec = i2c_smbus_pec(0, wbuf, 3); /* checksum */
		if (pec < 0)
			goto abort;

		msg = val | (pec << 8);
		dev_dbg(dev, "pwm write to pdb = %ld, Reg = 0x%x\n", val, PIC32_FAN_FAULT);

		err = i2c_smbus_write_word_data(data->client, PIC32_FAN_FAULT, msg);
		if (err < 0)
			goto abort;

		mutex_unlock(&data->update_lock);
		return err;
	default:
		err = -EOPNOTSUPP;
		break;
	}

abort:
	dev_warn(dev, "failed to write\n");
	mutex_unlock(&data->update_lock);
	return err;
}

static umode_t pic32_fan_is_visible(const void *_data, u32 attr, int channel)
{
	switch (attr) {
	case hwmon_fan_input:
		if (channel < 2 * NR_CHANNEL)
			return 0444;
		return 0;
	case hwmon_fan_fault:
		if (channel < NR_CHANNEL)
			return 0644;
		return 0;
	default:
		return 0;
	}
}

static umode_t pic32_pwm_is_visible(const void *_data, u32 attr, int channel)
{
	switch (attr) {
	case hwmon_pwm_input:
		if (channel < NR_CHANNEL)
			return 0644;
		return 0;
	default:
		return 0;
	}
}

static int pic32_read(struct device *dev, enum hwmon_sensor_types type,
		      u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_fan:
		return pic32_read_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return pic32_read_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int pic32_write(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_fan:
		return pic32_write_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return pic32_write_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t pic32_is_visible(const void *data, enum hwmon_sensor_types type,
				u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return pic32_fan_is_visible(data, attr, channel);
	case hwmon_pwm:
		return pic32_pwm_is_visible(data, attr, channel);
	default:
		return 0;
	}
}

static const struct hwmon_channel_info *pic32_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_FAULT,
			   HWMON_F_INPUT | HWMON_F_FAULT,
			   HWMON_F_INPUT | HWMON_F_FAULT,
			   HWMON_F_INPUT | HWMON_F_FAULT,
			   HWMON_F_INPUT | HWMON_F_FAULT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT,
			   HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT),
	NULL
};

static const struct hwmon_ops pic32_hwmon_ops = {
	.is_visible = pic32_is_visible,
	.read = pic32_read,
	.write = pic32_write,
};

static const struct hwmon_chip_info pic32_chip_info = {
	.ops = &pic32_hwmon_ops,
	.info = pic32_info,
};

static int pic32_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pic32_data *data;
	struct device *dev = &client->dev;

	data = devm_kzalloc(dev, sizeof(struct pic32_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->update_lock);

	data->hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							       data,
							       &pic32_chip_info,
							       NULL);

	if (IS_ERR(data->hwmon_dev))
		return PTR_ERR(data->hwmon_dev);

	return 0;
}

static void pic32_remove(struct i2c_client *client)
{
	struct pic32_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
}

static const struct i2c_device_id pic32_id[] = {
	{ "pic32", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pic32_id);

static struct i2c_driver pic32_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "pic32",
	},
	.probe		= pic32_probe,
	.remove		= pic32_remove,
	.id_table	= pic32_id,
	.address_list	= normal_i2c,
};

module_i2c_driver(pic32_driver);

MODULE_AUTHOR("Meghan Saitwal <meghan.saitwal@intel.com>");
MODULE_DESCRIPTION("PIC32MX130F128L on OPB PDB driver");
MODULE_LICENSE("GPL");
