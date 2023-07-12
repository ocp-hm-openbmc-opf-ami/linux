// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Intel Corporation

#include <linux/component.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

#include "pmbus.h"
#include "smart.h"

#define MAX_PECI_CPU_DEVICES 8
#define MAX_PECI_DIMM_DEVICES 8
#define MAX_PECI_PLATFORM_DEVICES 1
#define MAX_PSU_NUM_SUPPORTED 4
#define MAX_SUPPORTED_PECI_DEVICES 17
#define PMBUS_REV_1_2 2
#define PSU_EVENT_NO_EVENT  0
#define REDUNDANCY_ENABLED	0x01
#define POWERGOOD_OK		0x02
#define MORE_THAN_ONE_PSU	0x04
#define UNIT_OFF		0x08
#define SMART_MONITOR_MASK_PSU_NOT_PRESENT 0xFF
#define SMART_STATUS_INPUT_UV_FAULT_MASK 0x10
#define SMART_STATUS_INPUT_UNIT_OFF_MASK 0x08
#define SMART_STATUS_IOUT_OC_WARNING_MASK 0x20
#define SMART_STATUS_TEMPERATURE_OT_WARNING_MASK 0x40
#define SMART_STATUS_WORD_POWERGOOD_MASK 0x0800
#define SMART_STATUS_MASK_ALL 0xFF
#define SMART_PMBUS_PAGE 0x01

/**
 * struct smart_ot_psu_list_entry - Contains list of PSUs which have OT
 * event present.
 * @psu: Handle to PSU entry registered in SmaRT.
 * @node: List head.
 */
struct smart_ot_psu_list_entry {
	struct smart_psu_entry *psu;
	struct list_head node;
};

/**
 * struct smart_peci_entry - Describes PECI hwmon device registered in SmaRT.
 * @dev: Handle to device struct.
 * @ops: Ops shared by hwmon PECI devices to SmaRT module, used to
 *	 enable/disable max power throttling
 * @node: List head.
 */
struct smart_peci_entry {
	struct device *dev;
	struct peci_throttling_ops *ops;
	struct list_head node;
};

struct smart_psu_state_word {
	u16 none_of_the_above : 1;
	u16 cml : 1;
	u16 temperature : 1;
	u16 vin_uv_fault : 1;
	u16 iout_oc_fault : 1;
	u16 vout_ov_fault : 1;
	u16 off : 1;
	u16 busy : 1;
	u16 unknown : 1;
	u16 other : 1;
	u16 fans : 1;
	u16 power_good : 1;
	u16 mfr_specific : 1;
	u16 input : 1;
	u16 iout_pout : 1;
	u16 vout : 1;
};
struct smart_psu_state_input {
	u8 pin_op_warning : 1;
	u8 iin_oc_warning : 1;
	u8 iin_oc_fault : 1;
	u8 unit_off_low_voltage : 1;
	u8 vin_uv_fault : 1;
	u8 vin_uv_warning : 1;
	u8 vin_ov_warning : 1;
	u8 vin_ov_fault : 1;
};

struct smart_psu_state_iout {
	u8 pout_op_warning : 1;
	u8 pout_op_fault : 1;
	u8 power_limiting_mode : 1;
	u8 current_share_fault : 1;
	u8 iout_uc_fault : 1;
	u8 iout_oc_warning : 1;
	u8 iout_oc_fault_lv_shutdown : 1;
	u8 iout_oc_fault : 1;
};

struct smart_psu_state_temperature {
	u8 reserved : 4;
	u8 ut_fault : 1;
	u8 ut_warning : 1;
	u8 ot_warning : 1;
	u8 ot_fault : 1;
};

union smart_psu_event {
	struct {
		union {
			struct smart_psu_state_word bit;
			u16 all;
		} word;
		union {
			struct smart_psu_state_input bit;
			u8 all;
		} input;
		union {
			struct smart_psu_state_iout bit;
			u8 all;
		} iout;
		union {
			struct smart_psu_state_temperature bit;
			u8 all;
		} temperature;
	};
	u64 all;
};

/**
 * struct smart_psu_entry - Describes PSU registered in SmaRT.
 * @dev: Handle to device struct.
 * @undervoltage_masked_default: Default for UV mask in PSU, based on redundancy flag,
 *				 number of PSUs, POWERGOOD status.
 * @overcurrent_masked_default: Default for OC mask in PSU, based on redundancy flag,
 *				number of PSUs, POWERGOOD status.
 * @overtemperature_masked_default: Default for OT mask in PSU, based on redundancy flag,
 *				    number of PSUs, POWERGOOD status.
 * @undervoltage_masked: True, when UV event is masked in PSU.
 * @overcurrent_masked: True, when OC event is masked in PSU.
 * @overtemperature_masked: True, when OT event is masked in PSU.
 * @powergood: Reflects POWERGOOD status read from hardware.
 * @event_reported: Stores events reported when SMBAlert# is triggered and
 *		    SmaRT checks PSU status.
 * @current_event: Stores latest read events
 * @timer: Used to force additional throttling when SMBAlert# processing is
 *	   completed. Stores time to end of throttling.
 * @unit_off: Reflects UNIT_OFF bit read from hardware,
 *	      helps distinguish between UV and UNIT_OFF events.
 * @additional_throttling_timestamp: Timestamp when additional throttling started.
 * @uv_start_timestamp: Timestamp when UV event occurred.
 * @uv_timestamp_saved: Indicates the timestamp has been saved.
 * @ot_start_timestamp: Timestamp when OT event occurred.
 * @ot_timestamp_saved: Indicates the timestamp has been saved.
 * @node: List of registered PSU devices.
 */
struct smart_psu_entry {
	struct device *dev;
	bool undervoltage_masked_default;
	bool overcurrent_masked_default;
	bool overtemperature_masked_default;
	bool undervoltage_masked;
	bool overcurrent_masked;
	bool overtemperature_masked;
	bool powergood;
	union smart_psu_event event_reported;
	union smart_psu_event current_event;
	int timer;
	bool unit_off;
	ktime_t additional_throttling_timestamp;
	ktime_t uv_start_timestamp;
	bool uv_timestamp_saved;
	ktime_t ot_start_timestamp;
	bool ot_timestamp_saved;
	bool fault_state;
	struct list_head node;
};

union smart_query_response {
	struct {
		u8 reserved : 2;
		u8 format: 3;
		u8 read_supported : 1;
		u8 write_supported : 1;
		u8 supported : 1;
	};
	u8 all;
};

/**
 * struct smart_attr - configures attribute created in sysfs.
 * @name: Attribute name.
 * @min: Minimum value, validated on write access.
 * @max: Maximum value, validated on write access.
 * @value: Stores a value. Initialize it with a default used on start.
 * @opposite: If applies, points to opposite attribute, e.g. min vs max.
 */
struct smart_attr {
	const char *name;
	const unsigned int min;
	const unsigned int max;
	unsigned int value;
	struct smart_attr *opposite;
};

/**
 * struct smart_data - Stores current state of SmaRT module. Stored in driver's
 * priv data.
 * @dev: Handle to SmaRT's device struct.
 * @psu_list: List of registered PMBus devices.
 * @peci_list: List of registered PECI devices.
 * @mutex: Locks access to PECI and PSU devices lists (@psu_list,
 *			@peci_cpu_list, @peci_dimm_list, @peci_platform_list).
 *			Used to prevent probe/remove functions accessing those
 *			lists while SMBAlert interrupt is being processed.
 * @gpio: Stores GPIO descriptor of SMBAlert# requested in probe().
 * @work: Stores work related to SMBAlert# processing (PSUs polling).
 * @powergood_work: Stores work related to POWERGOOD polling.
 * @workqueue: Stores workqueue related to SMBAlert# processing (PSUs polling).
 * @event_logged: Latch flag to indicate whether events has been logged to
 *		  journal.
 * @throttling_reason: Stores the ORed error status value gathered from all linked PSUs
 *		       at the time when SMBAlert# had been triggered.
 * @additional_throttling_not_done: Indicates whether additional throttling is
 *				    not done yet at the end of SMBAlert#
 *				    processing.
 */
struct smart_data {
	struct device *dev;
	struct list_head psu_list;
	struct list_head peci_list;
	/*
	 * To synchronize concurrent accesses to smart priv data structure,
	 * e.g. registering new PSU while processing interrupt.
	 */
	struct mutex mutex;
	/*
	 * To synchronize concurrent accesses to devices
	 */
	struct mutex device_access_mutex;
	struct gpio_desc *gpio;
	struct delayed_work work;
	struct delayed_work powergood_work;
	struct workqueue_struct *workqueue;
	bool event_logged;
	bool throttling_active;
	union smart_psu_event throttling_reason;
	bool additional_throttling_not_done;
};

static struct smart_data *smart_data;

enum smart_status {
	UNINITIALIZED,
	NO_GPIO,
	IDLE,
	INTERRUPT,
};

static int smart_read_byte(const struct i2c_client *client, u8 command)
{
	union i2c_smbus_data data;
	int ret;

	data.block[0] = 2;
	data.block[1] = SMART_PMBUS_PAGE;
	data.block[2] = command;

	ret = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			     I2C_SMBUS_WRITE, PMBUS_PAGE_PLUS_READ,
			     I2C_SMBUS_BLOCK_PROC_CALL, &data);

	if (ret < 0) {
		dev_dbg(&client->dev, "%s failed with status %d for command %x\n", __func__, ret,
				command);

		return ret;
	}

	return data.block[1];
}

static int smart_read_word(const struct i2c_client *client, u8 command)
{
	union i2c_smbus_data data;
	int ret;

	data.block[0] = 2;
	data.block[1] = SMART_PMBUS_PAGE;
	data.block[2] = command;
	ret = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			     I2C_SMBUS_WRITE, PMBUS_PAGE_PLUS_READ,
			     I2C_SMBUS_BLOCK_PROC_CALL, &data);

	if (ret < 0) {
		dev_dbg(&client->dev, "%s failed with status %d\n", __func__, ret);

		return ret;
	}

	return data.block[1] | (data.block[2] << 8);
}

static int smart_write_byte(const struct i2c_client *client, u8 command, u8 value)
{
	union i2c_smbus_data data;
	int ret;

	data.block[0] = 3;
	data.block[1] = SMART_PMBUS_PAGE;
	data.block[2] = command;
	data.block[3] = value;
	ret = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			     I2C_SMBUS_WRITE, PMBUS_PAGE_PLUS_WRITE,
			     I2C_SMBUS_BLOCK_DATA, &data);

	if (ret < 0) {
		dev_dbg(&client->dev, "%s failed with status %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int smart_write_word(const struct i2c_client *client, u8 command, u16 value)
{
	union i2c_smbus_data data;
	int ret;

	data.block[0] = 4;
	data.block[1] = SMART_PMBUS_PAGE;
	data.block[2] = command;
	data.block[3] = value & 0xFF;
	data.block[4] = value >> 8;
	ret = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			     I2C_SMBUS_WRITE, PMBUS_PAGE_PLUS_WRITE,
			     I2C_SMBUS_BLOCK_DATA, &data);

	if (ret < 0) {
		dev_dbg(&client->dev, "%s failed with status %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static unsigned int smart_is_command_supported(const struct i2c_client *client, u8 command)
{
	u8 buf[] = { PMBUS_QUERY, 0x01, command};
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.buf = buf,
			.len = 3
		},
		{
			.addr = client->addr,
			.flags = client->flags | I2C_M_RD,
			.buf = buf,
			.len = 2
		}
	};
	int ret;
	union smart_query_response response;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_dbg(&client->dev, "i2c_transfer failed with status %d\n", ret);
		return 0;
	}
	response.all = buf[1];
	return response.supported;
}

static union smart_psu_event smart_read_event(const struct smart_psu_entry *psu)
{
	struct i2c_client *i2c = to_i2c_client(psu->dev);
	union smart_psu_event event;
	int ret;

	event = psu->current_event;

	ret = smart_read_word(i2c, PMBUS_STATUS_WORD);
	if (ret >= 0)
		event.word.all = (u16)ret;
	ret = smart_read_byte(i2c, PMBUS_STATUS_INPUT);
	if (ret >= 0)
		event.input.all = (u8)ret;
	ret = smart_read_byte(i2c, PMBUS_STATUS_IOUT);
	if (ret >= 0)
		event.iout.all = (u8)ret;
	ret = smart_read_byte(i2c, PMBUS_STATUS_TEMPERATURE);
	if (ret >= 0)
		event.temperature.all = (u8)ret;

	return event;
}

static void smart_clear_events(const struct smart_psu_entry *psu)
{
	struct i2c_client *i2c = to_i2c_client(psu->dev);

	smart_write_byte(i2c, PMBUS_STATUS_INPUT, SMART_STATUS_INPUT_UV_FAULT_MASK);
	smart_write_byte(i2c, PMBUS_STATUS_IOUT, SMART_STATUS_IOUT_OC_WARNING_MASK);
	smart_write_byte(i2c, PMBUS_STATUS_TEMPERATURE, SMART_STATUS_TEMPERATURE_OT_WARNING_MASK);
}

static struct smart_attr max_overtemperature_time = {
	.name = "max_overtemperature_time",
	.min = 100,
	.max = 2000,
	.value = 500,
};

static struct smart_attr max_undervoltage_time = {
	.name = "max_undervoltage_time",
	.min = 100,
	.max = 2000,
	.value = 500,
};

static struct smart_attr overtemperature_throttling_time = {
	.name = "overtemperature_throttling_time",
	.min = 100,
	.max = 10000,
	.value = 500,
};

static struct smart_attr overcurrent_throttling_time = {
	.name = "overcurrent_throttling_time",
	.min = 100,
	.max = 10000,
	.value = 500,
};

static struct smart_attr undervoltage_throttling_time = {
	.name = "undervoltage_throttling_time",
	.min = 100,
	.max = 10000,
	.value = 500,
};

static struct smart_attr smart_psu_polling_interval_time = {
	.name = "smart_psu_polling_interval_time",
	.min = 10,
	.max = 10000,
	.value = 10,
};

static struct smart_attr smart_powergood_polling_interval_time = {
	.name = "smart_powergood_polling_interval_time",
	.min = 10,
	.max = 10000,
	.value = 100,
};

static struct smart_attr force_smbalert_mask_interval_time = {
	.name = "force_smbalert_mask_interval_time",
	.min = 1000,
	.max = 1000000,
	.value = 10000,
};

static struct smart_attr irqs_serviced_num = {
	.name = "irqs_serviced_num",
	.value = 0,
};

static struct smart_attr wq_calls_on_psu_registering_num = {
	.name = "wq_calls_on_psu_registering_num",
	.value = 0,
};

static struct smart_attr wq_calls_on_enable_num = {
	.name = "wq_calls_on_enable_num",
	.value = 0,
};

static struct smart_attr psu_polls_num = {
	.name = "psu_polls_num",
	.value = 0,
};

static struct smart_attr powergood_polls_num = {
	.name = "powergood_polls_num",
	.value = 0,
};

static struct smart_attr enable = {
	.name = "enable",
	.value = 0,
	.min = 0,
	.max = 1,
};

static struct smart_attr redundancy = {
	.name = "redundancy",
	.value = 0,
	.min = 0,
	.max = 1,
};

static struct smart_attr i2c_addr_max;
static struct smart_attr i2c_addr_min = {
	.name = "i2c_addr_min",
	.value = 0x58,
	.min = 0x00,
	.max = 0xFF,
	.opposite = &i2c_addr_max,
};

static struct smart_attr i2c_addr_max = {
	.name = "i2c_addr_max",
	.value = 0x68,
	.min = 0x00,
	.max = 0xFF,
	.opposite = &i2c_addr_min,
};

static struct smart_attr linked_psu_num = {
	.name = "linked_psu_num",
	.value = 0,
	.min = 0x00000000,
	.max = 0xFFFFFFFF,
};

static struct smart_attr linked_peci_num = {
	.name = "linked_peci_num",
	.value = 0,
	.min = 0x00000000,
	.max = 0xFFFFFFFF,
};

static struct smart_attr *attr_range[] = {
	&max_overtemperature_time,
	&max_undervoltage_time,
	&overtemperature_throttling_time,
	&overcurrent_throttling_time,
	&undervoltage_throttling_time,
	&smart_psu_polling_interval_time,
	&smart_powergood_polling_interval_time,
	&force_smbalert_mask_interval_time,
	&irqs_serviced_num,
	&wq_calls_on_psu_registering_num,
	&wq_calls_on_enable_num,
	&psu_polls_num,
	&powergood_polls_num,
	&enable,
	&redundancy,
	&i2c_addr_min,
	&i2c_addr_max,
	&linked_psu_num,
	&linked_peci_num,
	NULL
};

static ssize_t smart_read_param(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t smart_write_param(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
				 size_t count);
static ssize_t smart_write_param_range(struct kobject *kobj, struct kobj_attribute *attr,
				       const char *buf, size_t count);
static ssize_t smart_write_redundancy(struct kobject *kobj, struct kobj_attribute *attr,
				      const char *buf, size_t count);
static ssize_t smart_read_status(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t smart_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			    size_t count);

static struct kobj_attribute max_overtemperature_time_attr =
	__ATTR(max_overtemperature_time, 0660, smart_read_param, smart_write_param);
static struct kobj_attribute max_undervoltage_time_attr =
	__ATTR(max_undervoltage_time, 0660, smart_read_param, smart_write_param);
static struct kobj_attribute overtemperature_throttling_time_attr =
	__ATTR(overtemperature_throttling_time, 0660, smart_read_param, smart_write_param);
static struct kobj_attribute overcurrent_throttling_time_attr =
	__ATTR(overcurrent_throttling_time, 0660, smart_read_param, smart_write_param);
static struct kobj_attribute undervoltage_throttling_time_attr =
	__ATTR(undervoltage_throttling_time, 0660, smart_read_param, smart_write_param);
static struct kobj_attribute smart_psu_polling_interval_time_attr =
	__ATTR(smart_psu_polling_interval_time, 0660, smart_read_param, smart_write_param);
static struct kobj_attribute force_smbalert_interval_time_attr =
	__ATTR(force_smbalert_mask_interval_time, 0660, smart_read_param, smart_write_param);
static struct kobj_attribute smart_powergood_polling_interval_time_attr =
	__ATTR(smart_powergood_polling_interval_time, 0660, smart_read_param, smart_write_param);
static struct kobj_attribute irqs_serviced_num_attr =
	__ATTR(irqs_serviced_num, 0440, smart_read_param, NULL);
static struct kobj_attribute wq_calls_on_psu_registering_num_attr =
	__ATTR(wq_calls_on_psu_registering_num, 0440, smart_read_param, NULL);
static struct kobj_attribute wq_calls_on_enable_num_attr =
	__ATTR(wq_calls_on_enable_num, 0440, smart_read_param, NULL);
static struct kobj_attribute psu_polls_num_attr =
	__ATTR(psu_polls_num, 0440, smart_read_param, NULL);
static struct kobj_attribute powergood_polls_num_attr =
	__ATTR(powergood_polls_num, 0440, smart_read_param, NULL);
static struct kobj_attribute enable_attr =
	__ATTR(enable, 0660, smart_read_param, smart_enable);
static struct kobj_attribute redundancy_attr =
	__ATTR(redundancy, 0660, smart_read_param, smart_write_redundancy);
static struct kobj_attribute i2c_addr_min_attr =
	__ATTR(i2c_addr_min, 0660, smart_read_param, smart_write_param_range);
static struct kobj_attribute i2c_addr_max_attr =
	__ATTR(i2c_addr_max, 0660, smart_read_param, smart_write_param_range);
static struct kobj_attribute status_attr =
	__ATTR(status, 0440, smart_read_status, NULL);

static struct kobj_attribute linked_psu_num_attr =
	__ATTR(linked_psu_num, 0440, smart_read_param, NULL);

static struct kobj_attribute linked_peci_num_attr =
	__ATTR(linked_peci_num, 0440, smart_read_param, NULL);

static struct attribute *smart_attrs[] = {
	&max_overtemperature_time_attr.attr,
	&max_undervoltage_time_attr.attr,
	&overtemperature_throttling_time_attr.attr,
	&overcurrent_throttling_time_attr.attr,
	&undervoltage_throttling_time_attr.attr,
	&smart_psu_polling_interval_time_attr.attr,
	&smart_powergood_polling_interval_time_attr.attr,
	&force_smbalert_interval_time_attr.attr,
	&irqs_serviced_num_attr.attr,
	&wq_calls_on_psu_registering_num_attr.attr,
	&wq_calls_on_enable_num_attr.attr,
	&psu_polls_num_attr.attr,
	&powergood_polls_num_attr.attr,
	&enable_attr.attr,
	&redundancy_attr.attr,
	&i2c_addr_min_attr.attr,
	&i2c_addr_max_attr.attr,
	&status_attr.attr,
	&linked_psu_num_attr.attr,
	&linked_peci_num_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(smart);

static bool smart_in_range(unsigned int value, unsigned int min, unsigned int max)
{
	return value >= min && value <= max;
}

static enum smart_status status = UNINITIALIZED;

static void smart_request_max_throttling(const struct list_head *list)
{
	struct smart_peci_entry *peci;
	int ret;

	list_for_each_entry(peci, list, node) {
		ret = peci->ops->request_max_power_throttling(peci->dev);
		if (ret)
			dev_dbg(peci->dev, "throttling enabling failed, ret: %d", ret);
	}

	smart_data->throttling_active = true;
}

static void smart_remove_max_throttling(const struct list_head *list)
{
	struct smart_peci_entry *peci;
	int ret;

	list_for_each_entry(peci, list, node) {
		ret = peci->ops->remove_max_power_throttling(peci->dev);
		if (ret)
			dev_dbg(peci->dev, "throttling disabling failed, ret: %d", ret);
	}

	smart_data->throttling_active = false;
}

static bool smart_is_throttling_active(void)
{
	return smart_data->throttling_active;
}

static bool smart_check_powergood(const struct i2c_client *client)
{
	int ret;

	ret = smart_read_word(client, PMBUS_STATUS_WORD);
	if (ret < 0)
		return false;

	return !(SMART_STATUS_WORD_POWERGOOD_MASK & ret);
}

static void smart_update_default_smbalert_mask(void)
{
	struct smart_psu_entry *psu;
	unsigned int mask;

	list_for_each_entry(psu, &smart_data->psu_list, node) {
		if (enable.value) {
			psu->overtemperature_masked_default = false;
			psu->undervoltage_masked_default = false;
			psu->overcurrent_masked_default = false;
		} else {
			psu->overtemperature_masked_default = true;
			psu->undervoltage_masked_default = true;
			psu->overcurrent_masked_default = true;
		}

		if (psu->dev) {
			mask = 0;
			mask |= (psu->powergood ? POWERGOOD_OK : 0);
			mask |= (redundancy.value ? REDUNDANCY_ENABLED : 0);
			mask |= (linked_psu_num.value  > 1 ? MORE_THAN_ONE_PSU : 0);
			mask |= (psu->unit_off ? UNIT_OFF : 0);

			switch (mask) {
			case REDUNDANCY_ENABLED | POWERGOOD_OK | MORE_THAN_ONE_PSU:
			case UNIT_OFF | MORE_THAN_ONE_PSU:
				psu->undervoltage_masked_default = true;
				break;
			case REDUNDANCY_ENABLED | MORE_THAN_ONE_PSU:
				psu->overtemperature_masked_default = true;
				psu->undervoltage_masked_default = true;
				psu->overcurrent_masked_default = true;
				break;
			default:
				/* according to psu redundancy, no mask needed */
				break;
			}
		}
	}
}

static int smart_write_smbalert_mask(const struct i2c_client *client, u8 command,
				     u8 mask)
{
	u16 data = command | (mask << 8);

	return smart_write_word(client, PMBUS_SMBALERT_MASK, data);
}

static void smart_apply_default_smbalert_mask(void)
{
	struct smart_psu_entry *psu;
	struct i2c_client *i2c;

	list_for_each_entry(psu, &smart_data->psu_list, node) {
		i2c = to_i2c_client(psu->dev);

		if (psu->undervoltage_masked_default) {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_INPUT,
						  SMART_STATUS_MASK_ALL);
			psu->undervoltage_masked = true;
			dev_dbg(psu->dev, "mask UV");
		} else {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_INPUT,
						  ~SMART_STATUS_INPUT_UV_FAULT_MASK);
			psu->undervoltage_masked = false;
			dev_dbg(psu->dev, "unmask UV");
		}

		if (psu->overcurrent_masked_default) {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_IOUT,
						  SMART_STATUS_MASK_ALL);
			psu->overcurrent_masked = true;
			dev_dbg(psu->dev, "mask OC");
		} else {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_IOUT,
						  ~SMART_STATUS_IOUT_OC_WARNING_MASK);
			psu->overcurrent_masked = false;
			dev_dbg(psu->dev, "unmask OC");
		}

		if (psu->overtemperature_masked_default) {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_TEMPERATURE,
						  SMART_STATUS_MASK_ALL);
			psu->overtemperature_masked = true;
			dev_dbg(psu->dev, "mask OT");
		} else {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_TEMPERATURE,
						  ~SMART_STATUS_TEMPERATURE_OT_WARNING_MASK);
			psu->overtemperature_masked = false;
			dev_dbg(psu->dev, "unmask OT");
		}
	}
}

static void smart_update_and_apply_default_smbalert_mask(void)
{
	smart_update_default_smbalert_mask();
	smart_apply_default_smbalert_mask();
}

static void smart_enable_smbalert_generation(void)
{
	struct smart_psu_entry *psu;

	smart_update_and_apply_default_smbalert_mask();

	list_for_each_entry(psu, &smart_data->psu_list, node) {
		psu->event_reported.all = PSU_EVENT_NO_EVENT;
		psu->current_event.all = PSU_EVENT_NO_EVENT;
		psu->uv_timestamp_saved = false;
		psu->ot_timestamp_saved = false;
	}
}

static void smart_log_start_event(void)
{
	if (smart_data->throttling_reason.input.bit.vin_uv_fault)
		dev_info(smart_data->dev, "SMART_EVENT,START,Undervoltage");
	else if (smart_data->throttling_reason.iout.bit.iout_oc_warning)
		dev_info(smart_data->dev, "SMART_EVENT,START,Overcurrent");
	else if (smart_data->throttling_reason.temperature.bit.ot_warning)
		dev_info(smart_data->dev, "SMART_EVENT,START,Overtemperature");
}

static void smart_log_stop_event(void)
{
	if (smart_data->throttling_reason.input.bit.vin_uv_fault)
		dev_info(smart_data->dev, "SMART_EVENT,STOP,Undervoltage");
	else if (smart_data->throttling_reason.iout.bit.iout_oc_warning)
		dev_info(smart_data->dev, "SMART_EVENT,STOP,Overcurrent");
	else if (smart_data->throttling_reason.temperature.bit.ot_warning)
		dev_info(smart_data->dev, "SMART_EVENT,STOP,Overtemperature");
}

static bool smart_is_error(union smart_psu_event event, struct smart_psu_entry *psu)
{
	return (event.input.bit.vin_uv_fault && !event.input.bit.unit_off_low_voltage) ||
	       event.iout.bit.iout_oc_warning ||
	       event.temperature.bit.ot_warning;
}

static bool smart_check_status_after_iteration(void)
{
	struct smart_psu_entry *psu;
	bool fault_present = false;

	list_for_each_entry(psu, &smart_data->psu_list, node) {
		if (smart_is_error(psu->current_event, psu)) {
			fault_present = true;
		} else if (psu->event_reported.input.bit.unit_off_low_voltage && !psu->unit_off) {
			dev_dbg(psu->dev, "additional throttling needed due to PSU off");
			psu->event_reported.all = PSU_EVENT_NO_EVENT;
			psu->unit_off = true;
			psu->powergood = false;
			psu->timer = undervoltage_throttling_time.value;
			psu->additional_throttling_timestamp = ktime_get();
		} else if (psu->event_reported.input.bit.vin_uv_fault && !psu->unit_off) {
			dev_dbg(psu->dev, "additional throttling needed due to UV");
			psu->event_reported.all = PSU_EVENT_NO_EVENT;
			psu->timer = undervoltage_throttling_time.value;
			psu->additional_throttling_timestamp = ktime_get();
		} else if (psu->event_reported.iout.bit.iout_oc_warning) {
			dev_dbg(psu->dev, "additional throttling needed due to OC");
			psu->event_reported.all = PSU_EVENT_NO_EVENT;
			psu->timer = overcurrent_throttling_time.value;
			psu->additional_throttling_timestamp = ktime_get();
		} else if (psu->event_reported.temperature.bit.ot_warning) {
			dev_dbg(psu->dev, "additional throttling needed due to OT");
			psu->event_reported.all = PSU_EVENT_NO_EVENT;
			psu->timer = overtemperature_throttling_time.value;
			psu->additional_throttling_timestamp = ktime_get();
		}

		if (ktime_before(ktime_get(),
				 ktime_add_ms(psu->additional_throttling_timestamp, psu->timer)))
			smart_data->additional_throttling_not_done = true;
	}

	return fault_present;
}

static void smart_check_ot_time(struct smart_psu_entry *psu, struct list_head *list)
{
	struct smart_ot_psu_list_entry *list_entry;

	if (psu->ot_timestamp_saved && !psu->overtemperature_masked &&
	    (ktime_after(ktime_get(),
	     ktime_add_ms(psu->ot_start_timestamp, max_overtemperature_time.value)))) {
		list_entry = kzalloc(sizeof(*list_entry), GFP_KERNEL);
		if (list_entry) {
			dev_dbg(psu->dev, "OT Warning time elapsed exceeded threshold");
			list_entry->psu = psu;
			list_add(&list_entry->node, list);
		}
	}
}

static bool smart_is_psu_in_fault_state(struct smart_psu_entry *psu, union smart_psu_event event)
{
	return event.input.bit.unit_off_low_voltage || event.word.bit.vout ||
	    (psu->uv_timestamp_saved &&
	     ktime_after(ktime_get(),
			 ktime_add_ms(psu->uv_start_timestamp, max_undervoltage_time.value)));
}

static bool smart_is_any_psu_in_fault_state(void)
{
	struct smart_psu_entry *psu;

	list_for_each_entry(psu, &smart_data->psu_list, node) {
		if (psu->fault_state)
			return true;
	}

	return false;
}

static void smart_handle_uv_event(struct smart_psu_entry *psu, union smart_psu_event *event)
{
	struct i2c_client *i2c = to_i2c_client(psu->dev);

	if (smart_is_psu_in_fault_state(psu, *event)) {
		psu->fault_state = true;
		if (!psu->overtemperature_masked) {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_TEMPERATURE,
						  SMART_STATUS_MASK_ALL);
			psu->overtemperature_masked = true;
			dev_dbg(psu->dev, "mask OT");
			event->temperature.all = 0;
		}
		if (!psu->overcurrent_masked) {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_IOUT, SMART_STATUS_MASK_ALL);
			psu->overcurrent_masked = true;
			dev_dbg(psu->dev, "mask OC");
			event->iout.all = 0;
		}
		if (!psu->undervoltage_masked) {
			smart_write_smbalert_mask(i2c, PMBUS_STATUS_INPUT, SMART_STATUS_MASK_ALL);
			psu->undervoltage_masked = true;
			dev_dbg(psu->dev, "mask UV");
			event->input.all = 0;
		}
	} else {
		psu->fault_state = false;
	}
}

static void smart_handle_ot_event(const struct list_head *list)
{
	struct smart_ot_psu_list_entry *psu_entry, *tmp;
	struct i2c_client *i2c;

	if (!list_is_singular(list) || linked_psu_num.value < 2 || !redundancy.value)
		goto delete_entries;

	psu_entry = list_first_entry(list, struct smart_ot_psu_list_entry, node);
	i2c = to_i2c_client(psu_entry->psu->dev);
	dev_dbg(psu_entry->psu->dev, "mask OT");
	psu_entry->psu->overtemperature_masked = true;
	smart_write_smbalert_mask(i2c, PMBUS_STATUS_TEMPERATURE, SMART_STATUS_MASK_ALL);

delete_entries:
	list_for_each_entry_safe(psu_entry, tmp, list, node) {
		list_del(&psu_entry->node);
		kfree(psu_entry);
	}
}

static void smart_log_smbalert(union smart_psu_event events_ored)
{
	if (!smart_data->event_logged) {
		smart_data->throttling_reason = events_ored;
		smart_log_start_event();
		smart_data->event_logged = true;
	}
}

static void smart_schedule_poll_or_remove_throttling(const ktime_t psu_poll_start_timestamp)
{
	if (smart_check_status_after_iteration() ||
	    smart_data->additional_throttling_not_done) {
		s64 diff = ktime_ms_delta(ktime_get(), psu_poll_start_timestamp);

		dev_dbg(smart_data->dev, "smart_psu_polling execution time: %lld [ms]", diff);
		if (diff >= smart_psu_polling_interval_time.value) {
			queue_delayed_work(smart_data->workqueue, &smart_data->work, 0);
		} else {
			queue_delayed_work(smart_data->workqueue, &smart_data->work,
					   msecs_to_jiffies(smart_psu_polling_interval_time.value -
							    diff));
		}
	} else {
		if (smart_is_throttling_active()) {
			smart_log_stop_event();
			smart_remove_max_throttling(&smart_data->peci_list);
		}

		if (smart_is_any_psu_in_fault_state()) {
			s64 diff = ktime_ms_delta(ktime_get(), psu_poll_start_timestamp);

			dev_dbg(smart_data->dev, "smart_psu_polling execution time: %lld [ms]",
					diff);
			if (diff >= smart_psu_polling_interval_time.value) {
				queue_delayed_work(smart_data->workqueue, &smart_data->work, 0);
			} else {
				queue_delayed_work(smart_data->workqueue, &smart_data->work,
					msecs_to_jiffies(
						smart_psu_polling_interval_time.value - diff));
			}
		} else {
			status = IDLE;
			smart_data->event_logged = false;
			smart_enable_smbalert_generation();
			queue_delayed_work(smart_data->workqueue, &smart_data->powergood_work, 0);
		}
	}
}

static void smart_save_psu_event(struct smart_psu_entry *psu, union smart_psu_event event)
{
	if (!psu->uv_timestamp_saved && event.input.bit.vin_uv_fault) {
		psu->uv_start_timestamp = ktime_get();
		psu->uv_timestamp_saved = true;
	} else if (!event.input.bit.vin_uv_fault) {
		psu->uv_timestamp_saved = false;
	}

	if (!psu->ot_timestamp_saved && event.temperature.bit.ot_warning) {
		psu->ot_start_timestamp = ktime_get();
		psu->ot_timestamp_saved = true;
	} else if (!event.temperature.bit.ot_warning) {
		psu->ot_timestamp_saved = false;
	}

	psu->current_event.all = event.all;
	psu->event_reported.all |= event.all;
}

static void smart_psu_polling(struct work_struct *work)
{
	union smart_psu_event event, events;
	struct smart_psu_entry *psu;
	struct list_head list;
	bool is_first_poll = (status != INTERRUPT);
	ktime_t psu_poll_start_timestamp = ktime_get();

	INIT_LIST_HEAD(&list);

	mutex_lock(&smart_data->mutex);
	smart_data->additional_throttling_not_done = false;
	psu_polls_num.value++;
	status = INTERRUPT;
	events.all = PSU_EVENT_NO_EVENT;
	mutex_unlock(&smart_data->mutex);

	mutex_lock(&smart_data->device_access_mutex);

	smart_request_max_throttling(&smart_data->peci_list);

	list_for_each_entry(psu, &smart_data->psu_list, node) {
		if (!is_first_poll)
			smart_clear_events(psu);
		event = smart_read_event(psu);
		events.all |= event.all;

		smart_handle_uv_event(psu, &event);
		smart_save_psu_event(psu, event);
		smart_check_ot_time(psu, &list);
	}

	smart_handle_ot_event(&list);
	smart_log_smbalert(events);
	smart_schedule_poll_or_remove_throttling(psu_poll_start_timestamp);
	mutex_unlock(&smart_data->device_access_mutex);
}

static void smart_powergood_polling(struct work_struct *work)
{
	unsigned int force_update_num;
	bool need_update = false;
	struct i2c_client *i2c;
	struct list_head list;
	struct smart_psu_entry *psu;
	bool powergood;

	if (!enable.value || status == INTERRUPT)
		return;

	INIT_LIST_HEAD(&list);

	mutex_lock(&smart_data->device_access_mutex);
	list_for_each_entry(psu, &smart_data->psu_list, node) {
		i2c = to_i2c_client(psu->dev);
		powergood = smart_check_powergood(i2c);
		if (powergood != psu->powergood) {
			psu->powergood = powergood;
			need_update = true;
		}

		if (powergood)
			psu->unit_off = false;
	}
	mutex_unlock(&smart_data->device_access_mutex);

	mutex_lock(&smart_data->mutex);
	force_update_num = force_smbalert_mask_interval_time.value /
			smart_powergood_polling_interval_time.value;
	if (powergood_polls_num.value >= force_update_num) {
		powergood_polls_num.value = 0;
		need_update = true;
	}
	powergood_polls_num.value++;
	mutex_unlock(&smart_data->mutex);

	mutex_lock(&smart_data->device_access_mutex);
	if (need_update) {
		dev_dbg(smart_data->dev, "powergood poll update");
		smart_update_and_apply_default_smbalert_mask();
	}

	if (enable.value) {
		queue_delayed_work(smart_data->workqueue, &smart_data->powergood_work,
				   msecs_to_jiffies(smart_powergood_polling_interval_time.value));
	}
	mutex_unlock(&smart_data->device_access_mutex);
}

static irqreturn_t smart_smbalert_handler(int irq, void *private)
{
	irqs_serviced_num.value++;

	dev_dbg(smart_data->dev, "interrupt triggered");
	if (enable.value) {
		dev_dbg(smart_data->dev, "SmaRT enabled, kicking workqueue");
		cancel_delayed_work(&smart_data->powergood_work);
		queue_delayed_work(smart_data->workqueue, &smart_data->work, 0);
	}

	return IRQ_HANDLED;
}

int smart_unregister_peci(struct device *dev)
{
	struct smart_peci_entry *tmp, *peci;

	dev_dbg(dev, "unregister %s in SmaRT", dev_name(dev->parent));
	mutex_lock(&smart_data->device_access_mutex);

	list_for_each_entry_safe(peci, tmp, &smart_data->peci_list, node) {
		if (peci->dev == dev) {
			list_del(&peci->node);
			kfree(peci);
			linked_peci_num.value--;
			mutex_unlock(&smart_data->device_access_mutex);
			return 0;
		}
	}

	mutex_unlock(&smart_data->device_access_mutex);

	return -EINVAL;
}
EXPORT_SYMBOL(smart_unregister_peci);

int smart_register_peci(struct device *dev, struct peci_throttling_ops *peci_smart_ops)
{
	struct smart_peci_entry *new, *peci;

	mutex_lock(&smart_data->device_access_mutex);

	dev_dbg(dev, "register %s in SmaRT", dev_name(dev->parent));
	list_for_each_entry(peci, &smart_data->peci_list, node) {
		if (peci->dev == dev) {
			dev_dbg(dev, "already registered %s", dev_name(dev->parent));
			mutex_unlock(&smart_data->device_access_mutex);
			return -EPERM;
		}
	}

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new) {
		mutex_unlock(&smart_data->device_access_mutex);
		return -ENOMEM;
	}

	new->dev = dev;
	new->ops = peci_smart_ops;
	list_add(&new->node, &smart_data->peci_list);
	linked_peci_num.value++;

	mutex_unlock(&smart_data->device_access_mutex);

	return 0;
}
EXPORT_SYMBOL(smart_register_peci);

int smart_unregister_psu(struct device *dev)
{
	struct smart_psu_entry *tmp, *psu;

	dev_dbg(dev, "unregister in SmaRT");
	mutex_lock(&smart_data->device_access_mutex);

	list_for_each_entry_safe(psu, tmp, &smart_data->psu_list, node) {
		if (psu->dev == dev) {
			list_del(&psu->node);
			kfree(psu);
			linked_psu_num.value--;
			mutex_unlock(&smart_data->device_access_mutex);
			return 0;
		}
	}

	smart_update_and_apply_default_smbalert_mask();

	mutex_unlock(&smart_data->device_access_mutex);

	return -EINVAL;
}
EXPORT_SYMBOL(smart_unregister_psu);

int smart_register_psu(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct smart_psu_entry *new, *psu;
	union smart_psu_event event;

	dev_dbg(dev, "register in SmaRT");
	mutex_lock(&smart_data->device_access_mutex);

	list_for_each_entry(psu, &smart_data->psu_list, node) {
		if (psu->dev == dev) {
			dev_dbg(dev, "already registered %s", dev_name(dev->parent));
			mutex_unlock(&smart_data->device_access_mutex);
			return -EPERM;
		}
	}

	if (!smart_is_command_supported(i2c, PMBUS_PAGE_PLUS_WRITE) ||
	    !smart_is_command_supported(i2c, PMBUS_PAGE_PLUS_READ)) {
		dev_dbg(dev, "does not support PAGE_PLUS commands");
		mutex_unlock(&smart_data->device_access_mutex);
		return 0;
	}

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new) {
		mutex_unlock(&smart_data->device_access_mutex);
		return -ENOMEM;
	}
	new->dev = dev;
	list_add(&new->node, &smart_data->psu_list);
	linked_psu_num.value++;

	event = smart_read_event(new);
	if (smart_is_error(event, new)) {
		dev_dbg(new->dev, "has non-zero status, kicking workqueue");
		wq_calls_on_psu_registering_num.value++;
		queue_delayed_work(smart_data->workqueue, &smart_data->work, 0);
	}

	smart_update_and_apply_default_smbalert_mask();

	mutex_unlock(&smart_data->device_access_mutex);

	return 0;
}
EXPORT_SYMBOL(smart_register_psu);

static ssize_t smart_read_param(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; attr_range[i]; i++) {
		if (!strcmp(attr_range[i]->name, attr->attr.name)) {
			mutex_lock(&smart_data->mutex);
			ret = sprintf(buf, "%u\n", attr_range[i]->value);
			mutex_unlock(&smart_data->mutex);
		}
	}

	if (ret > 0)
		return ret;

	return -EBADF;
}

static ssize_t smart_read_status(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t size = -EPERM;

	mutex_lock(&smart_data->mutex);

	switch (status) {
	case UNINITIALIZED:
		size = sprintf(buf, "%s", "uninitialized\n");
		break;
	case NO_GPIO:
		size = sprintf(buf, "%s", "no_gpio\n");
		break;
	case IDLE:
		size = sprintf(buf, "%s", "idle\n");
		break;
	case INTERRUPT:
		size = sprintf(buf, "%s", "interrupt_handling\n");
		break;
	}

	mutex_unlock(&smart_data->mutex);

	return size;
}

static ssize_t smart_write_param(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
				 size_t count)
{
	unsigned int i, value;
	int ret;

	ret = kstrtou32(buf, 0, &value);
	if (ret)
		return -EINVAL;

	for (i = 0; attr_range[i]; i++) {
		if (!strcmp(attr_range[i]->name, attr->attr.name)) {
			if (smart_in_range(value, attr_range[i]->min, attr_range[i]->max)) {
				mutex_lock(&smart_data->mutex);
				attr_range[i]->value = value;
				mutex_unlock(&smart_data->mutex);
				return count;
			}

			return -EINVAL;
		}
	}

	return -EBADF;
}

static ssize_t smart_write_redundancy(struct kobject *kobj, struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;

	ret = smart_write_param(kobj, attr, buf, count);
	if (ret < 0)
		return ret;

	mutex_lock(&smart_data->device_access_mutex);
	smart_update_and_apply_default_smbalert_mask();
	mutex_unlock(&smart_data->device_access_mutex);

	return ret;
}

static ssize_t smart_write_param_range(struct kobject *kobj, struct kobj_attribute *attr,
				       const char *buf, size_t count)
{
	struct smart_attr *attr_def;
	unsigned int i, value;
	int ret;

	ret = kstrtou32(buf, 0, &value);
	if (ret)
		return -EINVAL;

	for (i = 0; attr_range[i]; i++) {
		if (!strcmp(attr_range[i]->name, attr->attr.name))
			attr_def = attr_range[i];
	}

	if (!attr_def->opposite)
		return -EINVAL;

	if (strstr(attr->attr.name, "min")) {
		if (value > attr_def->opposite->value)
			return -EINVAL;
	} else if (value < attr_def->opposite->value) {
		return -EINVAL;
	}

	mutex_lock(&smart_data->mutex);
	attr_def->value = value;
	mutex_unlock(&smart_data->mutex);

	return count;
}

static void smart_mask_all_events(void)
{
	struct smart_psu_entry *psu;

	list_for_each_entry(psu, &smart_data->psu_list, node) {
		smart_write_smbalert_mask(to_i2c_client(psu->dev), PMBUS_STATUS_TEMPERATURE,
					  SMART_STATUS_MASK_ALL);
		smart_write_smbalert_mask(to_i2c_client(psu->dev), PMBUS_STATUS_IOUT,
					  SMART_STATUS_MASK_ALL);
		smart_write_smbalert_mask(to_i2c_client(psu->dev), PMBUS_STATUS_INPUT,
					  SMART_STATUS_MASK_ALL);
		dev_dbg(psu->dev, "masked all events");
		psu->overtemperature_masked = true;
		psu->undervoltage_masked = true;
		psu->overcurrent_masked = true;
	}
}

static ssize_t smart_enable(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			    size_t count)
{
	union smart_psu_event event;
	struct smart_psu_entry *psu;
	unsigned int value;
	bool is_error = false;
	int ret;

	ret = kstrtou32(buf, 0, &value);
	if (ret)
		return -EINVAL;

	ret = smart_write_param(kobj, attr, buf, count);
	if (ret < 0)
		return ret;

	mutex_lock(&smart_data->device_access_mutex);
	if (value) {
		smart_enable_smbalert_generation();
		list_for_each_entry(psu, &smart_data->psu_list, node) {
			event = smart_read_event(psu);
			is_error |= smart_is_error(event, psu);
		}
		queue_delayed_work(smart_data->workqueue, &smart_data->powergood_work, 0);
	} else {
		smart_mask_all_events();
	}
	mutex_unlock(&smart_data->device_access_mutex);

	if (is_error) {
		dev_dbg(smart_data->dev, "detected PSU(s) with non-zero status, kicking workqueue");
		mutex_lock(&smart_data->device_access_mutex);
		wq_calls_on_enable_num.value++;
		mutex_unlock(&smart_data->device_access_mutex);
		queue_delayed_work(smart_data->workqueue, &smart_data->work, 0);
	}

	return ret;
}

static int smart_probe(struct platform_device *pdev)
{
	int irq_num, ret;

	smart_data = devm_kzalloc(&pdev->dev, sizeof(*smart_data), GFP_KERNEL);
	if (!smart_data)
		return -ENOMEM;

	smart_data->dev = &pdev->dev;
	INIT_LIST_HEAD(&smart_data->psu_list);
	INIT_LIST_HEAD(&smart_data->peci_list);
	mutex_init(&smart_data->mutex);
	mutex_init(&smart_data->device_access_mutex);

	INIT_DELAYED_WORK(&smart_data->work, smart_psu_polling);
	INIT_DELAYED_WORK(&smart_data->powergood_work, smart_powergood_polling);
	smart_data->workqueue = create_workqueue("smart_workqueue");
	if (!smart_data->workqueue) {
		dev_err(&pdev->dev, "creating workqueue failed\n");
		return -ENOMEM;
	}

	smart_data->gpio = devm_gpiod_get(&pdev->dev, "smbalert", GPIOD_IN);
	if (IS_ERR(smart_data->gpio)) {
		dev_err(&pdev->dev, "cannot obtain GPIO\n");
		status = NO_GPIO;
		return PTR_ERR(smart_data->gpio);
	}

	irq_num = gpiod_to_irq(smart_data->gpio);
	if (irq_num < 0) {
		dev_err(&pdev->dev, "GPIO cannot be mapped to IRQ number\n");
		return irq_num;
	}

	ret = devm_request_irq(&pdev->dev, irq_num, smart_smbalert_handler,
			       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			       "smbalert_event", smart_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot setup IRQ\n");
		return ret;
	}

	if (enable.value)
		queue_delayed_work(smart_data->workqueue, &smart_data->powergood_work, 0);

	status = IDLE;

	return 0;
}

static int smart_remove(struct platform_device *pdev)
{
	cancel_delayed_work(&smart_data->work);
	cancel_delayed_work(&smart_data->powergood_work);
	flush_workqueue(smart_data->workqueue);
	destroy_workqueue(smart_data->workqueue);
	status = UNINITIALIZED;

	return 0;
}

static const struct of_device_id match[] = {
	{.compatible = "intel,smart"},
	{}
};

static struct platform_driver smart_driver = {
	.probe = smart_probe,
	.remove = smart_remove,
	.driver = {
		.name = "smart",
		.owner = THIS_MODULE,
		.of_match_table = match,
		.dev_groups = smart_groups,
	}
};

module_platform_driver(smart_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Agata Wiatrowska <agata.wiatrowska@intel.com>");
MODULE_AUTHOR("Wojciech Dembinski <wojciech.dembinski@intel.com>");
MODULE_DESCRIPTION("SmaRT & CLST module");
MODULE_VERSION("1.0");
MODULE_IMPORT_NS(PMBUS);
