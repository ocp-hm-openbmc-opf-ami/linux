// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Intel Corporation.*/

#include <linux/ktime.h>
#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include <linux/i3c/device.h>
#include <linux/i3c/master.h>

#define I3C_HUB_TP_MAX_COUNT				0x08

#define I3C_HUB_LOGICAL_BUS_MAX_COUNT			0x08

/* I3C HUB REGISTERS */

/*
 * In this driver Controller - Target convention is used. All the abbreviations are
 * based on this convention. For instance: CP - Controller Port, TP - Target Port.
 */

/* Device Information Registers */
#define I3C_HUB_DEV_INFO_0				0x00
#define I3C_HUB_DEV_INFO_1				0x01
#define I3C_HUB_PID_5					0x02
#define I3C_HUB_PID_4					0x03
#define I3C_HUB_PID_3					0x04
#define I3C_HUB_PID_2					0x05
#define I3C_HUB_PID_1					0x06
#define I3C_HUB_PID_0					0x07
#define I3C_HUB_BCR					0x08
#define I3C_HUB_DCR					0x09
#define I3C_HUB_DEV_CAPAB				0x0A
#define I3C_HUB_DEV_REV					0x0B

/* Device Configuration Registers */
#define I3C_HUB_PROTECTION_CODE				0x10
#define  REGISTERS_LOCK_CODE				0x00
#define  REGISTERS_UNLOCK_CODE				0x69
#define  CP1_REGISTERS_UNLOCK_CODE			0x6A

#define I3C_HUB_CP_CONF					0x11
#define I3C_HUB_TP_ENABLE				0x12
#define  TPn_ENABLE(n)					BIT(n)

#define I3C_HUB_DEV_CONF				0x13
#define I3C_HUB_IO_STRENGTH				0x14
#define  TP0145_IO_STRENGTH_MASK			GENMASK(1, 0)
#define  TP0145_IO_STRENGTH(x)				(((x) << 0) & TP0145_IO_STRENGTH_MASK)
#define  TP2367_IO_STRENGTH_MASK			GENMASK(3, 2)
#define  TP2367_IO_STRENGTH(x)				(((x) << 2) & TP2367_IO_STRENGTH_MASK)
#define  CP0_IO_STRENGTH_MASK				GENMASK(5, 4)
#define  CP0_IO_STRENGTH(x)				(((x) << 4) & CP0_IO_STRENGTH_MASK)
#define  CP1_IO_STRENGTH_MASK				GENMASK(7, 6)
#define  CP1_IO_STRENGTH(x)				(((x) << 6) & CP1_IO_STRENGTH_MASK)
#define  IO_STRENGTH_20_OHM				0x00
#define  IO_STRENGTH_30_OHM				0x01
#define  IO_STRENGTH_40_OHM				0x02
#define  IO_STRENGTH_50_OHM				0x03

#define I3C_HUB_NET_OPER_MODE_CONF			0x15
#define I3C_HUB_LDO_CONF				0x16
#define  CP0_LDO_VOLTAGE_MASK				GENMASK(1, 0)
#define  CP0_LDO_VOLTAGE(x)				(((x) << 0) & CP0_LDO_VOLTAGE_MASK)
#define  CP1_LDO_VOLTAGE_MASK				GENMASK(3, 2)
#define  CP1_LDO_VOLTAGE(x)				(((x) << 2) & CP1_LDO_VOLTAGE_MASK)
#define  TP0145_LDO_VOLTAGE_MASK			GENMASK(5, 4)
#define  TP0145_LDO_VOLTAGE(x)				(((x) << 4) & TP0145_LDO_VOLTAGE_MASK)
#define  TP2367_LDO_VOLTAGE_MASK			GENMASK(7, 6)
#define  TP2367_LDO_VOLTAGE(x)				(((x) << 6) & TP2367_LDO_VOLTAGE_MASK)
#define  LDO_VOLTAGE_1_0V				0x00
#define  LDO_VOLTAGE_1_1V				0x01
#define  LDO_VOLTAGE_1_2V				0x02
#define  LDO_VOLTAGE_1_8V				0x03

#define I3C_HUB_TP_IO_MODE_CONF				0x17
#define I3C_HUB_TP_SMBUS_AGNT_EN			0x18
#define  TPn_SMBUS_MODE_EN(n)				BIT(n)

#define I3C_HUB_LDO_AND_PULLUP_CONF			0x19
#define  LDO_ENABLE_DISABLE_MASK			GENMASK(3, 0)
#define  CP0_LDO_EN					BIT(0)
#define  CP1_LDO_EN					BIT(1)
/*
 * I3C HUB does not provide a way to control LDO or pull-up for individual ports. It is possible
 * for group of ports TP0/TP1/TP4/TP5 and TP2/TP3/TP6/TP7.
 */
#define  TP0145_LDO_EN					BIT(2)
#define  TP2367_LDO_EN					BIT(3)
#define  TP0145_PULLUP_CONF_MASK			GENMASK(7, 6)
#define  TP0145_PULLUP_CONF(x)				(((x) << 6) & TP0145_PULLUP_CONF_MASK)
#define  TP2367_PULLUP_CONF_MASK			GENMASK(5, 4)
#define  TP2367_PULLUP_CONF(x)				(((x) << 4) & TP2367_PULLUP_CONF_MASK)
#define  PULLUP_250R					0x00
#define  PULLUP_500R					0x01
#define  PULLUP_1K					0x02
#define  PULLUP_2K					0x03

#define I3C_HUB_CP_IBI_CONF				0x1A
#define I3C_HUB_TP_IBI_CONF				0x1B
#define I3C_HUB_IBI_MDB_CUSTOM				0x1C
#define I3C_HUB_JEDEC_CONTEXT_ID			0x1D
#define I3C_HUB_TP_GPIO_MODE_EN				0x1E
#define  TPn_GPIO_MODE_EN(n)				BIT(n)

/* Device Status and IBI Registers */
#define I3C_HUB_DEV_AND_IBI_STS				0x20
#define I3C_HUB_TP_SMBUS_AGNT_IBI_STS			0x21

/* Controller Port Control/Status Registers */
#define I3C_HUB_CP_MUX_SET				0x38
#define  CONTROLLER_PORT_MUX_REQ			BIT(0)
#define I3C_HUB_CP_MUX_STS				0x39
#define  CONTROLLER_PORT_MUX_CONNECTION_STATUS		BIT(0)

/* Target Ports Control Registers */
#define I3C_HUB_TP_SMBUS_AGNT_TRANS_START		0x50
#define  I3C_HUB_TP_SMBUS_AGNTx_TRANS_START_REQ(x)	BIT(x)
#define I3C_HUB_TP_NET_CON_CONF				0x51
#define  TPn_NET_CON(n)					BIT(n)

#define I3C_HUB_TP_PULLUP_EN				0x53
#define  TPn_PULLUP_EN(n)				BIT(n)

#define I3C_HUB_TP_SCL_OUT_EN				0x54
#define I3C_HUB_TP_SDA_OUT_EN				0x55
#define I3C_HUB_TP_SCL_OUT_LEVEL			0x56
#define I3C_HUB_TP_SDA_OUT_LEVEL			0x57
#define I3C_HUB_TP_IN_DETECT_MODE_CONF			0x58
#define I3C_HUB_TP_SCL_IN_DETECT_IBI_EN			0x59
#define I3C_HUB_TP_SDA_IN_DETECT_IBI_EN			0x5A

/* Target Ports Status Registers */
#define I3C_HUB_TP_SCL_IN_LEVEL_STS			0x60
#define I3C_HUB_TP_SDA_IN_LEVEL_STS			0x61
#define I3C_HUB_TP_SCL_IN_DETECT_FLG			0x62
#define I3C_HUB_TP_SDA_IN_DETECT_FLG			0x63

/* SMBus Agent Configuration and Status Registers */
#define I3C_HUB_TPx_SMBUS_AGNT_STS(x)			(0x64 + (x))
#define  I3C_HUB_TP_SMBUS_AGNT_STS_TRANS_FINISHED_MASK	BIT(0)
#define  I3C_HUB_TP_SMBUS_AGNT_STS_BUF0_RECEIVED_MASK	BIT(1)
#define  I3C_HUB_TP_SMBUS_AGNT_STS_BUF1_RECEIVED_MASK	BIT(2)
#define  I3C_HUB_TP_SMBUS_AGNT_STS_BUF_OVERFLOW_MASK	BIT(3)
#define  I3C_HUB_TP_SMBUS_AGNT_STS_ALL_MASK	\
						(I3C_HUB_TP_SMBUS_AGNT_STS_BUF0_RECEIVED_MASK | \
						I3C_HUB_TP_SMBUS_AGNT_STS_BUF1_RECEIVED_MASK | \
						I3C_HUB_TP_SMBUS_AGNT_STS_BUF_OVERFLOW_MASK | \
						I3C_HUB_TP_SMBUS_AGNT_STS_TRANS_FINISHED_MASK)
#define  I3C_HUB_TP_SMBUS_AGNT_STS_TRANS_CODE_MASK	0xF0
#define  I3C_HUB_TP_TRANSACTION_CODE_GET(x)		\
		(((x) & I3C_HUB_TP_SMBUS_AGNT_STS_TRANS_CODE_MASK) >> 4)
#define  I3C_HUB_TP_SMBUS_AGNT_STS_TRANS_CODE_SUCCESS	0x00

#define I3C_HUB_ONCHIP_TD_AND_SMBUS_AGNT_CONF		0x6C

/* Special Function Registers */
#define I3C_HUB_LDO_AND_CPSEL_STS			0x79
#define  CP_SDA1_LEVEL					BIT(7)
#define  CP_SCL1_LEVEL					BIT(6)
#define  CP_SEL_PIN_INPUT_CODE_MASK			GENMASK(5, 4)
#define  CP_SEL_PIN_INPUT_CODE_GET(x)			(((x) & CP_SEL_PIN_INPUT_CODE_MASK) >> 4)
#define  CP_SDA1_SCL1_PINS_CODE_MASK			GENMASK(7, 6)
#define  CP_SDA1_SCL1_PINS_CODE_GET(x)			(((x) & CP_SDA1_SCL1_PINS_CODE_MASK) >> 6)
#define  VCCIO1_PWR_GOOD				BIT(3)
#define  VCCIO0_PWR_GOOD				BIT(2)
#define  CP1_VCCIO_PWR_GOOD				BIT(1)
#define  CP0_VCCIO_PWR_GOOD				BIT(0)

#define I3C_HUB_BUS_RESET_SCL_TIMEOUT			0x7A
#define I3C_HUB_ONCHIP_TD_PROTO_ERR_FLG			0x7B

#define I3C_HUB_DEV_CMD					0x7C
#define I3C_HUB_DEV_CMD_LOCK_ALL_PROT_REGS		0x53

#define I3C_HUB_ONCHIP_TD_STS				0x7D
#define I3C_HUB_ONCHIP_TD_ADDR_CONF			0x7E
#define I3C_HUB_PAGE_PTR				0x7F

/* LDO Disable/Enable DT settings */
#define I3C_HUB_DT_LDO_DISENA_ENABLED			0x01
#define I3C_HUB_DT_LDO_DISENA_DISABLED			0x00
#define I3C_HUB_DT_LDO_DISENA_NOT_DEFINED		0xFF

/* LDO Voltage DT settings */
#define I3C_HUB_DT_LDO_VOLT_1_0V			0x01
#define I3C_HUB_DT_LDO_VOLT_1_1V			0x02
#define I3C_HUB_DT_LDO_VOLT_1_2V			0x03
#define I3C_HUB_DT_LDO_VOLT_1_8V			0x04
#define I3C_HUB_DT_LDO_VOLT_NOT_DEFINED			0xFF

/* Paged Transaction Registers */
#define I3C_HUB_CONTROLLER_BUFFER_PAGE			0x10
#define I3C_HUB_CONTROLLER_AGENT_BUFF			0x80
#define I3C_HUB_CONTROLLER_AGENT_BUFF_DATA		0x84
#define I3C_HUB_TARGET_BUFF_LENGTH			0x80
#define I3C_HUB_TARGET_BUFF_ADDRESS			0x81
#define I3C_HUB_TARGET_BUFF_DATA			0x82

/* Pull-up DT settings */
#define I3C_HUB_DT_PULLUP_DISABLED			0x00
#define I3C_HUB_DT_PULLUP_250R				0x01
#define I3C_HUB_DT_PULLUP_500R				0x02
#define I3C_HUB_DT_PULLUP_1K				0x03
#define I3C_HUB_DT_PULLUP_2K				0x04
#define I3C_HUB_DT_PULLUP_NOT_DEFINED			0xFF

/* TP DT setting */
#define I3C_HUB_DT_TP_MODE_DISABLED			0x00
#define I3C_HUB_DT_TP_MODE_I3C				0x01
#define I3C_HUB_DT_TP_MODE_I3C_PERF			0x02
#define I3C_HUB_DT_TP_MODE_SMBUS			0x03
#define I3C_HUB_DT_TP_MODE_GPIO				0x04
#define I3C_HUB_DT_TP_MODE_NOT_DEFINED			0xFF

/* TP pull-up status */
#define I3C_HUB_DT_TP_PULLUP_DISABLED			0x00
#define I3C_HUB_DT_TP_PULLUP_ENABLED			0x01
#define I3C_HUB_DT_TP_PULLUP_NOT_DEFINED		0xFF

/* CP/TP IO strength */
#define I3C_HUB_DT_IO_STRENGTH_20_OHM			0x00
#define I3C_HUB_DT_IO_STRENGTH_30_OHM			0x01
#define I3C_HUB_DT_IO_STRENGTH_40_OHM			0x02
#define I3C_HUB_DT_IO_STRENGTH_50_OHM			0x03
#define I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED		0xFF

/* SMBus polling */
#define I3C_HUB_POLLING_ROLL_PERIOD_MS			10

/* SMBus Transaction Descriptor definitions */
/* Descriptor byte 0 */
#define I3C_HUB_SMBUS_RNW_TRANSACTION			BIT(0)

/* Descriptor byte 1 */
#define I3C_HUB_SMBUS_WRITE_AND_READ_TRANSACTION	BIT(0)
#define I3C_HUB_SMBUS_400kHz				BIT(2)

/* Hub buffer size */
#define I3C_HUB_CONTROLLER_BUFFER_SIZE			88
#define I3C_HUB_TARGET_BUFFER_SIZE			80
#define I3C_HUB_SMBUS_DESCRIPTOR_SIZE			4
#define I3C_HUB_SMBUS_PAYLOAD_SIZE			(I3C_HUB_CONTROLLER_BUFFER_SIZE - \
							I3C_HUB_SMBUS_DESCRIPTOR_SIZE)
#define I3C_HUB_SMBUS_TARGET_PAYLOAD_SIZE		(I3C_HUB_TARGET_BUFFER_SIZE - 2)

/* Hub SMBus timeout time period in nanoseconds */
#define I3C_HUB_SMBUS_400kHz_TIMEOUT			(10e9 * 8 * \
							I3C_HUB_CONTROLLER_BUFFER_SIZE / 4e5)

/* ID Extraction */
#define I3C_HUB_ID_CP_SDA_SCL				0x00
#define I3C_HUB_ID_CP_SEL				0x01

struct tp_setting {
	u8 mode;
	u8 pullup_en;
};

struct dt_settings {
	u8 cp0_ldo_en;
	u8 cp1_ldo_en;
	u8 cp0_ldo_volt;
	u8 cp1_ldo_volt;
	u8 tp0145_ldo_en;
	u8 tp2367_ldo_en;
	u8 tp0145_ldo_volt;
	u8 tp2367_ldo_volt;
	u8 tp0145_pullup;
	u8 tp2367_pullup;
	u8 cp0_io_strength;
	u8 cp1_io_strength;
	u8 tp0145_io_strength;
	u8 tp2367_io_strength;
	struct tp_setting tp[I3C_HUB_TP_MAX_COUNT];
};

struct i2c_adapter_group {
	u8 tp_mask;
	u8 tp_port;
	u8 used;

	struct delayed_work delayed_work_polling;
	struct i2c_client *client;
	const char *compatible;
	u8 polling_last_status;
	int addr;
};

struct logical_bus {
	bool available; /* Indicates that logical bus configuration is available in DT. */
	bool registered; /* Indicates that logical bus was registered in the framework. */
	u8 tp_map;
	struct i3c_master_controller controller;
	struct i2c_adapter_group smbus_port_adapter;
	struct device_node *of_node;
	struct i3c_hub *priv;
};

struct i3c_hub {
	struct i3c_device *i3cdev;
	struct i3c_master_controller *controller;
	struct regmap *regmap;
	struct dt_settings settings;
	struct delayed_work delayed_work;
	int hub_pin_sel_id;
	int hub_pin_cp1_id;
	int hub_dt_sel_id;
	int hub_dt_cp1_id;

	struct logical_bus logical_bus[I3C_HUB_LOGICAL_BUS_MAX_COUNT];

	/* Offset for reading HUB's register. */
	u8 reg_addr;
	struct dentry *debug_dir;
};

struct hub_setting {
	const char * const name;
	const u8 value;
};

static const struct hub_setting ldo_en_settings[] = {
	{"disabled",	I3C_HUB_DT_LDO_DISENA_DISABLED},
	{"enabled",	I3C_HUB_DT_LDO_DISENA_ENABLED},
};

static const struct hub_setting ldo_volt_settings[] = {
	{"1.0V",	I3C_HUB_DT_LDO_VOLT_1_0V},
	{"1.1V",	I3C_HUB_DT_LDO_VOLT_1_1V},
	{"1.2V",	I3C_HUB_DT_LDO_VOLT_1_2V},
	{"1.8V",	I3C_HUB_DT_LDO_VOLT_1_8V},
};

static const struct hub_setting pullup_settings[] = {
	{"disabled",	I3C_HUB_DT_PULLUP_DISABLED},
	{"250R",	I3C_HUB_DT_PULLUP_250R},
	{"500R",	I3C_HUB_DT_PULLUP_500R},
	{"1k",		I3C_HUB_DT_PULLUP_1K},
	{"2k",		I3C_HUB_DT_PULLUP_2K},
};

static const struct hub_setting tp_mode_settings[] = {
	{"disabled",	I3C_HUB_DT_TP_MODE_DISABLED},
	{"i3c",		I3C_HUB_DT_TP_MODE_I3C},
	{"i3c-perf",	I3C_HUB_DT_TP_MODE_I3C_PERF},
	{"smbus",	I3C_HUB_DT_TP_MODE_SMBUS},
	{"gpio",	I3C_HUB_DT_TP_MODE_GPIO},
};

static const struct hub_setting tp_pullup_settings[] = {
	{"disabled",	I3C_HUB_DT_TP_PULLUP_DISABLED},
	{"enabled",	I3C_HUB_DT_TP_PULLUP_ENABLED},
};

static const struct hub_setting io_strength_settings[] = {
	{"20Ohms",	I3C_HUB_DT_IO_STRENGTH_20_OHM},
	{"30Ohms",	I3C_HUB_DT_IO_STRENGTH_30_OHM},
	{"40Ohms",	I3C_HUB_DT_IO_STRENGTH_40_OHM},
	{"50Ohms",	I3C_HUB_DT_IO_STRENGTH_50_OHM},
};

static u8 i3c_hub_ldo_dt_to_reg(u8 dt_value)
{
	switch (dt_value) {
	case I3C_HUB_DT_LDO_VOLT_1_1V:
		return LDO_VOLTAGE_1_1V;
	case I3C_HUB_DT_LDO_VOLT_1_2V:
		return LDO_VOLTAGE_1_2V;
	case I3C_HUB_DT_LDO_VOLT_1_8V:
		return LDO_VOLTAGE_1_8V;
	default:
		return LDO_VOLTAGE_1_0V;
	}
}

static u8 i3c_hub_pullup_dt_to_reg(u8 dt_value)
{
	switch (dt_value) {
	case I3C_HUB_DT_PULLUP_250R:
		return PULLUP_250R;
	case I3C_HUB_DT_PULLUP_500R:
		return PULLUP_500R;
	case I3C_HUB_DT_PULLUP_1K:
		return PULLUP_1K;
	default:
		return PULLUP_2K;
	}
}

static u8 i3c_hub_io_strength_dt_to_reg(u8 dt_value)
{
	switch (dt_value) {
	case I3C_HUB_DT_IO_STRENGTH_50_OHM:
		return IO_STRENGTH_50_OHM;
	case I3C_HUB_DT_IO_STRENGTH_40_OHM:
		return IO_STRENGTH_40_OHM;
	case I3C_HUB_DT_IO_STRENGTH_30_OHM:
		return IO_STRENGTH_30_OHM;
	default:
		return IO_STRENGTH_20_OHM;
	}
}

static void i3c_hub_of_get_setting(struct device *dev, const struct device_node *node,
				   const char *setting_name, const struct hub_setting settings[],
				   const u8 settings_count, u8 *setting_value)
{
	const char *sval;
	int ret;
	int i;

	ret = of_property_read_string(node, setting_name, &sval);
	if (ret) {
		if (ret != -EINVAL) /* Lack of property is not considered as a problem. */
			dev_warn(dev, "No setting or invalid setting for %s, err=%i\n",
				 setting_name, ret);
		return;
	}

	for (i = 0; i < settings_count; ++i) {
		const struct hub_setting * const setting = &settings[i];

		if (!strcmp(setting->name, sval)) {
			*setting_value = setting->value;
			return;
		}
	}
	dev_warn(dev, "Unknown setting for %s: '%s'\n", setting_name, sval);
}

static void i3c_hub_tp_of_get_setting(struct device *dev, const struct device_node *node,
				      struct tp_setting tp_setting[])
{
	struct device_node *tp_node;
	u32 id;

	for_each_available_child_of_node(node, tp_node) {
		if (!tp_node->name || of_node_cmp(tp_node->name, "target-port"))
			continue;

		if (!tp_node->full_name ||
		    (sscanf(tp_node->full_name, "target-port@%u", &id) != 1)) {
			dev_warn(dev, "Invalid target port node found in DT: %s\n",
				 tp_node->full_name);
			continue;
		}

		if (id >= I3C_HUB_TP_MAX_COUNT) {
			dev_warn(dev, "Invalid target port index found in DT: %i\n", id);
			continue;
		}
		i3c_hub_of_get_setting(dev, tp_node, "mode", tp_mode_settings,
				       ARRAY_SIZE(tp_mode_settings), &tp_setting[id].mode);
		i3c_hub_of_get_setting(dev, tp_node, "pullup", tp_pullup_settings,
				       ARRAY_SIZE(tp_pullup_settings), &tp_setting[id].pullup_en);
	}
}

static void i3c_hub_of_get_conf_static(struct device *dev, const struct device_node *node)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);

	i3c_hub_of_get_setting(dev, node, "cp0-ldo-en", ldo_en_settings,
			       ARRAY_SIZE(ldo_en_settings), &priv->settings.cp0_ldo_en);
	i3c_hub_of_get_setting(dev, node, "cp1-ldo-en", ldo_en_settings,
			       ARRAY_SIZE(ldo_en_settings), &priv->settings.cp1_ldo_en);
	i3c_hub_of_get_setting(dev, node, "cp0-ldo-volt", ldo_volt_settings,
			       ARRAY_SIZE(ldo_volt_settings), &priv->settings.cp0_ldo_volt);
	i3c_hub_of_get_setting(dev, node, "cp1-ldo-volt", ldo_volt_settings,
			       ARRAY_SIZE(ldo_volt_settings), &priv->settings.cp1_ldo_volt);
	i3c_hub_of_get_setting(dev, node, "tp0145-ldo-en", ldo_en_settings,
			       ARRAY_SIZE(ldo_en_settings), &priv->settings.tp0145_ldo_en);
	i3c_hub_of_get_setting(dev, node, "tp2367-ldo-en", ldo_en_settings,
			       ARRAY_SIZE(ldo_en_settings), &priv->settings.tp2367_ldo_en);
	i3c_hub_of_get_setting(dev, node, "tp0145-ldo-volt", ldo_volt_settings,
			       ARRAY_SIZE(ldo_volt_settings), &priv->settings.tp0145_ldo_volt);
	i3c_hub_of_get_setting(dev, node, "tp2367-ldo-volt", ldo_volt_settings,
			       ARRAY_SIZE(ldo_volt_settings), &priv->settings.tp2367_ldo_volt);
	i3c_hub_of_get_setting(dev, node, "tp0145-pullup", pullup_settings,
			       ARRAY_SIZE(pullup_settings), &priv->settings.tp0145_pullup);
	i3c_hub_of_get_setting(dev, node, "tp2367-pullup", pullup_settings,
			       ARRAY_SIZE(pullup_settings), &priv->settings.tp2367_pullup);
	i3c_hub_of_get_setting(dev, node, "cp0-io-strength", io_strength_settings,
			       ARRAY_SIZE(io_strength_settings), &priv->settings.cp0_io_strength);
	i3c_hub_of_get_setting(dev, node, "cp1-io-strength", io_strength_settings,
			       ARRAY_SIZE(io_strength_settings), &priv->settings.cp1_io_strength);
	i3c_hub_of_get_setting(dev, node, "tp0145-io-strength", io_strength_settings,
			       ARRAY_SIZE(io_strength_settings),
			       &priv->settings.tp0145_io_strength);
	i3c_hub_of_get_setting(dev, node, "tp2367-io-strength", io_strength_settings,
			       ARRAY_SIZE(io_strength_settings),
			       &priv->settings.tp2367_io_strength);

	i3c_hub_tp_of_get_setting(dev, node, priv->settings.tp);
}

static void i3c_hub_of_default_configuration(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	int id;

	priv->settings.cp0_ldo_en = I3C_HUB_DT_LDO_DISENA_NOT_DEFINED;
	priv->settings.cp1_ldo_en = I3C_HUB_DT_LDO_DISENA_NOT_DEFINED;
	priv->settings.cp0_ldo_volt = I3C_HUB_DT_LDO_VOLT_NOT_DEFINED;
	priv->settings.cp1_ldo_volt = I3C_HUB_DT_LDO_VOLT_NOT_DEFINED;
	priv->settings.tp0145_ldo_en = I3C_HUB_DT_LDO_DISENA_NOT_DEFINED;
	priv->settings.tp2367_ldo_en = I3C_HUB_DT_LDO_DISENA_NOT_DEFINED;
	priv->settings.tp0145_ldo_volt = I3C_HUB_DT_LDO_VOLT_NOT_DEFINED;
	priv->settings.tp2367_ldo_volt = I3C_HUB_DT_LDO_VOLT_NOT_DEFINED;
	priv->settings.tp0145_pullup = I3C_HUB_DT_PULLUP_NOT_DEFINED;
	priv->settings.tp2367_pullup = I3C_HUB_DT_PULLUP_NOT_DEFINED;
	priv->settings.cp0_io_strength = I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED;
	priv->settings.cp1_io_strength = I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED;
	priv->settings.tp0145_io_strength = I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED;
	priv->settings.tp2367_io_strength = I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED;

	for (id = 0; id < I3C_HUB_TP_MAX_COUNT; ++id) {
		priv->settings.tp[id].mode = I3C_HUB_DT_TP_MODE_NOT_DEFINED;
		priv->settings.tp[id].pullup_en = I3C_HUB_DT_TP_PULLUP_NOT_DEFINED;
	}
}

static int i3c_hub_hw_configure_pullup(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 mask = 0, value = 0;

	if (priv->settings.tp0145_pullup != I3C_HUB_DT_PULLUP_NOT_DEFINED) {
		mask |= TP0145_PULLUP_CONF_MASK;
		value |= TP0145_PULLUP_CONF(i3c_hub_pullup_dt_to_reg(priv->settings.tp0145_pullup));
	}

	if (priv->settings.tp2367_pullup != I3C_HUB_DT_PULLUP_NOT_DEFINED) {
		mask |= TP2367_PULLUP_CONF_MASK;
		value |= TP2367_PULLUP_CONF(i3c_hub_pullup_dt_to_reg(priv->settings.tp2367_pullup));
	}

	return regmap_update_bits(priv->regmap, I3C_HUB_LDO_AND_PULLUP_CONF, mask, value);
}

static int i3c_hub_hw_configure_ldo(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 ldo_config_mask = 0, ldo_config_val = 0;
	u8 ldo_disable_mask = 0, ldo_en_val = 0;
	u32 reg_val;
	int ret;
	u8 val;

	if (priv->settings.cp0_ldo_en == I3C_HUB_DT_LDO_DISENA_ENABLED)
		ldo_en_val |= CP0_LDO_EN;
	if (priv->settings.cp1_ldo_en == I3C_HUB_DT_LDO_DISENA_ENABLED)
		ldo_en_val |= CP1_LDO_EN;
	if (priv->settings.tp0145_ldo_en == I3C_HUB_DT_LDO_DISENA_ENABLED)
		ldo_en_val |= TP0145_LDO_EN;
	if (priv->settings.tp2367_ldo_en == I3C_HUB_DT_LDO_DISENA_ENABLED)
		ldo_en_val |= TP2367_LDO_EN;

	ret = regmap_read(priv->regmap, I3C_HUB_LDO_CONF, &reg_val);
	if (ret)
		return ret;

	if (priv->settings.cp0_ldo_volt != I3C_HUB_DT_LDO_VOLT_NOT_DEFINED) {
		val = CP0_LDO_VOLTAGE(i3c_hub_ldo_dt_to_reg(priv->settings.cp0_ldo_volt));
		if ((reg_val & CP0_LDO_VOLTAGE_MASK) != val) {
			ldo_config_mask |= CP0_LDO_VOLTAGE_MASK;
			ldo_disable_mask |= CP0_LDO_EN;
			ldo_config_val |= val;
		}
	}
	if (priv->settings.cp1_ldo_volt != I3C_HUB_DT_LDO_VOLT_NOT_DEFINED) {
		val = CP1_LDO_VOLTAGE(i3c_hub_ldo_dt_to_reg(priv->settings.cp1_ldo_volt));
		if ((reg_val & CP1_LDO_VOLTAGE_MASK) != val) {
			ldo_config_mask |= CP1_LDO_VOLTAGE_MASK;
			ldo_disable_mask |= CP1_LDO_EN;
			ldo_config_val |= val;
		}
	}
	if (priv->settings.tp0145_ldo_volt != I3C_HUB_DT_LDO_VOLT_NOT_DEFINED) {
		val = TP0145_LDO_VOLTAGE(i3c_hub_ldo_dt_to_reg(priv->settings.tp0145_ldo_volt));
		if ((reg_val & TP0145_LDO_VOLTAGE_MASK) != val) {
			ldo_config_mask |= TP0145_LDO_VOLTAGE_MASK;
			ldo_disable_mask |= TP0145_LDO_EN;
			ldo_config_val |= val;
		}
	}
	if (priv->settings.tp2367_ldo_volt != I3C_HUB_DT_LDO_VOLT_NOT_DEFINED) {
		val = TP2367_LDO_VOLTAGE(i3c_hub_ldo_dt_to_reg(priv->settings.tp2367_ldo_volt));
		if ((reg_val & TP2367_LDO_VOLTAGE_MASK) != val) {
			ldo_config_mask |= TP2367_LDO_VOLTAGE_MASK;
			ldo_disable_mask |= TP2367_LDO_EN;
			ldo_config_val |= val;
		}
	}

	if (ldo_config_mask) {
		ret = regmap_update_bits(priv->regmap, I3C_HUB_LDO_AND_PULLUP_CONF,
					 ldo_disable_mask, 0);
		if (ret)
			return ret;

		ret = regmap_update_bits(priv->regmap, I3C_HUB_LDO_CONF, ldo_config_mask,
					 ldo_config_val);
		if (ret)
			return ret;
	}

	return regmap_update_bits(priv->regmap, I3C_HUB_LDO_AND_PULLUP_CONF,
				  LDO_ENABLE_DISABLE_MASK, ldo_en_val);
}

static int i3c_hub_hw_configure_io_strength(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 mask_all = 0, val_all = 0;
	u32 reg_val;
	u8 val;
	int ret;

	/* Get IO strength configuration to figure out what needs to be changed */
	ret = regmap_read(priv->regmap, I3C_HUB_IO_STRENGTH, &reg_val);
	if (ret)
		return ret;

	if (priv->settings.cp0_io_strength != I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED) {
		val = CP0_IO_STRENGTH
			(i3c_hub_io_strength_dt_to_reg(priv->settings.cp0_io_strength));
		mask_all |= CP0_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (priv->settings.cp1_io_strength != I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED) {
		val = CP1_IO_STRENGTH
			(i3c_hub_io_strength_dt_to_reg(priv->settings.cp1_io_strength));
		mask_all |= CP1_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (priv->settings.tp0145_io_strength != I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED) {
		val = TP0145_IO_STRENGTH
			(i3c_hub_io_strength_dt_to_reg(priv->settings.tp0145_io_strength));
		mask_all |= TP0145_IO_STRENGTH_MASK;
		val_all |= val;
	}
	if (priv->settings.tp2367_io_strength != I3C_HUB_DT_IO_STRENGTH_NOT_DEFINED) {
		val = TP2367_IO_STRENGTH
			(i3c_hub_io_strength_dt_to_reg(priv->settings.tp2367_io_strength));
		mask_all |= TP2367_IO_STRENGTH_MASK;
		val_all |= val;
	}

	/* Set IO strength if required */
	return regmap_update_bits(priv->regmap, I3C_HUB_IO_STRENGTH, mask_all, val_all);
}

static int i3c_hub_hw_configure_tp(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u8 pullup_mask = 0, pullup_val = 0;
	u8 smbus_mask = 0, smbus_val = 0;
	u8 gpio_mask = 0, gpio_val = 0;
	u8 i3c_mask = 0, i3c_val = 0;
	int ret;
	int i;

	/* TBD: Read type of HUB from register I3C_HUB_DEV_INFO_0 to learn target ports count. */
	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i) {
		if (priv->settings.tp[i].mode != I3C_HUB_DT_TP_MODE_NOT_DEFINED) {
			i3c_mask |= TPn_NET_CON(i);
			smbus_mask |= TPn_SMBUS_MODE_EN(i);
			gpio_mask |= TPn_GPIO_MODE_EN(i);

			if (priv->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_I3C)
				i3c_val |= TPn_NET_CON(i);
			else if (priv->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_SMBUS)
				smbus_val |= TPn_SMBUS_MODE_EN(i);
			else if (priv->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_GPIO)
				gpio_val |= TPn_GPIO_MODE_EN(i);
		}
		if (priv->settings.tp[i].pullup_en != I3C_HUB_DT_TP_PULLUP_NOT_DEFINED) {
			pullup_mask |= TPn_PULLUP_EN(i);
			if (priv->settings.tp[i].pullup_en == I3C_HUB_DT_TP_PULLUP_ENABLED)
				pullup_val |= TPn_PULLUP_EN(i);
		}
	}

	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_IO_MODE_CONF, smbus_mask, smbus_val);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_PULLUP_EN, pullup_mask, pullup_val);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_SMBUS_AGNT_EN, smbus_mask, smbus_val);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_GPIO_MODE_EN, gpio_mask, gpio_val);
	if (ret)
		return ret;

	/* Request for HUB Network connection in case any TP is configured in I3C mode */
	if (i3c_val) {
		ret = regmap_write(priv->regmap, I3C_HUB_CP_MUX_SET, CONTROLLER_PORT_MUX_REQ);
		if (ret)
			return ret;
		/* TODO: verify if connection is done */
	}

	/* Enable TP here in case TP was configured */
	ret = regmap_update_bits(priv->regmap, I3C_HUB_TP_ENABLE, i3c_mask | smbus_mask | gpio_mask,
				 i3c_val | smbus_val | gpio_val);
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, I3C_HUB_TP_NET_CON_CONF, i3c_mask, i3c_val);
}

static int i3c_hub_configure_hw(struct device *dev)
{
	int ret;

	ret = i3c_hub_hw_configure_ldo(dev);
	if (ret)
		return ret;

	ret = i3c_hub_hw_configure_io_strength(dev);
	if (ret)
		return ret;

	ret = i3c_hub_hw_configure_pullup(dev);
	if (ret)
		return ret;

	return i3c_hub_hw_configure_tp(dev);
}

static void i3c_hub_of_get_conf_runtime(struct device *dev, const struct device_node *node)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	struct device_node *i3c_node;
	int i3c_id;
	u8 tp_mask;

	for_each_available_child_of_node(node, i3c_node) {
		if (!i3c_node->full_name ||
		    (sscanf(i3c_node->full_name, "i3c%i@%hhx", &i3c_id, &tp_mask) != 2)) {
			continue;
		}

		if (i3c_id < I3C_HUB_LOGICAL_BUS_MAX_COUNT) {
			priv->logical_bus[i3c_id].available = true;
			priv->logical_bus[i3c_id].of_node = i3c_node;
			priv->logical_bus[i3c_id].tp_map = tp_mask;
			priv->logical_bus[i3c_id].priv = priv;
		}
	}
}

static const struct i3c_device_id i3c_hub_ids[] = {
	I3C_CLASS(I3C_DCR_HUB, NULL),
	{ },
};

static int i3c_hub_read_id(struct device *dev)
{
	struct i3c_hub *priv = dev_get_drvdata(dev);
	u32 reg_val;
	int ret;

	ret = regmap_read(priv->regmap, I3C_HUB_LDO_AND_CPSEL_STS, &reg_val);
	if (ret) {
		dev_err(dev, "Failed to read status register\n");
		return -1;
	}

	priv->hub_pin_sel_id = CP_SEL_PIN_INPUT_CODE_GET(reg_val);
	priv->hub_pin_cp1_id = CP_SDA1_SCL1_PINS_CODE_GET(reg_val);
	return 0;
}

static struct device_node *i3c_hub_get_dt_hub_node(struct device_node *node,
						   struct i3c_hub *priv)
{
	struct device_node *hub_node_no_id = NULL;
	struct device_node *hub_node;
	u32 hub_id;
	int found_id = 0;

	for_each_available_child_of_node(node, hub_node) {
		if (strstr(hub_node->name, "hub")) {
			if (!of_property_read_u32(hub_node, "id", &hub_id)) {
				if (hub_id == (u32)priv->hub_pin_sel_id)
					found_id = 1;
				priv->hub_dt_sel_id = hub_id;
			}

			if (!of_property_read_u32(hub_node, "id-cp1", &hub_id)) {
				if (hub_id == (u32)priv->hub_pin_cp1_id)
					found_id = 1;
				priv->hub_dt_cp1_id = hub_id;
			}

			if (!found_id) {
				/*
				 * Just keep reference to first HUB node with no ID in case no ID
				 * matching
				 */
				if (!hub_node_no_id && priv->hub_dt_sel_id == -1 &&
				    priv->hub_dt_cp1_id == -1)
					hub_node_no_id = hub_node;
			} else {
				return hub_node;
			}
		}
	}

	return hub_node_no_id;
}

static int fops_access_reg_get(void *ctx, u64 *val)
{
	struct i3c_hub *priv = ctx;
	u32 reg_val;
	int ret;

	ret = regmap_read(priv->regmap, priv->reg_addr, &reg_val);
	if (ret)
		return ret;

	*val = reg_val & 0xFF;
	return 0;
}

static int fops_access_reg_set(void *ctx, u64 val)
{
	struct i3c_hub *priv = ctx;

	return regmap_write(priv->regmap, priv->reg_addr, val & 0xFF);
}
DEFINE_DEBUGFS_ATTRIBUTE(fops_access_reg, fops_access_reg_get, fops_access_reg_set, "0x%llX\n");

static int i3c_hub_debugfs_init(struct i3c_hub *priv, const char *hub_id)
{
	struct dentry  *entry, *dt_conf_dir, *reg_dir;
	int i;

	entry = debugfs_create_dir(hub_id, NULL);
	if (IS_ERR(entry))
		return PTR_ERR(entry);

	priv->debug_dir = entry;

	entry = debugfs_create_dir("dt-conf", priv->debug_dir);
	if (IS_ERR(entry))
		goto err_remove;

	dt_conf_dir = entry;

	debugfs_create_u8("cp0-ldo-en", 0400, dt_conf_dir, &priv->settings.cp0_ldo_en);
	debugfs_create_u8("cp1-ldo-en", 0400, dt_conf_dir, &priv->settings.cp1_ldo_en);
	debugfs_create_u8("cp0-ldo-volt", 0400, dt_conf_dir, &priv->settings.cp0_ldo_volt);
	debugfs_create_u8("cp1-ldo-volt", 0400, dt_conf_dir, &priv->settings.cp1_ldo_volt);
	debugfs_create_u8("tp0145-ldo-en", 0400, dt_conf_dir, &priv->settings.tp0145_ldo_en);
	debugfs_create_u8("tp2367-ldo-en", 0400, dt_conf_dir, &priv->settings.tp2367_ldo_en);
	debugfs_create_u8("tp0145-ldo-volt", 0400, dt_conf_dir, &priv->settings.tp0145_ldo_volt);
	debugfs_create_u8("tp2367-ldo-volt", 0400, dt_conf_dir, &priv->settings.tp2367_ldo_volt);
	debugfs_create_u8("tp0145-pullup", 0400, dt_conf_dir, &priv->settings.tp0145_pullup);
	debugfs_create_u8("tp2367-pullup", 0400, dt_conf_dir, &priv->settings.tp2367_pullup);

	for (i = 0; i < I3C_HUB_TP_MAX_COUNT; ++i) {
		char file_name[32];

		sprintf(file_name, "tp%i.mode", i);
		debugfs_create_u8(file_name, 0400, dt_conf_dir, &priv->settings.tp[i].mode);
		sprintf(file_name, "tp%i.pullup_en", i);
		debugfs_create_u8(file_name, 0400, dt_conf_dir, &priv->settings.tp[i].pullup_en);
	}

	entry = debugfs_create_dir("reg", priv->debug_dir);
	if (IS_ERR(entry))
		goto err_remove;

	reg_dir = entry;

	entry = debugfs_create_file_unsafe("access", 0600, reg_dir, priv, &fops_access_reg);
	if (IS_ERR(entry))
		goto err_remove;

	debugfs_create_u8("offset", 0600, reg_dir, &priv->reg_addr);

	return 0;

err_remove:
	debugfs_remove_recursive(priv->debug_dir);
	return PTR_ERR(entry);
}

static void i3c_hub_trans_pre_cb(struct i3c_master_controller *controller)
{
	struct logical_bus *bus = container_of(controller, struct logical_bus, controller);
	struct i3c_hub *priv = bus->priv;
	struct device *dev = i3cdev_to_dev(priv->i3cdev);
	int ret;

	ret = regmap_write(priv->regmap, I3C_HUB_TP_NET_CON_CONF, bus->tp_map);
	if (ret)
		dev_warn(dev, "Failed to open Target Port(s)\n");
}

static void i3c_hub_trans_post_cb(struct i3c_master_controller *controller)
{
	struct logical_bus *bus = container_of(controller, struct logical_bus, controller);
	struct i3c_hub *priv = bus->priv;
	struct device *dev = i3cdev_to_dev(priv->i3cdev);
	int ret;

	ret = regmap_write(priv->regmap, I3C_HUB_TP_NET_CON_CONF, 0x00);
	if (ret)
		dev_warn(dev, "Failed to close Target Port(s)\n");
}

static struct logical_bus *bus_from_i3c_desc(struct i3c_dev_desc *desc)
{
	struct i3c_master_controller *controller = i3c_dev_get_master(desc);

	return container_of(controller, struct logical_bus, controller);
}

static struct i3c_master_controller
	*parent_from_controller(struct i3c_master_controller *controller)
{
	struct logical_bus *bus = container_of(controller, struct logical_bus, controller);

	return bus->priv->controller;
}

static struct i3c_master_controller *parent_controller_from_i3c_desc(struct i3c_dev_desc *desc)
{
	struct i3c_master_controller *controller = i3c_dev_get_master(desc);
	struct logical_bus *bus = container_of(controller, struct logical_bus, controller);

	return bus->priv->controller;
}

static struct i3c_master_controller *parent_controller_from_i2c_desc(struct i2c_dev_desc *desc)
{
	struct i3c_master_controller *controller = desc->common.master;
	struct logical_bus *bus = container_of(controller, struct logical_bus, controller);

	return bus->priv->controller;
}

static int i3c_hub_read_transaction_status(struct i3c_hub *priv, u8 offset)
{
	ktime_t time_to_timeout = 0;
	unsigned int status;
	ktime_t start, end;
	int ret;

	start = ktime_get_real();
	while (time_to_timeout < (ktime_t)I3C_HUB_SMBUS_400kHz_TIMEOUT) {
		ret = regmap_read(priv->regmap, offset, &status);
		if (ret)
			return ret;

		if (status & I3C_HUB_TP_SMBUS_AGNT_STS_TRANS_FINISHED_MASK)
			return I3C_HUB_TP_TRANSACTION_CODE_GET(status);

		end = ktime_get_real();
		time_to_timeout = end - start;
	}
	dev_dbg(i3cdev_to_dev(priv->i3cdev), "SMBus transaction completion timeout reached\n");
	return -ETIMEDOUT;
}

static int i3c_hub_smbus_msg_raw(struct i3c_hub *priv, u8 target_port,
				 u8 desc[I3C_HUB_SMBUS_DESCRIPTOR_SIZE],
				 u8 *buff_w, size_t buff_w_len,
				 u8 *buff_r, size_t buff_r_len)
{
	u8 controller_buffer_page = I3C_HUB_CONTROLLER_BUFFER_PAGE + 4 * target_port;
	u8 status_offset = I3C_HUB_TPx_SMBUS_AGNT_STS(target_port);
	int ret;

	ret = regmap_write(priv->regmap, I3C_HUB_PAGE_PTR, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, status_offset, I3C_HUB_TP_SMBUS_AGNT_STS_ALL_MASK);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, I3C_HUB_PAGE_PTR, controller_buffer_page);
	if (ret)
		return ret;

	ret = regmap_bulk_write(priv->regmap, I3C_HUB_CONTROLLER_AGENT_BUFF, desc,
				I3C_HUB_SMBUS_DESCRIPTOR_SIZE);
	if (ret)
		return ret;

	if (buff_w) {
		ret = regmap_bulk_write(priv->regmap, I3C_HUB_CONTROLLER_AGENT_BUFF_DATA, buff_w,
					buff_w_len);
		if (ret)
			return ret;
	}

	ret = regmap_write(priv->regmap, I3C_HUB_TP_SMBUS_AGNT_TRANS_START,
			   I3C_HUB_TP_SMBUS_AGNTx_TRANS_START_REQ(target_port));
	if (ret)
		return ret;

	ret = i3c_hub_read_transaction_status(priv, status_offset);
	if (ret)
		return ret;

	if (buff_r) {
		ret = regmap_bulk_read(priv->regmap, I3C_HUB_CONTROLLER_AGENT_BUFF_DATA +
				       buff_w_len, buff_r, buff_r_len);
		if (ret)
			return ret;
	}

	return regmap_write(priv->regmap, I3C_HUB_PAGE_PTR, 0x00);
}

/*
 * i3c_hub_smbus_msg() - This starts a smbus write transaction by writing a descriptor
 * and a message to the hub registers. Controller buffer page is determined by multiplying the
 * target port index by four and adding the base page number to it.
 * @priv: a pointer to the i3c hub main structure
 * @target_port: a number of the port where the transaction will happen
 * @xfer: i2c_msg struct received from the master_xfers callback - single xfer
 *
 * Return: 0 returned on success. Negative value returned when accessing HUB's registers failed.
 * Positive value returned when SMBus transaction failed.
 */
static int i3c_hub_smbus_msg(struct i3c_hub *priv, u8 target_port, struct i2c_msg *xfer)
{
	u8 desc[I3C_HUB_SMBUS_DESCRIPTOR_SIZE];

	desc[0] = 2 * xfer->addr; /* converting 7-bit address to 8-bit address */
	if (xfer->flags & I2C_M_RD)
		desc[0] |= I3C_HUB_SMBUS_RNW_TRANSACTION;
	desc[1] = I3C_HUB_SMBUS_400kHz;
	desc[2] = xfer->len;
	desc[3] = (xfer->flags & I2C_M_RD) ? xfer->len : 0;

	if (xfer->flags & I2C_M_RD)
		return i3c_hub_smbus_msg_raw(priv, target_port, desc, NULL, 0, xfer->buf,
					     xfer->len);
	else
		return i3c_hub_smbus_msg_raw(priv, target_port, desc, xfer->buf, xfer->len, NULL,
					     0);
}

/*
 * i3c_hub_smbus_msg_wr_rd() - This starts a smbus write+read transaction by writing a descriptor
 * and a message to the hub registers. Controller buffer page is determined by multiplying the
 * target port index by four and adding the base page number to it.
 * @priv: a pointer to the i3c hub main structure
 * @target_port: a number of the port where the transaction will happen
 * @xfer_wr: i2c_msg struct received from the master_xfers callback - single xfer for write
 * @xfer_rd: i2c_msg struct received from the master_xfers callback - single xfer for read
 *
 * Return: 0 returned on success. Negative value returned when accessing HUB's registers failed.
 * Positive value returned when SMBus transaction failed.
 */
static int i3c_hub_smbus_msg_wr_rd(struct i3c_hub *priv, u8 target_port, struct i2c_msg *xfer_wr,
				   struct i2c_msg *xfer_rd)
{
	u8 desc[I3C_HUB_SMBUS_DESCRIPTOR_SIZE];

	desc[0] = (2 * xfer_wr->addr); /* converting 7-bit address to 8-bit address */
	desc[1] = I3C_HUB_SMBUS_400kHz | I3C_HUB_SMBUS_WRITE_AND_READ_TRANSACTION;
	desc[2] = xfer_wr->len;
	desc[3] = xfer_rd->len;

	return i3c_hub_smbus_msg_raw(priv, target_port, desc, xfer_wr->buf, xfer_wr->len,
				     xfer_rd->buf, xfer_rd->len);
}

/**
 * i3c_controller_smbus_port_adapter_xfer() - i3c hub smbus transfer logic
 * @adap: i2c_adapter corresponding with single port in the i3c hub
 * @xfers: all messages descriptors and data
 * @nxfers: amount of single messages in a transfer
 *
 * Return: function returns the sum of correctly sent messages (only those with hub return
 * status 0x01)
 */
static int i3c_controller_smbus_port_adapter_xfer(struct i2c_adapter *adap,
						  struct i2c_msg *xfers,
						  int nxfers)
{
	struct i3c_master_controller *controller =
		container_of(adap, struct i3c_master_controller, i2c);
	struct logical_bus *bus =
		container_of(controller, struct logical_bus, controller);
	struct i3c_hub *priv = bus->priv;
	int ret_sum = 0;
	u8 nxfers_i;
	int ret;

	for (nxfers_i = 0 ; nxfers_i < nxfers ; nxfers_i++) {
		/*
		 * If there is a write followed by read, send those two transactions with repeated
		 * start instead of stop and start.
		 */
		if ((nxfers_i + 1) < nxfers && !(xfers[nxfers_i].flags & I2C_M_RD) &&
		    xfers[nxfers_i + 1].flags & I2C_M_RD) {
			ret = i3c_hub_smbus_msg_wr_rd(priv, bus->smbus_port_adapter.tp_port,
						      &xfers[nxfers_i], &xfers[nxfers_i + 1]);
			/* Negative result means error not related to SMBus transaction itself. */
			if (ret < 0)
				return ret;
			/* Other results are related to SMBus transactions themself. */
			else if (ret == I3C_HUB_TP_SMBUS_AGNT_STS_TRANS_CODE_SUCCESS)
				ret_sum += 2;
			else
				dev_dbg(i3cdev_to_dev(priv->i3cdev),
					"SMBus transactions ([%i,%i/%i]) failed, status: %i\n",
					nxfers_i, nxfers_i + 1, nxfers, ret);
			/*
			 * Need to bump up one more time because two transactions (write + read)
			 * were handled.
			 */
			nxfers_i++;
		} else {
			ret = i3c_hub_smbus_msg(priv, bus->smbus_port_adapter.tp_port,
						&xfers[nxfers_i]);
			/* Negative result means error not related to SMBus transaction itself. */
			if (ret < 0)
				return ret;
			/* Other results are related to SMBus transactions themself. */
			else if (ret == I3C_HUB_TP_SMBUS_AGNT_STS_TRANS_CODE_SUCCESS)
				ret_sum++;
			else
				dev_dbg(i3cdev_to_dev(priv->i3cdev),
					"SMBus transaction ([%i/%i]) failed, status: %i\n",
					nxfers_i, nxfers, ret);
		}
	}
	return ret_sum;
}

static int i3c_hub_bus_init(struct i3c_master_controller *controller)
{
	struct logical_bus *bus = container_of(controller, struct logical_bus, controller);

	controller->this = bus->priv->i3cdev->desc;
	return 0;
}

static void i3c_hub_bus_cleanup(struct i3c_master_controller *controller)
{
	controller->this = NULL;
}

static int i3c_hub_attach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);

	return parent->ops->attach_i3c_dev(dev);
}

static int i3c_hub_reattach_i3c_dev(struct i3c_dev_desc *dev, u8 old_dyn_addr)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);

	return parent->ops->reattach_i3c_dev(dev, old_dyn_addr);
}

static void i3c_hub_detach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);

	parent->ops->detach_i3c_dev(dev);
}

static int i3c_hub_do_daa(struct i3c_master_controller *controller)
{
	struct i3c_master_controller *parent = parent_from_controller(controller);

	return parent->ops->do_daa(controller);
}

static bool i3c_hub_supports_ccc_cmd(struct i3c_master_controller *controller,
				     const struct i3c_ccc_cmd *cmd)
{
	struct i3c_master_controller *parent = parent_from_controller(controller);

	return parent->ops->supports_ccc_cmd(controller, cmd);
}

static int i3c_hub_send_ccc_cmd(struct i3c_master_controller *controller, struct i3c_ccc_cmd *cmd)
{
	struct i3c_master_controller *parent = parent_from_controller(controller);

	return parent->ops->send_ccc_cmd(controller, cmd);
}

static int i3c_hub_priv_xfers(struct i3c_dev_desc *dev, struct i3c_priv_xfer *xfers, int nxfers)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);
	struct logical_bus *bus = bus_from_i3c_desc(dev);
	int res;

	i3c_hub_trans_pre_cb(&bus->controller);
	res = parent->ops->priv_xfers(dev, xfers, nxfers);
	i3c_hub_trans_post_cb(&bus->controller);

	return res;
}

static int i3c_hub_attach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct i3c_master_controller *parent = parent_controller_from_i2c_desc(dev);

	return parent->ops->attach_i2c_dev(dev);
}

static void i3c_hub_detach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct i3c_master_controller *parent = parent_controller_from_i2c_desc(dev);

	parent->ops->detach_i2c_dev(dev);
}

static int i3c_hub_i2c_xfers(struct i2c_dev_desc *dev, const struct i2c_msg *xfers, int nxfers)
{
	struct i3c_master_controller *parent = parent_controller_from_i2c_desc(dev);

	return parent->ops->i2c_xfers(dev, xfers, nxfers);
}

static int i3c_hub_request_ibi(struct i3c_dev_desc *dev, const struct i3c_ibi_setup *req)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);

	return parent->ops->request_ibi(dev, req);
}

static void i3c_hub_free_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);

	parent->ops->free_ibi(dev);
}

static int i3c_hub_enable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);

	return parent->ops->enable_ibi(dev);
}

static int i3c_hub_disable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);

	return parent->ops->disable_ibi(dev);
}

static void i3c_hub_recycle_ibi_slot(struct i3c_dev_desc *dev, struct i3c_ibi_slot *slot)
{
	struct i3c_master_controller *parent = parent_controller_from_i3c_desc(dev);

	return parent->ops->recycle_ibi_slot(dev, slot);
}

static const struct i3c_master_controller_ops i3c_hub_i3c_ops = {
	.bus_init = i3c_hub_bus_init,
	.bus_cleanup = i3c_hub_bus_cleanup,
	.attach_i3c_dev = i3c_hub_attach_i3c_dev,
	.reattach_i3c_dev = i3c_hub_reattach_i3c_dev,
	.detach_i3c_dev = i3c_hub_detach_i3c_dev,
	.do_daa = i3c_hub_do_daa,
	.supports_ccc_cmd = i3c_hub_supports_ccc_cmd,
	.send_ccc_cmd = i3c_hub_send_ccc_cmd,
	.priv_xfers = i3c_hub_priv_xfers,
	.attach_i2c_dev = i3c_hub_attach_i2c_dev,
	.detach_i2c_dev = i3c_hub_detach_i2c_dev,
	.i2c_xfers = i3c_hub_i2c_xfers,
	.request_ibi = i3c_hub_request_ibi,
	.free_ibi = i3c_hub_free_ibi,
	.enable_ibi = i3c_hub_enable_ibi,
	.disable_ibi = i3c_hub_disable_ibi,
	.recycle_ibi_slot = i3c_hub_recycle_ibi_slot,
};

/* SMBus virtual i3c_master_controller_ops */

static int i3c_hub_do_daa_smbus(struct i3c_master_controller *controller)
{
	return 0;
}

static int i3c_hub_send_ccc_cmd_smbus(struct i3c_master_controller *controller,
				      struct i3c_ccc_cmd *cmd)
{
	return 0;
}

static int i3c_hub_priv_xfers_smbus(struct i3c_dev_desc *dev,
				    struct i3c_priv_xfer *xfers,
				    int nxfers)
{
	return 0;
}

static int i3c_hub_i2c_xfers_smbus(struct i2c_dev_desc *dev,
				   const struct i2c_msg *xfers,
				   int nxfers)
{
	return 0;
}

static const struct i3c_master_controller_ops i3c_hub_i3c_ops_smbus = {
	.bus_init = i3c_hub_bus_init,
	.bus_cleanup = i3c_hub_bus_cleanup,
	.do_daa = i3c_hub_do_daa_smbus,
	.send_ccc_cmd = i3c_hub_send_ccc_cmd_smbus,
	.priv_xfers = i3c_hub_priv_xfers_smbus,
	.i2c_xfers = i3c_hub_i2c_xfers_smbus,
};

int i3c_hub_logic_register(struct i3c_master_controller *controller,
			   struct i3c_master_controller *top_controller, struct device *parent)
{
	controller->bus_driver_context = top_controller->bus_driver_context;
	return i3c_master_register(controller, parent, &i3c_hub_i3c_ops, false);
}

int i3c_hub_logic_register_smbus(struct i3c_master_controller *controller,
				 struct i3c_master_controller *top_controller,
				 struct device *parent)
{
	controller->bus_driver_context = top_controller->bus_driver_context;
	return i3c_master_register(controller, parent, &i3c_hub_i3c_ops_smbus, false);
}

static u32 i3c_controller_smbus_funcs(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_EMUL | I2C_FUNC_I2C;
}

static int reg_i2c_target(struct i2c_client *client)
{
	return 0;
}

static int unreg_i2c_target(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_algorithm i3c_controller_smbus_algo = {
	.master_xfer = i3c_controller_smbus_port_adapter_xfer,
	.functionality = i3c_controller_smbus_funcs,
	.reg_slave = reg_i2c_target,
	.unreg_slave = unreg_i2c_target,
};

static void i3c_hub_delayed_work(struct work_struct *work)
{
	struct i3c_hub *priv = container_of(work, typeof(*priv), delayed_work.work);
	struct device *dev = i3cdev_to_dev(priv->i3cdev);
	struct i2c_board_info host_notify_board_info;
	int ret;
	int i;

	for (i = 0; i < I3C_HUB_LOGICAL_BUS_MAX_COUNT; ++i) {
		if (priv->logical_bus[i].available) {
			ret = regmap_write(priv->regmap, I3C_HUB_TP_NET_CON_CONF,
					   priv->logical_bus[i].tp_map);
			if (ret)
				dev_warn(dev, "Failed to open Target Port(s)\n");

			dev->of_node = priv->logical_bus[i].of_node;
			ret = i3c_hub_logic_register(&priv->logical_bus[i].controller,
						     i3c_dev_get_master(priv->i3cdev->desc), dev);
			if (ret)
				dev_warn(dev, "Failed to register i3c controller - bus id:%i\n", i);
			else
				priv->logical_bus[i].registered = true;

			ret = regmap_write(priv->regmap, I3C_HUB_TP_NET_CON_CONF, 0x00);
			if (ret)
				dev_warn(dev, "Failed to close Target Port(s)\n");
		}
	}

	ret = i3c_master_do_daa(priv->controller);
	if (ret)
		dev_warn(dev, "Failed to run DAA\n");

	for (i = 0 ; i < I3C_HUB_TP_MAX_COUNT ; i++) {
		if (!priv->logical_bus[i].smbus_port_adapter.used)
			continue;

		priv->logical_bus[i].controller.i2c.algo = &i3c_controller_smbus_algo;

		if (!priv->logical_bus[i].smbus_port_adapter.compatible)
			continue;

		host_notify_board_info.addr = priv->logical_bus[i].smbus_port_adapter.addr;
		host_notify_board_info.flags = I2C_CLIENT_SLAVE;
		snprintf(host_notify_board_info.type,
			 I2C_NAME_SIZE,
			 priv->logical_bus[i].smbus_port_adapter.compatible);

		priv->logical_bus[i].smbus_port_adapter.client =
			i2c_new_client_device(&priv->logical_bus[i].controller.i2c,
					      &host_notify_board_info);
		if (IS_ERR(priv->logical_bus[i].smbus_port_adapter.client)) {
			dev_warn(dev, "Error while registering backend\n");
			return;
		}

		schedule_delayed_work(&priv->logical_bus[i].smbus_port_adapter.delayed_work_polling,
					msecs_to_jiffies(I3C_HUB_POLLING_ROLL_PERIOD_MS));
	}
}

static int i3c_hub_register_smbus_controller(struct i3c_hub *priv, int i)
{
	struct device *dev = i3cdev_to_dev(priv->i3cdev);
	int ret;

	ret = regmap_write(priv->regmap,
			   I3C_HUB_TP_NET_CON_CONF,
			   priv->logical_bus[i].smbus_port_adapter.tp_mask);
	if (ret) {
		dev_warn(dev, "Failed to open Target Port");
		return ret;
	}

	ret = i3c_hub_logic_register_smbus(&priv->logical_bus[i].controller,
					   priv->controller, dev);
	if (ret) {
		dev_warn(dev, "Failed to register i3c controller\n");
		return ret;
	}

	ret = regmap_write(priv->regmap, I3C_HUB_TP_NET_CON_CONF, 0x00);
	if (ret) {
		dev_warn(dev, "Failed to close Target Port");
		return ret;
	}

	return 0;
}

/**
 * i3c_hub_delayed_work_polling() - This delayed work is a polling mechanism to
 * find if any transaction happened. After a transaction was found it is saved with
 * the slave-mqueue backend and can be read from the fs. Controller buffer page is
 * determined by adding the first buffer page number to port index multiplied by four.
 * The two target buffer page numbers are determined the same way but they are offset
 * by 2 and 3 from the controller page.
 */
static void i3c_hub_delayed_work_polling(struct work_struct *work)
{
	struct i2c_adapter_group *g_adap = container_of(work,
							typeof(*g_adap),
							delayed_work_polling.work);
	struct logical_bus *bus = container_of(g_adap, struct logical_bus, smbus_port_adapter);
	u8 controller_buffer_page = I3C_HUB_CONTROLLER_BUFFER_PAGE + 4 * g_adap->tp_port;
	u8 target_port_status = I3C_HUB_TPx_SMBUS_AGNT_STS(g_adap->tp_port);
	u8 local_buffer[I3C_HUB_SMBUS_TARGET_PAYLOAD_SIZE] = {0};
	u8 target_buffer_page, address, test, len, tmp;
	struct i3c_hub *priv = bus->priv;
	struct device *dev = i3cdev_to_dev(priv->i3cdev);
	u32 local_last_status, i;

	regmap_read(priv->regmap, target_port_status, &local_last_status);

	tmp = local_last_status;
	if (tmp & I3C_HUB_TP_SMBUS_AGNT_STS_BUF_OVERFLOW_MASK) {
		regmap_write(priv->regmap, I3C_HUB_PAGE_PTR, 0x00);
		regmap_write(priv->regmap, target_port_status, I3C_HUB_TP_SMBUS_AGNT_STS_ALL_MASK);
		regmap_read(priv->regmap, target_port_status, &local_last_status);
		g_adap->polling_last_status = local_last_status;
	} else if (local_last_status != g_adap->polling_last_status) {
		if (tmp & I3C_HUB_TP_SMBUS_AGNT_STS_BUF0_RECEIVED_MASK)
			target_buffer_page = controller_buffer_page + 2;
		else if (tmp & I3C_HUB_TP_SMBUS_AGNT_STS_BUF1_RECEIVED_MASK)
			target_buffer_page = controller_buffer_page + 3;
		else
			goto reschedule;

		regmap_write(priv->regmap, I3C_HUB_PAGE_PTR, target_buffer_page);

		regmap_read(priv->regmap, I3C_HUB_TARGET_BUFF_LENGTH, &local_last_status);

		len = local_last_status - 1;
		if (len > I3C_HUB_SMBUS_TARGET_PAYLOAD_SIZE) {
			dev_err(dev, "Received message too big for hub buffer\n");
			goto reschedule;
		}

		regmap_read(priv->regmap, I3C_HUB_TARGET_BUFF_ADDRESS, &local_last_status);

		address = local_last_status;
		if ((address >> 1) != g_adap->addr)
			goto reschedule;

		regmap_bulk_read(priv->regmap, I3C_HUB_TARGET_BUFF_DATA, local_buffer, len);

		i2c_slave_event(g_adap->client, I2C_SLAVE_WRITE_RECEIVED, &address);

		for (i = 0 ; i < len ; i++) {
			tmp = local_buffer[i];
			i2c_slave_event(g_adap->client, I2C_SLAVE_WRITE_RECEIVED, &tmp);
		}
		i2c_slave_event(g_adap->client, I2C_SLAVE_STOP, &test);

reschedule:
		regmap_write(priv->regmap, I3C_HUB_PAGE_PTR, 0x00);
		regmap_write(priv->regmap, target_port_status, I3C_HUB_TP_SMBUS_AGNT_STS_ALL_MASK);
		regmap_read(priv->regmap, target_port_status, &local_last_status);
		g_adap->polling_last_status = local_last_status;
	}

	schedule_delayed_work(&g_adap->delayed_work_polling,
			      msecs_to_jiffies(I3C_HUB_POLLING_ROLL_PERIOD_MS));
}

static int i3c_hub_smbus_tp_algo(struct i3c_hub *priv, int i)
{
	int ret;

	if (priv->hub_dt_cp1_id != priv->hub_pin_cp1_id)
		return 1;

	priv->logical_bus[i].priv = priv;
	priv->logical_bus[i].smbus_port_adapter.tp_port = i;
	priv->logical_bus[i].smbus_port_adapter.tp_mask = BIT(i);

	/* Register controller for target port*/
	ret = i3c_hub_register_smbus_controller(priv, i);
	if (ret)
		return ret;

	priv->logical_bus[i].smbus_port_adapter.used = 1;

	INIT_DELAYED_WORK(&priv->logical_bus[i].smbus_port_adapter.delayed_work_polling,
			  i3c_hub_delayed_work_polling);

	priv->logical_bus[i].controller.i2c.dev.parent =
		priv->logical_bus[i].controller.dev.parent;
	priv->logical_bus[i].controller.i2c.owner =
		priv->logical_bus[i].controller.dev.parent->driver->owner;

	sprintf(priv->logical_bus[i].controller.i2c.name, "hub0x%X.port%d",
		priv->hub_dt_cp1_id, i);

	priv->logical_bus[i].controller.i2c.timeout = 1000;
	priv->logical_bus[i].controller.i2c.retries = 3;

	return 0;
}

static int read_backend_from_i3c_hub_dts(struct device_node *i3c_node_target,
					 struct i3c_hub *priv)
{
	struct device_node *i3c_node_tp;
	const char *compatible;
	int tp_port, ret;
	u32 addr_dts;

	if (sscanf(i3c_node_target->full_name, "target-port@%d", &tp_port) == 0)
		return -EINVAL;

	if (tp_port > I3C_HUB_TP_MAX_COUNT)
		return -ERANGE;

	if (tp_port < 0)
		return -EINVAL;

	for_each_available_child_of_node(i3c_node_target, i3c_node_tp) {
		if (strcmp(i3c_node_tp->full_name, "backend@0,0"))
			continue;

		ret = of_property_read_u32(i3c_node_tp, "target-reg", &addr_dts);
		if (ret)
			return ret;

		ret = of_property_read_string(i3c_node_tp, "compatible", &compatible);
		if (ret)
			return ret;

		/* Currently only the slave-mqueue backend is supported */
		if (strcmp("slave-mqueue", compatible))
			return -EINVAL;

		priv->logical_bus[tp_port].smbus_port_adapter.addr = addr_dts;
		priv->logical_bus[tp_port].smbus_port_adapter.compatible = compatible;

		break;
	}
	return 0;
}

/**
 * This function saves information about the i3c_hub's ports
 * working in slave mode. It takes its data from the DTs
 * (aspeed-bmc-intel-avc.dts) and saves the parameters
 * into the coresponding target port i2c_adapter_group structure
 * in the i3c_hub
 *
 * @dev: device used by i3c_hub
 * @i3c_node_hub: device node pointing to the hub
 * @priv: pointer to the i3c_hub structure
 */
static void i3c_hub_parse_dt_tp(struct device *dev,
				const struct device_node *i3c_node_hub,
				struct i3c_hub *priv)
{
	struct device_node *i3c_node_target;
	int ret;

	for_each_available_child_of_node(i3c_node_hub, i3c_node_target) {
		if (!strcmp(i3c_node_target->name, "target-port")) {
			ret = read_backend_from_i3c_hub_dts(i3c_node_target, priv);
			if (ret)
				dev_err(dev, "DTS entry invalid - error %d", ret);
		}
	}
}

static int i3c_hub_probe(struct i3c_device *i3cdev)
{
	struct regmap_config i3c_hub_regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};
	struct device *dev = &i3cdev->dev;
	struct device_node *node = NULL;
	struct regmap *regmap;
	struct i3c_hub *priv;
	char hub_id[32];
	int ret;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->i3cdev = i3cdev;
	priv->controller = i3c_dev_get_master(i3cdev->desc);
	i3cdev_set_drvdata(i3cdev, priv);
	INIT_DELAYED_WORK(&priv->delayed_work, i3c_hub_delayed_work);
	sprintf(hub_id, "i3c-hub-%d-%llx", i3c_dev_get_master(i3cdev->desc)->bus_id,
		i3cdev->desc->info.pid);
	ret = i3c_hub_debugfs_init(priv, hub_id);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialized DebugFS.\n");

	i3c_hub_of_default_configuration(dev);

	regmap = devm_regmap_init_i3c(i3cdev, &i3c_hub_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "Failed to register I3C HUB regmap\n");
		goto error;
	}
	priv->regmap = regmap;

	ret = i3c_hub_read_id(dev);
	if (ret)
		goto error;

	priv->hub_dt_sel_id = -1;
	priv->hub_dt_cp1_id = -1;
	if (priv->hub_pin_cp1_id >= 0 && priv->hub_pin_sel_id >= 0)
		/* Find hub node in DT matching HW ID or just first without ID provided in DT */
		node = i3c_hub_get_dt_hub_node(dev->parent->of_node, priv);

	if (!node) {
		dev_info(dev, "No DT entry - running with hardware defaults.\n");
	} else {
		of_node_get(node);
		i3c_hub_of_get_conf_static(dev, node);
		i3c_hub_of_get_conf_runtime(dev, node);
		of_node_put(node);

		/* Parse DTS to find data on the SMBus target mode */
		i3c_hub_parse_dt_tp(dev, node, priv);
	}

	/* Unlock access to protected registers */
	ret = regmap_write(priv->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_UNLOCK_CODE);
	if (ret) {
		dev_err(dev, "Failed to unlock HUB's protected registers\n");
		goto error;
	}

	/* Register logic for native smbus ports */
	for (i = 0 ; i < I3C_HUB_TP_MAX_COUNT ; i++) {
		priv->logical_bus[i].smbus_port_adapter.used = 0;
		if (priv->settings.tp[i].mode == I3C_HUB_DT_TP_MODE_SMBUS)
			ret = i3c_hub_smbus_tp_algo(priv, i);
	}

	ret = i3c_hub_configure_hw(dev);
	if (ret) {
		dev_err(dev, "Failed to configure the HUB\n");
		goto error;
	}

	/* Apply security lock for all protected registers */
	ret = regmap_write(priv->regmap, I3C_HUB_DEV_CMD, I3C_HUB_DEV_CMD_LOCK_ALL_PROT_REGS);
	if (ret) {
		dev_err(dev, "Failed to apply security lock for all HUB's protected registers\n");
		goto error;
	}

	/* Lock access to protected registers */
	ret = regmap_write(priv->regmap, I3C_HUB_PROTECTION_CODE, REGISTERS_LOCK_CODE);
	if (ret) {
		dev_err(dev, "Failed to lock HUB's protected registers\n");
		goto error;
	}

	schedule_delayed_work(&priv->delayed_work, msecs_to_jiffies(100));

	return 0;

error:
	debugfs_remove_recursive(priv->debug_dir);
	return ret;
}

static void i3c_hub_remove(struct i3c_device *i3cdev)
{
	struct i3c_hub *priv = i3cdev_get_drvdata(i3cdev);
	struct i2c_adapter_group *g_adap;
	int i;

	for (i = 0; i < I3C_HUB_TP_MAX_COUNT ; i++) {
		if (priv->logical_bus[i].smbus_port_adapter.used) {
			g_adap = &priv->logical_bus[i].smbus_port_adapter;
			cancel_delayed_work_sync(&g_adap->delayed_work_polling);
			i2c_unregister_device(g_adap->client);
		}

		if (priv->logical_bus[i].smbus_port_adapter.used || priv->logical_bus[i].registered)
			i3c_master_unregister(&priv->logical_bus[i].controller);
	}

	cancel_delayed_work_sync(&priv->delayed_work);
	debugfs_remove_recursive(priv->debug_dir);
}

static struct i3c_driver i3c_hub = {
	.driver.name = "i3c-hub",
	.id_table = i3c_hub_ids,
	.probe = i3c_hub_probe,
	.remove = i3c_hub_remove,
};

module_i3c_driver(i3c_hub);

MODULE_AUTHOR("Zbigniew Lukwinski <zbigniew.lukwinski@linux.intel.com>");
MODULE_DESCRIPTION("I3C HUB driver");
MODULE_LICENSE("GPL");
