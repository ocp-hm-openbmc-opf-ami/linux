// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020-2020 Intel Corporation

#include <linux/hwmon.h>
#include <linux/jiffies.h>
#include <linux/mfd/intel-peci-client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include "peci-hwmon.h"
#ifdef CONFIG_SMART_MODULE
 #include "pmbus/smart.h"
#endif /* CONFIG_SMART_MODULE */

enum PECI_PLATFORMPOWER_POWER_SENSOR_TYPES {
	PECI_PLATFORMPOWER_SENSOR_TYPE_POWER = 0,
	PECI_PLATFORMPOWER_SENSOR_TYPE_ENERGY,
	PECI_PLATFORMPOWER_SENSOR_TYPES_COUNT,
};

#define PECI_PLATFORMPOWER_POWER_CHANNEL_COUNT		1 /* Supported channels number. */
#define PECI_PLATFORMPOWER_ENERGY_CHANNEL_COUNT	1 /* Supported channels number. */

#define PECI_PLATFORMPOWER_POWER_SENSOR_COUNT	4 /* Supported sensors number. */
#define PECI_PLATFORMPOWER_ENERGY_SENSOR_COUNT	1 /* Supported sensors number. */

#define PECI_PLATFORMPOWER_ENERGY_UNIT	0

#define PECI_PLATFORMPOWER_PCS_PPL_MAX_VALUE 0x1FFFF /* Maximum limit value */

/**
 * union peci_platform_power_info_low - Platform and Package Power Info (Low)
 * PCS. This PCS coresponds to the MSR@665h - PLATFORM_POWER_INFO, bits [31:0]
 * Accessing over PECI: PCS=0x1C, parameter=0x00FE
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@max_ppl1:	Bits [15:0] - Max PP L1 value
 *		@rsvd0:		Bits [16:16]
 *		@min_ppl1:	Bits [31:17] - Min PP L1 value
 */
union peci_platform_power_info_low {
	u32 value;
	struct {
		u32 max_ppl1        : 17;
		u32 min_ppl1        : 15;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_platform_power_info_low) ==
		     PECI_PCS_REGISTER_SIZE);

/**
 * union peci_platform_power_limit_high - Platform Power Limit 2 PCS
 * This PCS coresponds to the MSR@65Ch - PLATFORM_POWER_LIMIT_SRVR, bits [63:32]
 * Accessing over PECI: PCS=0x3B, Parameter=0x0000
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@pwr_lim_2:	Bits [16:0] - Power Limit 2
 *		@pwr_lim_2_en:	Bits [17:17] - Power Limit 2 Enable
 *		@pwr_clmp_lim_2:Bits [18:18] - Critical Power Clamp 2
 *		@pwr_lim_2_time:Bits [25:19] - Power Limit 2 Time Window
 *		@rsvd0:		Bits [31:26]
 */
union peci_platform_power_limit_high {
	u32 value;
	struct {
		u32 pwr_lim_2       : 17;
		u32 pwr_lim_2_en    : 1;
		u32 pwr_clmp_lim_2  : 1;
		u32 pwr_lim_2_time  : 7;
		u32 rsvd0           : 6;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_platform_power_limit_high) ==
	      PECI_PCS_REGISTER_SIZE);

/**
 * union peci_platform_power_limit_low - Platform Power Limit 1 PCS
 * This PCS coresponds to the MSR@65Ch - PLATFORM_POWER_LIMIT_SRVR, bits [31:0]
 * Accessing over PECI: PCS=0x3A, Parameter=0x0000
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@pwr_lim_1:	Bits [16:0] - Power Limit 1
 *		@pwr_lim_1_en:	Bits [17:17] - Power Limit 1 Enable
 *		@pwr_clmp_lim_1:Bits [18:18] - Critical Power Clamp 1
 *		@pwr_lim_1_time:Bits [25:19] - Power Limit 1 Time Window
 *		@rsvd0:		Bits [31:26]
 */
union peci_platform_power_limit_low {
	u32 value;
	struct {
		u32 pwr_lim_1       : 17;
		u32 pwr_lim_1_en    : 1;
		u32 pwr_clmp_lim_1  : 1;
		u32 pwr_lim_1_time  : 7;
		u32 rsvd0           : 6;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_platform_power_limit_low) ==
		     PECI_PCS_REGISTER_SIZE);

struct peci_platformpower {
	struct device *dev;
	struct peci_client_manager *mgr;
	char name[PECI_NAME_SIZE];
	const struct cpu_gen_info *gen_info;
	u32 power_config[PECI_PLATFORMPOWER_POWER_CHANNEL_COUNT + 1];
	u32 energy_config[PECI_PLATFORMPOWER_ENERGY_CHANNEL_COUNT + 1];

	struct hwmon_channel_info power_info;
	struct hwmon_channel_info energy_info;
	const struct hwmon_channel_info *info[PECI_PLATFORMPOWER_SENSOR_TYPES_COUNT + 1];
	struct hwmon_chip_info chip;

	struct peci_sensor_data
		power_sensor_data_list[PECI_PLATFORMPOWER_POWER_CHANNEL_COUNT]
				      [PECI_PLATFORMPOWER_POWER_SENSOR_COUNT];
	struct peci_sensor_data
		energy_sensor_data_list[PECI_PLATFORMPOWER_ENERGY_CHANNEL_COUNT]
				       [PECI_PLATFORMPOWER_ENERGY_SENSOR_COUNT];

	/* Below structs are not exposed to any sensor directly */
	struct peci_sensor_data energy_cache; /* used to limit PECI communication */
	struct peci_sensor_data power_sensor_prev_energy;
	struct peci_sensor_data energy_sensor_prev_energy;
	bool extended_energy_supported;

	union peci_pkg_power_sku_unit units;
	bool units_valid;

	u32 ppl1_time_window;
	u32 ppl2_time_window;
	bool ppl_time_windows_valid;

#ifdef CONFIG_SMART_MODULE
	bool block_set_power_limit;
	s32 latest_power_limit_set;
#endif /* CONFIG_SMART_MODULE */
};

static const u8 peci_platformpower_models[] = {
	INTEL_FAM6_SAPPHIRERAPIDS,
	INTEL_FAM6_EMERALDRAPIDS,
	INTEL_FAM6_GRANITERAPIDS,
	INTEL_FAM6_SIERRAFOREST,
};

static const char
*peci_platformpower_labels[PECI_PLATFORMPOWER_SENSOR_TYPES_COUNT] = {
	"platform power",
	"platform energy",
};

/**
 * peci_platformpower_read_platf_pwr_info_low - read PCS Platform Power Info low
 * @peci_mgr: PECI client manager handle
 * @reg: Pointer to the variable read value is going to be put
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static inline int
peci_platformpower_read_platf_pwr_info_low(struct peci_client_manager *peci_mgr,
					   union peci_platform_power_info_low
					   *reg)
{
	return peci_pcs_read(peci_mgr, PECI_MBX_INDEX_TDP,
			     PECI_PKG_ID_PLATFORM, (u8 *)&reg->value,
			     sizeof(reg->value));
}

/**
 * peci_platformpower_read_platf_pwr_lim_low - read PCS Platform Power Limit Low
 * @peci_mgr: PECI client manager handle
 * @reg: Pointer to the variable read value is going to be put
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static inline int
peci_platformpower_read_platf_pwr_lim_low(struct peci_client_manager *peci_mgr,
					  union peci_platform_power_limit_low
					  *reg)
{
	return peci_pcs_read(peci_mgr, PECI_MBX_INDEX_PKG_PSYS_PWR_LIM1,
			     PECI_PCS_PARAM_ZERO, (u8 *)&reg->value,
			     sizeof(reg->value));
}

/**
 * peci_platformpower_get_energy_counter - get actual energy counter value
 * @priv: Pointer to the peci platformpower context
 * @sensor_data: Sensor data
 * @update_interval: time in jiffies needs to elapse to read sensor again
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static int
peci_platformpower_get_energy_counter(struct peci_platformpower *priv,
				      struct peci_sensor_data *sensor_data,
				      ulong update_interval)
{
	int ret = 0;

	mutex_lock(&sensor_data->lock);
	if (!peci_sensor_need_update_with_time(sensor_data,
					       update_interval)) {
		dev_dbg(priv->dev, "skip reading package energy over peci\n");
		goto unlock;
	}

	if (priv->extended_energy_supported)
		ret = peci_pcs_read(priv->mgr, PECI_MBX_INDEX_ENERGY_COUNTER,
				    PECI_PKG_ID_PLATFORM, (u8 *)&sensor_data->uvalue,
				    sizeof(sensor_data->uvalue));
	else
		ret = peci_pcs_read(priv->mgr, PECI_MBX_INDEX_ENERGY_COUNTER,
				    PECI_PKG_ID_PLATFORM, (u8 *)&sensor_data->uvalue,
				    sizeof(u32));

	if (ret) {
		dev_dbg(priv->dev, "not able to read package energy\n");
		goto unlock;
	}

	peci_sensor_mark_updated(sensor_data);

	dev_dbg(priv->dev,
		"energy counter updated %lluuJ, jif %lu, HZ is %d jiffies\n",
		sensor_data->uvalue, sensor_data->last_updated, HZ);

unlock:
	mutex_unlock(&sensor_data->lock);
	return ret;
}

/**
 * peci_platformpower_get_average_power - get avarage platform power
 * @ctx: Pointer to the driver context
 * @sensor_conf: Sensor configuration
 * @sensor_data: Sensor data
 * @val: Pointer to the variable read value is going to be put, in milliwatts
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static int
peci_platformpower_get_average_power(void *ctx,
				     struct peci_sensor_conf *sensor_conf,
				     struct peci_sensor_data *sensor_data)
{
	struct peci_platformpower *priv = (struct peci_platformpower *)ctx;
	int ret = 0;

	mutex_lock(&sensor_data->lock);
	if (!peci_sensor_need_update_with_time(sensor_data,
					       sensor_conf->update_interval)) {
		dev_dbg(priv->dev,
			"skip generating new power value %dmW jif %lu\n",
			sensor_data->value, jiffies);
		goto unlock;
	}

	ret = peci_platformpower_get_energy_counter(priv, &priv->energy_cache,
						    sensor_conf->update_interval);
	if (ret) {
		dev_dbg(priv->dev, "cannot update energy counter\n");
		goto unlock;
	}

	ret = peci_pcs_calc_pwr_from_eng(priv->dev,
					 &priv->power_sensor_prev_energy,
					 &priv->energy_cache,
					 PECI_PLATFORMPOWER_ENERGY_UNIT,
					 priv->extended_energy_supported,
					 &sensor_data->value);
	if (ret) {
		dev_dbg(priv->dev, "power calculation failed\n");
		goto unlock;
	}

	peci_sensor_mark_updated_with_time(sensor_data,
					   priv->energy_cache.last_updated);

	dev_dbg(priv->dev, "average power %dmW, jif %lu, HZ is %d jiffies\n",
		sensor_data->value, sensor_data->last_updated, HZ);

unlock:
	mutex_unlock(&sensor_data->lock);
	return ret;
}

/**
 * peci_platformpower_get_power_limit - get current platform power limit
 * @ctx: Pointer to the driver context
 * @sensor_conf: Sensor configuration
 * @sensor_data: Sensor data
 * @val: Pointer to the variable read value is going to be put, in milliwatts
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static int
peci_platformpower_get_power_limit(void *ctx,
				   struct peci_sensor_conf *sensor_conf,
				   struct peci_sensor_data *sensor_data)
{
	struct peci_platformpower *priv = (struct peci_platformpower *)ctx;
	union peci_platform_power_limit_low power_limit;
	int ret = 0;

	mutex_lock(&sensor_data->lock);

	/*
	 * Check whether need to update reading. If not just return cached
	 * value.
	 */
	if (!peci_sensor_need_update_with_time(sensor_data,
					       sensor_conf->update_interval)) {
		dev_dbg(priv->dev, "skip reading peci, power limit %dmW\n",
			sensor_data->value);
		goto unlock;
	}

	/*
	 * Read units to the cache. Units are needed to convert power values
	 * correctly. Units are read from CPU only once.
	 */
	ret = peci_pcs_get_units(priv->mgr, &priv->units, &priv->units_valid);
	if (ret) {
		dev_dbg(priv->dev, "not able to read units\n");
		goto unlock;
	}

	/* Read power limit 1 (PPL1). */
	ret = peci_platformpower_read_platf_pwr_lim_low(priv->mgr,
							&power_limit);
	if (ret) {
		dev_dbg(priv->dev, "not able to read power limit 1 (PPL1)\n");
		goto unlock;
	}

	peci_sensor_mark_updated(sensor_data);
	sensor_data->value = peci_pcs_xn_to_munits(power_limit.bits.pwr_lim_1,
						   priv->units.bits.pwr_unit);

	dev_dbg(priv->dev, "raw power limit %u, unit %u, power limit %d\n",
		power_limit.bits.pwr_lim_1, priv->units.bits.pwr_unit,
		sensor_data->value);

unlock:
	mutex_unlock(&sensor_data->lock);
	return ret;
}

/**
 * peci_platformpower_set_power_limit - set platform power limit
 * @ctx: Pointer to the driver context
 * @sensor_conf: Sensor configuration
 * @sensor_data: Sensor data
 * @val: Power limit to set, in milliwatts
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static int
peci_platformpower_set_power_limit(void *ctx,
				   struct peci_sensor_conf *sensor_conf,
				   struct peci_sensor_data *sensor_data,
				   s32 val)
{
	struct peci_platformpower *priv = (struct peci_platformpower *)ctx;
	union peci_platform_power_limit_high power_limit_high;
	union peci_platform_power_limit_low power_limit_low;
	int ret;

	/*
	 * Read units to the cache. Units are needed to convert power values
	 * correctly. Units are read from CPU only once.
	 */
	ret = peci_pcs_get_units(priv->mgr, &priv->units, &priv->units_valid);
	if (ret) {
		dev_dbg(priv->dev, "not able to read units\n");
		return ret;
	}

	/* Read power limit 1 (PPL1). */
	ret = peci_platformpower_read_platf_pwr_lim_low(priv->mgr,
							&power_limit_low);
	if (ret) {
		dev_dbg(priv->dev, "not able to read power limit 1 (PPL1)\n");
		return ret;
	}

	/* Read power limit 2 (PPL2). */
	ret = peci_pcs_read(priv->mgr, PECI_MBX_INDEX_PKG_PSYS_PWR_LIM2,
			    PECI_PCS_PARAM_ZERO, (u8 *)&power_limit_high.value,
			    sizeof(power_limit_high.value));
	if (ret) {
		dev_dbg(priv->dev, "not able to read power limit 2 (PPL2)\n");
		return ret;
	}

	/* Calculate PPL time windows if needed */
	if (!priv->ppl_time_windows_valid) {
		priv->ppl1_time_window =
			peci_pcs_calc_plxy_time_window(peci_pcs_munits_to_xn
				(PECI_PCS_PPL1_TIME_WINDOW,
				priv->units.bits.tim_unit));
		priv->ppl2_time_window =
			peci_pcs_calc_plxy_time_window(peci_pcs_munits_to_xn
				(PECI_PCS_PPL2_TIME_WINDOW,
				priv->units.bits.tim_unit));
		priv->ppl_time_windows_valid = true;
	}

	/* Enable or disable power limitation */
	if (val > 0) {
		/* Calculate and enable PPL1 */
		power_limit_low.bits.pwr_lim_1 =
			min(peci_pcs_munits_to_xn(val, priv->units.bits.pwr_unit),
			    (u32)PECI_PLATFORMPOWER_PCS_PPL_MAX_VALUE);
		power_limit_low.bits.pwr_lim_1_en = 1u;
		power_limit_low.bits.pwr_clmp_lim_1 = 1u;
		power_limit_low.bits.pwr_lim_1_time = priv->ppl1_time_window;

		/* Calculate and enable PPL2 */
		power_limit_high.bits.pwr_lim_2 =
			min(peci_pcs_munits_to_xn
			    (PECI_PCS_PPL1_TO_PPL2(val), priv->units.bits.pwr_unit),
			    (u32)PECI_PLATFORMPOWER_PCS_PPL_MAX_VALUE);
		power_limit_high.bits.pwr_lim_2_en = 1u;
		power_limit_high.bits.pwr_clmp_lim_2 = 1u;
		power_limit_high.bits.pwr_lim_2_time = priv->ppl2_time_window;
	} else {
		/* Disable power limitation */
		power_limit_low.bits.pwr_lim_1 = 0u;
		power_limit_low.bits.pwr_lim_1_en = 0u;
		power_limit_low.bits.pwr_clmp_lim_1 = 0u;
		power_limit_low.bits.pwr_lim_1_time = 0u;
		power_limit_high.bits.pwr_lim_2 = 0u;
		power_limit_high.bits.pwr_lim_2_en = 0u;
		power_limit_high.bits.pwr_clmp_lim_2 = 0u;
		power_limit_high.bits.pwr_lim_2_time = 0u;
	}

	/* Write calculated PPL1 to the CPU */
	ret = peci_pcs_write(priv->mgr, PECI_MBX_INDEX_PKG_PSYS_PWR_LIM1,
			     PECI_PCS_PARAM_ZERO,
			     (u8 *)&power_limit_low.value,
			     sizeof(power_limit_low.value));
	if (ret) {
		dev_dbg(priv->dev, "not able to write power limit 1\n");
		return ret;
	}

	/* Write calculated PPL2 to the CPU */
	ret = peci_pcs_write(priv->mgr, PECI_MBX_INDEX_PKG_PSYS_PWR_LIM2,
			     PECI_PCS_PARAM_ZERO,
			     (u8 *)&power_limit_high.value,
			     sizeof(power_limit_high.value));

	if (ret) {
		dev_dbg(priv->dev, "not able to write power limit 2\n");
		return ret;
	}

	dev_dbg(priv->dev, "power limit %d, unit %u, raw power limit 1 %u,\n",
		val, priv->units.bits.pwr_unit, power_limit_low.bits.pwr_lim_1);

	return ret;
}

/**
 * peci_platformpower_read_max_power - get maximum value for the platform power
 * limit
 * @ctx: Pointer to the driver context
 * @sensor_conf: Sensor configuration
 * @sensor_data: Sensor data
 * @val: Pointer to the variable read value is going to be put, in milliwatts
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static int
peci_platformpower_read_max_power(void *ctx,
				  struct peci_sensor_conf *sensor_conf,
				  struct peci_sensor_data *sensor_data)
{
	struct peci_platformpower *priv = (struct peci_platformpower *)ctx;
	union peci_platform_power_info_low power_info;
	int ret = 0;

	mutex_lock(&sensor_data->lock);

	/*
	 * Check whether need to update reading. If not just return cached
	 * value.
	 */
	if (!peci_sensor_need_update_with_time(sensor_data,
					       sensor_conf->update_interval)) {
		dev_dbg(priv->dev, "skip reading peci, max power %dW\n",
			sensor_data->value);
		goto unlock;
	}

	/*
	 * Read units to the cache. Units are needed to convert power values
	 * correctly. Units are read from CPU only once.
	 */
	ret = peci_pcs_get_units(priv->mgr, &priv->units, &priv->units_valid);
	if (ret) {
		dev_dbg(priv->dev, "not able to read units\n");
		goto unlock;
	}

	/* Read platform power info. */
	ret = peci_platformpower_read_platf_pwr_info_low(priv->mgr,
							 &power_info);
	if (ret) {
		dev_dbg(priv->dev, "not able to read platform info\n");
		goto unlock;
	}

	peci_sensor_mark_updated(sensor_data);
	sensor_data->value = peci_pcs_xn_to_munits(power_info.bits.max_ppl1,
						   priv->units.bits.pwr_unit);

	dev_dbg(priv->dev, "raw max power %u, unit %u, max power %dmW\n",
		power_info.bits.max_ppl1, priv->units.bits.pwr_unit,
		sensor_data->value);

unlock:
	mutex_unlock(&sensor_data->lock);
	return ret;
}

/**
 * peci_platformpower_read_min_power - get minimal value for the platform power
 * limit
 * @ctx: Pointer to the driver context
 * @sensor_conf: Sensor configuration
 * @sensor_data: Sensor data
 * @val: Pointer to the variable read value is going to be put, in milliwatts
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static int
peci_platformpower_read_min_power(void *ctx,
				  struct peci_sensor_conf *sensor_conf,
				  struct peci_sensor_data *sensor_data)
{
	struct peci_platformpower *priv = (struct peci_platformpower *)ctx;
	union peci_platform_power_info_low power_info;
	int ret = 0;

	mutex_lock(&sensor_data->lock);

	/*
	 * Check whether need to update reading. If not just return cached
	 * value.
	 */
	if (!peci_sensor_need_update_with_time(sensor_data,
					       sensor_conf->update_interval)) {
		dev_dbg(priv->dev, "skip reading peci, min power %dmW\n",
			sensor_data->value);
		goto unlock;
	}

	/*
	 * Read units to the cache. Units are needed to convert power values
	 * correctly. Units are read from CPU only once.
	 */
	ret = peci_pcs_get_units(priv->mgr, &priv->units, &priv->units_valid);
	if (ret) {
		dev_dbg(priv->dev, "not able to read units\n");
		goto unlock;
	}

	/* Read platform power info. */
	ret = peci_platformpower_read_platf_pwr_info_low(priv->mgr,
							 &power_info);
	if (ret) {
		dev_dbg(priv->dev, "not able to read platform info\n");
		goto unlock;
	}

	peci_sensor_mark_updated(sensor_data);
	sensor_data->value = peci_pcs_xn_to_munits(power_info.bits.min_ppl1,
						   priv->units.bits.pwr_unit);

	dev_dbg(priv->dev, "raw min power %u, unit %u, min power %dmW\n",
		power_info.bits.min_ppl1, priv->units.bits.pwr_unit,
		sensor_data->value);

unlock:
	mutex_unlock(&sensor_data->lock);
	return ret;
}

static int
peci_platformpower_read_energy(void *ctx, struct peci_sensor_conf *sensor_conf,
			       struct peci_sensor_data *sensor_data)
{
	struct peci_platformpower *priv = (struct peci_platformpower *)ctx;
	int ret;

	mutex_lock(&sensor_data->lock);

	if (!peci_sensor_need_update_with_time(sensor_data,
					       sensor_conf->update_interval)) {
		dev_dbg(priv->dev,
			"skip generating new energy value %duJ jif %lu\n",
			sensor_data->value, jiffies);
		goto unlock;
	}

	ret = peci_platformpower_get_energy_counter(priv, &priv->energy_cache,
						    sensor_conf->update_interval);
	if (ret) {
		dev_dbg(priv->dev, "cannot update energy counter\n");
		goto unlock;
	}

	ret = peci_pcs_calc_acc_eng(priv->dev,
				    &priv->energy_sensor_prev_energy,
				    &priv->energy_cache,
				    PECI_PLATFORMPOWER_ENERGY_UNIT,
				    priv->extended_energy_supported,
				    &sensor_data->value);

	if (ret) {
		dev_dbg(priv->dev, "cumulative energy calculation failed\n");
		goto unlock;
	}

	peci_sensor_mark_updated_with_time(sensor_data,
					   priv->energy_cache.last_updated);

	dev_dbg(priv->dev, "energy %duJ, jif %lu, HZ is %d jiffies\n",
		sensor_data->value, sensor_data->last_updated, HZ);

unlock:
	mutex_unlock(&sensor_data->lock);
	return 0;
}

static struct peci_sensor_conf
peci_platformpower_power_cfg[PECI_PLATFORMPOWER_POWER_CHANNEL_COUNT]
			    [PECI_PLATFORMPOWER_POWER_SENSOR_COUNT] = {
	/* Channel 0  - Power */
	{
		{
			.attribute = hwmon_power_average,
			.config = HWMON_P_AVERAGE,
			.update_interval = UPDATE_INTERVAL_100MS,
			.read = peci_platformpower_get_average_power,
			.write = NULL,
		},
		{
			.attribute = hwmon_power_cap,
			.config = HWMON_P_CAP,
			.update_interval = UPDATE_INTERVAL_100MS,
			.read = peci_platformpower_get_power_limit,
			.write = peci_platformpower_set_power_limit,
		},
		{
			.attribute = hwmon_power_cap_max,
			.config = HWMON_P_CAP_MAX,
			.update_interval = UPDATE_INTERVAL_10S,
			.read = peci_platformpower_read_max_power,
			.write = NULL,
		},
		{
			.attribute = hwmon_power_cap_min,
			.config = HWMON_P_CAP_MIN,
			.update_interval = UPDATE_INTERVAL_10S,
			.read = peci_platformpower_read_min_power,
			.write = NULL,
		},
	},
};

static struct peci_sensor_conf
peci_platformpower_energy_cfg[PECI_PLATFORMPOWER_ENERGY_CHANNEL_COUNT]
			     [PECI_PLATFORMPOWER_ENERGY_SENSOR_COUNT] = {
	/* Channel 0  - Energy */
	{
		{
			.attribute = hwmon_energy_input,
			.config = HWMON_E_INPUT,
			.update_interval = UPDATE_INTERVAL_100MS,
			.read = peci_platformpower_read_energy,
			.write = NULL,
		},
	}
};

static bool
peci_platformpower_is_channel_valid(enum hwmon_sensor_types type,
				    int channel)
{
	if ((type == hwmon_power && channel < PECI_PLATFORMPOWER_POWER_CHANNEL_COUNT) ||
	    (type == hwmon_energy && channel < PECI_PLATFORMPOWER_ENERGY_CHANNEL_COUNT))
		return true;

	return false;
}

static int
peci_platformpower_read_string(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	if (!peci_platformpower_is_channel_valid(type, channel))
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_power_label:
		*str = peci_platformpower_labels[PECI_PLATFORMPOWER_SENSOR_TYPE_POWER];
		break;
	case hwmon_energy_label:
		*str = peci_platformpower_labels[PECI_PLATFORMPOWER_SENSOR_TYPE_ENERGY];
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int
peci_platformpower_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	struct peci_platformpower *priv = dev_get_drvdata(dev);
	struct peci_sensor_conf *sensor_conf;
	struct peci_sensor_data *sensor_data;
	int ret;

	if (!priv || !val)
		return -EINVAL;

	if (!peci_platformpower_is_channel_valid(type, channel))
		return -EOPNOTSUPP;

	/* Get sensor configuration and data. */
	switch (type) {
	case hwmon_power:
		ret = peci_sensor_get_ctx(attr, peci_platformpower_power_cfg[channel],
					  &sensor_conf,
					  priv->power_sensor_data_list[channel],
					  &sensor_data,
					  ARRAY_SIZE(peci_platformpower_power_cfg[channel]));
		break;
	case hwmon_energy:
		ret = peci_sensor_get_ctx(attr, peci_platformpower_energy_cfg[channel],
					  &sensor_conf,
					  priv->energy_sensor_data_list[channel],
					  &sensor_data,
					  ARRAY_SIZE(peci_platformpower_energy_cfg[channel]));
		break;
	default:
		ret = -EOPNOTSUPP;
	}

	if (ret)
		return ret;

	if (sensor_conf->read) {
		ret = sensor_conf->read(priv, sensor_conf, sensor_data);
		if (!ret)
			*val = (long)sensor_data->value;
	} else {
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int
peci_platformpower_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	struct peci_platformpower *priv = dev_get_drvdata(dev);
	struct peci_sensor_conf *sensor_conf;
	struct peci_sensor_data *sensor_data;
	int ret;

	if (!priv)
		return -EINVAL;

	if (!peci_platformpower_is_channel_valid(type, channel))
		return -EOPNOTSUPP;

	/* Get sensor configuration and data. */
	switch (type) {
	case hwmon_power:
		ret = peci_sensor_get_ctx(attr, peci_platformpower_power_cfg[channel],
					  &sensor_conf,
					  priv->power_sensor_data_list[channel],
					  &sensor_data,
					  ARRAY_SIZE(peci_platformpower_power_cfg[channel]));
		break;
	case hwmon_energy:
		ret = peci_sensor_get_ctx(attr, peci_platformpower_energy_cfg[channel],
					  &sensor_conf,
					  priv->energy_sensor_data_list[channel],
					  &sensor_data,
					  ARRAY_SIZE(peci_platformpower_energy_cfg[channel]));
		break;
	default:
		ret = -EOPNOTSUPP;
	}

	if (ret)
		return ret;

	if (sensor_conf->write) {
#ifdef CONFIG_SMART_MODULE
		if (sensor_conf->write == peci_platformpower_set_power_limit &&
		    priv->block_set_power_limit)
			return -EPERM;
		priv->latest_power_limit_set = val;
#endif /* CONFIG_SMART_MODULE */
		ret = sensor_conf->write(priv, sensor_conf, sensor_data,
					 (s32)val);
	} else {
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static umode_t
peci_platformpower_is_visible(const void *data, enum hwmon_sensor_types type,
			      u32 attr, int channel)
{
	struct peci_sensor_conf *sensor_conf;
	umode_t mode = 0;
	int ret;

	if (!peci_platformpower_is_channel_valid(type, channel))
		return mode;

	/* If it is about label add read access only. */
	if (attr == hwmon_power_label || attr == hwmon_energy_label)
		return 0444;

	/* Get sensor configuration and data. */
	switch (type) {
	case hwmon_power:
		ret = peci_sensor_get_ctx(attr, peci_platformpower_power_cfg[channel],
					  &sensor_conf, NULL, NULL,
					  ARRAY_SIZE(peci_platformpower_power_cfg[channel]));
		break;
	case hwmon_energy:
		ret = peci_sensor_get_ctx(attr, peci_platformpower_energy_cfg[channel],
					  &sensor_conf, NULL, NULL,
					  ARRAY_SIZE(peci_platformpower_energy_cfg[channel]));
		break;
	default:
		return mode;
	}

	if (!ret) {
		if (sensor_conf->read)
			mode |= 0444;
		if (sensor_conf->write)
			mode |= 0200;
	}

	return mode;
}

#ifdef CONFIG_SMART_MODULE
#define MINIMUM_POWER_CAP_VALUE	1

static int request_throttling(const struct device *dev)
{
	struct peci_platformpower *priv = dev_get_drvdata(dev);
	struct peci_sensor_conf *sensor_conf;
	struct peci_sensor_data *sensor_data;
	int ret;

	dev_dbg(dev, "Request max throttling\n");
	priv->block_set_power_limit = true;
	ret = peci_sensor_get_ctx(hwmon_power_cap, peci_platformpower_power_cfg[0],
				  &sensor_conf,
				  priv->power_sensor_data_list[0],
				  &sensor_data,
				  ARRAY_SIZE(peci_platformpower_power_cfg[0]));
	if (ret) {
		dev_dbg(dev, "Error while getting sensor context, ret: %d\n", ret);
		return -ENODATA;
	}
	peci_platformpower_set_power_limit(priv, sensor_conf, sensor_data,
					   MINIMUM_POWER_CAP_VALUE);

	return 0;
}

static int remove_throttling(const struct device *dev)
{
	struct peci_platformpower *priv = dev_get_drvdata(dev);
	struct peci_sensor_conf *sensor_conf;
	struct peci_sensor_data *sensor_data;
	int ret;

	dev_dbg(dev, "Remove max throttling\n");
	ret = peci_sensor_get_ctx(hwmon_power_cap, peci_platformpower_power_cfg[0],
				  &sensor_conf,
				  priv->power_sensor_data_list[0],
				  &sensor_data,
				  ARRAY_SIZE(peci_platformpower_power_cfg[0]));
	if (ret) {
		dev_dbg(dev, "Error while getting sensor context, ret: %d\n", ret);
		return -ENODATA;
	}

	peci_platformpower_set_power_limit(priv, sensor_conf, sensor_data,
					   priv->latest_power_limit_set);
	priv->block_set_power_limit = false;

	return 0;
}

static struct peci_throttling_ops peci_ops = {
	.request_max_power_throttling = request_throttling,
	.remove_max_power_throttling = remove_throttling
};
#endif /* CONFIG_SMART_MODULE */

static const struct hwmon_ops peci_platformpower_ops = {
	.is_visible = peci_platformpower_is_visible,
	.read_string = peci_platformpower_read_string,
	.read = peci_platformpower_read,
	.write = peci_platformpower_write,
};

static void peci_platformpower_sensor_init(struct peci_platformpower *priv)
{
	int i, j;

	mutex_init(&priv->energy_cache.lock);

	for (i = 0; i < PECI_PLATFORMPOWER_POWER_CHANNEL_COUNT; i++) {
		for (j = 0; j < PECI_PLATFORMPOWER_POWER_SENSOR_COUNT; j++)
			mutex_init(&priv->power_sensor_data_list[i][j].lock);
	}

	for (i = 0; i < PECI_PLATFORMPOWER_ENERGY_CHANNEL_COUNT; i++) {
		for (j = 0; j < PECI_PLATFORMPOWER_ENERGY_SENSOR_COUNT; j++)
			mutex_init(&priv->energy_sensor_data_list[i][j].lock);
	}
}

static int peci_platformpower_probe(struct platform_device *pdev)
{
	struct peci_client_manager *mgr = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct peci_platformpower *priv;
	struct device *hwmon_dev;
	u32 power_cfg_idx = 0;
	u32 energy_cfg_idx = 0;
	u32 cmd_mask;
	int iter;
#ifdef CONFIG_SMART_MODULE
	int ret;
#endif /* CONFIG_SMART_MODULE */

	cmd_mask = BIT(PECI_CMD_RD_PKG_CFG) | BIT(PECI_CMD_WR_PKG_CFG);
	if ((mgr->client->adapter->cmd_mask & cmd_mask) != cmd_mask)
		return -ENODEV;

	/* Psys is supported only on specific CPU generations. */
	for (iter = 0; iter < ARRAY_SIZE(peci_platformpower_models); ++iter) {
		if (mgr->gen_info->model == peci_platformpower_models[iter])
			break;
	}
	if (iter == ARRAY_SIZE(peci_platformpower_models)) {
		dev_dbg(dev, "not supported CPU model\n");
		return -ENODEV;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->mgr = mgr;
	priv->dev = dev;
	priv->gen_info = mgr->gen_info;

	snprintf(priv->name, PECI_NAME_SIZE, "peci_platformpower.cpu%d.%d",
		 mgr->client->addr - PECI_BASE_ADDR, mgr->client->domain_id);

	/* Prepare the right configuration for registration in hwmon. */
	priv->power_config[power_cfg_idx] = HWMON_P_LABEL |
		peci_sensor_get_config(peci_platformpower_power_cfg[power_cfg_idx],
				       ARRAY_SIZE(peci_platformpower_power_cfg[power_cfg_idx]));

	priv->energy_config[energy_cfg_idx] = HWMON_E_LABEL |
		peci_sensor_get_config(peci_platformpower_energy_cfg[energy_cfg_idx],
				       ARRAY_SIZE(peci_platformpower_energy_cfg[energy_cfg_idx]));

	priv->info[PECI_PLATFORMPOWER_SENSOR_TYPE_POWER] = &priv->power_info;
	priv->power_info.type = hwmon_power;
	priv->power_info.config = priv->power_config;

	priv->info[PECI_PLATFORMPOWER_SENSOR_TYPE_ENERGY] = &priv->energy_info;
	priv->energy_info.type = hwmon_energy;
	priv->energy_info.config = priv->energy_config;

	/* Extended energy read is supported on GNR. */
	if (mgr->gen_info->model == INTEL_FAM6_GRANITERAPIDS)
		priv->extended_energy_supported = true;
	else
		priv->extended_energy_supported = false;

	priv->chip.ops = &peci_platformpower_ops;
	priv->chip.info = priv->info;

	peci_platformpower_sensor_init(priv);

	hwmon_dev = devm_hwmon_device_register_with_info(priv->dev, priv->name,
							 priv, &priv->chip,
							 NULL);

	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

#ifdef CONFIG_SMART_MODULE
	ret = smart_register_peci(hwmon_dev, &peci_ops);
	if (ret)
		dev_warn(hwmon_dev, "Failed to register peci device in SmaRT");
#endif /* CONFIG_SMART_MODULE */

	return 0;
}

#ifdef CONFIG_SMART_MODULE
static int find_hwmon_dev(struct device *dev, void *data)
{
	if (strstr(dev_name(dev), "hwmon"))
		return 1;

	return 0;
}

static int peci_platformpower_remove(struct platform_device *pdev)
{
	struct device *dev;
	int ret;

	dev = device_find_child(&pdev->dev, NULL, find_hwmon_dev);
	if (dev) {
		ret = smart_unregister_peci(dev);
		if (ret)
			dev_warn(dev, "Failed to unregister peci device in SmaRT, ret %d", ret);
		put_device(dev);
	}

	return 0;
}
#endif /* CONFIG_SMART_MODULE */

static const struct platform_device_id peci_platformpower_ids[] = {
	{ .name = "peci-platformpower", .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(platform, peci_platformpower_ids);

static struct platform_driver peci_platformpower_driver = {
	.probe    = peci_platformpower_probe,
#ifdef CONFIG_SMART_MODULE
	.remove	  = peci_platformpower_remove,
#endif /* CONFIG_SMART_MODULE */
	.id_table = peci_platformpower_ids,
	.driver   = { .name = KBUILD_MODNAME, },
};
module_platform_driver(peci_platformpower_driver);

MODULE_AUTHOR("Zbigniew Lukwinski <zbigniew.lukwinski@linux.intel.com>");
MODULE_DESCRIPTION("PECI platformpower driver");
MODULE_LICENSE("GPL v2");
