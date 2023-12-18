/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018-2020 Intel Corporation */

#ifndef __PECI_HWMON_H
#define __PECI_HWMON_H

#include <linux/peci-legacy.h>
#include <asm/div64.h>

#define TEMP_TYPE_PECI			6 /* Sensor type 6: Intel PECI */
#define UPDATE_INTERVAL_DEFAULT		HZ
#define UPDATE_INTERVAL_100MS		(HZ / 10)
#define UPDATE_INTERVAL_10S		(HZ * 10)

#define EXTENDED_ENERGY_TIME_UNIT_10NS	100000000

#define PECI_HWMON_LABEL_STR_LEN	10

/**
 * struct peci_sensor_data - PECI sensor information
 * @valid: flag to indicate the sensor value is valid
 * @value: hwmon sensor value in milli units
 * @uvalue: used to store PECI reg value
 * @last_updated: time of the last update in jiffies
 */
struct peci_sensor_data {
	uint valid;
	union {
		s32 value;
		u64 uvalue;
	};
	ulong last_updated;
	struct mutex lock; /* protect sensor access */
};

/**
 * peci_sensor_need_update - check whether sensor update is needed or not
 * @sensor: pointer to sensor data struct
 *
 * Return: true if update is needed, false if not.
 */
static inline bool peci_sensor_need_update(struct peci_sensor_data *sensor)
{
	return !sensor->valid ||
	       time_after(jiffies,
			  sensor->last_updated + UPDATE_INTERVAL_DEFAULT);
}

/**
 * peci_sensor_need_update_with_time - check whether sensor update is needed
 * or not
 * @sensor: pointer to sensor data struct
 * @update_interval: update interval to check
 *
 * Return: true if update is needed, false if not.
 */
static inline bool
peci_sensor_need_update_with_time(struct peci_sensor_data *sensor,
				  ulong update_interval)
{
	return !sensor->valid ||
	       time_after(jiffies, sensor->last_updated + update_interval);
}

/**
 * peci_sensor_mark_updated - mark the sensor is updated
 * @sensor: pointer to sensor data struct
 */
static inline void peci_sensor_mark_updated(struct peci_sensor_data *sensor)
{
	sensor->valid = 1;
	sensor->last_updated = jiffies;
}

/**
 * peci_sensor_mark_updated_with_time - mark the sensor is updated
 * @sensor: pointer to sensor data struct
 * @jif: jiffies value to update with
 */
static inline void
peci_sensor_mark_updated_with_time(struct peci_sensor_data *sensor, ulong jif)
{
	sensor->valid = 1;
	sensor->last_updated = jif;
}

/**
 * struct peci_sensor_conf - PECI sensor information
 * @attribute: Sensor attribute
 * @config: Part of channel parameters brought by single sensor
 * @update_interval: time in jiffies needs to elapse to read sensor again
 * @read:	Read callback for data attributes. Mandatory if readable
 *		data attributes are present.
 *		Parameters are:
 *		@module_ctx:	Pointer peci module context
 *		@sensor_conf:	Pointer to sensor configuration object
 *		@sensor_data:	Pointer to sensor data object
 *		@val:	Pointer to returned value
 *		The function returns 0 on success or a negative error number.
 * @write:	Write callback for data attributes. Mandatory if writeable
 *		data attributes are present.
 *		Parameters are:
 *		@module_ctx:	Pointer peci module context
 *		@sensor_conf:	Pointer to sensor configuration object
 *		@sensor_data:	Pointer to sensor data object
 *		@val:	Value to write
 *		The function returns 0 on success or a negative error number.
 */
struct peci_sensor_conf {
	const s32 attribute;
	const u32 config;
	const ulong update_interval;

	int (*const read)(void *priv, struct peci_sensor_conf *sensor_conf,
			  struct peci_sensor_data *sensor_data);
	int (*const write)(void *priv, struct peci_sensor_conf *sensor_conf,
			   struct peci_sensor_data *sensor_data, s32 val);
};

/**
 * peci_sensor_get_config - get peci sensor configuration for provided channel
 * @sensors: Sensors list
 * @sensor_count: Sensors count
 *
 * Return: sensor configuration
 */
static inline u32 peci_sensor_get_config(struct peci_sensor_conf sensors[],
					 u8 sensor_count)
{
	u32 config = 0u;
	int iter;

	for (iter = 0; iter < sensor_count; ++iter)
		config |= sensors[iter].config;

	return config;
}

/**
 * peci_sensor_get_ctx - get peci sensor context - both configuration and data
 * @attribute: Sensor attribute
 * @sensor_conf_list: Sensors configuration object list
 * @sensor_conf: Sensor configuration object found
 * @sensor_data_list: Sensors data object list, maybe NULL in case there is no
 *		need to find sensor data object
 * @sensor_data: Sensor data object found, maybe NULL in case there is no need
 *		to find sensor data object
 * @sensor_count: Sensor count
 *
 * Return: 0 on success or -EOPNOTSUPP in case sensor attribute not found
 */
static inline int
peci_sensor_get_ctx(s32 attribute, struct peci_sensor_conf sensor_conf_list[],
		    struct peci_sensor_conf **sensor_conf,
		    struct peci_sensor_data sensor_data_list[],
		    struct peci_sensor_data **sensor_data,
		    const u8 sensor_count)
{
	int iter;

	for (iter = 0; iter < sensor_count; ++iter) {
		if (attribute == sensor_conf_list[iter].attribute) {
			*sensor_conf = &sensor_conf_list[iter];
			if (sensor_data_list && sensor_data)
				*sensor_data = &sensor_data_list[iter];
			return 0;
		}
	}

	return -EOPNOTSUPP;
}

/* Value for the most common parameter used for PCS accessing */
#define PECI_PCS_PARAM_ZERO 0x0000u

#define PECI_PCS_REGISTER_SIZE 4u /* PCS register size in bytes */
#define PECI_PCS_EXT_REGISTER_SIZE 8u /* PCS extended register size in bytes */

/* PPL1 value to PPL2 value conversation macro */
#define PECI_PCS_PPL1_TO_PPL2(ppl1_value) ((((u32)(ppl1_value)) * 10uL) / 9uL)

#define PECI_PCS_PPL1_TIME_WINDOW 1000 /* PPL1 Time Window value in ms */

#define PECI_PCS_PPL2_TIME_WINDOW 12 /* PPL2 Time Window value in ms */

#define PECI_PCS_PPL_MAX_VALUE 0x7FFF /* Maximum PPL1/PPL2 Limit value */

/**
 * struct peci_acc_energy_status_extended - PECI Accumulated Energy Status PCS
 * Accessing over PECI: PCS=0x3, Parameters: 0xFF, 0xFE
 *     @energy:        Bits [31:0] - Free running accumulated energy counter
 *     @timestamp:     Bits [63:32] - Timestamp
 */
struct peci_acc_energy_status_extended {
	u32 energy;
	u32 timestamp;
} __packed;

static_assert(sizeof(struct peci_acc_energy_status_extended) == PECI_PCS_EXT_REGISTER_SIZE);

/**
 * union peci_pkg_power_sku_unit - PECI Package Power Unit PCS
 * This register coresponds to the MSR@606h - MSR_RAPL_POWER_UNIT
 * Accessing over PECI: PCS=0x1E, Parameter=0x0000
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@pwr_unit:	Bits [3:0] - Power Unit
 *		@rsvd0:		Bits [7:4]
 *		@eng_unit:	Bits [12:8] - Energy Unit
 *		@rsvd1:		Bits [15:13]
 *		@tim_unit:	Bits [19:16] - Time Unit
 *		@rsvd2:		Bits [31:20]
 */
union peci_pkg_power_sku_unit {
	u32 value;
	struct {
		u32 pwr_unit	: 4;
		u32 rsvd0	: 4;
		u32 eng_unit	: 5;
		u32 rsvd1	: 3;
		u32 tim_unit	: 4;
		u32 rsvd2	: 12;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_pkg_power_sku_unit) == PECI_PCS_REGISTER_SIZE);

/**
 * union peci_package_power_info_low - Platform and Package Power SKU (Low) PCS
 * This PCS coresponds to the MSR@614h - PACKAGE_POWER_SKU, bits [31:0]
 * Accessing over PECI: PCS=0x1C, parameter=0x00FF
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@pkg_tdp:	Bits [14:0] - TDP Package Power
 *		@rsvd0:		Bits [15:15]
 *		@pkg_min_pwr:	Bits [30:16] - Minimal Package Power
 *		@rsvd1:		Bits [31:31]
 */
union peci_package_power_info_low {
	u32 value;
	struct {
		u32 pkg_tdp	: 15;
		u32 rsvd0	: 1;
		u32 pkg_min_pwr	: 15;
		u32 rsvd1	: 1;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_package_power_info_low) ==
	      PECI_PCS_REGISTER_SIZE);

/**
 * union peci_package_power_limit_high - Package Power Limit 2 PCS
 * This PCS coresponds to the MSR@610h - PACKAGE_RAPL_LIMIT, bits [63:32]
 * Accessing over PECI: PCS=0x1B, Parameter=0x0000
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@pwr_lim_2:	Bits [14:0] - Power Limit 2
 *		@pwr_lim_2_en:	Bits [15:15] - Power Limit 2 Enable
 *		@pwr_clmp_lim_2:Bits [16:16] - Package Clamping Limitation 2
 *		@pwr_lim_2_time:Bits [23:17] - Power Limit 2 Time Window
 *		@rsvd0:		Bits [31:24]
 */
union peci_package_power_limit_high {
	u32 value;
	struct {
		u32 pwr_lim_2		: 15;
		u32 pwr_lim_2_en	: 1;
		u32 pwr_clmp_lim_2	: 1;
		u32 pwr_lim_2_time	: 7;
		u32 rsvd0		: 8;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_package_power_limit_high) ==
	      PECI_PCS_REGISTER_SIZE);

/**
 * union peci_package_power_limit_low - Package Power Limit 1 PCS
 * This PCS coresponds to the MSR@610h - PACKAGE_RAPL_LIMIT, bits [31:0]
 * Accessing over PECI: PCS=0x1A, Parameter=0x0000
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@pwr_lim_1:	Bits [14:0] - Power Limit 1
 *		@pwr_lim_1_en:	Bits [15:15] - Power Limit 1 Enable
 *		@pwr_clmp_lim_1:Bits [16:16] - Package Clamping Limitation 1
 *		@pwr_lim_1_time:Bits [23:17] - Power Limit 1 Time Window
 *		@rsvd0:		Bits [31:24]
 */
union peci_package_power_limit_low {
	u32 value;
	struct {
		u32 pwr_lim_1		: 15;
		u32 pwr_lim_1_en	: 1;
		u32 pwr_clmp_lim_1	: 1;
		u32 pwr_lim_1_time	: 7;
		u32 rsvd0		: 8;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_package_power_limit_low) ==
	      PECI_PCS_REGISTER_SIZE);

/**
 * union peci_dram_power_info_low - DRAM Power Info low PCS
 * This PCS coresponds to the MSR@61Ch - MSR_DRAM_POWER_INFO, bits [31:0]
 * Accessing over PECI: PCS=0x24, Parameter=0x0000
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@tdp:		Bits [14:0] - Spec DRAM Power
 *		@rsvd0:		Bits [15:15]
 *		@min_pwr:	Bits [30:16] - Minimal DRAM Power
 *		@rsvd1:		Bits [31:31]
 */
union peci_dram_power_info_low {
	u32 value;
	struct {
		u32 tdp		: 15;
		u32 rsvd0	: 1;
		u32 min_pwr	: 15;
		u32 rsvd1	: 1;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_dram_power_info_low) == PECI_PCS_REGISTER_SIZE);

/**
 * union peci_dram_power_limit - DRAM Power Limit PCS
 * This PCS coresponds to the MSR@618h - DRAM_PLANE_POWER_LIMIT, bits [31:0]
 * Accessing over PECI: PCS=0x22, Parameter=0x0000
 * @value: PCS register value
 * @bits:	PCS register bits
 *		@pp_pwr_lim:	Bits [14:0] - Power Limit[0] for DDR domain,
 *				format: U11.3
 *		@pwr_lim_ctrl_en:Bits [15:15] - Power Limit[0] enable bit for
 *				DDR domain
 *		@rsvd0:		Bits [16:16]
 *		@ctrl_time_win:	Bits [23:17] - Power Limit[0] time window for
 *				DDR domain
 *		@rsvd1:		Bits [31:24]
 */
union peci_dram_power_limit {
	u32 value;
	struct {
		u32 pp_pwr_lim		: 15;
		u32 pwr_lim_ctrl_en	: 1;
		u32 rsvd0		: 1;
		u32 ctrl_time_win	: 7;
		u32 rsvd1		: 8;
	} __attribute__((__packed__)) bits;
} __attribute__((__packed__));

static_assert(sizeof(union peci_dram_power_limit) == PECI_PCS_REGISTER_SIZE);

/**
 * peci_pcs_xn_to_uunits - function converting value in units in x.N format to
 * micro units (microjoules, microseconds, microdegrees) in regular format
 * @x_n_value: Value in units in x.n format
 * @n: n factor for x.n format

 *
 * Return: value in micro units (microjoules, microseconds, microdegrees)
 * in regular format
 */
static inline u64 peci_pcs_xn_to_uunits(u32 x_n_value, u8 n)
{
	u64 mx_n_value = (u64)x_n_value * 1000000uLL;

	return mx_n_value >> n;
}

/**
 * peci_pcs_xn_to_munits - function converting value in units in x.N format to
 * milli units (millijoules, milliseconds, millidegrees) in regular format
 * @x_n_value: Value in units in x.n format
 * @n: n factor for x.n format

 *
 * Return: value in milli units (millijoules, milliseconds, millidegrees)
 * in regular format
 */
static inline u64 peci_pcs_xn_to_munits(u32 x_n_value, u8 n)
{
	u64 mx_n_value = (u64)x_n_value * 1000uLL;

	return mx_n_value >> n;
}

/**
 * peci_pcs_munits_to_xn - function converting value in milli units
 * (millijoules,milliseconds, millidegrees) in regular format to value in units
 * in x.n format
 * @mu_value: Value in milli units (millijoules, milliseconds, millidegrees)
 * @n: n factor for x.n format, assumed here maximal value for n is 32
 *
 * Return: value in units in x.n format
 */
static inline u32 peci_pcs_munits_to_xn(u32 mu_value, u8 n)
{
	/* Convert value in milli units (regular format) to the x.n format */
	u64 mx_n_value = (u64)mu_value << n;
	/* Convert milli units (x.n format) to units (x.n format) */
	if (mx_n_value > (u64)U32_MAX) {
		do_div(mx_n_value, 1000uL);
		return (u32)mx_n_value;
	} else {
		return (u32)mx_n_value / 1000uL;
	}
}

/**
 * peci_get_energy_ext_value - get energy value from extended energy container
 * @energy: Pointer to peci sensor data with extended energy reading
 *
 * Return: Energy value
 */
static inline u32 peci_get_energy_ext_value(struct peci_sensor_data *energy)
{
	struct peci_acc_energy_status_extended *energy_ext =
		(struct peci_acc_energy_status_extended *)&energy->uvalue;

	return energy_ext->energy;
}

/**
 * peci_get_energy_ext_elapsed - get time elapsed since previous extended energy reading
 * @prev_energy: Pointer to peci sensor data with previous extended energy reading
 * @curr_energy: Pointer to peci sensor data with current extended energy reading
 *
 * Return: Time elapsed since previous energy reading
 */
static inline ulong peci_get_energy_ext_elapsed
	(struct peci_sensor_data *prev_energy,
	 struct peci_sensor_data *curr_energy)
{
	ulong elapsed;

	struct peci_acc_energy_status_extended *prev_energy_ext =
		(struct peci_acc_energy_status_extended *)&prev_energy->uvalue;
	struct peci_acc_energy_status_extended *curr_energy_ext =
		(struct peci_acc_energy_status_extended *)&curr_energy->uvalue;

	if (curr_energy_ext->timestamp > prev_energy_ext->timestamp)
		elapsed = curr_energy_ext->timestamp - prev_energy_ext->timestamp;
	else
		elapsed = (U32_MAX - prev_energy_ext->timestamp) +
					curr_energy_ext->timestamp + 1u;

	return elapsed;
}

/**
 * peci_pcs_parse_energy_data - parse CPU energy data
 * @prev_energy: Previous energy reading context with raw energy counter value
 * @curr_energy: Current energy reading context with raw energy counter value
 * @extended: True if energy timestamp was read from CPU PCS 3
 * @energy: Current energy reading context with raw energy counter value
 * @prev_energy_val: Pointer to previous energy value
 * @curr_energy_val: Pointer to current energy value
 * @elapsed: Pointer to time elapsed since previous energy reading
 * @time_unit: Pointer to time unit
 */
static inline void peci_pcs_parse_energy_data(struct peci_sensor_data *prev_energy,
					      struct peci_sensor_data *curr_energy,
					      bool extended,
					      u32 *prev_energy_val,
					      u32 *curr_energy_val,
					      ulong *elapsed,
					      u32 *time_unit)
{
	if (extended) {
		*prev_energy_val = peci_get_energy_ext_value(prev_energy);
		*curr_energy_val = peci_get_energy_ext_value(curr_energy);
		*elapsed = peci_get_energy_ext_elapsed(prev_energy, curr_energy);
		*time_unit = EXTENDED_ENERGY_TIME_UNIT_10NS;
	} else {
		*prev_energy_val = (u32)prev_energy->uvalue;
		*curr_energy_val = (u32)curr_energy->uvalue;
		*elapsed = curr_energy->last_updated - prev_energy->last_updated;
		*time_unit = HZ;
	}
}

/**
 * peci_pcs_read - read PCS register
 * @peci_mgr: PECI client manager handle
 * @index: PCS index
 * @parameter: PCS parameter
 * @reg: Pointer to the variable read value is going to be put
 * @data_len: Number of bytes to read into @reg
 *
 * Return: 0 if succeeded,
 *	-EINVAL if there are null pointers among arguments,
 *	other values in case other errors.
 */
static inline int peci_pcs_read(struct peci_client_manager *peci_mgr, u8 index,
				u16 parameter, u8 *reg, u8 data_len)
{
	u8 pcs_reg[PECI_RDPKGCFG_PKGCFG_LEN] = { 0 };
	int ret;

	if (!reg || data_len > PECI_RDPKGCFG_PKGCFG_LEN)
		return -EINVAL;

	ret = peci_client_read_package_config(peci_mgr, index, parameter,
					      pcs_reg, data_len);

	if (!ret) {
		if (data_len <= sizeof(u32)) {
			u32 pkgcfg_reg = le32_to_cpup((__le32 *)pcs_reg);

			memcpy(reg, &pkgcfg_reg, data_len);
		} else if (data_len == sizeof(u64)) {
			u64 pkgcfg_reg = le64_to_cpup((__le64 *)pcs_reg);

			memcpy(reg, &pkgcfg_reg, data_len);
		} else {
			return -EINVAL;
		}
	}
	return ret;
}

/**
 * peci_pcs_write - write PCS register
 * @peci_mgr: PECI client manager handle
 * @index: PCS index
 * @parameter: PCS parameter
 * @reg: Variable which value is going to be written to the PCS
 *
 * Return: 0 if succeeded, other values in case an error.
 */
static inline int peci_pcs_write(struct peci_client_manager *peci_mgr, u8 index,
				 u16 parameter, u8 *reg, u8 data_len)
{
	int ret;

	ret = peci_client_write_package_config(peci_mgr, index, parameter, reg,
					       data_len);

	return ret;
}

/**
 * peci_pcs_calc_pwr_from_eng - calculate power (in milliwatts) based on
 * two energy readings
 * @dev: Device handle
 * @prev_energy: Previous energy reading context with raw energy counter value
 * @energy: Current energy reading context with raw energy counter value
 * @unit: Calculation factor
 * @extended: True if energy timestamp was read from CPU PCS 3
 * @power_val_in_mW: Pointer to the variable calculation result is going to
 * be put
 *
 * Return: 0 if succeeded,
 *	-EINVAL if there are null pointers among arguments,
 *	-EAGAIN if calculation is skipped.
 */
static inline int peci_pcs_calc_pwr_from_eng(struct device *dev,
					     struct peci_sensor_data *prev_energy,
					     struct peci_sensor_data *energy,
					     u32 unit, bool extended,
					     s32 *power_in_mW)
{
	u32 prev_energy_val;
	u32 curr_energy_val;
	ulong elapsed;
	u32 time_unit;
	int ret;

	peci_pcs_parse_energy_data(prev_energy, energy, extended,
				   &prev_energy_val, &curr_energy_val,
				   &elapsed, &time_unit);

	dev_dbg(dev, "raw energy before %u, raw energy now %u, energy unit %u, time elapsed %lu, time unit %u\n",
		prev_energy_val, curr_energy_val, unit, elapsed, time_unit);

	/*
	 * TODO: Remove checking current energy value against 0.
	 * During host reset CPU resets its energy counters, hwmon treats such case
	 * as proper energy read (counter overflow) and calculates invalid
	 * power consumption. Currently hwmon is unable to determine if CPU was
	 * reset, stop treating 0 as invalid value when proper mechanism
	 * is implemented.
	 *
	 * Don't calculate average power for first counter read  last counter
	 * read was more than 60 minutes ago (jiffies did not wrap and power
	 * calculation does not overflow or underflow) or energy read time
	 * did not change.
	 */
	if (curr_energy_val > 0 && prev_energy->last_updated > 0 &&
	    elapsed < ((u64)time_unit * 3600) && elapsed) {
		u32 energy_consumed;
		u64 energy_consumed_in_mJ;
		u64 energy_by_time_unit;

		if (curr_energy_val >= prev_energy_val)
			energy_consumed = curr_energy_val - prev_energy_val;
		else
			energy_consumed = (U32_MAX - prev_energy_val) + curr_energy_val + 1u;

		energy_consumed_in_mJ =
				peci_pcs_xn_to_munits(energy_consumed, unit);
		energy_by_time_unit = energy_consumed_in_mJ * time_unit;

		if (energy_by_time_unit > (u64)U32_MAX) {
			do_div(energy_by_time_unit, elapsed);
			*power_in_mW = (long)energy_by_time_unit;
		} else {
			*power_in_mW = (u32)energy_by_time_unit / elapsed;
		}

		dev_dbg(dev, "raw energy consumed %u, scaled energy consumed %llumJ, scaled power %dmW\n",
			energy_consumed, energy_consumed_in_mJ, *power_in_mW);

		ret = 0;
	} else {
		dev_dbg(dev, "skipping calculate power, try again\n");
		*power_in_mW = 0;
		ret = -EAGAIN;
	}

	prev_energy->uvalue = energy->uvalue;
	peci_sensor_mark_updated_with_time(prev_energy, energy->last_updated);

	return ret;
}

/**
 * peci_pcs_calc_acc_eng - calculate accumulated energy (in microjoules) based
 * on two energy readings
 * @dev: Device handle
 * @prev_energy: Previous energy reading context with raw energy counter value
 * @energy: Current energy reading context with raw energy counter value
 * @unit: Calculation factor
 * @extended: True if energy timestamp was read from CPU PCS 3
 * @acc_energy_in_uJ: Pointer to the variable with cumulative energy counter
 *
 * Return: 0 if succeeded,
 *	-EINVAL if there are null pointers among arguments,
 *	-EAGAIN if calculation is skipped.
 */
static inline int peci_pcs_calc_acc_eng(struct device *dev,
					struct peci_sensor_data *prev_energy,
					struct peci_sensor_data *curr_energy,
					u32 unit, bool extended,
					s32 *acc_energy_in_uJ)
{
	u32 prev_energy_val;
	u32 curr_energy_val;
	ulong elapsed;
	u32 time_unit;
	int ret;

	peci_pcs_parse_energy_data(prev_energy, curr_energy, extended,
				   &prev_energy_val, &curr_energy_val,
				   &elapsed, &time_unit);

	dev_dbg(dev, "raw energy before %u, raw energy now %u, energy unit %u, time elapsed %lu, time unit %u\n",
		prev_energy_val, curr_energy_val, unit, elapsed, time_unit);

	/*
	 * TODO: Remove checking current energy value against 0.
	 * During host reset CPU resets its energy counters, hwmon treats such case
	 * as proper energy read (counter overflow) and calculates invalid
	 * energy increase. Currently hwmon is unable to determine if CPU was
	 * reset, stop treating 0 as invalid value when proper mechanism
	 * is implemented.
	 *
	 * Don't calculate cumulative energy for first counter read - last counter
	 * read was more than 17 minutes ago (jiffies and energy raw counter did not wrap
	 * and power calculation does not overflow or underflow).
	 */
	if (curr_energy_val > 0 && prev_energy->last_updated > 0 &&
	    elapsed < ((u64)time_unit * 17 * 60)) {
		u32 energy_consumed;
		u64 energy_consumed_in_uJ;

		if (curr_energy_val >= prev_energy_val)
			energy_consumed = curr_energy_val - prev_energy_val;
		else
			energy_consumed = (U32_MAX - prev_energy_val) +
					curr_energy_val + 1u;

		energy_consumed_in_uJ =
				peci_pcs_xn_to_uunits(energy_consumed, unit);
		*acc_energy_in_uJ = S32_MAX &
				((u64)*acc_energy_in_uJ + energy_consumed_in_uJ);

		dev_dbg(dev, "raw energy %u, scaled energy %llumJ, cumulative energy %dmJ\n",
			energy_consumed, energy_consumed_in_uJ,
			*acc_energy_in_uJ);

		ret = 0;
	} else {
		dev_dbg(dev, "skipping calculate cumulative energy, try again\n");

		*acc_energy_in_uJ = 0;
		ret = -EAGAIN;
	}

	prev_energy->uvalue = curr_energy->uvalue;
	peci_sensor_mark_updated_with_time(prev_energy,
					   curr_energy->last_updated);

	return ret;
}

/**
 * peci_pcs_get_units - read units (power, energy, time) from HW or cache
 * @peci_mgr: PECI client manager handle
 * @units: Pointer to the variable read value is going to be put in case reading
 * from HW
 * @valid: Flag telling cache is valid
 *
 * Return: 0 if succeeded
 *	-EINVAL if there are null pointers among arguments,
 *	other values in case other errors.
 */
static inline int peci_pcs_get_units(struct peci_client_manager *peci_mgr,
				     union peci_pkg_power_sku_unit *units,
				     bool *valid)
{
	int ret = 0;

	if (!valid)
		return -EINVAL;

	if (!(*valid)) {
		ret = peci_pcs_read(peci_mgr, PECI_MBX_INDEX_TDP_UNITS,
				    PECI_PCS_PARAM_ZERO, (u8 *)&units->value,
				    sizeof(units->value));
		if (!ret)
			*valid = true;
	}
	return ret;
}

/**
 * peci_pcs_calc_plxy_time_window - calculate power limit time window in
 * PCS format. To figure that value out needs to solve the following equation:
 * time_window = (1+(x/4)) * (2 ^ y), where time_window is known value and
 * x and y values are variables to find.
 * Return value is about X & Y compostion according to the following:
 * x = ret[6:5], y = ret[4:0].
 * @pl_tim_wnd_in_xn: PPL time window in X-n format
 *
 * Return: Power limit time window value
 */
static inline u32 peci_pcs_calc_plxy_time_window(u32 pl_tim_wnd_in_xn)
{
	u32 x = 0u;
	u32 y = 0u;

	/* Calculate y first */
	while (pl_tim_wnd_in_xn > 7u) {
		pl_tim_wnd_in_xn >>= 1;
		y++;
	}

	/* Correct y value */
	if (pl_tim_wnd_in_xn >= 4u)
		y += 2u;
	else if (pl_tim_wnd_in_xn >= 2u)
		y += 1u;

	/* Calculate x then */
	if (pl_tim_wnd_in_xn >= 4u)
		x = pl_tim_wnd_in_xn % 4;
	else
		x = 0u;

	return ((x & 0x3) << 5) | (y & 0x1F);
}

#endif /* __PECI_HWMON_H */
