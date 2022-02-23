/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022 Intel Corporation */

#ifndef SMART_H
#define SMART_H

#include <linux/device.h>

/**
 * struct peci_throttling_ops - ops shared by hwmon PECI devices to SmaRT
 * module, used to enable/disable max power throttling.
 * @request_max_power_throttling: Called by SmaRT module on SMBAlert# interrupt
 *				  to request max power limit to be set in
 *				  hardware over PECI interface.
 * @remove_max_power_throttling: Called by SmaRT module when SMBAlert# interrupt
 *				 handler ends processing. It requests to undo
 *				 previously set max power throttling in hardware
 *				 over PECI interface.
 */
struct peci_throttling_ops {
	int (*request_max_power_throttling)(const struct device *dev);
	int (*remove_max_power_throttling)(const struct device *dev);
};

int smart_register_psu(struct device *dev);
int smart_unregister_psu(struct device *dev);
int smart_register_peci(struct device *dev, struct peci_throttling_ops *peci_smart_ops);
int smart_unregister_peci(struct device *dev);

#endif // SMART_H
