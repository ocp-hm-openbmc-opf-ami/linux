// SPDX-License-Identifier: GPL-2.0
/*
 * configfs_ami.c - AMI-specific USB gadget configfs extensions
 *
 * Copyright (C) 2025 American Megatrends International, LLC
 */

#include <linux/configfs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget_configfs.h>

#include "function/u_ecm.h"
#include "function/u_ether.h"
#include "function/u_rndis.h"

void ami_bind_ecm_rndis_shared(struct usb_gadget *gadget,
			       struct list_head *available_func);

/**
 * find_ecm_rndis_pair - Locate ECM and RNDIS function instances
 * @available_func: pointer to the list of available function instances
 * @ecm_fi: pointer to store found ECM function instance
 * @rndis_fi: pointer to store found RNDIS function instance
 *
 * Searches for ECM (ecm.ami) and RNDIS (rndis.ami) function instances
 * within the provided available_func list.
 *
 * Return: true if both instances are found, false otherwise
 */
static bool
find_ecm_rndis_pair(struct list_head *available_func,
		    struct usb_function_instance **ecm_fi,
		    struct usb_function_instance **rndis_fi)
{
	struct usb_function_instance *tmp_fi;
	*ecm_fi = *rndis_fi = NULL;

	list_for_each_entry(tmp_fi, available_func, cfs_list) {

		/* Check if this is the ECM instance we're looking for */
		if (!*ecm_fi && strcmp(tmp_fi->group.cg_item.ci_name, "ecm.ami") == 0) {
			*ecm_fi = tmp_fi;
		}
		/* Check if this is the RNDIS instance we're looking for */
		else if (!*rndis_fi && strcmp(tmp_fi->group.cg_item.ci_name, "rndis.ami") == 0) {
			*rndis_fi = tmp_fi;
		}

		/* Early exit if both instances found */
		if (*ecm_fi && *rndis_fi)
			break;
	}

	return (*ecm_fi && *rndis_fi);
}

/**
 * ami_bind_ecm_rndis_shared - Bind ECM with RNDIS network sharing
 * @gadget: pointer to the USB gadget device
 * @available_func: pointer to the list of available function instances
 *
 * Early binds ECM network interface and configures RNDIS to borrow its
 * network device.
 *
 * Steps performed:
 * 1. Locate ECM and RNDIS function instances from the available_func list
 * 2. Early bind and register ECM's network interface
 * 3. Make RNDIS borrow ECM's network device (only on first UDC binding)
 */
void ami_bind_ecm_rndis_shared(struct usb_gadget *gadget,
			       struct list_head *available_func)
{
	struct usb_function_instance *ecm_fi, *rndis_fi;
	struct f_ecm_opts *ecm_opts;
	struct f_rndis_opts *rndis_opts;
	int ret;

	/* Find both ECM and RNDIS instances */
	if (!find_ecm_rndis_pair(available_func, &ecm_fi, &rndis_fi)) {
		return;
	}

	ecm_opts = container_of(ecm_fi, struct f_ecm_opts, func_inst);

	/* Early bind and register ECM network interface if not already bound */
	if (ecm_opts->net && !ecm_opts->bound) {
		gether_set_gadget(ecm_opts->net, gadget);
		ret = gether_register_netdev(ecm_opts->net);
		if (ret) {
			return;
		}
		ecm_opts->bound = true;
	}

	rndis_opts = container_of(rndis_fi, struct f_rndis_opts, func_inst);

	/*
	 * During configfs setup, RNDIS creates a default network device.
	 * To share the ECM network device to RNDIS, the original RNDIS
	 * device must be released. This only works during first UDC binding.
	 */
	if (!rndis_opts->borrowed_net && !device_is_registered(&rndis_opts->net->dev)) {
		rndis_borrow_net(rndis_fi, ecm_opts->net);
	}
}
EXPORT_SYMBOL_GPL(ami_bind_ecm_rndis_shared);
