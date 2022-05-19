/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022 Intel Corporation */

#ifndef _UAPI_LINUX_I3C_MCTP_H
#define _UAPI_LINUX_I3C_MCTP_H

#include <linux/ioctl.h>
#include <linux/types.h>

/*
 * maximum possible number of struct eid_info elements stored in list
 */
#define I3C_MCTP_EID_INFO_MAX 256

/*
 * MCTP operations
 * @I3C_MCTP_SET_EID_INFO: write or overwrite already existing list of
 * CPU EID and Domain ID mappings
 */

struct i3c_mctp_eid_info {
	__u8 eid;
	__u8 dyn_addr;
	__u8 domain_id;
};

struct i3c_mctp_set_eid_info {
	__u64 ptr;
	__u16 count;
};

#define I3C_MCTP_IOCTL_BASE    0x69

#define I3C_MCTP_IOCTL_SET_EID_INFO \
	_IOW(I3C_MCTP_IOCTL_BASE, 0x41, struct i3c_mctp_set_eid_info)

#endif /* _UAPI_LINUX_I3C_MCTP_H */
