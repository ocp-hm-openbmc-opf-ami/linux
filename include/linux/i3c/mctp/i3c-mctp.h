/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2022 Intel Corporation.*/

#ifndef I3C_MCTP_H
#define I3C_MCTP_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define I3C_MCTP_PACKET_SIZE		68
#define I3C_MCTP_PAYLOAD_SIZE		64
#define I3C_MCTP_HDR_SIZE		4

/* Extended packet structure for AST2700 */
#define I3C_MCTP_PACKET_SIZE_ast27	69
#define I3C_MCTP_PAYLOAD_SIZE_ast27	65
#define I3C_MCTP_MIN_PACKET_SIZE	5

/* PECI MCTP Intel VDM definitions */
#define MCTP_MSG_TYPE_VDM_PCI		0x7E
#define MCTP_VDM_PCI_INTEL_VENDOR_ID	0x8086
#define MCTP_VDM_PCI_INTEL_PECI		0x2

/* MCTP message header offsets */
#define MCTP_MSG_HDR_MSG_TYPE_OFFSET	0
#define MCTP_MSG_HDR_VENDOR_OFFSET	1
#define MCTP_MSG_HDR_OPCODE_OFFSET	4

struct i3c_mctp_client;

struct mctp_protocol_hdr {
	u8 ver;
	u8 dest;
	u8 src;
	u8 flags_seq_tag;
} __packed;

struct i3c_mctp_packet_data {
	u8 protocol_hdr[I3C_MCTP_HDR_SIZE];
	u8 payload[I3C_MCTP_PAYLOAD_SIZE];
};

struct i3c_mctp_packet {
	struct i3c_mctp_packet_data data;
	u32 size;
};

struct ast27_i3c_mctp_packet_data {
	u8 protocol_hdr[I3C_MCTP_HDR_SIZE];
	u8 payload[I3C_MCTP_PAYLOAD_SIZE_ast27];
};

struct ast27_i3c_mctp_packet {
	struct ast27_i3c_mctp_packet_data data;
	u32 size;
};

/* IOCTL interface for EID management */
#define I3C_MCTP_EID_INFO_MAX	256

struct i3c_mctp_eid_info {
	__u8 eid;
	__u8 dyn_addr;
	__u8 domain_id;
};

struct i3c_mctp_set_eid_info {
	__u64 ptr;
	__u16 count;
};

struct i3c_mctp_set_own_eid {
	__u8 eid;
};

#define I3C_MCTP_IOCTL_BASE		0x69

#define I3C_MCTP_IOCTL_SET_EID_INFO \
	_IOW(I3C_MCTP_IOCTL_BASE, 0x41, struct i3c_mctp_set_eid_info)
#define I3C_MCTP_IOCTL_SET_OWN_EID \
	_IOW(I3C_MCTP_IOCTL_BASE, 0x42, struct i3c_mctp_set_own_eid)
#define I3C_MCTP_IOCTL_REGISTER_DEFAULT_CLIENT \
	_IO(I3C_MCTP_IOCTL_BASE, 0x43)

void *i3c_mctp_packet_alloc(gfp_t flags);
void i3c_mctp_packet_free(void *packet);

int i3c_mctp_get_eid(struct i3c_mctp_client *client, u8 domain_id, u8 *eid);
int i3c_mctp_send_packet(struct i3c_device *i3c, struct i3c_mctp_packet *tx_packet);
struct i3c_mctp_packet *i3c_mctp_receive_packet(struct i3c_mctp_client *client,
						unsigned long timeout);
struct i3c_mctp_client *i3c_mctp_add_peci_client(struct i3c_device *i3c);
void i3c_mctp_remove_peci_client(struct i3c_mctp_client *client);

#endif /* I3C_MCTP_H */
