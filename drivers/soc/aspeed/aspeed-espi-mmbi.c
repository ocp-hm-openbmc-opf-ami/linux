// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021, Intel Corporation.

#include <linux/bitfield.h>
#include <linux/crc8.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <uapi/linux/aspeed-espi-mmbi.h>
#include <dt-bindings/mmbi/protocols.h>

#define DEVICE_NAME "mmbi"
#define MAX_NO_OF_SUPPORTED_CHANNELS 8
#define MAX_NO_OF_SUPPORTED_PROTOCOLS 5

/* 20 Bits for H2B/B2H Write/Read Pointers */
#define H2B_WRITE_POINTER_MASK GENMASK(19, 0)
#define B2H_READ_POINTER_MASK GENMASK(19, 0)
#define MMBI_HDR_LENGTH_MASK GENMASK(23, 0)
#define MMBI_HDR_TYPE_MASK GENMASK(31, 24)
#define HOST_RESET_REQUEST_BIT BIT(31)
#define HOST_READY_BIT BIT(31)
#define ESPI_SCI_STATUS_BIT BIT(24)

#define GET_H2B_WRITE_POINTER(x) ((x) & H2B_WRITE_POINTER_MASK)
#define GET_B2H_READ_POINTER(x) ((x) & B2H_READ_POINTER_MASK)
#define GET_HOST_RESET_REQ_BIT(x) ((x) & HOST_RESET_REQUEST_BIT)
#define GET_HOST_READY_BIT(x) ((x) & HOST_READY_BIT)
#define HOST_READ_SCI_STATUS_BIT(x) ((x) & ESPI_SCI_STATUS_BIT)

#define MMBI_CRC8_POLYNOMIAL 0x07
DECLARE_CRC8_TABLE(mmbi_crc8_table);

typedef u8 protocol_type;

struct host_rop {
	unsigned int
		b2h_wp : 20; /* Offset where BMC can write next data in B2H */
	unsigned int reserved1 : 11;
	unsigned int b_rdy : 1; /* BMC ready bit */
	unsigned int h2b_rp : 20; /* Offset till where bmc read data in H2B */
	unsigned int reserved2 : 11;
	unsigned int b_rst : 1; /* BMC reset request bit */
};

struct host_rwp {
	unsigned int
		h2b_wp : 20; /* Offset where HOST can write next data in H2B */
	unsigned int reserved1 : 11;
	unsigned int h_rdy : 1; /* Host ready bit */
	unsigned int b2h_rp : 20; /* Offset till where host read data in B2H */
	unsigned int reserved2 : 11;
	unsigned int h_rst : 1; /* host reset request bit */
};

struct buffer_type_desc {
	u32 host_rop_p;
	u32 host_rwp_p;
	u8 msg_protocol_type;
	u8 host_int_type;
	u16 global_sys_interrupt;
	u8 bmc_int_type;
	u32 bmc_int_a;
	u8 bmc_int_v;
} __packed;

struct mmbi_cap_desc {
	u8 signature[6];
	u8 version;
	u8 instance_num;
	u32 nex_inst_base_addr;
	u32 b2h_ba; /* B2H buffer base offset (i.e 0x48) */
	u32 h2b_ba; /* H2B buffer base offset (i.e 0x08) */
	u16 b2h_d; /* Multiple of 16 Bytes (Max 1MB) */
	u16 h2b_d; /* multiples of 16 bytes (Max 1MB) */
	u8 buffer_type; /* Type of buffer in B2H/H2B */
	u8 reserved1[7];
	struct buffer_type_desc bt_desc; /* 18 bytes */
	u8 reserved2[13];
	u8 crc8; /* CRC-8-CCITT of the whole data structure (bytes 0 to 62) */
} __packed;

struct mmbi_header {
	u32 data;
};

struct aspeed_mmbi_protocol {
	struct miscdevice miscdev;
	struct aspeed_mmbi_channel *chan_ref;
	protocol_type type;

	bool data_available;
	/*
	 * If user space application is opened for read, then only process
	 * the data and copy to userspace. Otherwise, discard the command and
	 * process the remaining commands (can be different protocol type)
	 */
	bool process_data;
	wait_queue_head_t queue;
};

struct aspeed_mmbi_channel {
	struct aspeed_mmbi_protocol protocol[MAX_NO_OF_SUPPORTED_PROTOCOLS];
	struct aspeed_espi_mmbi *priv;

	u8 chan_num;
	u8 supported_protocols[MAX_NO_OF_SUPPORTED_PROTOCOLS];
	u32 b2h_cb_size;
	u32 h2b_cb_size;
	u8 *desc_vmem;
	u8 *hrop_vmem;
	u8 *b2h_cb_vmem;
	u8 *hrwp_vmem;
	u8 *h2b_cb_vmem;
	bool enabled;
};

struct aspeed_espi_mmbi {
	struct regmap *map;
	struct regmap *pmap;
	struct regmap *lpc_map;
	struct device *dev;

	int irq;
	phys_addr_t host_map_addr;
	dma_addr_t mmbi_phys_addr;
	resource_size_t mmbi_size;
	u8 *dma_vaddr;

	struct aspeed_mmbi_channel chan[MAX_NO_OF_SUPPORTED_CHANNELS];
};

static const struct regmap_config aspeed_espi_mmbi_regmap_cfg = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = 0x04C,
};

static void raise_sci_interrupt(struct aspeed_mmbi_channel *channel)
{
	u32 val;
	int retry;
	struct regmap *lpc_regmap = channel->priv->lpc_map;

	dev_dbg(channel->priv->dev, "Raising SCI interrupt...\n");

	regmap_write_bits(lpc_regmap, AST_LPC_ACPIB7B4, LPC_BMC_TRIG_SCI_EVT_EN,
			  LPC_BMC_TRIG_SCI_EVT_EN);

	regmap_write_bits(lpc_regmap, AST_LPC_SWCR0704,
			  LPC_BMC_TRIG_WAKEUP_EVT_EN,
			  LPC_BMC_TRIG_WAKEUP_EVT_EN);

	regmap_write_bits(lpc_regmap, AST_LPC_SWCR0B08, LPC_BMC_TRIG_WAKEUP_EVT,
			  LPC_BMC_TRIG_WAKEUP_EVT);

	/*
	 * Just asserting the SCI VW will trigger the SCI event continuosly.
	 * So BMC must deassert SCI VW to avoid it.
	 * ESPI098[24] reading will confirm Host read data or not.
	 * - 0 means host read the data
	 * - 1 means host not yet read data, so retry with 1us delay.
	 */
	retry = 30;
	while (retry) {
		if (regmap_read(channel->priv->pmap, ASPEED_ESPI_SYS_EVENT,
				&val)) {
			dev_err(channel->priv->dev, "Unable to read ESPI098\n");
			break;
		}

		if (HOST_READ_SCI_STATUS_BIT(val) == 0)
			break;

		retry--;
		dev_dbg(channel->priv->dev,
			"Host SCI handler not invoked(ESPI098: 0x%0x), so retry(%d) after 1us...\n",
			val, retry);
		udelay(1);
	}

	regmap_write_bits(lpc_regmap, AST_LPC_SWCR0300,
			  LPC_BMC_TRIG_WAKEUP_EVT_STS,
			  LPC_BMC_TRIG_WAKEUP_EVT_STS);

	regmap_write_bits(lpc_regmap, AST_LPC_ACPIB3B0,
			  LPC_BMC_TRIG_SCI_EVT_STS, LPC_BMC_TRIG_SCI_EVT_STS);
}

static int read_host_rwp_val(struct aspeed_mmbi_channel *channel, u32 reg,
			     u32 *val)
{
	int rc;

	rc = regmap_read(channel->priv->map, reg, val);
	if (rc) {
		dev_err(channel->priv->dev,
			"Unable to read Host RWP pointer\n");
		return rc;
	}

	return 0;
}

static int get_b2h_avail_buf_len(struct aspeed_mmbi_channel *channel,
				 ssize_t *avail_buf_len)
{
	struct host_rop hrop;
	u32 b2h_rp, h_rwp1;

	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP1_INSTANCE0, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}

	b2h_rp = GET_B2H_READ_POINTER(h_rwp1);
	dev_dbg(channel->priv->dev, "MMBI HRWP - b2h_rp: 0x%0x\n", b2h_rp);

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	dev_dbg(channel->priv->dev, "HROP - b2h_wp: 0x%0x, h2b_rp: 0x%0x",
		hrop.b2h_wp, hrop.h2b_rp);

	if (hrop.b2h_wp >= b2h_rp)
		*avail_buf_len = channel->b2h_cb_size - hrop.b2h_wp + b2h_rp;
	else
		*avail_buf_len = b2h_rp - hrop.b2h_wp;

	return 0;
}

static int get_mmbi_header(struct aspeed_mmbi_channel *channel,
			   u32 *data_length, u8 *type, u32 *unread_data_len)
{
	u32 h2b_wp, b2h_rp, h_rwp0, h_rwp1;
	struct mmbi_header header;
	struct host_rop hrop;

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	dev_dbg(channel->priv->dev,
		"MMBI HROP - b2h_wp: 0x%0x, h2b_rp: 0x%0x\n", hrop.b2h_wp,
		hrop.h2b_rp);

	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP0_INSTANCE0, &h_rwp0)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}
	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP1_INSTANCE0, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}
	h2b_wp = GET_H2B_WRITE_POINTER(h_rwp0);
	b2h_rp = GET_B2H_READ_POINTER(h_rwp1);
	dev_dbg(channel->priv->dev, "MMBI HRWP - h2b_wp: 0x%0x, b2h_rp: 0x%0x\n", h2b_wp, b2h_rp);

	if (h2b_wp >= hrop.h2b_rp)
		*unread_data_len = h2b_wp - hrop.h2b_rp;
	else
		*unread_data_len = channel->h2b_cb_size - hrop.h2b_rp + h2b_wp;

	if (*unread_data_len < sizeof(struct mmbi_header)) {
		dev_dbg(channel->priv->dev, "No data to read(%d -%d)\n", h2b_wp,
			hrop.h2b_rp);
		return -EAGAIN;
	}

	dev_dbg(channel->priv->dev, "READ MMBI header from: 0x%0x\n",
		(u32)(channel->h2b_cb_vmem + hrop.h2b_rp));

	/* Extract MMBI protocol - protocol type and length */
	if ((hrop.h2b_rp + sizeof(header)) <= channel->h2b_cb_size) {
		memcpy(&header, channel->h2b_cb_vmem + hrop.h2b_rp,
		       sizeof(header));
	} else {
		ssize_t chunk_len = channel->h2b_cb_size - hrop.h2b_rp;

		memcpy(&header, channel->h2b_cb_vmem + hrop.h2b_rp, chunk_len);
		memcpy(((u8 *)&header) + chunk_len, channel->h2b_cb_vmem,
		       sizeof(header) - chunk_len);
	}

	*data_length = FIELD_GET(MMBI_HDR_LENGTH_MASK, header.data);
	*type = FIELD_GET(MMBI_HDR_TYPE_MASK, header.data);

	return 0;
}

static void raise_missing_sci(struct aspeed_mmbi_channel *channel)
{
	struct host_rop hrop;
	u32 h_rwp0, h_rwp1, b2h_rptr;

	/* Rise SCI only if Host is READY (h_rdy is 1). */
	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP0_INSTANCE0, &h_rwp0)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return;
	}
	if (!GET_HOST_READY_BIT(h_rwp0)) {
		// Host is not ready, no point in raising the SCI
		return;
	}

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP1_INSTANCE0, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return;
	}
	b2h_rptr = GET_B2H_READ_POINTER(h_rwp1);

	if (hrop.b2h_wp == b2h_rptr) {
		// Host has read all outstanding SCI data,
		// Do not raise another SCI.
		return;
	}

	dev_dbg(channel->priv->dev,
		"Host not read the data yet, so rising SCI interrupt again...\n");
	raise_sci_interrupt(channel);
}

static void update_host_rop(struct aspeed_mmbi_channel *channel,
			    unsigned int w_len, unsigned int r_len)
{
	struct host_rop hrop;

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	dev_dbg(channel->priv->dev,
		"MMBI HROP - b2h_wp: 0x%0x, h2b_rp: 0x%0x\n", hrop.b2h_wp,
		hrop.h2b_rp);

	/* Advance the B2H CB offset for next write */
	if ((hrop.b2h_wp + w_len) <= channel->b2h_cb_size)
		hrop.b2h_wp += w_len;
	else
		hrop.b2h_wp = hrop.b2h_wp + w_len - channel->b2h_cb_size;

	/* Advance the H2B CB offset till where BMC read data */
	if ((hrop.h2b_rp + r_len) <= channel->h2b_cb_size)
		hrop.h2b_rp += r_len;
	else
		hrop.h2b_rp = hrop.h2b_rp + r_len - channel->h2b_cb_size;

	/*
	 * Clear BMC reset request state its set:
	 * Set BMC reset request bit to 0
	 * Set BMC ready bit to 1
	 */
	if (hrop.b_rst) {
		dev_dbg(channel->priv->dev,
			"Clearing BMC reset request state\n");
		hrop.b_rst = 0;
		hrop.b_rdy = 1;
	}

	dev_dbg(channel->priv->dev,
		"Updating HROP - h2b_rp: 0x%0x, b2h_wp: 0x%0x\n", hrop.h2b_rp,
		hrop.b2h_wp);
	memcpy(channel->hrop_vmem, &hrop, sizeof(hrop));

	/*
	 * Raise SCI interrupt only if B2H buffer is updated
	 * Don't raise SCI, after BMC read the H2B buffer
	 */
	if (w_len != 0)
		raise_sci_interrupt(channel);
}

static int send_bmc_reset_request(struct aspeed_mmbi_channel *channel)
{
	struct host_rop hrop;

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	/*
	 * Send MMBI buffer reset request: First BMC should clear its own
	 * pointers, set Reset bit and reset BMC ready bit.
	 * B2H Write pointer - must be set to zero
	 * H2B read pointer - must be set to zero
	 * BMC ready bit - Set to 0
	 * BMC reset bit - Set to 1
	 */
	hrop.b2h_wp = 0;
	hrop.h2b_rp = 0;
	hrop.b_rdy = 0;
	hrop.b_rst = 1;

	dev_info(channel->priv->dev,
		 "Send BMC reset request on MMBI channel(%d)\n",
		 channel->chan_num);

	memcpy(channel->hrop_vmem, &hrop, sizeof(hrop));

	/* Raise SCI interrupt */
	raise_sci_interrupt(channel);

	return 0;
}

void check_host_reset_request(struct aspeed_mmbi_channel *channel)
{
	struct host_rop hrop;
	u32 h_rwp1;

	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP1_INSTANCE0, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return;
	}

	/* If its not host reset request, just discard */
	if (!GET_HOST_RESET_REQ_BIT(h_rwp1))
		return;

	/*  Host requested for MMBI buffer reset */
	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	/*
	 * When host request for reset MMBI buffer:
	 * B2H Write pointer - must be set to zero
	 * H2B read pointer - must be set to zero
	 * BMC ready bit - No change (Set to 1)
	 * BMC reset bit - No change (Set to 0)
	 */
	hrop.b2h_wp = 0;
	hrop.h2b_rp = 0;

	dev_info(channel->priv->dev,
		 "Handle Host reset request on MMBI channel(%d)\n",
		 channel->chan_num);

	memcpy(channel->hrop_vmem, &hrop, sizeof(hrop));
}

void wake_up_device(struct aspeed_mmbi_channel *channel)
{
	u32 req_data_len, unread_data_len;
	u8 type;
	int i;

	if (0 !=
	    get_mmbi_header(channel, &req_data_len, &type, &unread_data_len)) {
		/* Bail out as we can't read header */
		return;
	}
	dev_dbg(channel->priv->dev, "%s: Length: 0x%0x, Protocol Type: %d\n",
		__func__, req_data_len, type);

	for (i = 0; channel->supported_protocols[i] != 0; i++) {
		if (type == channel->supported_protocols[i]) {
			/*
			 * MMBI supports multiple protocols on each channel
			 * If userspace application is not opened the device
			 * for read /write the data, discard the data and
			 * advance the HROP for processing next command.
			 */
			if (channel->protocol[i].process_data) {
				channel->protocol[i].data_available = true;
				wake_up(&channel->protocol[i].queue);
			} else {
				/* Discard data and advance the hrop */
				update_host_rop(channel, 0,
						req_data_len +
						sizeof(struct mmbi_header));
			}
			/*
			 * Raise the missing SCI's by checking pointer for host
			 * read acknowledgment. This will work around the Missing
			 * SCI bug on host side.
			 */
			dev_warn(channel->priv->dev,
				 "%s: Check and raise missing SCI\n", __func__);
			raise_missing_sci(channel);
		}
	}
}

static struct aspeed_mmbi_protocol *file_aspeed_espi_mmbi(struct file *file)
{
	return container_of(file->private_data, struct aspeed_mmbi_protocol,
			    miscdev);
}

static int mmbi_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int mmbi_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static unsigned int mmbi_poll(struct file *filp, poll_table *wait)
{
	struct aspeed_mmbi_protocol *protocol = file_aspeed_espi_mmbi(filp);

	poll_wait(filp, &protocol->queue, wait);

	return protocol->data_available ? POLLIN : 0;
}

static ssize_t mmbi_read(struct file *filp, char *buff, size_t count,
			 loff_t *offp)
{
	struct aspeed_mmbi_protocol *protocol = file_aspeed_espi_mmbi(filp);
	struct aspeed_mmbi_channel *channel = protocol->chan_ref;
	struct aspeed_espi_mmbi *priv = channel->priv;
	struct host_rop hrop;
	ssize_t rd_offset, rd_len;
	ssize_t ret;
	u32 unread_data_len, req_data_len;
	u8 type;

	protocol->process_data = true;
	if (!protocol->data_available && (filp->f_flags & O_NONBLOCK)) {
		// Work around: The lack of response might be cause by missing SCI
		// (host didn't consume the last message), check the buffer state
		// and retry if it's needed
		raise_missing_sci(channel);
		return -EAGAIN;
	}
	dev_dbg(priv->dev, "%s: count:%d, Type: %d\n", __func__, count,
		protocol->type);

	ret = wait_event_interruptible(protocol->queue,
				       protocol->data_available);
	if (ret == -ERESTARTSYS) {
		ret = -EINTR;
		goto err_out;
	}

	ret = get_mmbi_header(channel, &req_data_len, &type, &unread_data_len);
	if (ret != 0) {
		/* Bail out as we can't read header. */
		goto err_out;
	}
	dev_dbg(priv->dev,
		"%s: Length: 0x%0x, Protocol Type: %d, Unread data: %d\n",
		__func__, req_data_len, type, unread_data_len);

	/* Check is data belongs to this device, if not wake_up corresponding device. */
	if (type != protocol->type) {
		ret = -EFAULT;
		goto err_out;
	}

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	if ((hrop.h2b_rp + sizeof(struct mmbi_header)) <=
	    channel->h2b_cb_size) {
		rd_offset = hrop.h2b_rp + sizeof(struct mmbi_header);
	} else {
		rd_offset = hrop.h2b_rp + sizeof(struct mmbi_header) -
			    channel->h2b_cb_size;
	}
	rd_len = req_data_len;

	/* Extract data and copy to user space application */
	dev_dbg(priv->dev, "READ MMBI Data from: 0x%0x and length: %d\n",
		(u32)(channel->h2b_cb_vmem + rd_offset), rd_len);
	if (unread_data_len < sizeof(struct mmbi_header) + rd_len) {
		dev_err(priv->dev, "Invalid H2B buffer (Read msg length: %d)\n",
			rd_len);
		ret = -EFAULT;
		goto err_out;
	}

	if ((channel->h2b_cb_size - rd_offset) >= rd_len) {
		if (copy_to_user(buff, channel->h2b_cb_vmem + rd_offset,
				 rd_len)) {
			dev_err(priv->dev,
				"Failed to copy data to user space\n");
			ret = -EFAULT;
			goto err_out;
		}
		rd_offset += rd_len;
	} else {
		ssize_t chunk_len;

		chunk_len = channel->h2b_cb_size - rd_offset;
		if (copy_to_user(buff, channel->h2b_cb_vmem + rd_offset,
				 chunk_len)) {
			dev_err(priv->dev,
				"Failed to copy data to user space\n");
			ret = -EFAULT;
			goto err_out;
		}
		rd_offset = 0;
		if (copy_to_user(buff + chunk_len,
				 channel->h2b_cb_vmem + rd_offset,
				 rd_len - chunk_len)) {
			dev_err(priv->dev,
				"Failed to copy data to user space\n");
			ret = -EFAULT;
			goto err_out;
		}
		rd_offset += (rd_len - chunk_len);
	}
	*offp += rd_len;
	ret = rd_len;

	update_host_rop(channel, 0, rd_len + sizeof(struct mmbi_header));

	dev_dbg(priv->dev, "%s: Return length: %d\n", __func__, ret);
err_out:
	/*
	 * Raise the missing SCI's by checking pointer for host
	 * read acknowledgment. This will work around the Missing
	 * SCI bug on host side. *
	 */
	dev_warn(priv->dev, "%s: Check and raise missing SCI\n", __func__);
	raise_missing_sci(channel);

	protocol->data_available = false;

	wake_up_device(channel);

	return ret;
}

static ssize_t mmbi_write(struct file *filp, const char *buffer, size_t len,
			  loff_t *offp)
{
	struct aspeed_mmbi_protocol *protocol = file_aspeed_espi_mmbi(filp);
	struct aspeed_mmbi_channel *channel = protocol->chan_ref;
	struct aspeed_espi_mmbi *priv = channel->priv;
	struct mmbi_header header;
	struct host_rop hrop;
	ssize_t wt_offset;
	ssize_t avail_buf_len;
	ssize_t chunk_len;
	ssize_t end_offset;
	u32 h_rwp0;

	dev_dbg(priv->dev, "%s: length:%d , type: %d\n", __func__, len,
		protocol->type);

	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP0_INSTANCE0, &h_rwp0)) {
		dev_err(priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}

	/* If Host READY bit is not set, Just discard the write. */
	if (!GET_HOST_READY_BIT(h_rwp0)) {
		dev_dbg(channel->priv->dev,
			"Host not ready, discarding request...\n");
		return -EAGAIN;
	}

	if (get_b2h_avail_buf_len(channel, &avail_buf_len)) {
		dev_dbg(priv->dev, "Failed to B2H empty buffer len\n");
		return -EAGAIN;
	}

	dev_dbg(priv->dev, "B2H buffer empty space: %d\n", avail_buf_len);

	/* Empty space should be more than write request data size */
	if (len + sizeof(header) > avail_buf_len) {
		dev_err(priv->dev, "Not enough space(%d) in B2H buffer\n",
			avail_buf_len);
		return -ENOSPC;
	}

	/* Fill multi-protocol header */
	header.data = ((protocol->type << 24) + len);

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	wt_offset = hrop.b2h_wp;
	end_offset = channel->b2h_cb_size;

	if ((end_offset - wt_offset) >= sizeof(header)) {
		memcpy(channel->b2h_cb_vmem + wt_offset, &header,
		       sizeof(header));
		wt_offset += sizeof(header);
	} else {
		chunk_len = end_offset - wt_offset;
		memcpy(channel->b2h_cb_vmem + wt_offset, &header, chunk_len);
		memcpy(channel->b2h_cb_vmem, &header + chunk_len,
		       (sizeof(header) - chunk_len));
		wt_offset = (sizeof(header) - chunk_len);
	}

	/* Write the data */
	if ((end_offset - wt_offset) >= len) {
		if (copy_from_user(&channel->b2h_cb_vmem[wt_offset], buffer,
				   len)) {
			return -EFAULT;
		}
		wt_offset += len;
	} else {
		chunk_len = end_offset - wt_offset;
		if (copy_from_user(&channel->b2h_cb_vmem[wt_offset], buffer,
				   chunk_len)) {
			return -EFAULT;
		}
		wt_offset = 0;
		if (copy_from_user(&channel->b2h_cb_vmem[wt_offset],
				   buffer + chunk_len, len - chunk_len)) {
			return -EFAULT;
		}
		wt_offset += len - chunk_len;
	}

	*offp += len;

	update_host_rop(channel, len + sizeof(struct mmbi_header), 0);

	return len;
}

static int get_mmbi_config(struct aspeed_mmbi_channel *channel, void __user *userbuf)
{
	bool h_ready;
	struct host_rop hrop;
	struct aspeed_mmbi_get_config get_conf;
	u32 h2b_wptr, b2h_rptr, h_rwp0, h_rwp1;

	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP0_INSTANCE0, &h_rwp0)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}
	if (read_host_rwp_val(channel, ASPEED_MMBI_HRWP1_INSTANCE0, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}

	h2b_wptr = GET_H2B_WRITE_POINTER(h_rwp0);
	b2h_rptr = GET_B2H_READ_POINTER(h_rwp1);

	h_ready = GET_HOST_READY_BIT(h_rwp0) ? true : false;

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));

	get_conf.h_rdy = h_ready;
	get_conf.h2b_wp = h2b_wptr;
	get_conf.b2h_rp = b2h_rptr;
	get_conf.h2b_rp = hrop.h2b_rp;
	get_conf.b2h_wp = hrop.b2h_wp;

	if (copy_to_user(userbuf, &get_conf, sizeof(get_conf))) {
		dev_err(channel->priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int get_b2h_empty_space(struct aspeed_mmbi_channel *channel,
			       void __user *userbuf)
{
	struct aspeed_mmbi_get_empty_space empty_space;
	ssize_t avail_buf_len;

	if (get_b2h_avail_buf_len(channel, &avail_buf_len)) {
		dev_dbg(channel->priv->dev, "Failed to B2H empty buffer len\n");
		return -EAGAIN;
	}

	dev_dbg(channel->priv->dev, "B2H buffer empty space: %d\n",
		avail_buf_len);

	empty_space.length = avail_buf_len;

	if (copy_to_user(userbuf, &empty_space, sizeof(empty_space))) {
		dev_err(channel->priv->dev, "copy to user failed\n");
		return -EFAULT;
	}

	return 0;
}

static long mmbi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct aspeed_mmbi_protocol *protocol = file_aspeed_espi_mmbi(filp);
	struct aspeed_mmbi_channel *channel = protocol->chan_ref;
	void __user *userbuf = (void __user *)arg;
	int ret;

	switch (cmd) {
	case ASPEED_MMBI_CTRL_IOCTL_GET_B2H_EMPTY_SPACE:
		ret = get_b2h_empty_space(channel, userbuf);
		break;

	case ASPEED_MMBI_CTRL_IOCTL_SEND_RESET_REQUEST:
		ret = send_bmc_reset_request(channel);
		break;

	case ASPEED_MMBI_CTRL_IOCTL_GET_CONFIG:
		ret = get_mmbi_config(channel, userbuf);
		break;

	default:
		dev_err(channel->priv->dev, "Command not found\n");
		ret = -ENOTTY;
	}

	return ret;
}

static const struct file_operations aspeed_espi_mmbi_fops = {
	.owner = THIS_MODULE,
	.open = mmbi_open,
	.release = mmbi_release,
	.read = mmbi_read,
	.write = mmbi_write,
	.unlocked_ioctl = mmbi_ioctl,
	.poll = mmbi_poll
};

static char *get_protocol_suffix(protocol_type type)
{
	switch (type) {
	case MMBI_PROTOCOL_IPMI:
		return "ipmi";
	case MMBI_PROTOCOL_SEAMLESS:
		return "seamless";
	case MMBI_PROTOCOL_RAS_OFFLOAD:
		return "ras_offload";
	case MMBI_PROTOCOL_MCTP:
		return "mctp";
	case MMBI_PROTOCOL_NODE_MANAGER:
		return "nm";
	}

	return NULL;
}

static struct mmbi_cap_desc mmbi_desc_init(struct aspeed_mmbi_channel channel)
{
	struct mmbi_cap_desc ch_desc;

	memset(&ch_desc, 0, sizeof(ch_desc));

	/* Per MMBI protoco spec, Set it to "$MMBI$" */
	strncpy(ch_desc.signature, "$MMBI$", sizeof(ch_desc.signature));
	ch_desc.version = 1;
	ch_desc.instance_num = channel.chan_num;
	/*
	 * TODO: Add multi-channel support. Handcoded H2B start offset
	 * to 0x8000 as we support single channel today.
	 */
	ch_desc.nex_inst_base_addr = 0;
	ch_desc.b2h_ba = sizeof(struct mmbi_cap_desc) + sizeof(struct host_rop);
	ch_desc.h2b_ba = 0x8000 + sizeof(struct host_rwp);
	ch_desc.b2h_d = 0x800; /* 32KB = 0x800 * 16 */
	ch_desc.h2b_d = 0x800; /* 32KB = 0x800 * 16 */

	ch_desc.buffer_type = 0x01; /* VMSCB */
	ch_desc.bt_desc.host_rop_p = sizeof(struct mmbi_cap_desc);
	ch_desc.bt_desc.host_rwp_p = 0x8000;
	ch_desc.bt_desc.msg_protocol_type = 0x01; /* Multiple protocol type */
	ch_desc.bt_desc.host_int_type =
		0x01; /* SCI Triggered through eSPI VW */
	ch_desc.bt_desc.global_sys_interrupt = 0x00; /* Not used */
	ch_desc.bt_desc.bmc_int_type = 0x00; /* Auto - AST HW Interrupt */
	ch_desc.bt_desc.bmc_int_a = 0x00; /* Not used, set to zero */
	ch_desc.bt_desc.bmc_int_v = 0x00; /* Not used, set to zero */

	ch_desc.crc8 = crc8(mmbi_crc8_table, (u8 *)&ch_desc,
			    (size_t)(sizeof(ch_desc) - 1), 0);

	return ch_desc;
}

static int mmbi_channel_init(struct aspeed_espi_mmbi *priv, struct device_node *node, u8 idx)
{
	struct device *dev = priv->dev;
	int rc;
	u8 i;
	u8 *h2b_vaddr, *b2h_vaddr;
	struct mmbi_cap_desc ch_desc;
	struct host_rop hrop;
	int no_of_protocols_enabled;
	u8 mmbi_supported_protocols[MAX_NO_OF_SUPPORTED_PROTOCOLS];

	u32 b2h_size = (priv->mmbi_size / 2);
	u32 h2b_size = (priv->mmbi_size / 2);

	b2h_vaddr = priv->dma_vaddr;
	h2b_vaddr = priv->dma_vaddr + (priv->mmbi_size / 2);

	memset(&priv->chan[idx], 0, sizeof(struct aspeed_mmbi_channel));
	priv->chan[idx].chan_num = idx;

	priv->chan[idx].desc_vmem = b2h_vaddr;
	priv->chan[idx].hrop_vmem = b2h_vaddr + sizeof(struct mmbi_cap_desc);
	priv->chan[idx].b2h_cb_vmem = b2h_vaddr + sizeof(struct mmbi_cap_desc) +
				      sizeof(struct host_rop);
	priv->chan[idx].b2h_cb_size = b2h_size - sizeof(struct mmbi_cap_desc) -
				      sizeof(struct host_rop);
	/* Set BMC ready bit */
	memcpy(&hrop, priv->chan[idx].hrop_vmem, sizeof(hrop));
	hrop.b_rdy = 1;
	memcpy(priv->chan[idx].hrop_vmem, &hrop, sizeof(hrop));

	priv->chan[idx].hrwp_vmem = h2b_vaddr;
	priv->chan[idx].h2b_cb_vmem = h2b_vaddr + sizeof(struct host_rwp);
	priv->chan[idx].h2b_cb_size = h2b_size - sizeof(struct host_rwp);

	dev_dbg(priv->dev,
		"B2H mapped addr - desc: 0x%0x, hrop: 0x%0x, b2h_cb: 0x%0x\n",
		(size_t)priv->chan[idx].desc_vmem,
		(size_t)priv->chan[idx].hrop_vmem,
		(size_t)priv->chan[idx].b2h_cb_vmem);
	dev_dbg(priv->dev, "H2B mapped addr - hrwp: 0x%0x, h2b_cb: 0x%0x\n",
		(size_t)priv->chan[idx].hrwp_vmem,
		(size_t)priv->chan[idx].h2b_cb_vmem);

	dev_dbg(priv->dev, "B2H buffer size: 0x%0x\n",
		(size_t)priv->chan[idx].b2h_cb_size);
	dev_dbg(priv->dev, "H2B buffer size: 0x%0x\n",
		(size_t)priv->chan[idx].h2b_cb_size);

	/* Initialize the MMBI channel descriptor */
	ch_desc = mmbi_desc_init(priv->chan[idx]);
	memcpy(priv->chan[idx].desc_vmem, &ch_desc, sizeof(ch_desc));

	priv->chan[idx].enabled = true;
	if (!node) {
		dev_err(priv->dev, "mmbi protocol : no instance found\n");
		goto err_destroy_channel;
	}
	no_of_protocols_enabled = of_property_count_u8_elems(node, "protocols");
	if (no_of_protocols_enabled <= 0 || no_of_protocols_enabled >
	    MAX_NO_OF_SUPPORTED_PROTOCOLS){
		dev_err(dev, "No supported mmbi protocol\n");
		goto err_destroy_channel;
	}
	rc = of_property_read_u8_array(node, "protocols", mmbi_supported_protocols,
				       no_of_protocols_enabled);
	if (!rc) {
		memset(&priv->chan[idx].supported_protocols, 0,
		       sizeof(priv->chan[idx].supported_protocols));
		memcpy(&priv->chan[idx].supported_protocols, mmbi_supported_protocols,
		       sizeof(mmbi_supported_protocols));
	}

	for (i = 0; i < no_of_protocols_enabled; i++) {
		char *dev_name;
		u8 proto_type;

		proto_type = priv->chan[idx].supported_protocols[i];
		dev_name = get_protocol_suffix(proto_type);
		if (!dev_name) {
			dev_err(dev,
				"Unable to get MMBI protocol suffix name\n");
			goto err_destroy_channel;
		}
		priv->chan[idx].protocol[i].type = proto_type;
		priv->chan[idx].protocol[i].miscdev.name =
			devm_kasprintf(dev, GFP_KERNEL, "%s_%d_%s", DEVICE_NAME,
				       idx, dev_name);
		priv->chan[idx].protocol[i].miscdev.minor = MISC_DYNAMIC_MINOR;
		priv->chan[idx].protocol[i].miscdev.fops =
			&aspeed_espi_mmbi_fops;
		priv->chan[idx].protocol[i].miscdev.parent = dev;
		rc = misc_register(&priv->chan[idx].protocol[i].miscdev);
		if (rc) {
			dev_err(dev, "Unable to register device\n");
			goto err_destroy_channel;
		}

		/* Hold the back reference of channel */
		priv->chan[idx].protocol[i].chan_ref = &priv->chan[idx];

		priv->chan[idx].protocol[i].data_available = false;
		priv->chan[idx].protocol[i].process_data = false;
		init_waitqueue_head(&priv->chan[idx].protocol[i].queue);
	}

	priv->chan[idx].priv = priv;

	/*
	 * When BMC goes for reset while host is in OS, SRAM memory will be
	 * remapped and the content in memory will be lost. This include
	 * host ready state which will block memory write transactions.
	 * Ideally this reset has to be done while mapping memory(u-boot).
	 * Since channel initialization (including descriptor) done at kernel,
	 * So added channel reset also during driver load. Future, when staged
	 * commands processing(IPMI commands for BIOS-BMC communication) is
	 * enabled, this check should be moved to u-boot.
	 */
	if (send_bmc_reset_request(&priv->chan[idx]))
		dev_info(dev, "MMBI channel(%d) reset failed\n", idx);

	dev_info(dev, "MMBI Channel(%d) initialized successfully\n", idx);

	return 0;

err_destroy_channel:
	if (b2h_vaddr)
		memunmap(b2h_vaddr);

	if (h2b_vaddr)
		memunmap(h2b_vaddr);

	priv->chan[idx].enabled = false;
	return -ENOMEM;
}

static irqreturn_t aspeed_espi_mmbi_irq(int irq, void *arg)
{
	struct aspeed_espi_mmbi *priv = arg;
	u32 status;
	int idx;

	regmap_read(priv->map, ASPEED_MMBI_IRQ_STATUS, &status);
	/* Clear interrupt */
	regmap_write(priv->map, ASPEED_MMBI_IRQ_STATUS, status);

	for (idx = 0; idx < MAX_NO_OF_SUPPORTED_CHANNELS; idx++) {
		/*
		 * Host RWP 1: It gets updated after Host reads data and also
		 * when host want to send reset MMBI buffer request. So
		 * Handle reset request and ignore read pointer update.
		 * Host RWP0: It gets updated when host write data on H2B,
		 * So process the request by invoking corresponding device.
		 */
		if (!priv->chan[idx].enabled)
			continue;
		if ((status >> (idx * 2)) & HRWP1_READ_MASK)
			check_host_reset_request(&priv->chan[idx]);
		else
			wake_up_device(&priv->chan[idx]);
	}
	dev_dbg(priv->dev, "MMBI IRQ Status: %d\n", status);

	return IRQ_HANDLED;
}

static const struct of_device_id aspeed_espi_mmbi_match[] = {
	{ .compatible = "aspeed,ast2600-espi-mmbi" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_espi_mmbi_match);

static int aspeed_espi_mmbi_probe(struct platform_device *pdev)
{
	const struct of_device_id *dev_id;
	struct aspeed_espi_mmbi *priv;
	struct device_node *node;
	struct resource resm;
	void __iomem *regs;
	u32 reg_val, enable_irqs;
	int rc, i;

	dev_dbg(&pdev->dev, "MMBI: Probing MMBI devices...\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_espi_mmbi),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	dev_id = of_match_device(aspeed_espi_mmbi_match, priv->dev);
	if (!dev_id) {
		dev_err(priv->dev, "MMBI: Failed to match mmbi device\n");
		return -EINVAL;
	}

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		dev_err(priv->dev, "MMBI: Failed to get regmap!\n");
		return PTR_ERR(regs);
	}

	/* MMBI register map */
	priv->map = devm_regmap_init_mmio(priv->dev, regs,
					  &aspeed_espi_mmbi_regmap_cfg);
	if (IS_ERR(priv->map)) {
		dev_err(priv->dev, "MMBI: Couldn't get regmap\n");
		return -ENODEV;
	}

	/* ESI register map */
	priv->pmap = syscon_regmap_lookup_by_phandle(priv->dev->of_node,
						     "aspeed,espi");
	if (IS_ERR(priv->pmap)) {
		dev_err(priv->dev, "MMBI: Failed to find ESPI regmap\n");
		return PTR_ERR(priv->pmap);
	}

	/* LPC register map */
	priv->lpc_map = syscon_regmap_lookup_by_phandle(priv->dev->of_node,
							"aspeed,lpc");
	if (IS_ERR(priv->lpc_map)) {
		dev_err(priv->dev, "MMBI: Failed to find LPC regmap\n");
		return PTR_ERR(priv->lpc_map);
	}

	/* If memory-region is described in device tree then store */
	node = of_parse_phandle(priv->dev->of_node, "memory-region", 0);
	if (node) {
		rc = of_property_read_u32(priv->dev->of_node, "host-map-addr",
					  &priv->host_map_addr);
		if (rc) {
			dev_info(priv->dev, "No host mapping address\n");
			priv->host_map_addr = PCH_ESPI_LGMR_BASE_ADDRESS;
		}

		rc = of_address_to_resource(node, 0, &resm);
		of_node_put(node);
		if (!rc) {
			priv->mmbi_size = resource_size(&resm);
			priv->mmbi_phys_addr = resm.start;
		} else {
			priv->mmbi_size = ESPI_MMBI_TOTAL_SIZE;
			priv->mmbi_phys_addr = BMC_SRAM_BASE_ADDRESS;
		}
	} else {
		dev_dbg(priv->dev,
			"No DTS config, assign default MMBI Address\n");
		priv->host_map_addr = PCH_ESPI_LGMR_BASE_ADDRESS;
		priv->mmbi_size = ESPI_MMBI_TOTAL_SIZE;
		priv->mmbi_phys_addr = BMC_SRAM_BASE_ADDRESS;
	}
	dev_dbg(priv->dev, "MMBI: HostAddr:0x%x, SramAddr:0x%x, Size: 0x%0x\n",
		priv->host_map_addr, priv->mmbi_phys_addr, priv->mmbi_size);
	priv->dma_vaddr = dma_alloc_coherent(priv->dev, priv->mmbi_size,
					     &priv->mmbi_phys_addr, GFP_KERNEL);

	if (!priv->dma_vaddr) {
		dev_err(priv->dev, "MMBI: DMA memory allocation failed\n");
		return -ENOMEM;
	}
	dev_dbg(priv->dev, "MMBI: DMA Addr: 0x%x\n", (u32)priv->dma_vaddr);
	memset(priv->dma_vaddr, 0, priv->mmbi_size);

	crc8_populate_msb(mmbi_crc8_table, MMBI_CRC8_POLYNOMIAL);

	/* eSPI Controller settings */
	regmap_write(priv->pmap, ASPEED_ESPI_PC_RX_SADDR, priv->host_map_addr);
	regmap_write(priv->pmap, ASPEED_ESPI_PC_RX_TADDR, priv->mmbi_phys_addr);
	regmap_write(priv->pmap, ASPEED_ESPI_PC_RX_TADDRM,
		     ASPEED_ESPI_PC_RX_TADDR_MASK);
	regmap_update_bits(priv->pmap, ASPEED_ESPI_CTRL2,
			   ESPI_DISABLE_PERP_MEM_READ |
			   ESPI_DISABLE_PERP_MEM_WRITE, 0);

	/* MMBI controller Settings */
	regmap_read(priv->map, ASPEED_MMBI_CTRL, &reg_val);
	regmap_read(priv->map, ASPEED_MMBI_IRQ_ENABLE, &reg_val);
	regmap_write(priv->map, ASPEED_MMBI_CTRL,
		     MMBI_ENABLE_FUNCTION | MMBI_TOTAL_SIZE_64K |
			     MMBI_INSTANCE_SIZE_64K);
	regmap_write(priv->map, ASPEED_MMBI_IRQ_ENABLE, 0x03);

	dev_set_drvdata(priv->dev, priv);
	for_each_child_of_node(priv->dev->of_node, node) {
		rc = of_property_read_u32(node, "channel", &i);
		if (rc || i >= MAX_NO_OF_SUPPORTED_CHANNELS || priv->chan[i].enabled)
			continue;
		rc = mmbi_channel_init(priv, node, i);
		if (rc) {
			dev_err(priv->dev, "MMBI: Channel(%d) init failed\n",
				i);
		} else {
			enable_irqs += (0x03 << i);
		}
	}
	regmap_write(priv->map, ASPEED_MMBI_IRQ_ENABLE, enable_irqs);

	/* Enable IRQ */
	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		dev_err(priv->dev, "MMBI: No irq specified\n");
		return priv->irq;
	}

	rc = devm_request_irq(priv->dev, priv->irq, aspeed_espi_mmbi_irq,
			      IRQF_SHARED, dev_name(priv->dev), priv);
	if (rc) {
		dev_err(priv->dev, "MMBI: Unable to get IRQ\n");
		return rc;
	}

	dev_dbg(priv->dev, "MMBI: aspeed MMBI driver loaded successfully\n");

	return 0;
}

static int aspeed_espi_mmbi_remove(struct platform_device *pdev)
{
	struct aspeed_espi_mmbi *priv = dev_get_drvdata(&pdev->dev);
	int i, j;

	dev_dbg(priv->dev, "MMBI: Removing MMBI device\n");

	for (i = 0; i < MAX_NO_OF_SUPPORTED_CHANNELS; i++) {
		if (!priv->chan[i].enabled)
			continue;
		for (j = 0; priv->chan[i].supported_protocols[j] != 0; j++)
			misc_deregister(&priv->chan[i].protocol[j].miscdev);
	}

	if (priv->dma_vaddr)
		dma_free_coherent(priv->dev, priv->mmbi_size, priv->dma_vaddr,
				  priv->mmbi_phys_addr);

	return 0;
}

static struct platform_driver aspeed_espi_mmbi_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = aspeed_espi_mmbi_match,
	},
	.probe  = aspeed_espi_mmbi_probe,
	.remove = aspeed_espi_mmbi_remove,
};
module_platform_driver(aspeed_espi_mmbi_driver);

MODULE_AUTHOR("AppaRao Puli <apparao.puli@intel.com>");
MODULE_DESCRIPTION("MMBI Driver");
MODULE_LICENSE("GPL v2");
