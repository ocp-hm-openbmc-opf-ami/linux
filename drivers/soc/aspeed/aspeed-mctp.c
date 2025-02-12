// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Intel Corporation.

#include <linux/aspeed-mctp.h>
#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/list_sort.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/ptr_ring.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/swab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <uapi/linux/aspeed-mctp.h>

/* AST2600 MCTP Controller registers */
#define ASPEED_MCTP_CTRL	0x000
#define  TX_CMD_TRIGGER		BIT(0)
#define  RX_CMD_READY		BIT(4)
#define  MATCHING_EID		BIT(9)

#define ASPEED_MCTP_TX_CMD	0x004
#define ASPEED_MCTP_RX_CMD	0x008

#define ASPEED_MCTP_INT_STS	0x00c
#define ASPEED_MCTP_INT_EN	0x010
#define  TX_CMD_SENT_INT	BIT(0)
#define  TX_CMD_LAST_INT	BIT(1)
#define  TX_CMD_WRONG_INT	BIT(2)
#define  RX_CMD_RECEIVE_INT	BIT(8)
#define  RX_CMD_NO_MORE_INT	BIT(9)

#define ASPEED_MCTP_EID		0x014
#define ASPEED_MCTP_OBFF_CTRL	0x018

#define ASPEED_MCTP_ENGINE_CTRL		0x01c
#define  TX_MAX_PAYLOAD_SIZE_SHIFT	0
#define  TX_MAX_PAYLOAD_SIZE_MASK	GENMASK(1, TX_MAX_PAYLOAD_SIZE_SHIFT)
#define  TX_MAX_PAYLOAD_SIZE(x) \
	(((x) << TX_MAX_PAYLOAD_SIZE_SHIFT) & TX_MAX_PAYLOAD_SIZE_MASK)
#define  RX_MAX_PAYLOAD_SIZE_SHIFT	4
#define  RX_MAX_PAYLOAD_SIZE_MASK	GENMASK(5, RX_MAX_PAYLOAD_SIZE_SHIFT)
#define  RX_MAX_PAYLOAD_SIZE(x) \
	(((x) << RX_MAX_PAYLOAD_SIZE_SHIFT) & RX_MAX_PAYLOAD_SIZE_MASK)
#define  FIFO_LAYOUT_SHIFT		8
#define  FIFO_LAYOUT_MASK		GENMASK(9, FIFO_LAYOUT_SHIFT)
#define  FIFO_LAYOUT(x) \
	(((x) << FIFO_LAYOUT_SHIFT) & FIFO_LAYOUT_MASK)

#define ASPEED_MCTP_RX_BUF_ADDR		0x020
#define ASPEED_MCTP_RX_BUF_SIZE		0x024
#define ASPEED_MCTP_RX_BUF_RD_PTR	0x028
#define  UPDATE_RX_RD_PTR		BIT(31)
#define  RX_BUF_RD_PTR_MASK		GENMASK(11, 0)
#define ASPEED_MCTP_RX_BUF_WR_PTR	0x02c
#define  RX_BUF_WR_PTR_MASK		GENMASK(11, 0)

#define ASPEED_MCTP_TX_BUF_ADDR		0x030
#define ASPEED_MCTP_TX_BUF_SIZE		0x034
#define ASPEED_MCTP_TX_BUF_RD_PTR	0x038
#define  UPDATE_TX_RD_PTR		BIT(31)
#define  TX_BUF_RD_PTR_MASK		GENMASK(11, 0)
#define ASPEED_MCTP_TX_BUF_WR_PTR	0x03c
#define  TX_BUF_WR_PTR_MASK		GENMASK(11, 0)

#define ADDR_LEN	(BIT(26) - 1)
#define DATA_ADDR(x)	(((x) >> 4) & ADDR_LEN)

/* TX command */
#define TX_LAST_CMD		BIT(31)
#define TX_DATA_ADDR_SHIFT	4
#define TX_DATA_ADDR_MASK	GENMASK(30, TX_DATA_ADDR_SHIFT)
#define TX_DATA_ADDR(x) \
	((DATA_ADDR(x) << TX_DATA_ADDR_SHIFT) & TX_DATA_ADDR_MASK)
#define TX_RESERVED_1_MASK	GENMASK(1, 0) /* must be 1 */
#define TX_RESERVED_1		1
#define TX_STOP_AFTER_CMD	BIT(16)
#define TX_INTERRUPT_AFTER_CMD	BIT(15)
#define TX_PACKET_SIZE_SHIFT	2
#define TX_PACKET_SIZE_MASK	GENMASK(12, TX_PACKET_SIZE_SHIFT)
#define TX_PACKET_SIZE(x) \
	(((x) << TX_PACKET_SIZE_SHIFT) & TX_PACKET_SIZE_MASK)
#define TX_RESERVED_0_MASK	GENMASK(1, 0) /* MBZ */
#define TX_RESERVED_0		0

/* RX command */
#define RX_INTERRUPT_AFTER_CMD	BIT(2)
#define RX_DATA_ADDR_SHIFT	4
#define RX_DATA_ADDR_MASK	GENMASK(30, RX_DATA_ADDR_SHIFT)
#define RX_DATA_ADDR(x) \
	((DATA_ADDR(x) << RX_DATA_ADDR_SHIFT) & RX_DATA_ADDR_MASK)

/* HW buffer sizes */
#define TX_PACKET_COUNT		48
#define RX_PACKET_COUNT		96
#define TX_MAX_PACKET_COUNT	(TX_BUF_RD_PTR_MASK + 1)
#define RX_MAX_PACKET_COUNT	(RX_BUF_RD_PTR_MASK + 1)

#define TX_CMD_BUF_SIZE \
	PAGE_ALIGN(TX_PACKET_COUNT * sizeof(struct aspeed_mctp_tx_cmd))
#define TX_DATA_BUF_SIZE \
	 PAGE_ALIGN(TX_PACKET_COUNT * sizeof(struct mctp_pcie_packet_data))
#define RX_CMD_BUF_SIZE PAGE_ALIGN(RX_PACKET_COUNT * sizeof(u32))
#define RX_DATA_BUF_SIZE \
	PAGE_ALIGN(RX_PACKET_COUNT * sizeof(struct mctp_pcie_packet_data))

/* Per client packet cache sizes */
#define RX_RING_COUNT		64
#define TX_RING_COUNT		64

/* PCIe Host Controller registers */
#define ASPEED_PCIE_MISC_STS_1	0x0c4

/* PCI address definitions */
#define PCI_DEV_NUM_MASK	GENMASK(4, 0)
#define PCI_BUS_NUM_SHIFT	5
#define PCI_BUS_NUM_MASK	GENMASK(12, PCI_BUS_NUM_SHIFT)
#define GET_PCI_DEV_NUM(x)	((x) & PCI_DEV_NUM_MASK)
#define GET_PCI_BUS_NUM(x)	(((x) & PCI_BUS_NUM_MASK) >> PCI_BUS_NUM_SHIFT)

/* MCTP header definitions */
#define MCTP_HDR_SRC_EID_OFFSET		14
#define MCTP_HDR_TAG_OFFSET		15
#define MCTP_HDR_SOM			BIT(7)
#define MCTP_HDR_EOM			BIT(6)
#define MCTP_HDR_SOM_EOM		(MCTP_HDR_SOM | MCTP_HDR_EOM)
#define MCTP_HDR_TYPE_OFFSET		16
#define MCTP_HDR_TYPE_CONTROL		0
#define MCTP_HDR_TYPE_VDM_PCI		0x7e
#define MCTP_HDR_TYPE_SPDM		0x5
#define MCTP_HDR_TYPE_BASE_LAST		MCTP_HDR_TYPE_SPDM
#define MCTP_HDR_VENDOR_OFFSET		17
#define MCTP_HDR_VDM_TYPE_OFFSET	19

/* FIXME: ast2600 supports variable max transmission unit */
#define ASPEED_MCTP_MTU 64

struct aspeed_mctp_tx_cmd {
	u32 tx_lo;
	u32 tx_hi;
};

struct mctp_buffer {
	void *vaddr;
	dma_addr_t dma_handle;
};

struct mctp_channel {
	struct mctp_buffer data;
	struct mctp_buffer cmd;
	struct tasklet_struct tasklet;
	u32 buffer_count;
	u32 rd_ptr;
	u32 wr_ptr;
	bool stopped;
};

struct aspeed_mctp {
	struct device *dev;
	struct regmap *map;
	struct reset_control *reset;
	struct mctp_channel tx;
	struct mctp_channel rx;
	struct list_head clients;
	struct mctp_client *default_client;
	struct list_head mctp_type_handlers;
	/*
	 * clients_lock protects list of clients, list of type handlers
	 * and default client
	 */
	spinlock_t clients_lock;
	struct list_head endpoints;
	size_t endpoints_count;
	/*
	 * endpoints_lock protects list of endpoints
	 */
	struct mutex endpoints_lock;
	struct {
		struct regmap *map;
		struct delayed_work rst_dwork;
		bool need_uevent;
		u16 bdf;
	} pcie;
	struct {
		bool enable;
		bool warmup;
		int packet_counter;
	} rx_runaway_wa;
	u8 eid;
	struct platform_device *peci_mctp;
};

struct mctp_client {
	struct kref ref;
	struct aspeed_mctp *priv;
	struct ptr_ring tx_queue;
	struct ptr_ring rx_queue;
	struct list_head link;
	void *user_data;
	void (*on_segment_change)(void *usr_buf);
	wait_queue_head_t wait_queue;
};

struct mctp_type_handler {
	u8 mctp_type;
	u16 pci_vendor_id;
	u16 vdm_type;
	u16 vdm_mask;
	struct mctp_client *client;
	struct list_head link;
};

union aspeed_mctp_eid_data_info {
	struct aspeed_mctp_eid_info eid_info;
	struct aspeed_mctp_eid_ext_info eid_ext_info;
};

enum mctp_address_type {
	ASPEED_MCTP_GENERIC_ADDR_FORMAT = 0,
	ASPEED_MCTP_EXTENDED_ADDR_FORMAT = 1
};

struct aspeed_mctp_endpoint {
	union  aspeed_mctp_eid_data_info data;
	struct list_head link;
};

struct kmem_cache *packet_cache;

void *aspeed_mctp_packet_alloc(gfp_t flags)
{
	return kmem_cache_alloc(packet_cache, flags);
}
EXPORT_SYMBOL_GPL(aspeed_mctp_packet_alloc);

void aspeed_mctp_packet_free(void *packet)
{
	kmem_cache_free(packet_cache, packet);
}
EXPORT_SYMBOL_GPL(aspeed_mctp_packet_free);

/*
 * HW produces and expects VDM header in little endian and payload in network order.
 * To allow userspace to use network order for the whole packet, PCIe VDM header needs
 * to be swapped.
 */
static void aspeed_mctp_swap_pcie_vdm_hdr(struct mctp_pcie_packet_data *data)
{
	int i;

	for (i = 0; i < PCIE_VDM_HDR_SIZE_DW; i++)
		data->hdr[i] = swab32(data->hdr[i]);
}

static void aspeed_mctp_rx_trigger(struct mctp_channel *rx)
{
	struct aspeed_mctp *priv = container_of(rx, typeof(*priv), rx);

	/*
	 * Even though rx_buf_addr doesn't change, if we don't do the write
	 * here, the HW doesn't trigger RX. We're also clearing the
	 * RX_CMD_READY bit, otherwise we're observing a rare case where
	 * trigger isn't registered by the HW, and we're ending up with stuck
	 * HW (not reacting to wr_ptr writes).
	 * Also, note that we're writing 0 as wr_ptr. If we're writing other
	 * value, the HW behaves in a bizarre way that's hard to explain...
	 */
	regmap_update_bits(priv->map, ASPEED_MCTP_CTRL, RX_CMD_READY, 0);
	regmap_write(priv->map, ASPEED_MCTP_RX_BUF_ADDR, rx->cmd.dma_handle);
	regmap_write(priv->map, ASPEED_MCTP_RX_BUF_WR_PTR, 0);

	/* After re-enabling RX we need to restart WA logic */
	if (priv->rx_runaway_wa.enable) {
		priv->rx_runaway_wa.warmup = true;
		priv->rx_runaway_wa.packet_counter = 0;
		priv->rx.buffer_count = RX_PACKET_COUNT;
	}

	regmap_update_bits(priv->map, ASPEED_MCTP_CTRL, RX_CMD_READY,
			   RX_CMD_READY);
}

static void aspeed_mctp_tx_trigger(struct mctp_channel *tx, bool notify)
{
	struct aspeed_mctp *priv = container_of(tx, typeof(*priv), tx);

	if (notify) {
		struct aspeed_mctp_tx_cmd *last_cmd;

		last_cmd = (struct aspeed_mctp_tx_cmd *)tx->cmd.vaddr +
			   (tx->wr_ptr - 1) % TX_PACKET_COUNT;
		last_cmd->tx_lo |= TX_INTERRUPT_AFTER_CMD;
	}

	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_WR_PTR, tx->wr_ptr);
	regmap_update_bits(priv->map, ASPEED_MCTP_CTRL, TX_CMD_TRIGGER,
			   TX_CMD_TRIGGER);
}

static void aspeed_mctp_emit_tx_cmd(struct mctp_channel *tx,
				    struct mctp_pcie_packet *packet)
{
	struct aspeed_mctp_tx_cmd *tx_cmd =
		(struct aspeed_mctp_tx_cmd *)tx->cmd.vaddr + tx->wr_ptr;
	u32 packet_sz_dw = packet->size / sizeof(u32) -
		sizeof(packet->data.hdr) / sizeof(u32);
	u32 offset = tx->wr_ptr * sizeof(packet->data);

	aspeed_mctp_swap_pcie_vdm_hdr(&packet->data);

	memcpy((u8 *)tx->data.vaddr + offset, &packet->data,
	       sizeof(packet->data));

	tx_cmd->tx_lo = TX_PACKET_SIZE(packet_sz_dw);
	tx_cmd->tx_hi = TX_RESERVED_1;
	tx_cmd->tx_hi |= TX_DATA_ADDR(tx->data.dma_handle + offset);

	tx->wr_ptr = (tx->wr_ptr + 1) % TX_PACKET_COUNT;
}

static struct mctp_client *aspeed_mctp_client_alloc(struct aspeed_mctp *priv)
{
	struct mctp_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		goto out;

	kref_init(&client->ref);
	client->priv = priv;
	ptr_ring_init(&client->tx_queue, TX_RING_COUNT, GFP_KERNEL);
	ptr_ring_init(&client->rx_queue, RX_RING_COUNT, GFP_ATOMIC);

out:
	return client;
}

static void aspeed_mctp_client_free(struct kref *ref)
{
	struct mctp_client *client = container_of(ref, typeof(*client), ref);

	ptr_ring_cleanup(&client->rx_queue, &aspeed_mctp_packet_free);
	ptr_ring_cleanup(&client->tx_queue, &aspeed_mctp_packet_free);

	kfree(client);
}

static void aspeed_mctp_client_get(struct mctp_client *client)
{
	lockdep_assert_held(&client->priv->clients_lock);

	kref_get(&client->ref);
}

static void aspeed_mctp_client_put(struct mctp_client *client)
{
	kref_put(&client->ref, &aspeed_mctp_client_free);
}

static struct mctp_client *
aspeed_mctp_find_handler(struct aspeed_mctp *priv,
			 struct mctp_pcie_packet *packet)
{
	struct mctp_type_handler *handler;
	u8 *hdr = (u8 *)packet->data.hdr;
	struct mctp_client *client = NULL;
	u8 mctp_type, som_eom;
	u16 vendor = 0;
	u16 vdm_type = 0;

	lockdep_assert_held(&priv->clients_lock);

	/*
	 * Middle and EOM fragments cannot be matched to MCTP type.
	 * For consistency do not match type for any fragmented messages.
	 */
	som_eom = hdr[MCTP_HDR_TAG_OFFSET] & MCTP_HDR_SOM_EOM;
	if (som_eom != MCTP_HDR_SOM_EOM)
		return NULL;

	mctp_type = hdr[MCTP_HDR_TYPE_OFFSET];
	if (mctp_type == MCTP_HDR_TYPE_VDM_PCI) {
		vendor = *((u16 *)&hdr[MCTP_HDR_VENDOR_OFFSET]);
		vdm_type = *((u16 *)&hdr[MCTP_HDR_VDM_TYPE_OFFSET]);
	}

	list_for_each_entry(handler, &priv->mctp_type_handlers, link) {
		if (handler->mctp_type == mctp_type &&
		    handler->pci_vendor_id == vendor &&
		    handler->vdm_type == (vdm_type & handler->vdm_mask)) {
			dev_dbg(priv->dev, "Found client for type %x vdm %x\n",
				mctp_type, handler->vdm_type);
			client = handler->client;
			break;
		}
	}
	return client;
}

static void aspeed_mctp_dispatch_packet(struct aspeed_mctp *priv,
					struct mctp_pcie_packet *packet)
{
	struct mctp_client *client;
	int ret;

	spin_lock(&priv->clients_lock);

	client = aspeed_mctp_find_handler(priv, packet);

	if (!client)
		client = priv->default_client;

	if (client)
		aspeed_mctp_client_get(client);

	spin_unlock(&priv->clients_lock);

	if (client) {
		ret = ptr_ring_produce(&client->rx_queue, packet);
		if (ret) {
			/*
			 * This can happen if client process does not
			 * consume packets fast enough
			 */
			dev_dbg(priv->dev, "Failed to store packet in client RX queue\n");
			aspeed_mctp_packet_free(packet);
		} else {
			wake_up_all(&client->wait_queue);
		}
		aspeed_mctp_client_put(client);
	} else {
		dev_dbg(priv->dev, "Failed to dispatch RX packet\n");
		aspeed_mctp_packet_free(packet);
	}
}

static void aspeed_mctp_tx_tasklet(unsigned long data)
{
	struct mctp_channel *tx = (struct mctp_channel *)data;
	struct aspeed_mctp *priv = container_of(tx, typeof(*priv), tx);
	struct mctp_client *client;
	bool trigger = false;
	bool full = false;
	u32 rd_ptr;

	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_RD_PTR, UPDATE_RX_RD_PTR);
	regmap_read(priv->map, ASPEED_MCTP_TX_BUF_RD_PTR, &rd_ptr);
	rd_ptr &= TX_BUF_RD_PTR_MASK;

	spin_lock(&priv->clients_lock);

	list_for_each_entry(client, &priv->clients, link) {
		while (!(full = (tx->wr_ptr + 1) % TX_PACKET_COUNT == rd_ptr)) {
			struct mctp_pcie_packet *packet;

			packet = ptr_ring_consume(&client->tx_queue);
			if (!packet)
				break;

			aspeed_mctp_emit_tx_cmd(tx, packet);
			aspeed_mctp_packet_free(packet);
			trigger = true;
		}
	}

	spin_unlock(&priv->clients_lock);

	if (trigger)
		aspeed_mctp_tx_trigger(tx, full);
}

static void aspeed_mctp_rx_tasklet(unsigned long data)
{
	struct mctp_channel *rx = (struct mctp_channel *)data;
	struct aspeed_mctp *priv = container_of(rx, typeof(*priv), rx);
	struct mctp_pcie_packet *rx_packet;
	struct mctp_pcie_packet_data *rx_buf;
	u32 hw_read_ptr;
	u32 *hdr;

	/* Trigger HW read pointer update, must be done before RX loop */
	regmap_write(priv->map, ASPEED_MCTP_RX_BUF_RD_PTR, UPDATE_RX_RD_PTR);

	/*
	 * XXX: Using rd_ptr obtained from HW is unreliable so we need to
	 * maintain the state of buffer on our own by peeking into the buffer
	 * and checking where the packet was written.
	 */
	rx_buf = (struct mctp_pcie_packet_data *)rx->data.vaddr;
	hdr = (u32 *)&rx_buf[rx->wr_ptr];

	if (priv->rx_runaway_wa.warmup && !*hdr) {
		u32 tmp_wr_ptr = rx->wr_ptr;

		/*
		 * HACK: Right after start the RX hardware can put received
		 * packet into an unexpected offset - in order to locate
		 * received packet driver has to scan all RX data buffers.
		 */
		do {
			tmp_wr_ptr = (tmp_wr_ptr + 1) % RX_PACKET_COUNT;

			hdr = (u32 *)&rx_buf[tmp_wr_ptr];
		} while (!*hdr && tmp_wr_ptr != rx->wr_ptr);

		if (tmp_wr_ptr != rx->wr_ptr) {
			dev_dbg(priv->dev, "Runaway RX packet found %d -> %d\n",
				rx->wr_ptr, tmp_wr_ptr);
			rx->wr_ptr = tmp_wr_ptr;
		}

		/*
		 * Once we receive RX_PACKET_COUNT packets, hardware is
		 * guaranteed to use (RX_PACKET_COUNT - 4) buffers. Decrease
		 * buffer count by 4, then we can turn off scanning of RX
		 * buffers. RX buffer scanning should be enabled every time
		 * RX hardware is started.
		 * This is just a performance optimization - we could keep
		 * scanning RX buffers forever, but under heavy traffic it is
		 * fairly common that rx_tasklet is executed while RX buffer
		 * ring is empty.
		 */
		if (priv->rx_runaway_wa.packet_counter > RX_PACKET_COUNT) {
			priv->rx_runaway_wa.warmup = false;
			rx->buffer_count = RX_PACKET_COUNT - 4;
		}
	}

	while (*hdr != 0) {
		rx_packet = aspeed_mctp_packet_alloc(GFP_ATOMIC);
		if (rx_packet) {
			memcpy(&rx_packet->data, hdr, sizeof(rx_packet->data));

			aspeed_mctp_swap_pcie_vdm_hdr(&rx_packet->data);

			aspeed_mctp_dispatch_packet(priv, rx_packet);
		} else {
			dev_dbg(priv->dev, "Failed to allocate RX packet\n");
		}

		*hdr = 0;
		rx->wr_ptr = (rx->wr_ptr + 1) % rx->buffer_count;
		hdr = (u32 *)&rx_buf[rx->wr_ptr];

		priv->rx_runaway_wa.packet_counter++;
	}

	/*
	 * Update HW write pointer, this can be done only after driver consumes
	 * packets from RX ring.
	 */
	regmap_read(priv->map, ASPEED_MCTP_RX_BUF_RD_PTR, &hw_read_ptr);
	hw_read_ptr &= RX_BUF_RD_PTR_MASK;
	regmap_write(priv->map, ASPEED_MCTP_RX_BUF_WR_PTR, (hw_read_ptr));

	dev_dbg(priv->dev, "RX hw ptr %02d, sw ptr %2d\n",
		hw_read_ptr, rx->wr_ptr);

	/* Kick RX if it was stopped due to ring full condition */
	if (rx->stopped) {
		regmap_update_bits(priv->map, ASPEED_MCTP_CTRL, RX_CMD_READY,
				   RX_CMD_READY);
		rx->stopped = false;
	}
}

static void aspeed_mctp_rx_chan_init(struct mctp_channel *rx)
{
	struct aspeed_mctp *priv = container_of(rx, typeof(*priv), rx);
	u32 *rx_cmd = (u32 *)rx->cmd.vaddr;
	u32 data_size = sizeof(struct mctp_pcie_packet_data);
	u32 hw_rx_count = RX_PACKET_COUNT;
	int i;

	for (i = 0; i < RX_PACKET_COUNT; i++) {
		*rx_cmd = RX_DATA_ADDR(rx->data.dma_handle + data_size * i);
		*rx_cmd |= RX_INTERRUPT_AFTER_CMD;
		rx_cmd++;
	}

	rx->buffer_count = RX_PACKET_COUNT;

	/*
	 * TODO: Once read pointer runaway bug is fixed in some future AST2x00
	 * stepping then add chip revision detection and turn on this
	 * workaround only when needed
	 */
	priv->rx_runaway_wa.enable = true;

	/*
	 * Hardware does not wrap around ASPEED_MCTP_RX_BUF_SIZE
	 * correctly - we have to set number of buffers to n/4 -1
	 */
	if (priv->rx_runaway_wa.enable)
		hw_rx_count = (RX_PACKET_COUNT / 4 - 1);

	regmap_write(priv->map, ASPEED_MCTP_RX_BUF_SIZE, hw_rx_count);
}

static void aspeed_mctp_tx_chan_init(struct mctp_channel *tx)
{
	struct aspeed_mctp *priv = container_of(tx, typeof(*priv), tx);

	tx->wr_ptr = 0;
	regmap_update_bits(priv->map, ASPEED_MCTP_CTRL, TX_CMD_TRIGGER, 0);
	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_SIZE, TX_PACKET_COUNT);
	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_WR_PTR, 0);
	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_ADDR, tx->cmd.dma_handle);
}

struct mctp_client *aspeed_mctp_create_client(struct aspeed_mctp *priv,
					      void *user_data,
					      void (*on_segment_change)(void *))
{
	struct mctp_client *client;

	client = aspeed_mctp_client_alloc(priv);
	if (!client)
		return NULL;
	client->user_data = user_data;
	client->on_segment_change = on_segment_change;

	init_waitqueue_head(&client->wait_queue);

	spin_lock_bh(&priv->clients_lock);
	list_add_tail(&client->link, &priv->clients);
	spin_unlock_bh(&priv->clients_lock);

	return client;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_create_client);

static int aspeed_mctp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct platform_device *pdev = to_platform_device(misc->parent);
	struct aspeed_mctp *priv = platform_get_drvdata(pdev);
	struct mctp_client *client;

	client = aspeed_mctp_create_client(priv, NULL, NULL);
	if (!client)
		return -ENOMEM;

	file->private_data = client;

	return 0;
}

void aspeed_mctp_delete_client(struct mctp_client *client)
{
	struct aspeed_mctp *priv = client->priv;
	struct mctp_type_handler *handler, *tmp;

	spin_lock_bh(&priv->clients_lock);

	list_del(&client->link);

	if (priv->default_client == client)
		priv->default_client = NULL;

	list_for_each_entry_safe(handler, tmp, &priv->mctp_type_handlers,
				 link) {
		if (handler->client == client) {
			list_del(&handler->link);
			kfree(handler);
		}
	}
	spin_unlock_bh(&priv->clients_lock);

	/* Disable the tasklet to appease lockdep */
	local_bh_disable();
	aspeed_mctp_client_put(client);
	local_bh_enable();
}
EXPORT_SYMBOL_GPL(aspeed_mctp_delete_client);

static int aspeed_mctp_release(struct inode *inode, struct file *file)
{
	struct mctp_client *client = file->private_data;

	aspeed_mctp_delete_client(client);

	return 0;
}

static u16 _get_bdf(struct aspeed_mctp *priv)
{
	u16 bdf;

	bdf = READ_ONCE(priv->pcie.bdf);
	smp_rmb(); /* enforce ordering between flush and producer */

	return bdf;
}

static void _set_bdf(struct aspeed_mctp *priv, u16 bdf)
{
	smp_wmb(); /* enforce ordering between flush and producer */
	WRITE_ONCE(priv->pcie.bdf, bdf);
}

#define LEN_MASK_HI GENMASK(9, 8)
#define LEN_MASK_LO GENMASK(7, 0)
#define PCI_VDM_HDR_LEN_MASK_LO GENMASK(31, 24)
#define PCI_VDM_HDR_LEN_MASK_HI GENMASK(17, 16)
#define PCIE_VDM_HDR_REQUESTER_BDF_MASK GENMASK(31, 16)

int aspeed_mctp_send_packet(struct mctp_client *client,
			    struct mctp_pcie_packet *packet)
{
	struct aspeed_mctp *priv = client->priv;
	u32 *hdr_dw = (u32 *)packet->data.hdr;
	u8 *hdr = (u8 *)packet->data.hdr;
	u16 packet_data_sz_dw;
	u16 pci_data_len_dw;
	int ret;
	u16 bdf;

	bdf = _get_bdf(priv);
	if (bdf == 0)
		return -EIO;

	/*
	 * If the data size is different from contents of PCIe VDM header,
	 * aspeed_mctp_tx_cmd will be programmed incorrectly. This may cause
	 * MCTP HW to stop working.
	 */
	pci_data_len_dw = FIELD_PREP(LEN_MASK_LO, FIELD_GET(PCI_VDM_HDR_LEN_MASK_LO, hdr_dw[0])) |
			FIELD_PREP(LEN_MASK_HI, FIELD_GET(PCI_VDM_HDR_LEN_MASK_HI, hdr_dw[0]));
	if (pci_data_len_dw == 0) /* According to PCIe Spec, 0 means 1024 DW */
		pci_data_len_dw = SZ_1K;

	packet_data_sz_dw = packet->size / sizeof(u32) - sizeof(packet->data.hdr) / sizeof(u32);
	if (packet_data_sz_dw != pci_data_len_dw)
		return -EINVAL;

	be32p_replace_bits(&hdr_dw[1], bdf, PCIE_VDM_HDR_REQUESTER_BDF_MASK);

	/*
	 * XXX Don't update EID for MCTP Control messages - old EID may
	 * interfere with MCTP discovery flow.
	 */
	if (priv->eid && hdr[MCTP_HDR_TYPE_OFFSET] != MCTP_HDR_TYPE_CONTROL)
		hdr[MCTP_HDR_SRC_EID_OFFSET] = priv->eid;

	ret = ptr_ring_produce_bh(&client->tx_queue, packet);
	if (!ret)
		tasklet_hi_schedule(&priv->tx.tasklet);

	return ret;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_send_packet);

struct mctp_pcie_packet *aspeed_mctp_receive_packet(struct mctp_client *client,
						    unsigned long timeout)
{
	struct aspeed_mctp *priv = client->priv;
	u16 bdf = _get_bdf(priv);
	int ret;

	if (bdf == 0)
		return ERR_PTR(-EIO);

	ret = wait_event_interruptible_timeout(client->wait_queue,
					       __ptr_ring_peek(&client->rx_queue),
					       timeout);
	if (ret < 0)
		return ERR_PTR(ret);
	else if (ret == 0)
		return ERR_PTR(-ETIME);

	return ptr_ring_consume_bh(&client->rx_queue);
}
EXPORT_SYMBOL_GPL(aspeed_mctp_receive_packet);

void aspeed_mctp_flush_rx_queue(struct mctp_client *client)
{
	struct mctp_pcie_packet *packet;

	while ((packet = ptr_ring_consume_bh(&client->rx_queue)))
		aspeed_mctp_packet_free(packet);
}
EXPORT_SYMBOL_GPL(aspeed_mctp_flush_rx_queue);

static ssize_t aspeed_mctp_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct mctp_client *client = file->private_data;
	struct aspeed_mctp *priv = client->priv;
	struct mctp_pcie_packet *rx_packet;
	u16 bdf;

	if (count < PCIE_MCTP_MIN_PACKET_SIZE)
		return -EINVAL;

	bdf = _get_bdf(priv);
	if (bdf == 0)
		return -EIO;

	if (count > sizeof(rx_packet->data))
		count = sizeof(rx_packet->data);

	rx_packet = ptr_ring_consume_bh(&client->rx_queue);
	if (!rx_packet)
		return -EAGAIN;

	if (copy_to_user(buf, &rx_packet->data, count)) {
		dev_err(priv->dev, "copy to user failed\n");
		count = -EFAULT;
	}

	aspeed_mctp_packet_free(rx_packet);

	return count;
}

static void aspeed_mctp_flush_tx_queue(struct mctp_client *client)
{
	struct mctp_pcie_packet *packet;

	while ((packet = ptr_ring_consume_bh(&client->tx_queue)))
		aspeed_mctp_packet_free(packet);
}

static void aspeed_mctp_flush_all_tx_queues(struct aspeed_mctp *priv)
{
	struct mctp_client *client;

	spin_lock_bh(&priv->clients_lock);
	list_for_each_entry(client, &priv->clients, link)
		aspeed_mctp_flush_tx_queue(client);
	spin_unlock_bh(&priv->clients_lock);
}

static ssize_t aspeed_mctp_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct mctp_client *client = file->private_data;
	struct aspeed_mctp *priv = client->priv;
	struct mctp_pcie_packet *tx_packet;
	int ret;

	if (count < PCIE_MCTP_MIN_PACKET_SIZE)
		return -EINVAL;

	if (count > sizeof(tx_packet->data))
		return -ENOSPC;

	tx_packet = aspeed_mctp_packet_alloc(GFP_KERNEL);
	if (!tx_packet) {
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(&tx_packet->data, buf, count)) {
		dev_err(priv->dev, "copy from user failed\n");
		ret = -EFAULT;
		goto out_packet;
	}

	tx_packet->size = count;

	ret = aspeed_mctp_send_packet(client, tx_packet);
	if (ret)
		goto out_packet;

	return count;

out_packet:
	aspeed_mctp_packet_free(tx_packet);
out:
	return ret;
}

int aspeed_mctp_add_type_handler(struct mctp_client *client, u8 mctp_type,
				 u16 pci_vendor_id, u16 vdm_type, u16 vdm_mask)
{
	struct aspeed_mctp *priv = client->priv;
	struct mctp_type_handler *handler, *new_handler;
	int ret = 0;

	if (mctp_type <= MCTP_HDR_TYPE_BASE_LAST) {
		/* Vendor, type and type mask must be zero for types 0-5 */
		if (pci_vendor_id != 0 || vdm_type != 0 || vdm_mask != 0)
			return -EINVAL;
	} else if (mctp_type == MCTP_HDR_TYPE_VDM_PCI) {
		/* For Vendor Defined PCI type the the vendor ID must be nonzero */
		if (pci_vendor_id == 0 || pci_vendor_id == 0xffff)
			return -EINVAL;
	} else {
		return -EINVAL;
	}

	new_handler = kzalloc(sizeof(*new_handler), GFP_KERNEL);
	if (!new_handler)
		return -ENOMEM;
	new_handler->mctp_type = mctp_type;
	new_handler->pci_vendor_id = pci_vendor_id;
	new_handler->vdm_type = vdm_type & vdm_mask;
	new_handler->vdm_mask = vdm_mask;
	new_handler->client = client;

	spin_lock_bh(&priv->clients_lock);
	list_for_each_entry(handler, &priv->mctp_type_handlers, link) {
		if (handler->mctp_type == new_handler->mctp_type &&
		    handler->pci_vendor_id == new_handler->pci_vendor_id &&
		    handler->vdm_type == new_handler->vdm_type) {
			if (handler->client != new_handler->client)
				ret = -EBUSY;
			kfree(new_handler);
			goto out_unlock;
		}
	}
	list_add_tail(&new_handler->link, &priv->mctp_type_handlers);
out_unlock:
	spin_unlock_bh(&priv->clients_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_add_type_handler);

int aspeed_mctp_remove_type_handler(struct mctp_client *client,
				    u8 mctp_type, u16 pci_vendor_id,
				    u16 vdm_type, u16 vdm_mask)
{
	struct aspeed_mctp *priv = client->priv;
	struct mctp_type_handler *handler, *tmp;
	int ret = -EINVAL;

	vdm_type &= vdm_mask;

	spin_lock_bh(&priv->clients_lock);
	list_for_each_entry_safe(handler, tmp, &priv->mctp_type_handlers,
				 link) {
		if (handler->client == client &&
		    handler->mctp_type == mctp_type &&
		    handler->pci_vendor_id == pci_vendor_id &&
		    handler->vdm_type == vdm_type) {
			list_del(&handler->link);
			kfree(handler);
			ret = 0;
			break;
		}
	}
	spin_unlock_bh(&priv->clients_lock);
	return ret;
}

static int aspeed_mctp_register_default_handler(struct mctp_client *client)
{
	struct aspeed_mctp *priv = client->priv;
	int ret = 0;

	spin_lock_bh(&priv->clients_lock);

	if (!priv->default_client)
		priv->default_client = client;
	else if (priv->default_client != client)
		ret = -EBUSY;

	spin_unlock_bh(&priv->clients_lock);

	return ret;
}

static int
aspeed_mctp_register_type_handler(struct mctp_client *client,
				  void __user *userbuf)
{
	struct aspeed_mctp *priv = client->priv;
	struct aspeed_mctp_type_handler_ioctl handler;

	if (copy_from_user(&handler, userbuf, sizeof(handler))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	return aspeed_mctp_add_type_handler(client, handler.mctp_type,
					    handler.pci_vendor_id,
					    handler.vendor_type,
					    handler.vendor_type_mask);
}

static int
aspeed_mctp_unregister_type_handler(struct mctp_client *client,
				    void __user *userbuf)
{
	struct aspeed_mctp *priv = client->priv;
	struct aspeed_mctp_type_handler_ioctl handler;

	if (copy_from_user(&handler, userbuf, sizeof(handler))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	return aspeed_mctp_remove_type_handler(client, handler.mctp_type,
					       handler.pci_vendor_id,
					       handler.vendor_type,
					       handler.vendor_type_mask);
}

static int
aspeed_mctp_filter_eid(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_filter_eid eid;

	if (copy_from_user(&eid, userbuf, sizeof(eid))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	if (eid.enable) {
		regmap_write(priv->map, ASPEED_MCTP_EID, eid.eid);
		regmap_update_bits(priv->map, ASPEED_MCTP_CTRL,
				   MATCHING_EID, MATCHING_EID);
	} else {
		regmap_update_bits(priv->map, ASPEED_MCTP_CTRL,
				   MATCHING_EID, 0);
	}
	return 0;
}

static int aspeed_mctp_get_bdf(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_bdf bdf = { _get_bdf(priv) };

	if (copy_to_user(userbuf, &bdf, sizeof(bdf))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int
aspeed_mctp_get_medium_id(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_medium_id id = { 0x09 }; /* PCIe revision 2.0 */

	if (copy_to_user(userbuf, &id, sizeof(id))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int
aspeed_mctp_get_mtu(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_mtu id = { ASPEED_MCTP_MTU };

	if (copy_to_user(userbuf, &id, sizeof(id))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

int aspeed_mctp_get_eid_bdf(struct mctp_client *client, u8 eid, u16 *bdf)
{
	struct aspeed_mctp_endpoint *endpoint;
	int ret = -ENOENT;

	mutex_lock(&client->priv->endpoints_lock);
	list_for_each_entry(endpoint, &client->priv->endpoints, link) {
		if (endpoint->data.eid_info.eid == eid) {
			*bdf = endpoint->data.eid_info.bdf;
			ret = 0;
			break;
		}
	}
	mutex_unlock(&client->priv->endpoints_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_get_eid_bdf);

int aspeed_mctp_get_eid(struct mctp_client *client, u16 bdf,
			u8 domain_id, u8 socket_id, u8 *eid)
{
	struct aspeed_mctp_endpoint *endpoint;
	int ret = -ENOENT;

	mutex_lock(&client->priv->endpoints_lock);

	list_for_each_entry(endpoint, &client->priv->endpoints, link) {
		if (endpoint->data.eid_ext_info.domain_id == domain_id &&
		    endpoint->data.eid_ext_info.bdf == bdf &&
			endpoint->data.eid_ext_info.socket_id == socket_id) {
			*eid = endpoint->data.eid_ext_info.eid;
			ret = 0;
			break;
		}
	}

	mutex_unlock(&client->priv->endpoints_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(aspeed_mctp_get_eid);

static int
aspeed_mctp_get_eid_info(struct aspeed_mctp *priv, void __user *userbuf,
			 enum mctp_address_type addr_format)
{
	int count = 0;
	int ret = 0;
	struct aspeed_mctp_get_eid_info get_eid;
	struct aspeed_mctp_endpoint *endpoint;
	void *user_ptr;
	size_t count_to_copy;

	if (copy_from_user(&get_eid, userbuf, sizeof(get_eid))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	mutex_lock(&priv->endpoints_lock);

	if (get_eid.count == 0) {
		count = priv->endpoints_count;
		goto out_unlock;
	}

	user_ptr = u64_to_user_ptr(get_eid.ptr);
	count_to_copy = get_eid.count > priv->endpoints_count ?
					priv->endpoints_count : get_eid.count;
	list_for_each_entry(endpoint, &priv->endpoints, link) {
		if (endpoint->data.eid_info.eid < get_eid.start_eid)
			continue;
		if (count >= count_to_copy)
			break;

		if (addr_format == ASPEED_MCTP_EXTENDED_ADDR_FORMAT)
			ret = copy_to_user(&(((struct aspeed_mctp_eid_ext_info *)
								user_ptr)[count]),
							&endpoint->data,
							sizeof(struct aspeed_mctp_eid_ext_info));
		else
			ret = copy_to_user(&(((struct aspeed_mctp_eid_info *)
								user_ptr)[count]),
							&endpoint->data,
							sizeof(struct aspeed_mctp_eid_info));

		if (ret) {
			dev_err(priv->dev, "copy to user failed\n");
			ret = -EFAULT;
			goto out_unlock;
		}
		count++;
	}

out_unlock:
	get_eid.count = count;
	if (copy_to_user(userbuf, &get_eid, sizeof(get_eid))) {
		dev_err(priv->dev, "copy to user failed\n");
		ret = -EFAULT;
	}

	mutex_unlock(&priv->endpoints_lock);
	return ret;
}

static int
eid_info_cmp(void *priv, const struct list_head *a, const struct list_head *b)
{
	struct aspeed_mctp_endpoint *endpoint_a;
	struct aspeed_mctp_endpoint *endpoint_b;

	if (a == b)
		return 0;

	endpoint_a = list_entry(a, typeof(*endpoint_a), link);
	endpoint_b = list_entry(b, typeof(*endpoint_b), link);

	if (endpoint_a->data.eid_info.eid < endpoint_b->data.eid_info.eid)
		return -1;
	else if (endpoint_a->data.eid_info.eid > endpoint_b->data.eid_info.eid)
		return 1;

	return 0;
}

static void aspeed_mctp_eid_info_list_remove(struct list_head *list)
{
	struct aspeed_mctp_endpoint *endpoint;
	struct aspeed_mctp_endpoint *tmp;

	list_for_each_entry_safe(endpoint, tmp, list, link) {
		list_del(&endpoint->link);
		kfree(endpoint);
	}
}

static bool
aspeed_mctp_eid_info_list_valid(struct list_head *list)
{
	struct aspeed_mctp_endpoint *endpoint;
	struct aspeed_mctp_endpoint *next;

	list_for_each_entry(endpoint, list, link) {
		next = list_next_entry(endpoint, link);
		if (&next->link == list)
			break;

		/* duplicted eids */
		if (next->data.eid_info.eid == endpoint->data.eid_info.eid)
			return false;
	}

	return true;
}

static int
aspeed_mctp_set_eid_info(struct aspeed_mctp *priv, void __user *userbuf,
			 enum mctp_address_type addr_format)
{
	struct list_head list = LIST_HEAD_INIT(list);
	struct aspeed_mctp_set_eid_info set_eid;
	void *user_ptr;
	struct aspeed_mctp_endpoint *endpoint;
	struct mctp_client *client;
	int ret = 0;
	u8 eid = 0;
	size_t i;

	if (copy_from_user(&set_eid, userbuf, sizeof(set_eid))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	if (set_eid.count > ASPEED_MCTP_EID_INFO_MAX)
		return -EINVAL;

	user_ptr = u64_to_user_ptr(set_eid.ptr);
	for (i = 0; i < set_eid.count; i++) {
		endpoint = kzalloc(sizeof(*endpoint), GFP_KERNEL);
		if (!endpoint) {
			ret = -ENOMEM;
			goto out;
		}
		memset(endpoint, 0, sizeof(*endpoint));

		if (addr_format == ASPEED_MCTP_EXTENDED_ADDR_FORMAT)
			ret = copy_from_user(&endpoint->data,
					     &(((struct aspeed_mctp_eid_ext_info *)
								user_ptr)[i]),
					     sizeof(struct aspeed_mctp_eid_ext_info));
		else
			ret = copy_from_user(&endpoint->data,
					     &(((struct aspeed_mctp_eid_info *)
								user_ptr)[i]),
						 sizeof(struct aspeed_mctp_eid_info));

		if (ret) {
			dev_err(priv->dev, "copy from user failed\n");
			kfree(endpoint);
			ret = -EFAULT;
			goto out;
		}

		/* Detect self EID */
		if (_get_bdf(priv) == endpoint->data.eid_info.bdf) {
			/*
			 * XXX Use smallest EID with matching BDF.
			 * On some platforms there could be multiple endpoints
			 * with same BDF in routing table.
			 */
			if (eid == 0 || endpoint->data.eid_info.eid < eid)
				eid = endpoint->data.eid_info.eid;
		}

	list_add_tail(&endpoint->link, &list);
	}

	list_sort(NULL, &list, eid_info_cmp);
	if (!aspeed_mctp_eid_info_list_valid(&list)) {
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&priv->endpoints_lock);
	if (list_empty(&priv->endpoints))
		list_splice_init(&list, &priv->endpoints);
	else
		list_swap(&list, &priv->endpoints);
	priv->endpoints_count = set_eid.count;
	priv->eid = eid;
	mutex_unlock(&priv->endpoints_lock);

	list_for_each_entry(client, &priv->clients, link) {
		if (client->on_segment_change && client->user_data)
			client->on_segment_change(client->user_data);
	}

out:
	aspeed_mctp_eid_info_list_remove(&list);
	return ret;
}

static int aspeed_mctp_set_own_eid(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_set_own_eid data;

	if (copy_from_user(&data, userbuf, sizeof(data))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	priv->eid = data.eid;

	return 0;
}

static long
aspeed_mctp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mctp_client *client = file->private_data;
	struct aspeed_mctp *priv = client->priv;
	void __user *userbuf = (void __user *)arg;
	int ret;

	switch (cmd) {
	case ASPEED_MCTP_IOCTL_FILTER_EID:
		ret = aspeed_mctp_filter_eid(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_BDF:
		ret = aspeed_mctp_get_bdf(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_MEDIUM_ID:
		ret = aspeed_mctp_get_medium_id(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_MTU:
		ret = aspeed_mctp_get_mtu(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_REGISTER_DEFAULT_HANDLER:
		ret = aspeed_mctp_register_default_handler(client);
	break;

	case ASPEED_MCTP_IOCTL_REGISTER_TYPE_HANDLER:
		ret = aspeed_mctp_register_type_handler(client, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_UNREGISTER_TYPE_HANDLER:
		ret = aspeed_mctp_unregister_type_handler(client, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_EID_INFO:
		ret = aspeed_mctp_get_eid_info(priv, userbuf, ASPEED_MCTP_GENERIC_ADDR_FORMAT);
	break;

	case ASPEED_MCTP_IOCTL_GET_EID_EXT_INFO:
		ret = aspeed_mctp_get_eid_info(priv, userbuf, ASPEED_MCTP_EXTENDED_ADDR_FORMAT);
	break;

	case ASPEED_MCTP_IOCTL_SET_EID_INFO:
		ret = aspeed_mctp_set_eid_info(priv, userbuf, ASPEED_MCTP_GENERIC_ADDR_FORMAT);
	break;

	case ASPEED_MCTP_IOCTL_SET_EID_EXT_INFO:
		ret = aspeed_mctp_set_eid_info(priv, userbuf, ASPEED_MCTP_EXTENDED_ADDR_FORMAT);
	break;

	case ASPEED_MCTP_IOCTL_SET_OWN_EID:
		ret = aspeed_mctp_set_own_eid(priv, userbuf);
	break;

	default:
		dev_err(priv->dev, "Command not found\n");
		ret = -ENOTTY;
	}

	return ret;
}

static __poll_t aspeed_mctp_poll(struct file *file,
				 struct poll_table_struct *pt)
{
	struct mctp_client *client = file->private_data;
	__poll_t ret = 0;

	poll_wait(file, &client->wait_queue, pt);

	if (!ptr_ring_full_bh(&client->tx_queue))
		ret |= EPOLLOUT;

	if (__ptr_ring_peek(&client->rx_queue))
		ret |= EPOLLIN;

	return ret;
}

static const struct file_operations aspeed_mctp_fops = {
	.owner = THIS_MODULE,
	.open = aspeed_mctp_open,
	.release = aspeed_mctp_release,
	.read = aspeed_mctp_read,
	.write = aspeed_mctp_write,
	.unlocked_ioctl = aspeed_mctp_ioctl,
	.poll = aspeed_mctp_poll,
};

static const struct regmap_config aspeed_mctp_regmap_cfg = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= ASPEED_MCTP_TX_BUF_WR_PTR,
};

struct device_type aspeed_mctp_type = {
	.name		= "aspeed-mctp",
};

static struct miscdevice aspeed_mctp_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aspeed-mctp",
	.fops = &aspeed_mctp_fops,
};

static void aspeed_mctp_send_pcie_uevent(struct kobject *kobj, bool ready)
{
	char *pcie_not_ready_event[] = { ASPEED_MCTP_READY "=0", NULL };
	char *pcie_ready_event[] = { ASPEED_MCTP_READY "=1", NULL };

	kobject_uevent_env(kobj, KOBJ_CHANGE,
			   ready ? pcie_ready_event : pcie_not_ready_event);
}

static u16 aspeed_mctp_pcie_setup(struct aspeed_mctp *priv)
{
	u32 reg;
	u16 bdf;

	regmap_read(priv->pcie.map, ASPEED_PCIE_MISC_STS_1, &reg);

	bdf = PCI_DEVID(GET_PCI_BUS_NUM(reg), GET_PCI_DEV_NUM(reg));
	if (bdf != 0)
		cancel_delayed_work(&priv->pcie.rst_dwork);
	else
		schedule_delayed_work(&priv->pcie.rst_dwork,
				      msecs_to_jiffies(1000));

	return bdf;
}

static void aspeed_mctp_irq_enable(struct aspeed_mctp *priv)
{
	u32 enable = TX_CMD_SENT_INT | TX_CMD_WRONG_INT |
		     RX_CMD_RECEIVE_INT | RX_CMD_NO_MORE_INT;

	regmap_write(priv->map, ASPEED_MCTP_INT_EN, enable);
}

static void aspeed_mctp_irq_disable(struct aspeed_mctp *priv)
{
	regmap_write(priv->map, ASPEED_MCTP_INT_EN, 0);
}

static void aspeed_mctp_reset_work(struct work_struct *work)
{
	struct aspeed_mctp *priv = container_of(work, typeof(*priv),
						pcie.rst_dwork.work);
	struct kobject *kobj = &aspeed_mctp_miscdev.this_device->kobj;
	u16 bdf;

	if (priv->pcie.need_uevent) {
		aspeed_mctp_send_pcie_uevent(kobj, false);
		priv->pcie.need_uevent = false;
	}

	bdf = aspeed_mctp_pcie_setup(priv);
	if (bdf) {
		aspeed_mctp_flush_all_tx_queues(priv);
		aspeed_mctp_irq_enable(priv);
		aspeed_mctp_rx_trigger(&priv->rx);
		_set_bdf(priv, bdf);
		aspeed_mctp_send_pcie_uevent(kobj, true);
	}
}

static void aspeed_mctp_channels_init(struct aspeed_mctp *priv)
{
	aspeed_mctp_rx_chan_init(&priv->rx);
	aspeed_mctp_tx_chan_init(&priv->tx);
}

static void aspeed_mctp_mtu_init(struct aspeed_mctp *priv)
{
	regmap_update_bits(priv->map, ASPEED_MCTP_ENGINE_CTRL, RX_MAX_PAYLOAD_SIZE_MASK, 0);

	/*
	 * XXX: HW sets EOM bit in first packet of multipacket frame if its
	 * content size is equal or greater MTU, what causes such frames cannot be
	 * send correctly by aspeed controller.
	 * According to recomendation TX MTU size shall be set to 128 bytes, and
	 * EOM bit shall be control by software.
	 */
	regmap_update_bits(priv->map, ASPEED_MCTP_ENGINE_CTRL, TX_MAX_PAYLOAD_SIZE_MASK, 1);
}

static irqreturn_t aspeed_mctp_irq_handler(int irq, void *arg)
{
	struct aspeed_mctp *priv = arg;
	u32 handled = 0;
	u32 status;

	regmap_read(priv->map, ASPEED_MCTP_INT_STS, &status);
	regmap_write(priv->map, ASPEED_MCTP_INT_STS, status);

	if (status & TX_CMD_SENT_INT) {
		tasklet_hi_schedule(&priv->tx.tasklet);

		handled |= TX_CMD_SENT_INT;
	}

	if (status & TX_CMD_WRONG_INT) {
		/* TODO: print the actual command */
		dev_warn(priv->dev, "TX wrong");

		handled |= TX_CMD_WRONG_INT;
	}

	if (status & RX_CMD_RECEIVE_INT) {
		tasklet_hi_schedule(&priv->rx.tasklet);

		handled |= RX_CMD_RECEIVE_INT;
	}

	if (status & RX_CMD_NO_MORE_INT) {
		dev_dbg(priv->dev, "RX full");
		priv->rx.stopped = true;
		tasklet_hi_schedule(&priv->rx.tasklet);

		handled |= RX_CMD_NO_MORE_INT;
	}

	if (!handled)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_mctp_pcie_rst_irq_handler(int irq, void *arg)
{
	struct aspeed_mctp *priv = arg;

	aspeed_mctp_channels_init(priv);

	priv->pcie.need_uevent = true;
	_set_bdf(priv, 0);
	priv->eid = 0;

	schedule_delayed_work(&priv->pcie.rst_dwork, 0);

	return IRQ_HANDLED;
}

static void aspeed_mctp_drv_init(struct aspeed_mctp *priv)
{
	INIT_LIST_HEAD(&priv->clients);
	INIT_LIST_HEAD(&priv->mctp_type_handlers);
	INIT_LIST_HEAD(&priv->endpoints);

	spin_lock_init(&priv->clients_lock);
	mutex_init(&priv->endpoints_lock);

	INIT_DELAYED_WORK(&priv->pcie.rst_dwork, aspeed_mctp_reset_work);

	tasklet_init(&priv->tx.tasklet, aspeed_mctp_tx_tasklet,
		     (unsigned long)&priv->tx);
	tasklet_init(&priv->rx.tasklet, aspeed_mctp_rx_tasklet,
		     (unsigned long)&priv->rx);
}

static void aspeed_mctp_drv_fini(struct aspeed_mctp *priv)
{
	aspeed_mctp_eid_info_list_remove(&priv->endpoints);
	tasklet_disable(&priv->tx.tasklet);
	tasklet_kill(&priv->tx.tasklet);
	tasklet_disable(&priv->rx.tasklet);
	tasklet_kill(&priv->rx.tasklet);

	cancel_delayed_work_sync(&priv->pcie.rst_dwork);
}

static int aspeed_mctp_resources_init(struct aspeed_mctp *priv)
{
	struct platform_device *pdev = to_platform_device(priv->dev);
	void __iomem *regs;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		dev_err(priv->dev, "Failed to get regmap!\n");
		return PTR_ERR(regs);
	}

	priv->map = devm_regmap_init_mmio(priv->dev, regs,
					  &aspeed_mctp_regmap_cfg);
	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);

	priv->reset = devm_reset_control_get(priv->dev, 0);
	if (IS_ERR(priv->reset)) {
		dev_err(priv->dev, "Failed to get reset!\n");
		return PTR_ERR(priv->reset);
	}

	priv->pcie.map =
		syscon_regmap_lookup_by_phandle(priv->dev->of_node,
						"aspeed,pcieh");
	if (IS_ERR(priv->pcie.map)) {
		dev_err(priv->dev, "Failed to find PCIe Host regmap!\n");
		return PTR_ERR(priv->pcie.map);
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int aspeed_mctp_dma_init(struct aspeed_mctp *priv)
{
	struct mctp_channel *tx = &priv->tx;
	struct mctp_channel *rx = &priv->rx;
	int ret = -ENOMEM;

	BUILD_BUG_ON(TX_PACKET_COUNT >= TX_MAX_PACKET_COUNT);
	BUILD_BUG_ON(RX_PACKET_COUNT >= RX_MAX_PACKET_COUNT);

	tx->cmd.vaddr = dma_alloc_coherent(priv->dev, TX_CMD_BUF_SIZE,
					   &tx->cmd.dma_handle, GFP_KERNEL);

	if (!tx->cmd.vaddr)
		return ret;

	tx->data.vaddr = dma_alloc_coherent(priv->dev, TX_DATA_BUF_SIZE,
					    &tx->data.dma_handle, GFP_KERNEL);

	if (!tx->data.vaddr)
		goto out_tx_data;

	rx->cmd.vaddr = dma_alloc_coherent(priv->dev, RX_CMD_BUF_SIZE,
					   &rx->cmd.dma_handle, GFP_KERNEL);

	if (!rx->cmd.vaddr)
		goto out_tx_cmd;

	rx->data.vaddr = dma_alloc_coherent(priv->dev, RX_DATA_BUF_SIZE,
					    &rx->data.dma_handle, GFP_KERNEL);

	if (!rx->data.vaddr)
		goto out_rx_data;

	return 0;
out_rx_data:
	dma_free_coherent(priv->dev, RX_CMD_BUF_SIZE, rx->cmd.vaddr,
			  rx->cmd.dma_handle);

out_tx_cmd:
	dma_free_coherent(priv->dev, TX_DATA_BUF_SIZE, tx->data.vaddr,
			  tx->data.dma_handle);

out_tx_data:
	dma_free_coherent(priv->dev, TX_CMD_BUF_SIZE, tx->cmd.vaddr,
			  tx->cmd.dma_handle);
	return ret;
}

static void aspeed_mctp_dma_fini(struct aspeed_mctp *priv)
{
	struct mctp_channel *tx = &priv->tx;
	struct mctp_channel *rx = &priv->rx;

	dma_free_coherent(priv->dev, TX_CMD_BUF_SIZE, tx->cmd.vaddr,
			  tx->cmd.dma_handle);

	dma_free_coherent(priv->dev, RX_CMD_BUF_SIZE, rx->cmd.vaddr,
			  rx->cmd.dma_handle);

	dma_free_coherent(priv->dev, TX_DATA_BUF_SIZE, tx->data.vaddr,
			  tx->data.dma_handle);

	dma_free_coherent(priv->dev, RX_DATA_BUF_SIZE, rx->data.vaddr,
			  rx->data.dma_handle);
}

static int aspeed_mctp_irq_init(struct aspeed_mctp *priv)
{
	struct platform_device *pdev = to_platform_device(priv->dev);
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (!irq)
		return -ENODEV;

	ret = devm_request_irq(priv->dev, irq, aspeed_mctp_irq_handler,
			       IRQF_SHARED, "aspeed-mctp", priv);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 1);
	if (!irq)
		return -ENODEV;

	ret = devm_request_irq(priv->dev, irq,
			       aspeed_mctp_pcie_rst_irq_handler,
			       IRQF_SHARED, "aspeed-mctp", priv);
	if (ret)
		return ret;

	return 0;
}

static void aspeed_mctp_hw_reset(struct aspeed_mctp *priv)
{
	u32 reg;

	/*
	 * XXX: We need to skip the reset when we probe multiple times.
	 * Currently calling reset more than once seems to make the HW upset,
	 * however, we do need to reset once after the first boot before we're
	 * able to use the HW.
	 */
	regmap_read(priv->map, ASPEED_MCTP_TX_BUF_ADDR, &reg);

	if (reg) {
		dev_info(priv->dev,
			 "Already initialized - skipping hardware reset\n");
		return;
	}

	if (reset_control_assert(priv->reset) != 0)
		dev_warn(priv->dev, "Failed to assert reset\n");

	if (reset_control_deassert(priv->reset) != 0)
		dev_warn(priv->dev, "Failed to deassert reset\n");
}

static int aspeed_mctp_probe(struct platform_device *pdev)
{
	struct aspeed_mctp *priv;
	int ret;
	u16 bdf;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto out;
	}
	priv->dev = &pdev->dev;

	aspeed_mctp_drv_init(priv);

	ret = aspeed_mctp_resources_init(priv);
	if (ret) {
		dev_err(priv->dev, "Failed to init resources\n");
		goto out_drv;
	}

	ret = aspeed_mctp_dma_init(priv);
	if (ret) {
		dev_err(priv->dev, "Failed to init DMA\n");
		goto out_drv;
	}

	aspeed_mctp_mtu_init(priv);

	aspeed_mctp_hw_reset(priv);

	aspeed_mctp_channels_init(priv);

	aspeed_mctp_miscdev.parent = priv->dev;
	ret = misc_register(&aspeed_mctp_miscdev);
	if (ret) {
		dev_err(priv->dev, "Failed to register miscdev\n");
		goto out_dma;
	}
	aspeed_mctp_miscdev.this_device->type = &aspeed_mctp_type;

	ret = aspeed_mctp_irq_init(priv);
	if (ret) {
		dev_err(priv->dev, "Failed to init IRQ!\n");
		goto out_dma;
	}

	aspeed_mctp_irq_enable(priv);

	bdf = aspeed_mctp_pcie_setup(priv);
	if (bdf != 0)
		_set_bdf(priv, bdf);

	priv->peci_mctp =
		platform_device_register_data(priv->dev, "peci-mctp",
					      PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(priv->peci_mctp))
		dev_err(priv->dev, "Failed to register peci-mctp device\n");

	aspeed_mctp_rx_trigger(&priv->rx);

	return 0;

out_dma:
	aspeed_mctp_dma_fini(priv);
out_drv:
	aspeed_mctp_drv_fini(priv);
out:
	dev_err(&pdev->dev, "Failed to probe Aspeed MCTP: %d\n", ret);
	return ret;
}

static int aspeed_mctp_remove(struct platform_device *pdev)
{
	struct aspeed_mctp *priv = platform_get_drvdata(pdev);

	platform_device_unregister(priv->peci_mctp);

	misc_deregister(&aspeed_mctp_miscdev);

	aspeed_mctp_irq_disable(priv);

	aspeed_mctp_dma_fini(priv);

	aspeed_mctp_drv_fini(priv);

	return 0;
}

static const struct of_device_id aspeed_mctp_match_table[] = {
	{ .compatible = "aspeed,ast2600-mctp" },
	{ }
};

static struct platform_driver aspeed_mctp_driver = {
	.driver	= {
		.name		= "aspeed-mctp",
		.of_match_table	= of_match_ptr(aspeed_mctp_match_table),
	},
	.probe	= aspeed_mctp_probe,
	.remove	= aspeed_mctp_remove,
};

static int __init aspeed_mctp_init(void)
{
	packet_cache =
		kmem_cache_create_usercopy("mctp-packet",
					   sizeof(struct mctp_pcie_packet),
					   0, 0, 0,
					   sizeof(struct mctp_pcie_packet),
					   NULL);
	if (!packet_cache)
		return -ENOMEM;

	return platform_driver_register(&aspeed_mctp_driver);
}

static void __exit aspeed_mctp_exit(void)
{
	platform_driver_unregister(&aspeed_mctp_driver);
	kmem_cache_destroy(packet_cache);
}

module_init(aspeed_mctp_init)
module_exit(aspeed_mctp_exit)

MODULE_DEVICE_TABLE(of, aspeed_mctp_match_table);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Iwona Winiarska <iwona.winiarska@intel.com>");
MODULE_DESCRIPTION("Aspeed AST2600 MCTP driver");
