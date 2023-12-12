// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2022 Intel Corporation.*/

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/list_sort.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/preempt.h>
#include <linux/ptr_ring.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <linux/i3c/device.h>

#include <linux/i3c/mctp/i3c-mctp.h>

#include <uapi/linux/i3c/i3c-mctp.h>

#define I3C_MCTP_MINORS				32
#define CCC_DEVICE_STATUS_PENDING_INTR(x)	(((x) & GENMASK(3, 0)) >> 0)
#define POLLING_TIMEOUT_MS			50
#define MCTP_INTERRUPT_NUMBER			1
#define RX_RING_COUNT				16
#define I3C_MCTP_MIN_TRANSFER_SIZE		69
#define I3C_MCTP_IBI_PAYLOAD_SIZE		2

/* MCTP header definitions */
#define MCTP_HDR_SRC_EID_OFFSET			2

#define MAX_PROCESS_COUNT			255

#define IS_BMC_NON_LEGACY(pid) (((pid) & 0xF) == 0xB)

struct i3c_mctp {
	struct i3c_device *i3c;
	struct cdev cdev;
	struct device *dev;
	struct delayed_work polling_work;
	struct platform_device *i3c_peci;
	int id;
	/* Currently only one userspace client is supported */
	struct i3c_mctp_client *default_client;
	struct i3c_mctp_client *peci_client;
	u16 max_read_len;
	u16 max_write_len;
	struct list_head endpoints;
	size_t endpoints_count;
	/*
	 * endpoints_lock protects list of endpoints
	 */
	struct mutex endpoints_lock;
	spinlock_t clients_lock; /* to protect PECI and default client accesses */
	u8 eid;
	/*
	 * As there can be more than one process opening the /dev file - we need
	 * a counter to take care of device cleanup in case the file is opened
	 * on device removal. We also need a locker to avoid potential race
	 * conditions.
	 */
	spinlock_t file_lock;
	u8 process_count;
	bool ibi_enabled;
};

struct i3c_mctp_client {
	struct kref ref;
	struct i3c_mctp *priv;
	struct ptr_ring rx_queue;
	wait_queue_head_t wait_queue;
};

struct i3c_mctp_endpoint {
	struct i3c_mctp_eid_info eid_info;
	struct list_head link;
};

static struct class *i3c_mctp_class;
static dev_t i3c_mctp_devt;
static DEFINE_IDA(i3c_mctp_ida);

static struct kmem_cache *packet_cache;

/**
 * i3c_mctp_packet_alloc() - allocates i3c_mctp_packet
 *
 * @flags: the type of memory to allocate
 *
 * Allocates i3c_mctp_packet via slab allocation
 * Return: pointer to the packet, NULL if some error occurred
 */
void *i3c_mctp_packet_alloc(gfp_t flags)
{
	return kmem_cache_alloc(packet_cache, flags);
}
EXPORT_SYMBOL_GPL(i3c_mctp_packet_alloc);

/**
 * i3c_mctp_packet_free() - frees i3c_mctp_packet
 *
 * @packet: pointer to the packet which should be freed
 *
 * Frees i3c_mctp_packet previously allocated via slab allocation
 */
void i3c_mctp_packet_free(void *packet)
{
	kmem_cache_free(packet_cache, packet);
}
EXPORT_SYMBOL_GPL(i3c_mctp_packet_free);

/**
 * i3c_mctp_flush_rx_queue() - flushes mctp client rx queue
 *
 * @client: pointer to the i3c mctp client whose rx queue should be flushed
 */
void i3c_mctp_flush_rx_queue(struct i3c_mctp_client *client)
{
	struct i3c_mctp_packet *packet;

	while ((packet = ptr_ring_consume_bh(&client->rx_queue)))
		i3c_mctp_packet_free(packet);
}
EXPORT_SYMBOL_GPL(i3c_mctp_flush_rx_queue);

static void i3c_mctp_client_free(struct kref *ref)
{
	struct i3c_mctp_client *client = container_of(ref, typeof(*client), ref);

	ptr_ring_cleanup(&client->rx_queue, &i3c_mctp_packet_free);

	kfree(client);
}

static void i3c_mctp_client_get(struct i3c_mctp_client *client)
{
	kref_get(&client->ref);
}

static void i3c_mctp_client_put(struct i3c_mctp_client *client)
{
	kref_put(&client->ref, &i3c_mctp_client_free);
}

static struct i3c_mctp_client *i3c_mctp_client_alloc(struct i3c_mctp *priv)
{
	struct i3c_mctp_client *client;
	int ret;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	kref_init(&client->ref);
	client->priv = priv;
	ret = ptr_ring_init(&client->rx_queue, RX_RING_COUNT, GFP_KERNEL);
	if (ret)
		goto out;

	init_waitqueue_head(&client->wait_queue);

	return client;
out:
	i3c_mctp_client_put(client);
	return ERR_PTR(ret);
}

static int i3c_mctp_register_default_client(struct i3c_mctp *priv, struct file *file)
{
	struct i3c_mctp_client *client;

	if (priv->default_client)
		return -EBUSY;

	client = i3c_mctp_client_alloc(priv);
	if (IS_ERR(client))
		return PTR_ERR(client);

	file->private_data = client;
	spin_lock(&priv->clients_lock);
	priv->default_client = client;
	spin_unlock(&priv->clients_lock);

	return 0;
}

static struct i3c_mctp_client *i3c_mctp_find_client(struct i3c_mctp *priv,
						    struct i3c_mctp_packet *packet)
{
	u8 *msg_hdr = (u8 *)packet->data.payload;
	u8 mctp_type = msg_hdr[MCTP_MSG_HDR_MSG_TYPE_OFFSET];
	u16 vendor = (msg_hdr[MCTP_MSG_HDR_VENDOR_OFFSET] << 8
		      | msg_hdr[MCTP_MSG_HDR_VENDOR_OFFSET + 1]);
	u8 intel_msg_op_code = msg_hdr[MCTP_MSG_HDR_OPCODE_OFFSET];

	if (priv->peci_client && mctp_type == MCTP_MSG_TYPE_VDM_PCI &&
	    vendor == MCTP_VDM_PCI_INTEL_VENDOR_ID && intel_msg_op_code == MCTP_VDM_PCI_INTEL_PECI)
		return priv->peci_client;

	return priv->default_client;
}

static struct i3c_mctp_packet *i3c_mctp_read_packet(struct i3c_device *i3c)
{
	struct i3c_mctp *priv = i3cdev_get_drvdata(i3c);
	struct i3c_mctp_packet *rx_packet;
	struct i3c_priv_xfer xfers = {
		.rnw = true,
	};
	int ret;

	rx_packet = i3c_mctp_packet_alloc(GFP_KERNEL);
	if (!rx_packet)
		return ERR_PTR(-ENOMEM);

	rx_packet->size = I3C_MCTP_PACKET_SIZE;
	xfers.len = rx_packet->size;
	xfers.data.in = &rx_packet->data;

	if (priv->max_read_len < xfers.len + 1) {
		dev_dbg(i3cdev_to_dev(i3c), "Length mismatch. MRL = %d, xfers.len = %d",
			priv->max_read_len, xfers.len);
		i3c_mctp_packet_free(rx_packet);
		return ERR_PTR(-EINVAL);
	}

	ret = i3c_device_do_priv_xfers(i3c, &xfers, 1);
	if (ret) {
		i3c_mctp_packet_free(rx_packet);
		return ERR_PTR(ret);
	}

	rx_packet->size = xfers.len;

	return rx_packet;
}

static void i3c_mctp_dispatch_packet(struct i3c_mctp *priv, struct i3c_mctp_packet *packet)
{
	struct i3c_mctp_client *client;
	int ret;

	spin_lock(&priv->clients_lock);
	client = i3c_mctp_find_client(priv, packet);
	if (client)
		i3c_mctp_client_get(client);
	spin_unlock(&priv->clients_lock);

	if (!client) {
		i3c_mctp_packet_free(packet);
		return;
	}

	ret = ptr_ring_produce(&client->rx_queue, packet);
	if (ret)
		i3c_mctp_packet_free(packet);
	else
		wake_up_all(&client->wait_queue);

	i3c_mctp_client_put(client);
}

static void i3c_mctp_polling_work(struct work_struct *work)
{
	struct i3c_mctp *priv = container_of(to_delayed_work(work), struct i3c_mctp, polling_work);
	struct i3c_device *i3cdev = priv->i3c;
	struct i3c_mctp_packet *rx_packet;
	struct i3c_device_info info;
	int ret;

	i3c_device_get_info(i3cdev, &info);
	ret = i3c_device_getstatus_ccc(i3cdev, &info);
	if (ret)
		goto out;

	if (CCC_DEVICE_STATUS_PENDING_INTR(info.status) != MCTP_INTERRUPT_NUMBER)
		goto out;

	rx_packet = i3c_mctp_read_packet(i3cdev);
	if (IS_ERR(rx_packet))
		goto out;

	i3c_mctp_dispatch_packet(priv, rx_packet);
out:
	schedule_delayed_work(&priv->polling_work, msecs_to_jiffies(POLLING_TIMEOUT_MS));
}

static int i3c_mctp_open(struct inode *inode, struct file *file)
{
	struct i3c_mctp *priv = container_of(inode->i_cdev, struct i3c_mctp, cdev);
	int ret = 0;

	spin_lock(&priv->file_lock);
	if (priv->process_count >= MAX_PROCESS_COUNT) {
		ret = -EBUSY;
		goto out_unlock;
	}

	priv->process_count++;

out_unlock:
	spin_unlock(&priv->file_lock);

	return ret;
}

static ssize_t i3c_mctp_write(struct file *file, const char __user *buf, size_t count,
			      loff_t *f_pos)
{
	struct i3c_mctp_client *client = file->private_data;
	struct i3c_mctp_packet *tx_packet;
	int ret;

	if (!client || !client->priv)
		return -EBADF;

	if (count < I3C_MCTP_MIN_PACKET_SIZE)
		return -EINVAL;

	if (count > sizeof(tx_packet->data))
		return -ENOSPC;

	tx_packet = i3c_mctp_packet_alloc(GFP_KERNEL);
	if (!tx_packet)
		return -ENOMEM;

	if (copy_from_user(&tx_packet->data, buf, count)) {
		dev_err(client->priv->dev, "copy from user failed\n");
		ret = -EFAULT;
		goto out_packet;
	}

	tx_packet->size = count;

	ret = i3c_mctp_send_packet(client->priv->i3c, tx_packet);
	if (ret)
		goto out_packet;

	ret = count;

out_packet:
	i3c_mctp_packet_free(tx_packet);
	return ret;
}

static ssize_t i3c_mctp_read(struct file *file, char __user *buf, size_t count, loff_t *f_pos)
{
	struct i3c_mctp_client *client = file->private_data;
	struct i3c_mctp_packet *rx_packet;

	if (!client)
		return -EBADF;

	if (count < I3C_MCTP_MIN_PACKET_SIZE)
		return -EINVAL;

	if (count > sizeof(rx_packet->data))
		count = sizeof(rx_packet->data);

	rx_packet = ptr_ring_consume(&client->rx_queue);
	if (!rx_packet)
		return -EAGAIN;

	if (count > rx_packet->size)
		count = rx_packet->size;

	if (copy_to_user(buf, &rx_packet->data, count))
		count = -EFAULT;

	i3c_mctp_packet_free(rx_packet);

	return count;
}

static int i3c_mctp_release(struct inode *inode, struct file *file)
{
	struct i3c_mctp *priv = container_of(inode->i_cdev, struct i3c_mctp, cdev);
	struct i3c_mctp_client *client = file->private_data;

	if (inode->i_cdev && priv) {
		spin_lock(&priv->file_lock);
		priv->process_count--;
		spin_unlock(&priv->file_lock);
	}

	if (!client)
		return 0;

	if (!client->priv)
		goto out;

	spin_lock(&client->priv->clients_lock);
	client->priv->default_client = NULL;
	spin_unlock(&client->priv->clients_lock);

out:
	i3c_mctp_client_put(client);

	file->private_data = NULL;

	return 0;
}

static __poll_t i3c_mctp_poll(struct file *file, struct poll_table_struct *pt)
{
	struct i3c_mctp_client *client = file->private_data;
	__poll_t ret = 0;

	if (!client)
		return ret;

	poll_wait(file, &client->wait_queue, pt);

	if (__ptr_ring_peek(&client->rx_queue))
		ret |= EPOLLIN;

	return ret;
}

static int
eid_info_cmp(void *priv, const struct list_head *a, const struct list_head *b)
{
	struct i3c_mctp_endpoint *endpoint_a;
	struct i3c_mctp_endpoint *endpoint_b;

	if (a == b)
		return 0;

	endpoint_a = list_entry(a, typeof(*endpoint_a), link);
	endpoint_b = list_entry(b, typeof(*endpoint_b), link);

	if (endpoint_a->eid_info.eid < endpoint_b->eid_info.eid)
		return -1;
	else if (endpoint_a->eid_info.eid > endpoint_b->eid_info.eid)
		return 1;

	return 0;
}

static void i3c_mctp_eid_info_list_remove(struct list_head *list)
{
	struct i3c_mctp_endpoint *endpoint;
	struct i3c_mctp_endpoint *tmp;

	list_for_each_entry_safe(endpoint, tmp, list, link) {
		list_del(&endpoint->link);
		kfree(endpoint);
	}
}

static bool
i3c_mctp_eid_info_list_valid(struct list_head *list)
{
	struct i3c_mctp_endpoint *endpoint;
	struct i3c_mctp_endpoint *next;

	list_for_each_entry(endpoint, list, link) {
		next = list_next_entry(endpoint, link);
		if (&next->link == list)
			break;

		/* duplicated eids */
		if (next->eid_info.eid == endpoint->eid_info.eid)
			return false;
	}

	return true;
}

static int
i3c_mctp_set_eid_info(struct i3c_mctp *priv, struct i3c_mctp_set_eid_info __user *userbuf)
{
	struct list_head list = LIST_HEAD_INIT(list);
	struct i3c_mctp_set_eid_info set_eid;
	struct i3c_mctp_endpoint *endpoint;
	void *user_ptr;
	int ret = 0;
	size_t i;

	if (copy_from_user(&set_eid, userbuf, sizeof(set_eid))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	if (set_eid.count > I3C_MCTP_EID_INFO_MAX)
		return -EINVAL;

	user_ptr = u64_to_user_ptr(set_eid.ptr);
	for (i = 0; i < set_eid.count; i++) {
		endpoint = kzalloc(sizeof(*endpoint), GFP_KERNEL);
		if (!endpoint) {
			ret = -ENOMEM;
			goto out;
		}
		memset(endpoint, 0, sizeof(*endpoint));

		ret = copy_from_user(&endpoint->eid_info,
				     &(((struct i3c_mctp_eid_info *)user_ptr)[i]),
				     sizeof(struct i3c_mctp_eid_info));

		if (ret) {
			dev_err(priv->dev, "copy from user failed\n");
			kfree(endpoint);
			ret = -EFAULT;
			goto out;
		}

		list_add_tail(&endpoint->link, &list);
	}

	list_sort(NULL, &list, eid_info_cmp);
	if (!i3c_mctp_eid_info_list_valid(&list)) {
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&priv->endpoints_lock);
	if (list_empty(&priv->endpoints))
		list_splice_init(&list, &priv->endpoints);
	else
		list_swap(&list, &priv->endpoints);
	priv->endpoints_count = set_eid.count;
	mutex_unlock(&priv->endpoints_lock);
out:
	i3c_mctp_eid_info_list_remove(&list);
	return ret;
}

static int i3c_mctp_set_own_eid(struct i3c_mctp *priv, void __user *userbuf)
{
	struct i3c_mctp_set_own_eid data;

	if (copy_from_user(&data, userbuf, sizeof(data))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	priv->eid = data.eid;

	return 0;
}

static long
i3c_mctp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i3c_mctp *priv = container_of(file_inode(file)->i_cdev, struct i3c_mctp, cdev);
	void __user *userbuf = (void __user *)arg;
	int ret;

	if (!file_inode(file)->i_cdev)
		return -ENODEV;

	switch (cmd) {
	case I3C_MCTP_IOCTL_SET_EID_INFO:
		ret = i3c_mctp_set_eid_info(priv, userbuf);
		break;
	case I3C_MCTP_IOCTL_SET_OWN_EID:
		ret = i3c_mctp_set_own_eid(priv, userbuf);
		break;
	case I3C_MCTP_IOCTL_REGISTER_DEFAULT_CLIENT:
		ret = i3c_mctp_register_default_client(priv, file);
		break;
	default:
		break;
	}
	return ret;
}

static const struct file_operations i3c_mctp_fops = {
	.owner = THIS_MODULE,
	.open = i3c_mctp_open,
	.read = i3c_mctp_read,
	.write = i3c_mctp_write,
	.poll = i3c_mctp_poll,
	.release = i3c_mctp_release,
	.unlocked_ioctl = i3c_mctp_ioctl,
};

/**
 * i3c_mctp_add_peci_client() - registers PECI client
 * @i3c: I3C device to get the PECI client for
 *
 * Return: pointer to PECI client, -ENOMEM - in case of client alloc fault
 */
struct i3c_mctp_client *i3c_mctp_add_peci_client(struct i3c_device *i3c)
{
	struct i3c_mctp *priv = i3cdev_get_drvdata(i3c);
	struct i3c_mctp_client *client;

	client = i3c_mctp_client_alloc(priv);
	if (IS_ERR(client))
		return client;

	spin_lock(&priv->clients_lock);
	priv->peci_client = client;
	spin_unlock(&priv->clients_lock);

	return priv->peci_client;
}
EXPORT_SYMBOL_GPL(i3c_mctp_add_peci_client);

/**
 * i3c_mctp_remove_peci_client() - un-registers PECI client
 * @client: i3c_mctp_client to be freed
 */
void i3c_mctp_remove_peci_client(struct i3c_mctp_client *client)
{
	struct i3c_mctp *priv = client->priv;

	spin_lock(&priv->clients_lock);
	priv->peci_client = NULL;
	spin_unlock(&priv->clients_lock);

	i3c_mctp_client_put(client);
}
EXPORT_SYMBOL_GPL(i3c_mctp_remove_peci_client);

static struct i3c_mctp *i3c_mctp_alloc(struct i3c_device *i3c)
{
	struct i3c_mctp *priv;
	int id;

	priv = devm_kzalloc(i3cdev_to_dev(i3c), sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	id = ida_alloc(&i3c_mctp_ida, GFP_KERNEL);
	if (id < 0) {
		pr_err("i3c_mctp: no minor number available!\n");
		return ERR_PTR(id);
	}

	priv->id = id;
	priv->i3c = i3c;
	priv->eid = 0;
	priv->process_count = 0;

	INIT_LIST_HEAD(&priv->endpoints);
	mutex_init(&priv->endpoints_lock);

	spin_lock_init(&priv->clients_lock);
	spin_lock_init(&priv->file_lock);

	return priv;
}

static void i3c_mctp_ibi_handler(struct i3c_device *dev, const struct i3c_ibi_payload *payload)
{
	struct i3c_mctp *priv = i3cdev_get_drvdata(dev);
	struct i3c_mctp_packet *rx_packet;

	rx_packet = i3c_mctp_read_packet(dev);
	if (IS_ERR(rx_packet))
		return;

	i3c_mctp_dispatch_packet(priv, rx_packet);
}

static int i3c_mctp_init(struct i3c_driver *drv)
{
	int ret;

	packet_cache = kmem_cache_create_usercopy("mctp-i3c-packet",
						  sizeof(struct i3c_mctp_packet), 0, 0, 0,
						  sizeof(struct i3c_mctp_packet), NULL);
	if (IS_ERR(packet_cache)) {
		ret = PTR_ERR(packet_cache);
		goto out;
	}

	/* Dynamically request unused major number */
	ret = alloc_chrdev_region(&i3c_mctp_devt, 0, I3C_MCTP_MINORS, "i3c-mctp");
	if (ret)
		goto out;

	/* Create a class to populate sysfs entries*/
	i3c_mctp_class = class_create(THIS_MODULE, "i3c-mctp");
	if (IS_ERR(i3c_mctp_class)) {
		ret = PTR_ERR(i3c_mctp_class);
		goto out_unreg_chrdev;
	}

	i3c_driver_register(drv);

	return 0;

out_unreg_chrdev:
	unregister_chrdev_region(i3c_mctp_devt, I3C_MCTP_MINORS);
out:
	pr_err("i3c_mctp: driver initialisation failed\n");
	return ret;
}

static void i3c_mctp_free(struct i3c_driver *drv)
{
	i3c_driver_unregister(drv);
	class_destroy(i3c_mctp_class);
	unregister_chrdev_region(i3c_mctp_devt, I3C_MCTP_MINORS);
	kmem_cache_destroy(packet_cache);
}

static int i3c_mctp_enable_ibi(struct i3c_device *i3cdev)
{
	struct i3c_mctp *priv = i3cdev_get_drvdata(i3cdev);
	struct i3c_ibi_setup ibireq = {
		.handler = i3c_mctp_ibi_handler,
		.max_payload_len = 2,
		.num_slots = 10,
	};
	int ret = 0;

	if (priv->ibi_enabled)
		return ret;
	ret = i3c_device_request_ibi(i3cdev, &ibireq);
	if (ret)
		return ret;
	ret = i3c_device_enable_ibi(i3cdev);
	if (ret)
		i3c_device_free_ibi(i3cdev);
	else
		priv->ibi_enabled = true;

	return ret;
}

static void i3c_mctp_disable_ibi(struct i3c_device *i3cdev)
{
	struct i3c_mctp *priv = i3cdev_get_drvdata(i3cdev);
	int ret;

	ret = i3c_device_disable_ibi(i3cdev);
	if (!ret) {
		i3c_device_free_ibi(i3cdev);
	} else {
		dev_warn(i3cdev_to_dev(i3cdev), "Failed to disable IBI, ret = %d", ret);
		return;
	}

	priv->ibi_enabled = false;
}

/**
 * i3c_mctp_get_eid() - receive MCTP EID assigned to the device
 *
 * @client: client for the device to get the EID for
 * @domain_id: requested domain ID
 * @eid: pointer to store EID value
 *
 * Receive MCTP endpoint ID dynamically assigned by the MCTP Bus Owner
 * Return: 0 in case of success, a negative error code otherwise.
 */
int i3c_mctp_get_eid(struct i3c_mctp_client *client, u8 domain_id, u8 *eid)
{
	struct i3c_mctp_endpoint *endpoint;
	struct i3c_device_info info;
	int ret = -ENOENT;

	i3c_device_get_info(client->priv->i3c, &info);

	mutex_lock(&client->priv->endpoints_lock);

	list_for_each_entry(endpoint, &client->priv->endpoints, link) {
		if (endpoint->eid_info.domain_id == domain_id &&
		    endpoint->eid_info.dyn_addr == info.dyn_addr) {
			*eid = endpoint->eid_info.eid;
			ret = 0;
			break;
		}
	}

	mutex_unlock(&client->priv->endpoints_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(i3c_mctp_get_eid);

/**
 * i3c_mctp_send_packet() - send mctp packet
 *
 * @tx_packet: the allocated packet that needs to be send via I3C
 * @i3c: i3c device to send the packet to
 *
 * Return: 0 in case of success, a negative error code otherwise.
 */
int i3c_mctp_send_packet(struct i3c_device *i3c, struct i3c_mctp_packet *tx_packet)
{
	struct i3c_mctp *priv = i3cdev_get_drvdata(i3c);
	u8 *protocol_hdr = (u8 *)tx_packet->data.protocol_hdr;
	struct i3c_priv_xfer xfers = {
		.rnw = false,
		.len = tx_packet->size,
		.data.out = &tx_packet->data,
	};

	/*
	 * Check against packet size + PEC byte
	 * to not send more data than it was set in the probe
	 */
	if (priv->max_write_len < xfers.len + 1) {
		dev_dbg(i3cdev_to_dev(i3c), "Length mismatch. MWL = %d, xfers.len = %d",
			priv->max_write_len, xfers.len);
		return -EINVAL;
	}

	spin_lock(&priv->clients_lock);
	if (i3c_mctp_find_client(priv, tx_packet) == priv->peci_client)
		protocol_hdr[MCTP_HDR_SRC_EID_OFFSET] = priv->eid;
	spin_unlock(&priv->clients_lock);

	return i3c_device_do_priv_xfers(i3c, &xfers, 1);
}
EXPORT_SYMBOL_GPL(i3c_mctp_send_packet);

/**
 * i3c_mctp_receive_packet() - receive mctp packet
 *
 * @client: i3c_mctp_client to receive the packet from
 * @timeout: timeout, in jiffies
 *
 * The function will sleep for up to @timeout if no packet is ready to read.
 *
 * Returns struct i3c_mctp_packet from or ERR_PTR in case of error or the
 * timeout elapsed.
 */
struct i3c_mctp_packet *i3c_mctp_receive_packet(struct i3c_mctp_client *client,
						unsigned long timeout)
{
	struct i3c_mctp_packet *rx_packet;
	int ret;

	ret = wait_event_interruptible_timeout(client->wait_queue,
					       __ptr_ring_peek(&client->rx_queue), timeout);
	if (ret < 0)
		return ERR_PTR(ret);
	else if (ret == 0)
		return ERR_PTR(-ETIME);

	rx_packet = ptr_ring_consume(&client->rx_queue);
	if (!rx_packet)
		return ERR_PTR(-EAGAIN);

	return rx_packet;
}
EXPORT_SYMBOL_GPL(i3c_mctp_receive_packet);

static void i3c_mctp_i3c_event_cb(struct i3c_device *dev, enum i3c_event event)
{
	struct i3c_mctp *priv = i3cdev_get_drvdata(dev);
	struct i3c_device_info info;

	switch (event) {
	case i3c_event_prepare_for_rescan:
		/*
		 * Disable IBI and polling mode blindly.
		 */
		i3c_mctp_disable_ibi(dev);
		cancel_delayed_work(&priv->polling_work);
		break;
	case i3c_event_rescan_done:
		i3c_device_get_info(dev, &info);
		if (i3c_mctp_enable_ibi(dev)) {
			if (!delayed_work_pending(&priv->polling_work)) {
				INIT_DELAYED_WORK(&priv->polling_work, i3c_mctp_polling_work);
				schedule_delayed_work(&priv->polling_work,
						      msecs_to_jiffies(POLLING_TIMEOUT_MS));
			}
		}
		break;
	default:
		break;
	}
}

static int i3c_mctp_probe(struct i3c_device *i3cdev)
{
	struct device *dev = i3cdev_to_dev(i3cdev);
	struct i3c_device_info info;
	struct i3c_mctp *priv;
	int ret;

	priv = i3c_mctp_alloc(i3cdev);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	cdev_init(&priv->cdev, &i3c_mctp_fops);

	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, MKDEV(MAJOR(i3c_mctp_devt), priv->id), 1);
	if (ret)
		goto error_cdev;

	/* register this i3c device with the driver core */
	priv->dev = device_create(i3c_mctp_class, dev,
				  MKDEV(MAJOR(i3c_mctp_devt), priv->id),
				  NULL, "i3c-mctp-%d", priv->id);
	if (IS_ERR(priv->dev)) {
		ret = PTR_ERR(priv->dev);
		goto error;
	}

	ret = i3c_device_control_pec(i3cdev, true);
	if (ret)
		goto error;

	i3cdev_set_drvdata(i3cdev, priv);

	i3c_device_get_info(i3cdev, &info);

	if (!IS_BMC_NON_LEGACY(info.pid))
		priv->i3c_peci = platform_device_register_data(i3cdev_to_dev(i3cdev), "peci-i3c",
							       priv->id, NULL, 0);
	if (IS_ERR(priv->i3c_peci))
		dev_warn(priv->dev, "failed to register peci-i3c device\n");

	if (info.max_read_len < I3C_MCTP_MIN_TRANSFER_SIZE)
		ret = i3c_device_setmrl_ccc(i3cdev, &info, cpu_to_be16(I3C_MCTP_MIN_TRANSFER_SIZE),
					    I3C_MCTP_IBI_PAYLOAD_SIZE);
	if (ret && info.max_read_len < I3C_MCTP_MIN_TRANSFER_SIZE) {
		dev_info(dev, "Failed to set MRL, ret = %d, running with default: %d\n", ret,
			 I3C_MCTP_MIN_TRANSFER_SIZE);
		info.max_read_len = I3C_MCTP_MIN_TRANSFER_SIZE;
	}
	priv->max_read_len = info.max_read_len;

	if (info.max_write_len < I3C_MCTP_MIN_TRANSFER_SIZE)
		ret = i3c_device_setmwl_ccc(i3cdev, &info, cpu_to_be16(I3C_MCTP_MIN_TRANSFER_SIZE));
	if (ret && info.max_write_len < I3C_MCTP_MIN_TRANSFER_SIZE) {
		dev_info(dev, "Failed to set MWL, ret = %d, running with default: %d\n", ret,
			 I3C_MCTP_MIN_TRANSFER_SIZE);
		info.max_write_len = I3C_MCTP_MIN_TRANSFER_SIZE;
	}
	priv->max_write_len = info.max_write_len;

	if (i3c_mctp_enable_ibi(i3cdev)) {
		INIT_DELAYED_WORK(&priv->polling_work, i3c_mctp_polling_work);
		schedule_delayed_work(&priv->polling_work, msecs_to_jiffies(POLLING_TIMEOUT_MS));
	}
	i3c_device_register_event_cb(i3cdev, i3c_mctp_i3c_event_cb);

	return 0;

error:
	cdev_del(&priv->cdev);
error_cdev:
	put_device(dev);
	return ret;
}

static void i3c_mctp_remove(struct i3c_device *i3cdev)
{
	struct i3c_mctp *priv = i3cdev_get_drvdata(i3cdev);
	int i;

	if (priv->default_client)
		priv->default_client->priv = NULL;

	spin_lock(&priv->file_lock);
	for (i = 0; i < priv->process_count; i++) {
		kobject_put(&priv->cdev.kobj);
		module_put(priv->cdev.owner);
	}
	spin_unlock(&priv->file_lock);

	cancel_delayed_work(&priv->polling_work);
	platform_device_unregister(priv->i3c_peci);

	device_destroy(i3c_mctp_class, MKDEV(MAJOR(i3c_mctp_devt), priv->id));
	cdev_del(&priv->cdev);
	i3c_mctp_eid_info_list_remove(&priv->endpoints);
	ida_free(&i3c_mctp_ida, priv->id);
}

static const struct i3c_device_id i3c_mctp_ids[] = {
	I3C_CLASS(0xCC, 0x0),
	{ },
};

static struct i3c_driver i3c_mctp_drv = {
	.driver.name = "i3c-mctp",
	.id_table = i3c_mctp_ids,
	.probe = i3c_mctp_probe,
	.remove = i3c_mctp_remove,
};

module_driver(i3c_mctp_drv, i3c_mctp_init, i3c_mctp_free);
MODULE_AUTHOR("Oleksandr Shulzhenko <oleksandr.shulzhenko.viktorovych@intel.com>");
MODULE_DESCRIPTION("I3C MCTP driver");
MODULE_LICENSE("GPL");
