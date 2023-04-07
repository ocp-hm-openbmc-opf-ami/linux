// SPDX-License-Identifier: GPL-2.0-only
/*
 *  Aspeed 24XX/25XX I2C Controller.
 *
 *  Copyright (C) 2012-2017 ASPEED Technology Inc.
 *  Copyright 2017 IBM Corporation
 *  Copyright 2017 Google, Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>

/* I2C Global Registers */
/* 0x00 : I2CG Interrupt Status Register  */
/* 0x08 : I2CG Interrupt Target Assignment  */
/* 0x0c : I2CG Global Control Register (AST2500)  */
#define ASPEED_I2CG_GLOBAL_CTRL_REG			0x0c
#define  ASPEED_I2CG_SRAM_BUFFER_EN			BIT(0)

/* I2C Bus Registers */
#define ASPEED_I2C_FUN_CTRL_REG				0x00
#define ASPEED_I2C_AC_TIMING_REG1			0x04
#define ASPEED_I2C_AC_TIMING_REG2			0x08
#define ASPEED_I2C_INTR_CTRL_REG			0x0c
#define ASPEED_I2C_INTR_STS_REG				0x10
#define ASPEED_I2C_CMD_REG				0x14
#define ASPEED_I2C_DEV_ADDR_REG				0x18
#define ASPEED_I2C_BUF_CTRL_REG				0x1c
#define ASPEED_I2C_BYTE_BUF_REG				0x20
#define ASPEED_I2C_DMA_ADDR_REG				0x24
#define ASPEED_I2C_DMA_LEN_REG				0x28

/* Device Register Definition */
/* 0x00 : I2CD Function Control Register  */
#define ASPEED_I2CD_BUFFER_PAGE_SEL_MASK		GENMASK(22, 20)
#define ASPEED_I2CD_BUS_AUTO_RECOVERY_EN		BIT(17)
#define ASPEED_I2CD_MULTI_MASTER_DIS			BIT(15)
#define ASPEED_I2CD_SDA_DRIVE_1T_EN			BIT(8)
#define ASPEED_I2CD_M_SDA_DRIVE_1T_EN			BIT(7)
#define ASPEED_I2CD_M_HIGH_SPEED_EN			BIT(6)
#define ASPEED_I2CD_GCALL_EN				BIT(2)
#define ASPEED_I2CD_SLAVE_EN				BIT(1)
#define ASPEED_I2CD_MASTER_EN				BIT(0)

/* 0x04 : I2CD Clock and AC Timing Control Register #1 */
#define ASPEED_I2CD_TIME_TBUF_MASK			GENMASK(31, 28)
#define ASPEED_I2CD_TIME_THDSTA_MASK			GENMASK(27, 24)
#define ASPEED_I2CD_TIME_TACST_MASK			GENMASK(23, 20)
#define ASPEED_I2CD_TIME_SCL_HIGH_SHIFT			16
#define ASPEED_I2CD_TIME_SCL_HIGH_MASK			GENMASK(19, 16)
#define ASPEED_I2CD_TIME_SCL_LOW_SHIFT			12
#define ASPEED_I2CD_TIME_SCL_LOW_MASK			GENMASK(15, 12)
#define ASPEED_I2CD_TIME_TIMEOUT_BASE_DIVISOR_SHIFT	8
#define ASPEED_I2CD_TIME_TIMEOUT_BASE_DIVISOR_MASK	GENMASK(9, 8)
#define ASPEED_I2CD_TIME_BASE_DIVISOR_MASK		GENMASK(3, 0)
#define ASPEED_I2CD_TIME_SCL_REG_MAX			GENMASK(3, 0)

/* 0x08 : I2CD Clock and AC Timing Control Register #2 */
#define ASPEED_I2CD_TIMEOUT_CYCLES_SHIFT		0
#define ASPEED_I2CD_TIMEOUT_CYCLES_MASK			GENMASK(4, 0)

/* 0x0c : I2CD Interrupt Control Register &
 * 0x10 : I2CD Interrupt Status Register
 *
 * These share bit definitions, so use the same values for the enable &
 * status bits.
 */
#define ASPEED_I2CD_INTR_RECV_MASK			0xf000ffff
#if defined(CONFIG_MACH_ASPEED_G6)
#define ASPEED_I2CD_INTR_SLAVE_ADDR_RECEIVED_PENDING	BIT(29)
#else
#define ASPEED_I2CD_INTR_SLAVE_ADDR_RECEIVED_PENDING	BIT(30)
#endif
#define ASPEED_I2CD_INTR_SLAVE_INACTIVE_TIMEOUT		BIT(15)
#define ASPEED_I2CD_INTR_SDA_DL_TIMEOUT			BIT(14)
#define ASPEED_I2CD_INTR_BUS_RECOVER_DONE		BIT(13)
#define ASPEED_I2CD_INTR_GCALL_ADDR			BIT(8)
#define ASPEED_I2CD_INTR_SLAVE_MATCH			BIT(7)
#define ASPEED_I2CD_INTR_SCL_TIMEOUT			BIT(6)
#define ASPEED_I2CD_INTR_ABNORMAL			BIT(5)
#define ASPEED_I2CD_INTR_NORMAL_STOP			BIT(4)
#define ASPEED_I2CD_INTR_ARBIT_LOSS			BIT(3)
#define ASPEED_I2CD_INTR_RX_DONE			BIT(2)
#define ASPEED_I2CD_INTR_TX_NAK				BIT(1)
#define ASPEED_I2CD_INTR_TX_ACK				BIT(0)
#define ASPEED_I2CD_INTR_MASTER_ERRORS					       \
		(ASPEED_I2CD_INTR_SDA_DL_TIMEOUT |			       \
		 ASPEED_I2CD_INTR_SCL_TIMEOUT |				       \
		 ASPEED_I2CD_INTR_ABNORMAL |				       \
		 ASPEED_I2CD_INTR_ARBIT_LOSS)
#define ASPEED_I2CD_INTR_SLAVE_ERRORS					       \
		ASPEED_I2CD_INTR_SLAVE_INACTIVE_TIMEOUT
#define ASPEED_I2CD_INTR_ALL						       \
		(ASPEED_I2CD_INTR_SLAVE_INACTIVE_TIMEOUT |		       \
		 ASPEED_I2CD_INTR_SDA_DL_TIMEOUT |			       \
		 ASPEED_I2CD_INTR_BUS_RECOVER_DONE |			       \
		 ASPEED_I2CD_INTR_SCL_TIMEOUT |				       \
		 ASPEED_I2CD_INTR_ABNORMAL |				       \
		 ASPEED_I2CD_INTR_NORMAL_STOP |				       \
		 ASPEED_I2CD_INTR_ARBIT_LOSS |				       \
		 ASPEED_I2CD_INTR_RX_DONE |				       \
		 ASPEED_I2CD_INTR_TX_NAK |				       \
		 ASPEED_I2CD_INTR_TX_ACK)
#define ASPEED_I2CD_INTR_STATUS_MASK					       \
		(ASPEED_I2CD_INTR_SLAVE_ADDR_RECEIVED_PENDING |		       \
		 ASPEED_I2CD_INTR_GCALL_ADDR |				       \
		 ASPEED_I2CD_INTR_SLAVE_MATCH |				       \
		 ASPEED_I2CD_INTR_ALL)

/* 0x14 : I2CD Command/Status Register   */
#define ASPEED_I2CD_SCL_LINE_STS			BIT(18)
#define ASPEED_I2CD_SDA_LINE_STS			BIT(17)
#define ASPEED_I2CD_BUS_BUSY_STS			BIT(16)
#define ASPEED_I2CD_BUS_RECOVER_CMD			BIT(11)

/* Command Bit */
#define ASPEED_I2CD_RX_DMA_ENABLE			BIT(9)
#define ASPEED_I2CD_TX_DMA_ENABLE			BIT(8)
#define ASPEED_I2CD_RX_BUFF_ENABLE			BIT(7)
#define ASPEED_I2CD_TX_BUFF_ENABLE			BIT(6)
#define ASPEED_I2CD_M_STOP_CMD				BIT(5)
#define ASPEED_I2CD_M_S_RX_CMD_LAST			BIT(4)
#define ASPEED_I2CD_M_RX_CMD				BIT(3)
#define ASPEED_I2CD_S_TX_CMD				BIT(2)
#define ASPEED_I2CD_M_TX_CMD				BIT(1)
#define ASPEED_I2CD_M_START_CMD				BIT(0)
#define ASPEED_I2CD_MASTER_CMDS_MASK					       \
		(ASPEED_I2CD_M_STOP_CMD |				       \
		 ASPEED_I2CD_M_S_RX_CMD_LAST |				       \
		 ASPEED_I2CD_M_RX_CMD |					       \
		 ASPEED_I2CD_M_TX_CMD |					       \
		 ASPEED_I2CD_M_START_CMD)

/* 0x18 : I2CD Slave Device Address Register   */
#define ASPEED_I2CD_DEV_ADDR_MASK			GENMASK(6, 0)

/* 0x1c : I2CD Buffer Control Register */
/* Use 8-bits or 6-bits wide bit fileds to support both AST2400 and AST2500 */
#define ASPEED_I2CD_BUF_RX_COUNT_MASK			GENMASK(31, 24)
#define ASPEED_I2CD_BUF_RX_SIZE_MASK			GENMASK(23, 16)
#define ASPEED_I2CD_BUF_TX_COUNT_MASK			GENMASK(15, 8)
#define ASPEED_I2CD_BUF_OFFSET_MASK			GENMASK(5, 0)

/* 0x24 : I2CD DMA Mode Buffer Address Register */
#define ASPEED_I2CD_DMA_ADDR_MASK			GENMASK(31, 2)
#define ASPEED_I2CD_DMA_ALIGN				4

/* 0x28 : I2CD DMA Transfer Length Register */
#define ASPEED_I2CD_DMA_LEN_SHIFT			0
#define ASPEED_I2CD_DMA_LEN_MASK			GENMASK(11, 0)

enum aspeed_i2c_master_state {
	ASPEED_I2C_MASTER_INACTIVE,
	ASPEED_I2C_MASTER_PENDING,
	ASPEED_I2C_MASTER_START,
	ASPEED_I2C_MASTER_TX_FIRST,
	ASPEED_I2C_MASTER_TX,
	ASPEED_I2C_MASTER_RX_FIRST,
	ASPEED_I2C_MASTER_RX,
	ASPEED_I2C_MASTER_STOP,
};

enum aspeed_i2c_slave_state {
	ASPEED_I2C_SLAVE_INACTIVE,
	ASPEED_I2C_SLAVE_START,
	ASPEED_I2C_SLAVE_READ_REQUESTED,
	ASPEED_I2C_SLAVE_READ_PROCESSED,
	ASPEED_I2C_SLAVE_WRITE_REQUESTED,
	ASPEED_I2C_SLAVE_WRITE_RECEIVED,
	ASPEED_I2C_SLAVE_GCALL_START,
	ASPEED_I2C_SLAVE_GCALL_REQUESTED,
	ASPEED_I2C_SLAVE_STOP,
};

struct aspeed_i2c_bus {
	struct i2c_adapter		adap;
	struct device			*dev;
	void __iomem			*base;
	struct reset_control		*rst;
	/* Synchronizes I/O mem access to base. */
	spinlock_t			lock;
	struct completion		cmd_complete;
	u32				(*get_clk_reg_val)(struct device *dev,
							   u32 divisor);
	unsigned long			parent_clk_frequency;
	u32				bus_frequency;
	u32				hw_timeout_ms;
	/* Transaction state. */
	enum aspeed_i2c_master_state	master_state;
	struct i2c_msg			*msgs;
	size_t				buf_index;
	size_t				msgs_index;
	size_t				msgs_count;
	bool				send_stop;
	int				cmd_err;
	/* Protected only by i2c_lock_bus */
	int				master_xfer_result;
	/* Multi-master */
	bool				multi_master;
	/* Buffer mode */
	void __iomem			*buf_base;
	u8				buf_offset;
	u8				buf_page;
	/* DMA mode */
	struct dma_pool			*dma_pool;
	dma_addr_t			dma_handle;
	u8				*dma_buf;
	size_t				dma_len;
	/* Buffer/DMA mode */
	size_t				buf_size;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct i2c_client		*slave;
	enum aspeed_i2c_slave_state	slave_state;
	/* General call */
	bool				general_call;
#endif /* CONFIG_I2C_SLAVE */
};

static bool dump_debug __read_mostly;
static int dump_debug_bus_id __read_mostly;

#define I2C_HEX_DUMP(bus, addr, flags, buf, len) \
	do { \
		if (dump_debug && (bus)->adap.nr == dump_debug_bus_id) { \
			char dump_info[100] = {0,}; \
			char task_info[TASK_COMM_LEN]; \
			get_task_comm(task_info, current); \
			snprintf(dump_info, sizeof(dump_info), \
				 "bus_id:%d, addr:0x%02x, flags:0x%02x, task:%s(%d): ", \
				 (bus)->adap.nr, addr, flags, task_info, \
				 task_pid_nr(current)); \
			print_hex_dump(KERN_ERR, dump_info, DUMP_PREFIX_NONE, \
				       16, 1, buf, len, true); \
		} \
	} while (0)

static int aspeed_i2c_reset(struct aspeed_i2c_bus *bus);

static int aspeed_i2c_recover_bus(struct aspeed_i2c_bus *bus)
{
	unsigned long time_left, flags;
	int ret = 0;
	u32 command;

	spin_lock_irqsave(&bus->lock, flags);
	command = readl(bus->base + ASPEED_I2C_CMD_REG);

	if (command & ASPEED_I2CD_SDA_LINE_STS) {
		/* Bus is idle: no recovery needed. */
		if (command & ASPEED_I2CD_SCL_LINE_STS)
			goto out;
		dev_dbg(bus->dev, "SCL hung (state %x), attempting recovery\n",
			command);

		reinit_completion(&bus->cmd_complete);
		writel(ASPEED_I2CD_M_STOP_CMD, bus->base + ASPEED_I2C_CMD_REG);
		spin_unlock_irqrestore(&bus->lock, flags);

		time_left = wait_for_completion_timeout(
				&bus->cmd_complete, bus->adap.timeout);

		spin_lock_irqsave(&bus->lock, flags);
		if (time_left == 0)
			goto reset_out;
		else if (bus->cmd_err)
			goto reset_out;
		/* Recovery failed. */
		else if (!(readl(bus->base + ASPEED_I2C_CMD_REG) &
			   ASPEED_I2CD_SCL_LINE_STS))
			goto reset_out;
	/* Bus error. */
	} else {
		dev_dbg(bus->dev, "SDA hung (state %x), attempting recovery\n",
			command);

		reinit_completion(&bus->cmd_complete);
		/* Writes 1 to 8 SCL clock cycles until SDA is released. */
		writel(ASPEED_I2CD_BUS_RECOVER_CMD,
		       bus->base + ASPEED_I2C_CMD_REG);
		spin_unlock_irqrestore(&bus->lock, flags);

		time_left = wait_for_completion_timeout(
				&bus->cmd_complete, bus->adap.timeout);

		spin_lock_irqsave(&bus->lock, flags);
		if (time_left == 0)
			goto reset_out;
		else if (bus->cmd_err)
			goto reset_out;
		/* Recovery failed. */
		else if (!(readl(bus->base + ASPEED_I2C_CMD_REG) &
			   ASPEED_I2CD_SDA_LINE_STS))
			goto reset_out;
	}

out:
	spin_unlock_irqrestore(&bus->lock, flags);

	return ret;

reset_out:
	spin_unlock_irqrestore(&bus->lock, flags);

	return aspeed_i2c_reset(bus);
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static int aspeed_i2c_check_slave_error(u32 irq_status)
{
	if (irq_status & ASPEED_I2CD_INTR_SLAVE_INACTIVE_TIMEOUT)
		return -EIO;

	return 0;
}

static inline void
aspeed_i2c_slave_handle_rx_done(struct aspeed_i2c_bus *bus, u32 irq_status,
				u8 *value)
{
	if (bus->dma_buf &&
	    bus->slave_state == ASPEED_I2C_SLAVE_WRITE_RECEIVED &&
	    !(irq_status & ASPEED_I2CD_INTR_NORMAL_STOP))
		*value = bus->dma_buf[0];
	else if (bus->buf_base &&
		 bus->slave_state == ASPEED_I2C_SLAVE_WRITE_RECEIVED &&
		 !(irq_status & ASPEED_I2CD_INTR_NORMAL_STOP))
		*value = readb(bus->buf_base);
	else
		*value = readl(bus->base + ASPEED_I2C_BYTE_BUF_REG) >> 8;
}

static inline void
aspeed_i2c_slave_handle_normal_stop(struct aspeed_i2c_bus *bus, u32 irq_status,
				    u8 *value)
{
	int i, len;

	if (bus->slave_state == ASPEED_I2C_SLAVE_WRITE_RECEIVED &&
	    irq_status & ASPEED_I2CD_INTR_RX_DONE) {
		if (bus->dma_buf) {
			len = bus->buf_size -
			      FIELD_GET(ASPEED_I2CD_DMA_LEN_MASK,
					readl(bus->base +
					      ASPEED_I2C_DMA_LEN_REG));
			for (i = 0; i < len; i++) {
				*value = bus->dma_buf[i];
				i2c_slave_event(bus->slave,
						I2C_SLAVE_WRITE_RECEIVED,
						value);
			}
		} else if (bus->buf_base) {
			len = FIELD_GET(ASPEED_I2CD_BUF_RX_COUNT_MASK,
					readl(bus->base +
					      ASPEED_I2C_BUF_CTRL_REG));
			for (i = 0; i < len; i++) {
				*value = readb(bus->buf_base + i);
				i2c_slave_event(bus->slave,
						I2C_SLAVE_WRITE_RECEIVED,
						value);
			}
		}
	}
}

static inline void
aspeed_i2c_slave_handle_write_requested(struct aspeed_i2c_bus *bus, u8 *value)
{
	if (bus->dma_buf) {
		writel(bus->dma_handle & ASPEED_I2CD_DMA_ADDR_MASK,
		       bus->base + ASPEED_I2C_DMA_ADDR_REG);
		writel(FIELD_PREP(ASPEED_I2CD_DMA_LEN_MASK, bus->buf_size),
		       bus->base + ASPEED_I2C_DMA_LEN_REG);
		writel(ASPEED_I2CD_RX_DMA_ENABLE,
		       bus->base + ASPEED_I2C_CMD_REG);
	} else if (bus->buf_base) {
		writel(FIELD_PREP(ASPEED_I2CD_BUF_RX_SIZE_MASK,
				  bus->buf_size - 1) |
		       FIELD_PREP(ASPEED_I2CD_BUF_OFFSET_MASK,
				  bus->buf_offset),
		       bus->base + ASPEED_I2C_BUF_CTRL_REG);
		writel(ASPEED_I2CD_RX_BUFF_ENABLE,
		       bus->base + ASPEED_I2C_CMD_REG);
	}
}

static inline void
aspeed_i2c_slave_handle_write_received(struct aspeed_i2c_bus *bus, u8 *value)
{
	int i, len;

	if (bus->dma_buf) {
		len = bus->buf_size -
		      FIELD_GET(ASPEED_I2CD_DMA_LEN_MASK,
				readl(bus->base +
				      ASPEED_I2C_DMA_LEN_REG));
		for (i = 1; i < len; i++) {
			*value = bus->dma_buf[i];
			i2c_slave_event(bus->slave, I2C_SLAVE_WRITE_RECEIVED,
					value);
		}
		writel(bus->dma_handle & ASPEED_I2CD_DMA_ADDR_MASK,
		       bus->base + ASPEED_I2C_DMA_ADDR_REG);
		writel(FIELD_PREP(ASPEED_I2CD_DMA_LEN_MASK, bus->buf_size),
		       bus->base + ASPEED_I2C_DMA_LEN_REG);
		writel(ASPEED_I2CD_RX_DMA_ENABLE,
		       bus->base + ASPEED_I2C_CMD_REG);
	} else if (bus->buf_base) {
		len = FIELD_GET(ASPEED_I2CD_BUF_RX_COUNT_MASK,
				readl(bus->base +
				      ASPEED_I2C_BUF_CTRL_REG));
		for (i = 1; i < len; i++) {
			*value = readb(bus->buf_base + i);
			i2c_slave_event(bus->slave, I2C_SLAVE_WRITE_RECEIVED,
					value);
		}
		writel(FIELD_PREP(ASPEED_I2CD_BUF_RX_SIZE_MASK,
				  bus->buf_size - 1) |
		       FIELD_PREP(ASPEED_I2CD_BUF_OFFSET_MASK, bus->buf_offset),
		       bus->base + ASPEED_I2C_BUF_CTRL_REG);
		writel(ASPEED_I2CD_RX_BUFF_ENABLE,
		       bus->base + ASPEED_I2C_CMD_REG);
	}
}

static u32 aspeed_i2c_slave_irq(struct aspeed_i2c_bus *bus, u32 irq_status)
{
	u32 command, irq_handled = 0;
	struct i2c_client *slave = bus->slave;
	u8 value;
	int ret;

	if (!slave)
		return 0;

	if (aspeed_i2c_check_slave_error(irq_status)) {
		dev_dbg(bus->dev, "received slave error interrupt: 0x%08x\n",
			irq_status);
		irq_handled |= (irq_status & ASPEED_I2CD_INTR_SLAVE_ERRORS);
		bus->slave_state = ASPEED_I2C_SLAVE_INACTIVE;
		return irq_handled;
	}

	command = readl(bus->base + ASPEED_I2C_CMD_REG);

	/* Slave was requested, restart state machine. */
	if (irq_status & ASPEED_I2CD_INTR_SLAVE_MATCH) {
		irq_handled |= ASPEED_I2CD_INTR_SLAVE_MATCH;
		bus->slave_state = ASPEED_I2C_SLAVE_START;
	}

	/* General call was requested, restart state machine. */
	if (irq_status & ASPEED_I2CD_INTR_GCALL_ADDR) {
		irq_handled |= ASPEED_I2CD_INTR_GCALL_ADDR;
		bus->slave_state = ASPEED_I2C_SLAVE_GCALL_START;
	}

	/* Slave is not currently active, irq was for someone else. */
	if (bus->slave_state == ASPEED_I2C_SLAVE_INACTIVE)
		return irq_handled;

	dev_dbg(bus->dev, "slave irq status 0x%08x, cmd 0x%08x\n",
		irq_status, command);

	/*
	 * If a peer master sends messages too quickly before it processes
	 * previous slave DMA data handling, this indicator will be set. It's
	 * just a indicator and driver can't recover this case so just ignore
	 * it.
	 */
	if (unlikely(irq_status &
		     ASPEED_I2CD_INTR_SLAVE_ADDR_RECEIVED_PENDING)) {
		dev_dbg(bus->dev, "A slave addr match interrupt is pending.\n");
		irq_handled |= ASPEED_I2CD_INTR_SLAVE_ADDR_RECEIVED_PENDING;
	}

	/* Slave was sent something. */
	if (irq_status & ASPEED_I2CD_INTR_RX_DONE) {
		aspeed_i2c_slave_handle_rx_done(bus, irq_status, &value);
		/* Handle address frame. */
		if (bus->slave_state == ASPEED_I2C_SLAVE_START) {
			if (value & 0x1)
				bus->slave_state =
						ASPEED_I2C_SLAVE_READ_REQUESTED;
			else
				bus->slave_state =
						ASPEED_I2C_SLAVE_WRITE_REQUESTED;
		} else if (bus->slave_state == ASPEED_I2C_SLAVE_GCALL_START) {
			/*
			 * I2C spec defines the second byte meaning like below.
			 * 0x06 : Reset and write programmable part of slave
			 *        address by hardware.
			 * 0x04 : Write programmable part of slave address by
			 *        hardware.
			 * 0x00 : No allowed.
			 *
			 * But in OpenBMC, we are going to use this
			 * 'General call' feature for IPMB message broadcasting
			 * so it delivers all data as is without any specific
			 * handling of the second byte.
			 */
			bus->slave_state = ASPEED_I2C_SLAVE_GCALL_REQUESTED;
		}
		irq_handled |= ASPEED_I2CD_INTR_RX_DONE;
	}

	/* Slave was asked to stop. */
	if (irq_status & ASPEED_I2CD_INTR_NORMAL_STOP) {
		aspeed_i2c_slave_handle_normal_stop(bus, irq_status, &value);
		irq_handled |= ASPEED_I2CD_INTR_NORMAL_STOP;
		bus->slave_state = ASPEED_I2C_SLAVE_STOP;
	}

	if (irq_status & ASPEED_I2CD_INTR_TX_NAK &&
	    bus->slave_state == ASPEED_I2C_SLAVE_READ_PROCESSED) {
		irq_handled |= ASPEED_I2CD_INTR_TX_NAK;
		bus->slave_state = ASPEED_I2C_SLAVE_STOP;
	}

	switch (bus->slave_state) {
	case ASPEED_I2C_SLAVE_READ_REQUESTED:
		if (unlikely(irq_status & ASPEED_I2CD_INTR_TX_ACK))
			dev_err(bus->dev, "Unexpected ACK on read request.\n");
		bus->slave_state = ASPEED_I2C_SLAVE_READ_PROCESSED;
		i2c_slave_event(slave, I2C_SLAVE_READ_REQUESTED, &value);
		writel(value, bus->base + ASPEED_I2C_BYTE_BUF_REG);
		writel(ASPEED_I2CD_S_TX_CMD, bus->base + ASPEED_I2C_CMD_REG);
		break;
	case ASPEED_I2C_SLAVE_READ_PROCESSED:
		if (unlikely(!(irq_status & ASPEED_I2CD_INTR_TX_ACK))) {
			dev_err(bus->dev,
				"Expected ACK after processed read.\n");
			break;
		}
		irq_handled |= ASPEED_I2CD_INTR_TX_ACK;
		i2c_slave_event(slave, I2C_SLAVE_READ_PROCESSED, &value);
		writel(value, bus->base + ASPEED_I2C_BYTE_BUF_REG);
		writel(ASPEED_I2CD_S_TX_CMD, bus->base + ASPEED_I2C_CMD_REG);
		break;
	case ASPEED_I2C_SLAVE_WRITE_REQUESTED:
		bus->slave_state = ASPEED_I2C_SLAVE_WRITE_RECEIVED;
		ret = i2c_slave_event(slave, I2C_SLAVE_WRITE_REQUESTED, &value);
		/*
		 * Slave ACK's on this address phase already but as the backend driver
		 * returns an errno, the bus driver should nack the next incoming byte.
		 */
		if (ret < 0)
			writel(ASPEED_I2CD_M_S_RX_CMD_LAST, bus->base + ASPEED_I2C_CMD_REG);
		aspeed_i2c_slave_handle_write_requested(bus, &value);
		break;
	case ASPEED_I2C_SLAVE_WRITE_RECEIVED:
		i2c_slave_event(slave, I2C_SLAVE_WRITE_RECEIVED, &value);
		aspeed_i2c_slave_handle_write_received(bus, &value);
		break;
	case ASPEED_I2C_SLAVE_GCALL_REQUESTED:
		bus->slave_state = ASPEED_I2C_SLAVE_WRITE_RECEIVED;
		i2c_slave_event(slave, I2C_SLAVE_GCALL_REQUESTED, &value);
		break;
	case ASPEED_I2C_SLAVE_STOP:
		i2c_slave_event(slave, I2C_SLAVE_STOP, &value);
		bus->slave_state = ASPEED_I2C_SLAVE_INACTIVE;
		break;
	case ASPEED_I2C_SLAVE_START:
	case ASPEED_I2C_SLAVE_GCALL_START:
		/* Slave was just started. Waiting for the next event. */;
		break;
	default:
		dev_err(bus->dev, "unknown slave_state: %d\n",
			bus->slave_state);
		bus->slave_state = ASPEED_I2C_SLAVE_INACTIVE;
		break;
	}

	return irq_handled;
}
#endif /* CONFIG_I2C_SLAVE */

static inline u32
aspeed_i2c_prepare_rx_buf(struct aspeed_i2c_bus *bus, struct i2c_msg *msg)
{
	u32 command = 0;
	int len;

	if (msg->len > bus->buf_size) {
		len = bus->buf_size;
	} else {
		len = msg->len;
		command |= ASPEED_I2CD_M_S_RX_CMD_LAST;
	}

	if (bus->dma_buf) {
		command |= ASPEED_I2CD_RX_DMA_ENABLE;

		writel(bus->dma_handle & ASPEED_I2CD_DMA_ADDR_MASK,
		       bus->base + ASPEED_I2C_DMA_ADDR_REG);
		writel(FIELD_PREP(ASPEED_I2CD_DMA_LEN_MASK, len),
		       bus->base + ASPEED_I2C_DMA_LEN_REG);
		bus->dma_len = len;
	} else {
		command |= ASPEED_I2CD_RX_BUFF_ENABLE;

		writel(FIELD_PREP(ASPEED_I2CD_BUF_RX_SIZE_MASK, len - 1) |
		       FIELD_PREP(ASPEED_I2CD_BUF_OFFSET_MASK, bus->buf_offset),
		       bus->base + ASPEED_I2C_BUF_CTRL_REG);
	}

	return command;
}

static inline u32
aspeed_i2c_prepare_tx_buf(struct aspeed_i2c_bus *bus, struct i2c_msg *msg)
{
	u8 slave_addr = i2c_8bit_addr_from_msg(msg);
	u32 command = 0;
	int len;

	if (msg->len + 1 > bus->buf_size)
		len = bus->buf_size;
	else
		len = msg->len + 1;

	if (bus->dma_buf) {
		command |= ASPEED_I2CD_TX_DMA_ENABLE;

		bus->dma_buf[0] = slave_addr;
		memcpy(bus->dma_buf + 1, msg->buf, len);

		writel(bus->dma_handle & ASPEED_I2CD_DMA_ADDR_MASK,
		       bus->base + ASPEED_I2C_DMA_ADDR_REG);
		writel(FIELD_PREP(ASPEED_I2CD_DMA_LEN_MASK, len),
		       bus->base + ASPEED_I2C_DMA_LEN_REG);
		bus->dma_len = len;
	} else {
		u8 wbuf[4];
		int i;

		command |= ASPEED_I2CD_TX_BUFF_ENABLE;

		/*
		 * Yeah, it looks bad but byte writing on remapped I2C SRAM
		 * causes corruption so use this way to make dword writings.
		 */
		wbuf[0] = slave_addr;
		for (i = 1; i < len; i++) {
			wbuf[i % 4] = msg->buf[i - 1];
			if (i % 4 == 3)
				writel(*(u32 *)wbuf, bus->buf_base + i - 3);
		}
		if (--i % 4 != 3)
			writel(*(u32 *)wbuf, bus->buf_base + i - (i % 4));

		writel(FIELD_PREP(ASPEED_I2CD_BUF_TX_COUNT_MASK, len - 1) |
		       FIELD_PREP(ASPEED_I2CD_BUF_OFFSET_MASK, bus->buf_offset),
		       bus->base + ASPEED_I2C_BUF_CTRL_REG);
	}

	bus->buf_index = len - 1;

	return command;
}

/* precondition: bus.lock has been acquired. */
static void aspeed_i2c_do_start(struct aspeed_i2c_bus *bus)
{
	u32 command = ASPEED_I2CD_M_START_CMD | ASPEED_I2CD_M_TX_CMD;
	struct i2c_msg *msg = &bus->msgs[bus->msgs_index];

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	/*
	 * If it's requested in the middle of a slave session, set the master
	 * state to 'pending' then H/W will continue handling this master
	 * command when the bus comes back to the idle state.
	 */
	if (bus->slave_state != ASPEED_I2C_SLAVE_INACTIVE) {
		bus->master_state = ASPEED_I2C_MASTER_PENDING;
		return;
	}
#endif /* CONFIG_I2C_SLAVE */

	bus->master_state = ASPEED_I2C_MASTER_START;
	bus->buf_index = 0;

	if (msg->flags & I2C_M_RD) {
		command |= ASPEED_I2CD_M_RX_CMD;
		if (!(msg->flags & I2C_M_RECV_LEN)) {
			if (msg->len && (bus->dma_buf || bus->buf_base))
				command |= aspeed_i2c_prepare_rx_buf(bus, msg);

			/* Need to let the hardware know to NACK after RX. */
			if (msg->len <= 1)
				command |= ASPEED_I2CD_M_S_RX_CMD_LAST;
		}
	} else if (msg->len && (bus->dma_buf || bus->buf_base)) {
		command |= aspeed_i2c_prepare_tx_buf(bus, msg);
	}

	if (!(command & (ASPEED_I2CD_TX_BUFF_ENABLE |
			 ASPEED_I2CD_TX_DMA_ENABLE)))
		writel(i2c_8bit_addr_from_msg(msg),
		       bus->base + ASPEED_I2C_BYTE_BUF_REG);
	writel(command, bus->base + ASPEED_I2C_CMD_REG);
}

/* precondition: bus.lock has been acquired. */
static void aspeed_i2c_do_stop(struct aspeed_i2c_bus *bus)
{
	bus->master_state = ASPEED_I2C_MASTER_STOP;
	writel(ASPEED_I2CD_M_STOP_CMD, bus->base + ASPEED_I2C_CMD_REG);
}

/* precondition: bus.lock has been acquired. */
static void aspeed_i2c_next_msg_or_stop(struct aspeed_i2c_bus *bus)
{
	if (bus->msgs_index + 1 < bus->msgs_count) {
		bus->msgs_index++;
		aspeed_i2c_do_start(bus);
	} else {
		aspeed_i2c_do_stop(bus);
	}
}

static int aspeed_i2c_check_master_error(u32 irq_status)
{
	if (irq_status & ASPEED_I2CD_INTR_ARBIT_LOSS)
		return -EAGAIN;
	if (irq_status & (ASPEED_I2CD_INTR_SDA_DL_TIMEOUT |
			  ASPEED_I2CD_INTR_SCL_TIMEOUT))
		return -EBUSY;
	if (irq_status & (ASPEED_I2CD_INTR_ABNORMAL))
		return -EPROTO;

	return 0;
}

static inline u32
aspeed_i2c_master_handle_tx_first(struct aspeed_i2c_bus *bus,
				  struct i2c_msg *msg)
{
	u32 command = 0;

	if (bus->dma_buf || bus->buf_base) {
		int len;

		if (msg->len - bus->buf_index > bus->buf_size)
			len = bus->buf_size;
		else
			len = msg->len - bus->buf_index;

		if (bus->dma_buf) {
			command |= ASPEED_I2CD_TX_DMA_ENABLE;

			memcpy(bus->dma_buf, msg->buf + bus->buf_index, len);


			writel(bus->dma_handle & ASPEED_I2CD_DMA_ADDR_MASK,
			       bus->base + ASPEED_I2C_DMA_ADDR_REG);
			writel(FIELD_PREP(ASPEED_I2CD_DMA_LEN_MASK, len),
			       bus->base + ASPEED_I2C_DMA_LEN_REG);
			bus->dma_len = len;
		} else {
			u8 wbuf[4];
			int i;

			command |= ASPEED_I2CD_TX_BUFF_ENABLE;

			if (msg->len - bus->buf_index > bus->buf_size)
				len = bus->buf_size;
			else
				len = msg->len - bus->buf_index;

			for (i = 0; i < len; i++) {
				wbuf[i % 4] = msg->buf[bus->buf_index + i];
				if (i % 4 == 3)
					writel(*(u32 *)wbuf,
					       bus->buf_base + i - 3);
			}
			if (--i % 4 != 3)
				writel(*(u32 *)wbuf,
				       bus->buf_base + i - (i % 4));

			writel(FIELD_PREP(ASPEED_I2CD_BUF_TX_COUNT_MASK,
					  len - 1) |
			       FIELD_PREP(ASPEED_I2CD_BUF_OFFSET_MASK,
					  bus->buf_offset),
			       bus->base + ASPEED_I2C_BUF_CTRL_REG);
		}

		bus->buf_index += len;
	} else {
		writel(msg->buf[bus->buf_index++],
		       bus->base + ASPEED_I2C_BYTE_BUF_REG);
	}

	return command;
}

static inline void
aspeed_i2c_master_handle_rx(struct aspeed_i2c_bus *bus, struct i2c_msg *msg)
{
	u8 recv_byte;
	int len;

	if (bus->dma_buf) {
		len = bus->dma_len -
		      FIELD_GET(ASPEED_I2CD_DMA_LEN_MASK,
				readl(bus->base + ASPEED_I2C_DMA_LEN_REG));

		memcpy(msg->buf + bus->buf_index, bus->dma_buf, len);
		bus->buf_index += len;
	} else if (bus->buf_base) {
		len = FIELD_GET(ASPEED_I2CD_BUF_RX_COUNT_MASK,
				readl(bus->base + ASPEED_I2C_BUF_CTRL_REG));
		memcpy_fromio(msg->buf + bus->buf_index, bus->buf_base, len);
		bus->buf_index += len;
	} else {
		recv_byte = readl(bus->base + ASPEED_I2C_BYTE_BUF_REG) >> 8;
		msg->buf[bus->buf_index++] = recv_byte;
	}
}

static inline u32
aspeed_i2c_master_handle_rx_next(struct aspeed_i2c_bus *bus,
				 struct i2c_msg *msg)
{
	u32 command = 0;

	if (bus->dma_buf || bus->buf_base) {
		int len;

		if (msg->len - bus->buf_index > bus->buf_size) {
			len = bus->buf_size;
		} else {
			len = msg->len - bus->buf_index;
			command |= ASPEED_I2CD_M_S_RX_CMD_LAST;
		}

		if (bus->dma_buf) {
			command |= ASPEED_I2CD_RX_DMA_ENABLE;

			writel(bus->dma_handle & ASPEED_I2CD_DMA_ADDR_MASK,
			       bus->base + ASPEED_I2C_DMA_ADDR_REG);
			writel(FIELD_PREP(ASPEED_I2CD_DMA_LEN_MASK, len),
			       bus->base + ASPEED_I2C_DMA_LEN_REG);
			bus->dma_len = len;
		} else {
			command |= ASPEED_I2CD_RX_BUFF_ENABLE;

			writel(FIELD_PREP(ASPEED_I2CD_BUF_RX_SIZE_MASK,
					  len - 1) |
			       FIELD_PREP(ASPEED_I2CD_BUF_TX_COUNT_MASK, 0) |
			       FIELD_PREP(ASPEED_I2CD_BUF_OFFSET_MASK,
					  bus->buf_offset),
			       bus->base + ASPEED_I2C_BUF_CTRL_REG);
		}
	} else {
		if (bus->buf_index + 1 == msg->len)
			command |= ASPEED_I2CD_M_S_RX_CMD_LAST;
	}

	return command;
}

static u32 aspeed_i2c_master_irq(struct aspeed_i2c_bus *bus, u32 irq_status)
{
	u32 irq_handled = 0, command = 0;
	struct i2c_msg *msg;
	u8 recv_byte;
	int ret;

	if (irq_status & ASPEED_I2CD_INTR_BUS_RECOVER_DONE) {
		bus->master_state = ASPEED_I2C_MASTER_INACTIVE;
		irq_handled |= ASPEED_I2CD_INTR_BUS_RECOVER_DONE;
		goto out_complete;
	}

	/*
	 * We encountered an interrupt that reports an error: the hardware
	 * should clear the command queue effectively taking us back to the
	 * INACTIVE state.
	 */
	ret = aspeed_i2c_check_master_error(irq_status);
	if (ret) {
		dev_dbg(bus->dev, "received master error interrupt: 0x%08x\n",
			irq_status);
		irq_handled |= (irq_status & ASPEED_I2CD_INTR_MASTER_ERRORS);
		if (bus->master_state != ASPEED_I2C_MASTER_INACTIVE) {
			bus->cmd_err = ret;
			if (bus->master_state == ASPEED_I2C_MASTER_STOP)
				irq_handled |= (irq_status &
				                ASPEED_I2CD_INTR_NORMAL_STOP);
			if (ret == -EAGAIN)
				irq_handled |= (irq_status &
						ASPEED_I2CD_INTR_TX_ACK);
			bus->master_state = ASPEED_I2C_MASTER_INACTIVE;
			goto out_complete;
		}
	}

	/* Master is not currently active, irq was for someone else. */
	if (bus->master_state == ASPEED_I2C_MASTER_INACTIVE ||
	    bus->master_state == ASPEED_I2C_MASTER_PENDING)
		goto out_no_complete;

	/* We are in an invalid state; reset bus to a known state. */
	if (!bus->msgs) {
		dev_err(bus->dev, "bus in unknown state. irq_status: 0x%x\n",
			irq_status);
		bus->cmd_err = -EIO;
		if (bus->master_state != ASPEED_I2C_MASTER_STOP &&
		    bus->master_state != ASPEED_I2C_MASTER_INACTIVE)
			aspeed_i2c_do_stop(bus);
		goto out_no_complete;
	}
	msg = &bus->msgs[bus->msgs_index];

	/*
	 * START is a special case because we still have to handle a subsequent
	 * TX or RX immediately after we handle it, so we handle it here and
	 * then update the state and handle the new state below.
	 */
	if (bus->master_state == ASPEED_I2C_MASTER_START) {
#if IS_ENABLED(CONFIG_I2C_SLAVE)
		/*
		 * If a peer master starts a xfer immediately after it queues a
		 * master command, clear the queued master command and change
		 * its state to 'pending'. To simplify handling of pending
		 * cases, it uses S/W solution instead of H/W command queue
		 * handling.
		 */
		if (unlikely(irq_status & ASPEED_I2CD_INTR_SLAVE_MATCH)) {
			writel(readl(bus->base + ASPEED_I2C_CMD_REG) &
				~ASPEED_I2CD_MASTER_CMDS_MASK,
			       bus->base + ASPEED_I2C_CMD_REG);
			bus->master_state = ASPEED_I2C_MASTER_PENDING;
			dev_dbg(bus->dev,
				"master goes pending due to a slave start\n");
			goto out_no_complete;
		}
#endif /* CONFIG_I2C_SLAVE */
		if (unlikely(!(irq_status & ASPEED_I2CD_INTR_TX_ACK))) {
			if (unlikely(!(irq_status & ASPEED_I2CD_INTR_TX_NAK))) {
				bus->cmd_err = -ENXIO;
				bus->master_state = ASPEED_I2C_MASTER_INACTIVE;
				goto out_complete;
			}
			pr_devel("no slave present at %02x\n", msg->addr);
			irq_handled |= ASPEED_I2CD_INTR_TX_NAK;
			bus->cmd_err = -ENXIO;
			aspeed_i2c_do_stop(bus);
			goto out_no_complete;
		}
		irq_handled |= ASPEED_I2CD_INTR_TX_ACK;
		if (msg->len == 0) { /* SMBUS_QUICK */
			aspeed_i2c_do_stop(bus);
			goto out_no_complete;
		}
		if (msg->flags & I2C_M_RD)
			bus->master_state = ASPEED_I2C_MASTER_RX_FIRST;
		else
			bus->master_state = ASPEED_I2C_MASTER_TX_FIRST;
	}

	switch (bus->master_state) {
	case ASPEED_I2C_MASTER_TX:
		if (unlikely(irq_status & ASPEED_I2CD_INTR_TX_NAK)) {
			dev_dbg(bus->dev, "slave NACKed TX\n");
			irq_handled |= ASPEED_I2CD_INTR_TX_NAK;
			goto error_and_stop;
		} else if (unlikely(!(irq_status & ASPEED_I2CD_INTR_TX_ACK))) {
			dev_err(bus->dev, "slave failed to ACK TX\n");
			goto error_and_stop;
		}
		irq_handled |= ASPEED_I2CD_INTR_TX_ACK;
		fallthrough;
	case ASPEED_I2C_MASTER_TX_FIRST:
		if (bus->buf_index < msg->len) {
			command = ASPEED_I2CD_M_TX_CMD;
			command |= aspeed_i2c_master_handle_tx_first(bus, msg);
			writel(command, bus->base + ASPEED_I2C_CMD_REG);
			bus->master_state = ASPEED_I2C_MASTER_TX;
		} else {
			aspeed_i2c_next_msg_or_stop(bus);
		}
		goto out_no_complete;
	case ASPEED_I2C_MASTER_RX_FIRST:
		/* RX may not have completed yet (only address cycle) */
		if (!(irq_status & ASPEED_I2CD_INTR_RX_DONE))
			goto out_no_complete;
		fallthrough;
	case ASPEED_I2C_MASTER_RX:
		if (unlikely(!(irq_status & ASPEED_I2CD_INTR_RX_DONE))) {
			dev_err(bus->dev, "master failed to RX\n");
			goto error_and_stop;
		}
		irq_handled |= ASPEED_I2CD_INTR_RX_DONE;

		if (msg->flags & I2C_M_RECV_LEN) {
			recv_byte = readl(bus->base +
					ASPEED_I2C_BYTE_BUF_REG) >> 8;
			if (unlikely(recv_byte > I2C_SMBUS_BLOCK_MAX)) {
				bus->cmd_err = -EPROTO;
				aspeed_i2c_do_stop(bus);
				goto out_no_complete;
			}
			msg->len = recv_byte + ((msg->flags & I2C_CLIENT_PEC) ?
						2 : 1);
			msg->flags &= ~I2C_M_RECV_LEN;
		} else if (msg->len) {
			aspeed_i2c_master_handle_rx(bus, msg);
		}

		if (bus->buf_index < msg->len) {
			command = ASPEED_I2CD_M_RX_CMD;
			command |= aspeed_i2c_master_handle_rx_next(bus, msg);
			writel(command, bus->base + ASPEED_I2C_CMD_REG);
			bus->master_state = ASPEED_I2C_MASTER_RX;
		} else {
			aspeed_i2c_next_msg_or_stop(bus);
		}
		goto out_no_complete;
	case ASPEED_I2C_MASTER_STOP:
		if (unlikely(!(irq_status & ASPEED_I2CD_INTR_NORMAL_STOP))) {
			dev_err(bus->dev,
				"master failed to STOP. irq_status:0x%x\n",
				irq_status);
			bus->cmd_err = -EIO;
			/* Do not STOP as we have already tried. */
		} else {
			irq_handled |= ASPEED_I2CD_INTR_NORMAL_STOP;
		}

		bus->master_state = ASPEED_I2C_MASTER_INACTIVE;
		goto out_complete;
	case ASPEED_I2C_MASTER_INACTIVE:
		dev_err(bus->dev,
			"master received interrupt 0x%08x, but is inactive\n",
			irq_status);
		bus->cmd_err = -EIO;
		/* Do not STOP as we should be inactive. */
		goto out_complete;
	default:
		WARN(1, "unknown master state\n");
		bus->master_state = ASPEED_I2C_MASTER_INACTIVE;
		bus->cmd_err = -EINVAL;
		goto out_complete;
	}
error_and_stop:
	bus->cmd_err = -EIO;
	aspeed_i2c_do_stop(bus);
	goto out_no_complete;
out_complete:
	bus->msgs = NULL;
	if (bus->cmd_err)
		bus->master_xfer_result = bus->cmd_err;
	else
		bus->master_xfer_result = bus->msgs_index + 1;
	complete(&bus->cmd_complete);
out_no_complete:
	return irq_handled;
}

static irqreturn_t aspeed_i2c_bus_irq(int irq, void *dev_id)
{
	struct aspeed_i2c_bus *bus = dev_id;
	u32 irq_received, irq_remaining, irq_handled;

	spin_lock(&bus->lock);
	irq_received = readl(bus->base + ASPEED_I2C_INTR_STS_REG);
	/* Ack all interrupts except for Rx done */
	writel(irq_received & ~ASPEED_I2CD_INTR_RX_DONE,
	       bus->base + ASPEED_I2C_INTR_STS_REG);
	readl(bus->base + ASPEED_I2C_INTR_STS_REG);
	irq_received &= ASPEED_I2CD_INTR_RECV_MASK;
	irq_remaining = irq_received;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	/*
	 * In most cases, interrupt bits will be set one by one, although
	 * multiple interrupt bits could be set at the same time. It's also
	 * possible that master interrupt bits could be set along with slave
	 * interrupt bits. Each case needs to be handled using corresponding
	 * handlers depending on the current state.
	 */
	if (bus->master_state != ASPEED_I2C_MASTER_INACTIVE &&
	    bus->master_state != ASPEED_I2C_MASTER_PENDING) {
		irq_handled = aspeed_i2c_master_irq(bus, irq_remaining);
		irq_remaining &= ~irq_handled;
		if (irq_remaining)
			irq_handled |= aspeed_i2c_slave_irq(bus, irq_remaining);
	} else {
		irq_handled = aspeed_i2c_slave_irq(bus, irq_remaining);
		irq_remaining &= ~irq_handled;
		if (irq_remaining) {
			irq_handled |= aspeed_i2c_master_irq(bus,
							     irq_remaining);
			if (bus->master_state == ASPEED_I2C_MASTER_INACTIVE &&
			    bus->slave_state == ASPEED_I2C_SLAVE_INACTIVE)
				irq_handled |= (irq_remaining &
						ASPEED_I2CD_INTR_NORMAL_STOP);
		}
	}

	/*
	 * Start a pending master command at here if a slave operation is
	 * completed.
	 */
	if (bus->master_state == ASPEED_I2C_MASTER_PENDING &&
	    bus->slave_state == ASPEED_I2C_SLAVE_INACTIVE)
		aspeed_i2c_do_start(bus);
#else
	irq_handled = aspeed_i2c_master_irq(bus, irq_remaining);
#endif /* CONFIG_I2C_SLAVE */

	irq_remaining &= ~irq_handled;
	if (irq_remaining)
		dev_err(bus->dev,
			"irq handled != irq. expected 0x%08x, but was 0x%08x\n",
			irq_received, irq_handled);

	/* Ack Rx done */
	if (irq_received & ASPEED_I2CD_INTR_RX_DONE) {
		writel(ASPEED_I2CD_INTR_RX_DONE,
		       bus->base + ASPEED_I2C_INTR_STS_REG);
		readl(bus->base + ASPEED_I2C_INTR_STS_REG);
	}
	spin_unlock(&bus->lock);
	return irq_remaining ? IRQ_NONE : IRQ_HANDLED;
}

static int aspeed_i2c_master_xfer(struct i2c_adapter *adap,
				  struct i2c_msg *msgs, int num)
{
	struct aspeed_i2c_bus *bus = i2c_get_adapdata(adap);
	unsigned long time_left, flags;
	int i;

	spin_lock_irqsave(&bus->lock, flags);
	bus->cmd_err = 0;

	/* If bus is busy in a single master environment, attempt recovery. */
	if (!bus->multi_master &&
	    (readl(bus->base + ASPEED_I2C_CMD_REG) &
	     ASPEED_I2CD_BUS_BUSY_STS)) {
		int ret;

		spin_unlock_irqrestore(&bus->lock, flags);
		ret = aspeed_i2c_recover_bus(bus);
		if (ret)
			return ret;
		spin_lock_irqsave(&bus->lock, flags);
	}

	bus->cmd_err = 0;
	bus->msgs = msgs;
	bus->msgs_index = 0;
	bus->msgs_count = num;

	reinit_completion(&bus->cmd_complete);
	aspeed_i2c_do_start(bus);
	spin_unlock_irqrestore(&bus->lock, flags);

	time_left = wait_for_completion_timeout(&bus->cmd_complete,
						bus->adap.timeout);

	if (time_left == 0) {
		/*
		 * If timed out and bus is still busy in a multi master
		 * environment, attempt recovery at here.
		 */
		if (bus->multi_master &&
		    (readl(bus->base + ASPEED_I2C_CMD_REG) &
		     ASPEED_I2CD_BUS_BUSY_STS))
			aspeed_i2c_recover_bus(bus);

		/*
		 * If timed out and the state is still pending, drop the pending
		 * master command.
		 */
		spin_lock_irqsave(&bus->lock, flags);
		if (bus->master_state == ASPEED_I2C_MASTER_PENDING)
			bus->master_state = ASPEED_I2C_MASTER_INACTIVE;
		spin_unlock_irqrestore(&bus->lock, flags);

		return -ETIMEDOUT;
	}

	for (i = 0; i < num; i++) {
		I2C_HEX_DUMP(bus, msgs[i].addr, msgs[i].flags,
			     msgs[i].buf, msgs[i].len);
	}

	return bus->master_xfer_result;
}

static u32 aspeed_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
/* precondition: bus.lock has been acquired. */
static void __aspeed_i2c_reg_slave(struct aspeed_i2c_bus *bus, u16 slave_addr)
{
	u32 addr_reg_val, func_ctrl_reg_val;

	/*
	 * Set slave addr.  Reserved bits can all safely be written with zeros
	 * on all of ast2[456]00, so zero everything else to ensure we only
	 * enable a single slave address (ast2500 has two, ast2600 has three,
	 * the enable bits for which are also in this register) so that we don't
	 * end up with additional phantom devices responding on the bus.
	 */
	addr_reg_val = slave_addr & ASPEED_I2CD_DEV_ADDR_MASK;
	writel(addr_reg_val, bus->base + ASPEED_I2C_DEV_ADDR_REG);

	/* Turn on slave mode. */
	func_ctrl_reg_val = readl(bus->base + ASPEED_I2C_FUN_CTRL_REG);
	func_ctrl_reg_val |= ASPEED_I2CD_SLAVE_EN;
	if (bus->general_call)
		func_ctrl_reg_val |= ASPEED_I2CD_GCALL_EN;
	writel(func_ctrl_reg_val, bus->base + ASPEED_I2C_FUN_CTRL_REG);
}

static int aspeed_i2c_reg_slave(struct i2c_client *client)
{
	struct aspeed_i2c_bus *bus = i2c_get_adapdata(client->adapter);
	unsigned long flags;

	spin_lock_irqsave(&bus->lock, flags);
	if (bus->slave) {
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EINVAL;
	}

	__aspeed_i2c_reg_slave(bus, client->addr);

	bus->slave = client;
	bus->slave_state = ASPEED_I2C_SLAVE_INACTIVE;
	spin_unlock_irqrestore(&bus->lock, flags);

	return 0;
}

static int aspeed_i2c_unreg_slave(struct i2c_client *client)
{
	struct aspeed_i2c_bus *bus = i2c_get_adapdata(client->adapter);
	u32 func_ctrl_reg_val;
	unsigned long flags;

	spin_lock_irqsave(&bus->lock, flags);
	if (!bus->slave) {
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EINVAL;
	}

	/* Turn off slave mode. */
	func_ctrl_reg_val = readl(bus->base + ASPEED_I2C_FUN_CTRL_REG);
	func_ctrl_reg_val &= ~ASPEED_I2CD_SLAVE_EN;
	if (bus->general_call)
		func_ctrl_reg_val &= ~ASPEED_I2CD_GCALL_EN;
	writel(func_ctrl_reg_val, bus->base + ASPEED_I2C_FUN_CTRL_REG);

	bus->slave = NULL;
	spin_unlock_irqrestore(&bus->lock, flags);

	return 0;
}
#endif /* CONFIG_I2C_SLAVE */

static const struct i2c_algorithm aspeed_i2c_algo = {
	.master_xfer	= aspeed_i2c_master_xfer,
	.functionality	= aspeed_i2c_functionality,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave	= aspeed_i2c_reg_slave,
	.unreg_slave	= aspeed_i2c_unreg_slave,
#endif /* CONFIG_I2C_SLAVE */
};

static u32 aspeed_i2c_get_clk_reg_val(struct device *dev,
				      u32 clk_high_low_mask,
				      u32 divisor)
{
	u32 base_clk_divisor, clk_high_low_max, clk_high, clk_low, tmp;

	/*
	 * SCL_high and SCL_low represent a value 1 greater than what is stored
	 * since a zero divider is meaningless. Thus, the max value each can
	 * store is every bit set + 1. Since SCL_high and SCL_low are added
	 * together (see below), the max value of both is the max value of one
	 * them times two.
	 */
	clk_high_low_max = (clk_high_low_mask + 1) * 2;

	/*
	 * The actual clock frequency of SCL is:
	 *	SCL_freq = APB_freq / (base_freq * (SCL_high + SCL_low))
	 *		 = APB_freq / divisor
	 * where base_freq is a programmable clock divider; its value is
	 *	base_freq = 1 << base_clk_divisor
	 * SCL_high is the number of base_freq clock cycles that SCL stays high
	 * and SCL_low is the number of base_freq clock cycles that SCL stays
	 * low for a period of SCL.
	 * The actual register has a minimum SCL_high and SCL_low minimum of 1;
	 * thus, they start counting at zero. So
	 *	SCL_high = clk_high + 1
	 *	SCL_low	 = clk_low + 1
	 * Thus,
	 *	SCL_freq = APB_freq /
	 *		((1 << base_clk_divisor) * (clk_high + 1 + clk_low + 1))
	 * The documentation recommends clk_high >= clk_high_max / 2 and
	 * clk_low >= clk_low_max / 2 - 1 when possible; this last constraint
	 * gives us the following solution:
	 */
	base_clk_divisor = divisor > clk_high_low_max ?
			ilog2((divisor - 1) / clk_high_low_max) + 1 : 0;

	if (base_clk_divisor > ASPEED_I2CD_TIME_BASE_DIVISOR_MASK) {
		base_clk_divisor = ASPEED_I2CD_TIME_BASE_DIVISOR_MASK;
		clk_low = clk_high_low_mask;
		clk_high = clk_high_low_mask;
		dev_err(dev,
			"clamping clock divider: divider requested, %u, is greater than largest possible divider, %u.\n",
			divisor, (1 << base_clk_divisor) * clk_high_low_max);
	} else {
		tmp = (divisor + (1 << base_clk_divisor) - 1)
				>> base_clk_divisor;
		clk_low = tmp / 2;
		clk_high = tmp - clk_low;

		if (clk_high)
			clk_high--;

		if (clk_low)
			clk_low--;
	}


	return ((clk_high << ASPEED_I2CD_TIME_SCL_HIGH_SHIFT)
		& ASPEED_I2CD_TIME_SCL_HIGH_MASK)
			| ((clk_low << ASPEED_I2CD_TIME_SCL_LOW_SHIFT)
			   & ASPEED_I2CD_TIME_SCL_LOW_MASK)
			| (base_clk_divisor
			   & ASPEED_I2CD_TIME_BASE_DIVISOR_MASK);
}

static u32 aspeed_i2c_24xx_get_clk_reg_val(struct device *dev, u32 divisor)
{
	/*
	 * clk_high and clk_low are each 3 bits wide, so each can hold a max
	 * value of 8 giving a clk_high_low_max of 16.
	 */
	return aspeed_i2c_get_clk_reg_val(dev, GENMASK(2, 0), divisor);
}

static u32 aspeed_i2c_25xx_get_clk_reg_val(struct device *dev, u32 divisor)
{
	/*
	 * clk_high and clk_low are each 4 bits wide, so each can hold a max
	 * value of 16 giving a clk_high_low_max of 32.
	 */
	return aspeed_i2c_get_clk_reg_val(dev, GENMASK(3, 0), divisor);
}

/* precondition: bus.lock has been acquired. */
static int aspeed_i2c_init_clk(struct aspeed_i2c_bus *bus)
{
	u32 timeout_base_divisor, timeout_tick_us, timeout_cycles;
	u32 divisor, clk_reg_val;

	divisor = DIV_ROUND_UP(bus->parent_clk_frequency, bus->bus_frequency);
	clk_reg_val = readl(bus->base + ASPEED_I2C_AC_TIMING_REG1);
	clk_reg_val &= (ASPEED_I2CD_TIME_TBUF_MASK |
			ASPEED_I2CD_TIME_THDSTA_MASK |
			ASPEED_I2CD_TIME_TACST_MASK);
	clk_reg_val |= bus->get_clk_reg_val(bus->dev, divisor);

	if (bus->hw_timeout_ms) {
		u8 div_max = ASPEED_I2CD_TIME_TIMEOUT_BASE_DIVISOR_MASK >>
			     ASPEED_I2CD_TIME_TIMEOUT_BASE_DIVISOR_SHIFT;
		u8 cycles_max = ASPEED_I2CD_TIMEOUT_CYCLES_MASK >>
				ASPEED_I2CD_TIMEOUT_CYCLES_SHIFT;

		timeout_base_divisor = 0;

		do {
			timeout_tick_us = 1000 * (16384 <<
						  (timeout_base_divisor << 1)) /
					  (bus->parent_clk_frequency / 1000);

			if (timeout_base_divisor == div_max ||
			    timeout_tick_us * ASPEED_I2CD_TIMEOUT_CYCLES_MASK >=
			    bus->hw_timeout_ms * 1000)
				break;
		} while (timeout_base_divisor++ < div_max);

		if (timeout_tick_us) {
			timeout_cycles = DIV_ROUND_UP(bus->hw_timeout_ms * 1000,
						      timeout_tick_us);
			if (timeout_cycles == 0)
				timeout_cycles = 1;
			else if (timeout_cycles > cycles_max)
				 timeout_cycles = cycles_max;
		} else {
			timeout_cycles = 0;
		}
	} else {
		timeout_base_divisor = 0;
		timeout_cycles = 0;
	}

	clk_reg_val |= FIELD_PREP(ASPEED_I2CD_TIME_TIMEOUT_BASE_DIVISOR_MASK,
				  timeout_base_divisor);

	writel(clk_reg_val, bus->base + ASPEED_I2C_AC_TIMING_REG1);
	writel(timeout_cycles, bus->base + ASPEED_I2C_AC_TIMING_REG2);

	return 0;
}

/* precondition: bus.lock has been acquired. */
static int aspeed_i2c_init(struct aspeed_i2c_bus *bus,
			     struct platform_device *pdev)
{
	u32 fun_ctrl_reg = ASPEED_I2CD_MASTER_EN;
	int ret;

	/* Disable everything. */
	writel(0, bus->base + ASPEED_I2C_FUN_CTRL_REG);

	device_property_read_u32(&pdev->dev, "aspeed,hw-timeout-ms",
				 &bus->hw_timeout_ms);
	if (bus->hw_timeout_ms)
		fun_ctrl_reg |= ASPEED_I2CD_BUS_AUTO_RECOVERY_EN;

	ret = aspeed_i2c_init_clk(bus);
	if (ret < 0)
		return ret;

	fun_ctrl_reg |= FIELD_PREP(ASPEED_I2CD_BUFFER_PAGE_SEL_MASK,
				   bus->buf_page);

	if (of_property_read_bool(pdev->dev.of_node, "multi-master"))
		bus->multi_master = true;
	else
		fun_ctrl_reg |= ASPEED_I2CD_MULTI_MASTER_DIS;

	/* Enable Master Mode */
	writel(readl(bus->base + ASPEED_I2C_FUN_CTRL_REG) | fun_ctrl_reg,
	       bus->base + ASPEED_I2C_FUN_CTRL_REG);

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	if (of_property_read_bool(pdev->dev.of_node, "general-call"))
		bus->general_call = true;

	/* If slave has already been registered, re-enable it. */
	if (bus->slave)
		__aspeed_i2c_reg_slave(bus, bus->slave->addr);
#endif /* CONFIG_I2C_SLAVE */

	/* Set interrupt generation of I2C controller */
	writel(ASPEED_I2CD_INTR_ALL, bus->base + ASPEED_I2C_INTR_CTRL_REG);

	return 0;
}

static int aspeed_i2c_reset(struct aspeed_i2c_bus *bus)
{
	struct platform_device *pdev = to_platform_device(bus->dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&bus->lock, flags);

	/* Disable and ack all interrupts. */
	writel(0, bus->base + ASPEED_I2C_INTR_CTRL_REG);
	writel(0xffffffff, bus->base + ASPEED_I2C_INTR_STS_REG);

	ret = aspeed_i2c_init(bus, pdev);

	spin_unlock_irqrestore(&bus->lock, flags);

	return ret;
}

static void aspeed_i2c_set_xfer_mode(struct aspeed_i2c_bus *bus)
{
	struct platform_device *pdev = to_platform_device(bus->dev);
	bool sram_enabled = true;
	int ret;

	/*
	 * Enable I2C SRAM in case of AST2500.
	 * SRAM is enabled by default in AST2400 and AST2600.
	 */
	if (of_device_is_compatible(pdev->dev.of_node,
				    "aspeed,ast2500-i2c-bus")) {
		struct regmap *gr_regmap = syscon_regmap_lookup_by_compatible("aspeed,ast2500-i2c-gr");

		if (IS_ERR(gr_regmap))
			ret = PTR_ERR(gr_regmap);
		else
			ret = regmap_update_bits(gr_regmap,
						 ASPEED_I2CG_GLOBAL_CTRL_REG,
						 ASPEED_I2CG_SRAM_BUFFER_EN,
						 ASPEED_I2CG_SRAM_BUFFER_EN);

		if (ret)
			sram_enabled = false;
	}

	/*
	 * Only AST2500 and AST2600 support DMA mode under some limitations:
	 * I2C is sharing the DMA H/W with UHCI host controller and MCTP
	 * controller. Since those controllers operate with DMA mode only, I2C
	 * has to use buffer mode or byte mode instead if one of those
	 * controllers is enabled. Also make sure that if SD/eMMC or Port80
	 * snoop uses DMA mode instead of PIO or FIFO respectively, I2C can't
	 * use DMA mode.
	 */
	if (sram_enabled && !IS_ENABLED(CONFIG_USB_UHCI_ASPEED) &&
	    !of_device_is_compatible(pdev->dev.of_node,
				     "aspeed,ast2400-i2c-bus")) {
		u32 dma_len_max = ASPEED_I2CD_DMA_LEN_MASK >>
				  ASPEED_I2CD_DMA_LEN_SHIFT;

		ret = device_property_read_u32(&pdev->dev,
					       "aspeed,dma-buf-size",
					       &bus->buf_size);
		if (!ret && bus->buf_size > dma_len_max)
			bus->buf_size = dma_len_max;
	}

	if (bus->buf_size) {
		if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))) {
			dev_warn(&pdev->dev, "No suitable DMA available\n");
		} else {
			bus->dma_pool = dma_pool_create("i2c-aspeed",
							&pdev->dev,
							bus->buf_size,
							ASPEED_I2CD_DMA_ALIGN,
							0);
			if (bus->dma_pool)
				bus->dma_buf = dma_pool_alloc(bus->dma_pool,
							      GFP_KERNEL,
							      &bus->dma_handle);

			if (!bus->dma_buf) {
				dev_warn(&pdev->dev,
					 "Cannot allocate DMA buffer\n");
				dma_pool_destroy(bus->dma_pool);
			}
		}
	}

	if (!bus->dma_buf && sram_enabled) {
		struct resource *res = platform_get_resource(pdev,
							     IORESOURCE_MEM, 1);

		if (res && resource_size(res) >= 2)
			bus->buf_base = devm_ioremap_resource(&pdev->dev, res);

		if (!IS_ERR_OR_NULL(bus->buf_base)) {
			bus->buf_size = resource_size(res);
			if (of_device_is_compatible(pdev->dev.of_node,
						    "aspeed,ast2400-i2c-bus")) {
				bus->buf_page = ((res->start >> 8) &
						 GENMASK(3, 0)) - 8;
				bus->buf_offset = (res->start >> 2) &
						  ASPEED_I2CD_BUF_OFFSET_MASK;
			}
		}
	}
}

static const struct of_device_id aspeed_i2c_bus_of_table[] = {
	{
		.compatible = "aspeed,ast2400-i2c-bus",
		.data = aspeed_i2c_24xx_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,ast2500-i2c-bus",
		.data = aspeed_i2c_25xx_get_clk_reg_val,
	},
	{
		.compatible = "aspeed,ast2600-i2c-bus",
		.data = aspeed_i2c_25xx_get_clk_reg_val,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, aspeed_i2c_bus_of_table);

static int aspeed_i2c_probe_bus(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct aspeed_i2c_bus *bus;
	struct clk *parent_clk;
	int irq, ret;

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	bus->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bus->base))
		return PTR_ERR(bus->base);

	bus->dev = &pdev->dev;

	/* Disable bus and clean up any left over interrupt state. */
	writel(0, bus->base + ASPEED_I2C_FUN_CTRL_REG);
	writel(0, bus->base + ASPEED_I2C_INTR_CTRL_REG);
	writel(0xffffffff, bus->base + ASPEED_I2C_INTR_STS_REG);

	/* Clear slave addresses. */
	writel(0, bus->base + ASPEED_I2C_DEV_ADDR_REG);

	parent_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(parent_clk))
		return PTR_ERR(parent_clk);
	bus->parent_clk_frequency = clk_get_rate(parent_clk);
	/* We just need the clock rate, we don't actually use the clk object. */
	devm_clk_put(&pdev->dev, parent_clk);

	bus->rst = devm_reset_control_get_shared(&pdev->dev, NULL);
	if (IS_ERR(bus->rst)) {
		dev_err(&pdev->dev,
			"missing or invalid reset controller device tree entry\n");
		return PTR_ERR(bus->rst);
	}
	reset_control_deassert(bus->rst);

	ret = of_property_read_u32(pdev->dev.of_node,
				   "bus-frequency", &bus->bus_frequency);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Could not read bus-frequency property\n");
		bus->bus_frequency = I2C_MAX_STANDARD_MODE_FREQ;
	}

	match = of_match_node(aspeed_i2c_bus_of_table, pdev->dev.of_node);
	if (!match)
		bus->get_clk_reg_val = aspeed_i2c_24xx_get_clk_reg_val;
	else
		bus->get_clk_reg_val = (u32 (*)(struct device *, u32))
				match->data;

	aspeed_i2c_set_xfer_mode(bus);

	/* Initialize the I2C adapter */
	spin_lock_init(&bus->lock);
	init_completion(&bus->cmd_complete);
	bus->adap.owner = THIS_MODULE;
	bus->adap.algo = &aspeed_i2c_algo;
	bus->adap.dev.parent = &pdev->dev;
	bus->adap.dev.of_node = pdev->dev.of_node;
	strscpy(bus->adap.name, pdev->name, sizeof(bus->adap.name));
	i2c_set_adapdata(&bus->adap, bus);

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	ret = devm_request_irq(&pdev->dev, irq, aspeed_i2c_bus_irq,
			       0, dev_name(&pdev->dev), bus);
	if (ret < 0)
		goto out_free_dma_buf;

	ret = aspeed_i2c_init(bus, pdev);
	if (ret < 0)
		goto out_free_dma_buf;

	ret = i2c_add_adapter(&bus->adap);
	if (ret < 0)
		goto out_free_dma_buf;

	platform_set_drvdata(pdev, bus);

	dev_info(bus->dev, "i2c bus %d registered (%s mode), irq %d\n",
		 bus->adap.nr, bus->dma_buf ? "dma" :
					      bus->buf_base ? "buffer" : "byte",
		 irq);

	return 0;

out_free_dma_buf:
	if (bus->dma_buf)
		dma_pool_free(bus->dma_pool, bus->dma_buf, bus->dma_handle);
	dma_pool_destroy(bus->dma_pool);

	return ret;
}

static int aspeed_i2c_remove_bus(struct platform_device *pdev)
{
	struct aspeed_i2c_bus *bus = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&bus->lock, flags);

	/* Disable everything. */
	writel(0, bus->base + ASPEED_I2C_FUN_CTRL_REG);
	writel(0, bus->base + ASPEED_I2C_INTR_CTRL_REG);

	spin_unlock_irqrestore(&bus->lock, flags);

	reset_control_assert(bus->rst);

	if (bus->dma_buf)
		dma_pool_free(bus->dma_pool, bus->dma_buf, bus->dma_handle);
	dma_pool_destroy(bus->dma_pool);

	i2c_del_adapter(&bus->adap);

	return 0;
}

static struct platform_driver aspeed_i2c_bus_driver = {
	.probe		= aspeed_i2c_probe_bus,
	.remove		= aspeed_i2c_remove_bus,
	.driver		= {
		.name		= "aspeed-i2c-bus",
		.of_match_table	= aspeed_i2c_bus_of_table,
	},
};
module_platform_driver(aspeed_i2c_bus_driver);

module_param_named(dump_debug, dump_debug, bool, 0644);
MODULE_PARM_DESC(dump_debug, "debug flag for dump printing");
module_param_named(dump_debug_bus_id, dump_debug_bus_id, int, 0644);
MODULE_PARM_DESC(dump_debug_bus_id, "bus id for dump debug printing");

MODULE_AUTHOR("Brendan Higgins <brendanhiggins@google.com>");
MODULE_DESCRIPTION("Aspeed I2C Bus Driver");
MODULE_LICENSE("GPL v2");
