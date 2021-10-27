/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2021, Intel Corporation. */

#ifndef _UAPI_LINUX_ASPEED_ESPI_MMBI_H
#define _UAPI_LINUX_ASPEED_ESPI_MMBI_H

struct aspeed_espi_mmbi;
struct aspeed_mmbi_channel;
struct aspeed_mmbi_protocol;

/* ESPI registers */
#define ASPEED_ESPI_CTRL2 0x80 /* Engine Control 2 */
#define ASPEED_ESPI_PC_RX_SADDR                                                \
	0x84 /* Mapping Source Address of Peripheral Channel Rx Package */
#define ASPEED_ESPI_PC_RX_TADDR                                                \
	0x88 /* Mapping Target Address of Peripheral Channel Rx Package */
#define ASPEED_ESPI_PC_RX_TADDRM                                               \
	0x8c /* Mapping Target Address Mask of Peripheral Channel Rx Package */

#ifdef CONFIG_ESPI_LGMR_ADDR
#define PCH_ESPI_LGMR_BASE_ADDRESS CONFIG_ESPI_LGMR_ADDR
#else
#define PCH_ESPI_LGMR_BASE_ADDRESS 0xFE0B0000
#endif
#define BMC_SRAM_BASE_ADDRESS 0x9EFF0000
#define ASPEED_ESPI_PC_RX_TADDR_MASK 0xFFFF0000
#define ESPI_MMBI_TOTAL_SIZE (1024 * 64) /* 64KB  MMBI size */

#define ASPEED_ESPI_CTRL2 0x80 /* Engine Control 2 */
#define ASPEED_ESPI_SYS_EVENT 0x98 /* ESPI098 - System Event from and to master */
#define ESPI_DISABLE_PERP_MEM_READ BIT(6)
#define ESPI_DISABLE_PERP_MEM_WRITE BIT(4)

/* MMBI Control registers */
#define ASPEED_MMBI_CTRL 0x00
#define ASPEED_MMBI_IRQ_STATUS 0x08
#define ASPEED_MMBI_IRQ_ENABLE 0x0C
#define ASPEED_MMBI_HRWP0_INSTANCE0 0x10
#define ASPEED_MMBI_HRWP1_INSTANCE0 0x14

#define MMBI_ENABLE_FUNCTION BIT(0) /* Enable MMBI Function */
#define MMBI_ENABLE_WRITE_PROTECTION BIT(1) /* Write protection of MMBI000[0] */
#define MMBI_TOTAL_SIZE_64K 0 /* [6:4] - 000b:64KB */
#define MMBI_INSTANCE_SIZE_64K (BIT(8) | BIT(9)) /* [10:8] - 011b:64KB */

/* MMBI008 - Interrupt status */
#define HRWP0_WRITE_MASK BIT(0)
#define HRWP1_READ_MASK BIT(1)

/* LPC register for SCI interrupt */
#define AST_LPC_SWCR0300 0x180
#define LPC_BMC_TRIG_WAKEUP_EVT_STS BIT(8)

#define AST_LPC_SWCR0704 0x184
#define LPC_BMC_TRIG_WAKEUP_EVT_EN BIT(8)

#define AST_LPC_SWCR0B08 0x188
#define LPC_BMC_TRIG_WAKEUP_EVT BIT(6)

#define AST_LPC_ACPIB3B0 0x1A8
#define LPC_BMC_TRIG_SCI_EVT_STS BIT(8)

#define AST_LPC_ACPIB7B4 0x1AC
#define LPC_BMC_TRIG_SCI_EVT_EN BIT(8)

struct aspeed_mmbi_get_empty_space {
	__u32 length;
};

#define __ASPEED_MMBI_CTRL_IOCTL_MAGIC 0xbb
/*
 * This IOCTL is meant to read empty space in B2H buffer
 * in a specific channel
 */
#define ASPEED_MMBI_CTRL_IOCTL_GET_B2H_EMPTY_SPACE                             \
	_IOWR(__ASPEED_MMBI_CTRL_IOCTL_MAGIC, 0x00,                            \
	      struct aspeed_mmbi_get_empty_space)

/* This IOCTL to send BMC reset request */
#define ASPEED_MMBI_CTRL_IOCTL_SEND_RESET_REQUEST	                       \
	_IOW(__ASPEED_MMBI_CTRL_IOCTL_MAGIC, 0x01, int)

#endif /* __LINUX_ASPEED_ESPI_MMBI_H */
