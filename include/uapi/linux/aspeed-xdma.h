/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/* Copyright IBM Corp 2019 */

#ifndef _UAPI_LINUX_ASPEED_XDMA_H_
#define _UAPI_LINUX_ASPEED_XDMA_H_

#include <linux/types.h>

/*
 * aspeed_xdma_direction
 *
 * ASPEED_XDMA_DIRECTION_DOWNSTREAM: transfers data from the host to the BMC
 *
 * ASPEED_XDMA_DIRECTION_UPSTREAM: transfers data from the BMC to the host
 *
 * ASPEED_XDMA_DIRECTION_RESET: resets the XDMA engine
 */
enum aspeed_xdma_direction {
	ASPEED_XDMA_DIRECTION_DOWNSTREAM = 0,
	ASPEED_XDMA_DIRECTION_UPSTREAM,
	ASPEED_XDMA_DIRECTION_RESET,
};

/*
 * aspeed_xdma_op
 *
 * host_addr: the DMA address on the host side, typically configured by PCI
 *            subsystem
 *
 * len: the size of the transfer in bytes
 *
 * direction: an enumerator indicating the direction of the DMA operation; see
 *            enum aspeed_xdma_direction
 *
 * bmc_addr: the virtual address to DMA on the BMC side; this parameter is
 *           unused on current platforms since the XDMA engine is restricted to
 *           accessing the VGA memory space
 *
 * reserved: for natural alignment purposes only
 */
struct aspeed_xdma_op {
	__u64 host_addr;
	__u32 len;
	__u32 direction;
	__u32 bmc_addr;
	__u32 reserved;
};

#endif /* _UAPI_LINUX_ASPEED_XDMA_H_ */
