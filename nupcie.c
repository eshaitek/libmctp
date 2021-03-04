/* SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later */
/* Copyright (c) 2020 Nuvoton Technology Corporation */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/nuvoton-mctp.h>

#include "container_of.h"
#include "libmctp-alloc.h"
#include "libmctp-nupcie.h"
#include "libmctp-log.h"
#include "nupcie.h"

#undef pr_fmt
#define pr_fmt(fmt) "nupcie: " fmt

/*
 * PCIe header template in "network format" - Big Endian
 */
static const struct mctp_pcie_hdr mctp_pcie_hdr_template_be = {
	.fmt_type = MSG_4DW_HDR,
	.mbz_attr_length = MCTP_PCIE_VDM_ATTR,
	.code = MSG_CODE_VDM_TYPE_1,
	.vendor = VENDOR_ID_DMTF_VDM
};

static int mctp_nupcie_get_error(struct mctp_binding_nupcie *nupcie)
{
    int ret, error = 0;

    ret = ioctl(nupcie->fd, PCIE_VDM_GET_ERRORS , &error);
    if (ret < 0) {
		mctp_prerr("ioctl PCIE_VDM_GET_ERRORS failed: %s, errno = %d", NU_DRV_FILE, ret);
		return ret;
	}

	return error;
}

static int mctp_nupcie_clear_error(struct mctp_binding_nupcie *nupcie, int error)
{
    int ret;

    ret = ioctl(nupcie->fd, PCIE_VDM_CLEAR_ERRORS , error);
    if (ret < 0) {
		mctp_prerr("ioctl PCIE_VDM_CLEAR_ERRORS failed: %s, errno = %d", NU_DRV_FILE, ret);
		return ret;
	}

	return 0;
}

#define PCIE_VDM_ERR_HW_FIFO_OVERFLOW				0x00000001
#define PCIE_VDM_ERR_DMA_BUFFER_OVERFLOW			0x00000002
#define PCIE_VDM_ERR_USER_BUFFER_OVERFLOW			0x00000004
#define PCIE_VDM_ERR_BUS_RESET_OCCURED				0x00000008


static int mctp_nupcie_error_reason(struct mctp_binding_nupcie *nupcie)
{
    int error;

    error = mctp_nupcie_get_error(nupcie);
	if (error < 0)
		return error;

	if (error | PCIE_VDM_ERR_HW_FIFO_OVERFLOW)
		mctp_prerr("%s: VDM HW FIFO OVERFLOW", NU_DRV_FILE);
	if (error | PCIE_VDM_ERR_DMA_BUFFER_OVERFLOW)
		mctp_prerr("%s: VDM DMA BUFFER OVERFLOW", NU_DRV_FILE);
	if (error | PCIE_VDM_ERR_USER_BUFFER_OVERFLOW)
		mctp_prerr("%s: VDM USER BUFFER OVERFLOW", NU_DRV_FILE);
	if (error | PCIE_VDM_ERR_BUS_RESET_OCCURED)
		mctp_prerr("%s: VDM BUS RESET OCCURED", NU_DRV_FILE);

	error = mctp_nupcie_clear_error(nupcie, error);
	if (error < 0)
		return error;

	return 0;
}

static int mctp_nupcie_vdm_init(struct mctp_binding_nupcie *nupcie)
{
    int ret = ioctl(nupcie->fd , PCIE_VDM_REINIT , 0);
    if (ret < 0) {
        mctp_prerr("Cannot init: %s, errno = %d", NU_DRV_FILE, ret);
		return ret;
	}

	return 0;
}

static int mctp_nupcie_open(struct mctp_binding_nupcie *nupcie)
{
	int fd = open(NU_DRV_FILE, O_RDWR);

	if (fd < 0) {
		mctp_prerr("Cannot open: %s, errno = %d", NU_DRV_FILE, errno);

		return fd;
	}

	nupcie->fd = fd;
	return 0;
}

/*
 * Start function. Opens driver and read bdf
 */
static int mctp_nupcie_start(struct mctp_binding *b)
{
	struct mctp_binding_nupcie *nupcie = binding_to_nupcie(b);
	int rc;

	assert(nupcie);

	rc = mctp_nupcie_open(nupcie);
	if (rc)
		return -errno;

	rc = mctp_nupcie_vdm_init(nupcie);
	if (rc)
		return -errno;

	return 0;
}

static uint8_t mctp_nupcie_tx_get_pad_len(struct mctp_pktbuf *pkt)
{
	size_t sz = mctp_pktbuf_size(pkt);

	return PCIE_PKT_ALIGN(sz) - sz;
}

static uint16_t mctp_nupcie_tx_get_payload_size_dw(struct mctp_pktbuf *pkt)
{
	size_t sz = mctp_pktbuf_size(pkt);

	return PCIE_PKT_ALIGN(sz) / sizeof(uint32_t) - MCTP_HDR_SIZE_DW;
}
/*
 * Tx function which writes single packet to device driver
 */
static int mctp_nupcie_tx(struct mctp_binding *b,
				   struct mctp_pktbuf *pkt)
{
	struct mctp_nupcie_pkt_private *pkt_prv =
		(struct mctp_nupcie_pkt_private *)pkt->msg_binding_private;
	struct mctp_binding_nupcie *nupcie = binding_to_nupcie(b);
	struct mctp_pcie_hdr *hdr = (struct mctp_pcie_hdr *)pkt->data;
	struct mctp_hdr *mctp_hdr = mctp_pktbuf_hdr(pkt);
	uint16_t payload_len_dw = mctp_nupcie_tx_get_payload_size_dw(pkt);
	uint8_t pad = mctp_nupcie_tx_get_pad_len(pkt);
	ssize_t write_len, len, i;

	memcpy(hdr, &mctp_pcie_hdr_template_be, sizeof(*hdr));

	mctp_prdebug("TX, len: %d, pad: %d", payload_len_dw, pad);

	PCIE_SET_ROUTING(hdr, pkt_prv->routing);
	PCIE_SET_DATA_LEN(hdr, payload_len_dw);
	PCIE_SET_REQ_ID(hdr, nupcie->bdf);
	PCIE_SET_TARGET_ID(hdr, pkt_prv->remote_id);
	PCIE_SET_PAD_LEN(hdr, pad);

	len = (payload_len_dw * sizeof(uint32_t)) +
	      NUVOTON_MCTP_PCIE_VDM_HDR_SIZE;

	mctp_trace_tx(pkt->data, len);

	write_len = write(nupcie->fd, pkt->data, len);
	if (write_len < 0) {
		mctp_prerr("TX error");
		mctp_nupcie_error_reason(nupcie);
		return -1;
	}

	return 0;
}

static size_t mctp_nupcie_rx_get_payload_size(struct mctp_pcie_hdr *hdr)
{
	size_t len = PCIE_GET_DATA_LEN(hdr) * sizeof(uint32_t);
	uint8_t pad = PCIE_GET_PAD_LEN(hdr);

	return len - pad;
}

/*
 * Simple poll implementation for use
 */
int mctp_nupcie_poll(struct mctp_binding_nupcie *nupcie, int timeout)
{
	struct pollfd fds[1];
	int rc;

	fds[0].fd = nupcie->fd;
	fds[0].events = POLLIN | POLLOUT;

	rc = poll(fds, 1, timeout);

	if (rc > 0)
		return fds[0].revents;

	if (rc < 0) {
		mctp_prwarn("Poll returned error status (errno=%d)", errno);

		return -1;
	}

	return 0;
}

static bool mctp_nupcie_is_routing_supported(int routing)
{
	switch (routing) {
	case PCIE_ROUTE_TO_RC:
	case PCIE_ROUTE_BY_ID:
	case PCIE_BROADCAST_FROM_RC:
		return true;
	default:
		return false;
	}
}

int mctp_nupcie_rx(struct mctp_binding_nupcie *nupcie)
{
	uint32_t data[MCTP_NUPCIE_BINDING_DEFAULT_BUFFER];
	struct mctp_nupcie_pkt_private pkt_prv;
	struct mctp_pktbuf *pkt;
	struct mctp_pcie_hdr *hdr;
	struct mctp_hdr *mctp_hdr;
	size_t read_len, payload_len;
	int rc;

	read_len = read(nupcie->fd, &data, sizeof(data));;
	if (read_len < 0) {
		mctp_prerr("Reading RX data failed (errno = %d)", errno);
		mctp_nupcie_error_reason(nupcie);
		return -1;
	}

	mctp_trace_rx(&data, read_len);

	hdr = (struct mctp_pcie_hdr *)data;
	payload_len = mctp_nupcie_rx_get_payload_size(hdr);

	pkt_prv.routing = PCIE_GET_ROUTING(hdr);

	if (!mctp_nupcie_is_routing_supported(pkt_prv.routing)) {
		mctp_prerr("unsupported routing value: %d", pkt_prv.routing);
		return -1;
	}

	pkt_prv.remote_id = PCIE_GET_REQ_ID(hdr);
	pkt_prv.own_id = PCIE_GET_TARGET_ID(hdr);

	pkt = mctp_pktbuf_alloc(&nupcie->binding, 0);
	if (!pkt) {
		mctp_prerr("pktbuf allocation failed");
		return -1;
	}

	rc = mctp_pktbuf_push(pkt, data + PCIE_HDR_SIZE_DW,
			      payload_len + sizeof(struct mctp_hdr));

	if (rc) {
		mctp_prerr("Cannot push to pktbuf");
		mctp_pktbuf_free(pkt);
		return -1;
	}

	mctp_hdr = mctp_pktbuf_hdr(pkt);
	memcpy(pkt->msg_binding_private, &pkt_prv, sizeof(pkt_prv));
	mctp_bus_rx(&nupcie->binding, pkt);

	return 0;
}

/*
 * Initializes PCIe binding structure
 */
struct mctp_binding_nupcie *mctp_nupcie_init(void)
{
	struct mctp_binding_nupcie *nupcie;

	nupcie = __mctp_alloc(sizeof(*nupcie));
	if (!nupcie)
		return NULL;

	memset(nupcie, 0, sizeof(*nupcie));

	nupcie->binding.name = "nupcie";
	nupcie->binding.version = 1;
	nupcie->binding.tx = mctp_nupcie_tx;
	nupcie->binding.start = mctp_nupcie_start;
	nupcie->binding.pkt_size = MCTP_PACKET_SIZE(MCTP_BTU);

	/* where mctp_hdr starts in in/out comming data
	 * note: there are two approaches: first (used here) that core
	 * allocates pktbuf to contain all binding metadata or this is handled
	 * other way by only by binding.
	 * This might change as smbus binding implements support for medium
	 * specific layer */
	nupcie->binding.pkt_pad = sizeof(struct mctp_pcie_hdr);
	nupcie->binding.pkt_priv_size = sizeof(struct mctp_nupcie_pkt_private);

	return nupcie;
}

/*
 * Closes file descriptor and releases binding memory
 */
void mctp_nupcie_free(struct mctp_binding_nupcie *b)
{
	close(b->fd);
	__mctp_free(b);
}

/*
 * Returns generic binder handler from PCIe binding handler
 */
struct mctp_binding *
mctp_nupcie_core(struct mctp_binding_nupcie *nupcie)
{
	return &nupcie->binding;
}

int mctp_nupcie_get_fd(struct mctp_binding_nupcie *nupcie)
{
	return nupcie->fd;
}