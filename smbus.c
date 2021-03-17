/* SPDX-License-Identifier: Apache-2.0 */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#ifdef MCTP_HAVE_FILEIO
#include <fcntl.h>
#endif

#define pr_fmt(x) "smbus: " x

#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <sys/poll.h>
#include <err.h>
#include <errno.h>

#include "libmctp-alloc.h"
#include "libmctp-log.h"
#include "libmctp-smbus.h"
#include "libmctp.h"

#ifndef container_of
#define container_of(ptr, type, member)                                        \
	(type *)((char *)(ptr) - (char *)&((type *)0)->member)
#endif

#define binding_to_smbus(b) container_of(b, struct mctp_binding_smbus, binding)

#define MCTP_COMMAND_CODE 0x0F
#define MCTP_SLAVE_ADDR_INDEX 0
//8bit:0x20;7bit:0x10 => NCT6681 only response to i2c slave address 0x20
#define MCTP_SOURCE_SLAVE_ADDRESS 0x21

#define SMBUS_COMMAND_CODE_SIZE 1
#define SMBUS_LENGTH_FIELD_SIZE 1
#define SMBUS_ADDR_OFFSET_SLAVE 0x1000
#define SMBUS_MAX_PKT_PAYLOAD_SIZE 64

struct mctp_smbus_header_tx {
	uint8_t command_code;
	uint8_t byte_count;
	uint8_t source_slave_address;
};

struct mctp_smbus_header_rx {
	uint8_t destination_slave_address;
	uint8_t command_code;
	uint8_t byte_count;
	uint8_t source_slave_address;
};

static int mctp_smbus_tx(struct mctp_binding_smbus *smbus, const uint8_t len,
			 struct mctp_smbus_extra_params *pkt_pvt)
{
	int ret = send(smbus->out_fd,&len,sizeof(uint8_t),0);
	ret = send(smbus->out_fd,smbus->txbuf,len,0);
	return ret;
}

int mctp_smbus_read(struct mctp_binding_smbus *smbus);

static int mctp_binding_smbus_tx(struct mctp_binding *b,
				 struct mctp_pktbuf *pkt)
{
	struct mctp_binding_smbus *smbus = binding_to_smbus(b);
	struct mctp_smbus_header_tx *smbus_hdr_tx = (void *)smbus->txbuf;
	struct mctp_smbus_extra_params *pkt_pvt =
		(struct mctp_smbus_extra_params *)pkt->msg_binding_private;
	struct mctp_hdr *mctp_hdr = (void *)(&pkt->data[pkt->start]);

	smbus_hdr_tx->command_code = MCTP_COMMAND_CODE;
	if (!pkt_pvt) {
		mctp_prerr("Binding private information not available");
		return -1;
	}
	/* the length field in the header excludes smbus framing
     * and escape sequences */
	size_t pkt_length = mctp_pktbuf_size(pkt);
	smbus_hdr_tx->byte_count = pkt_length;// + 1;
	smbus_hdr_tx->source_slave_address = MCTP_SOURCE_SLAVE_ADDRESS;

	memcpy(smbus->txbuf, &pkt->data[pkt->start], pkt_length);

	if (mctp_smbus_tx(smbus, pkt_length, pkt_pvt) < 0) {
		mctp_prerr("Error in tx of smbus message");
		return -1;
	}
	else
		mctp_smbus_read(smbus);

	return 0;
}

#ifdef MCTP_HAVE_FILEIO
int mctp_smbus_read(struct mctp_binding_smbus *smbus)
{
	ssize_t len = 0;
	uint8_t rsp_size;
	struct mctp_smbus_extra_params pvt_data;

	len = read(smbus->in_fd, &rsp_size, sizeof(uint8_t));

	len = read(smbus->in_fd, smbus->rxbuf, sizeof(smbus->rxbuf));

	if (len < 0 || len != rsp_size) {
		mctp_prerr("Failed to read");
		return -1;
	}

	mctp_trace_rx(smbus->rxbuf, len);

	smbus->rx_pkt = mctp_pktbuf_alloc(&(smbus->binding), 0);
	assert(smbus->rx_pkt);

	if (mctp_pktbuf_push(
		    smbus->rx_pkt, smbus->rxbuf,
		    len ) != 0) {
		mctp_prerr("Can't push tok pktbuf: %m");
		return -1;
	}

	mctp_bus_rx(&(smbus->binding), smbus->rx_pkt);

	smbus->rx_pkt = NULL;
	return 0;
}

int mctp_smbus_set_in_fd(struct mctp_binding_smbus *smbus, int fd)
{
	smbus->in_fd = fd;
}

int mctp_smbus_set_out_fd(struct mctp_binding_smbus *smbus, int fd)
{
	smbus->out_fd = fd;
}
#endif

int mctp_smbus_register_bus(struct mctp_binding_smbus *smbus, struct mctp *mctp,
			    mctp_eid_t eid)
{
	int rc = mctp_register_bus(mctp, &smbus->binding, eid);

	if (rc == 0) {
		/* TODO: Can we drop bus_id from mctp_binding_smbus? */
		smbus->bus_id = 0;
		mctp_binding_set_tx_enabled(&smbus->binding, true);
	}

	return rc;
}

struct mctp_binding_smbus *mctp_smbus_init(void)
{
	struct mctp_binding_smbus *smbus;

	smbus = __mctp_alloc(sizeof(*smbus));
	memset(&(smbus->binding), 0, sizeof(smbus->binding));

	smbus->in_fd = -1;
	smbus->out_fd = -1;

	smbus->rx_pkt = NULL;
	smbus->binding.name = "smbus";
	smbus->binding.version = 1;
	smbus->binding.pkt_size = SMBUS_MAX_PKT_PAYLOAD_SIZE;
	smbus->binding.pkt_priv_size = sizeof(struct mctp_smbus_extra_params);

	smbus->binding.tx = mctp_binding_smbus_tx;
	return smbus;
}

void mctp_smbus_free(struct mctp_binding_smbus *smbus)
{
	if (!(smbus->in_fd < 0)) {
		close(smbus->in_fd);
	}
	if (!(smbus->out_fd < 0)) {
		close(smbus->out_fd);
	}

	__mctp_free(smbus);
}
