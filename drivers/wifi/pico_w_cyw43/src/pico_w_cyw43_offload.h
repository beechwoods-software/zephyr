/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/net_offload.h>

#define PICOWCYW43_OFFLOAD_MAX_SOCKETS 4

enum pico_w_cyw43_transport_type {
	PICOWCYW43_TRANSPORT_TCP,
	PICOWCYW43_TRANSPORT_UDP,
	PICOWCYW43_TRANSPORT_UDP_LITE,
	PICOWCYW43_TRANSPORT_TCP_SSL,
};

enum pico_w_cyw43_socket_state {
	PICOWCYW43_SOCKET_STATE_NONE,
	PICOWCYW43_SOCKET_STATE_CONNECTING,
	PICOWCYW43_SOCKET_STATE_CONNECTED,
	PICOWCYW43_SOCKET_STATE_ACCEPTING,
};

struct pico_w_cyw43_off_socket {
	uint8_t index;
	enum pico_w_cyw43_transport_type type;
	enum pico_w_cyw43_socket_state state;
	struct net_context *context;
	net_context_recv_cb_t recv_cb;
	net_context_connect_cb_t conn_cb;
	net_context_send_cb_t send_cb;
	net_tcp_accept_cb_t accept_cb;
	void *recv_data;
	void *conn_data;
	void *send_data;
	void *accept_data;
	struct net_pkt *tx_pkt;
	struct k_work connect_work;
	struct k_work send_work;
	struct k_work_delayable read_work;
	struct sockaddr peer_addr;
	struct k_sem read_sem;
	struct k_sem accept_sem;
	uint16_t port;
	bool is_server;
	int usage;
	struct k_fifo fifo;
	struct net_pkt *prev_pkt_rem;
};

