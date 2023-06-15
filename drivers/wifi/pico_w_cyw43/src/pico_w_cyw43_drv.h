/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/net/wifi_mgmt.h>

#include "pico_w_cyw43_offload.h"

#define MAX_DATA_SIZE 1600

enum pico_w_cyw43_security_type {
	PICOWCYW43_SEC_OPEN,
	PICOWCYW43_SEC_WEP,
	PICOWCYW43_SEC_WPA,
	PICOWCYW43_SEC_WPA2_AES,
	PICOWCYW43_SEC_WPA2_MIXED,
	PICOWCYW43_SEC_MAX
};

enum pico_w_cyw43_request {
	PICOWCYW43_REQ_SCAN,
	PICOWCYW43_REQ_CONNECT,
	PICOWCYW43_REQ_DISCONNECT,
	PICOWCYW43_REQ_NONE
};

enum pico_w_cyw43_role {
	PICOWCYW43_ROLE_CLIENT,
	PICOWCYW43_ROLE_AP,
};

struct pico_w_cyw43_cfg {
	struct gpio_dt_spec data;
	struct gpio_dt_spec clock;
  	struct gpio_dt_spec power;
  	struct gpio_dt_spec irq_gpio;  
};



struct pico_w_cyw43_sta {
	char ssid[WIFI_SSID_MAX_LEN + 1];
	enum pico_w_cyw43_security_type security;
	char pass[65];
	bool connected;
	uint8_t channel;
	int rssi;
};

struct pico_w_cyw43_bus_ops;



struct pico_w_cyw43_dev {
	struct net_if *iface;
	struct pico_w_cyw43_bus_ops *bus;
	scan_result_cb_t scan_cb;
	struct k_work_q work_q;
	struct k_work request_work;
	struct k_work_delayable status_work;
	struct pico_w_cyw43_sta sta;
	enum pico_w_cyw43_request req;
	enum pico_w_cyw43_role role;
	uint8_t mac[6];
	char buf[MAX_DATA_SIZE];
	struct k_mutex mutex;
	atomic_val_t mutex_owner;
	unsigned int mutex_depth;
	void *bus_data;
	struct pico_w_cyw43_off_socket socket[PICOWCYW43_OFFLOAD_MAX_SOCKETS];
};


static inline void pico_w_cyw43_lock(struct pico_w_cyw43_dev *pico_w_cyw43)
{
	/* Nested locking */
	if (atomic_get(&pico_w_cyw43->mutex_owner) != (atomic_t)(uintptr_t)_current) {
		k_mutex_lock(&pico_w_cyw43->mutex, K_FOREVER);
		atomic_set(&pico_w_cyw43->mutex_owner, (atomic_t)(uintptr_t)_current);
		pico_w_cyw43->mutex_depth = 1;
	} else {
		pico_w_cyw43->mutex_depth++;
	}
}

static inline void pico_w_cyw43_unlock(struct pico_w_cyw43_dev *pico_w_cyw43)
{
	if (!--pico_w_cyw43->mutex_depth) {
		atomic_set(&pico_w_cyw43->mutex_owner, -1);
		k_mutex_unlock(&pico_w_cyw43->mutex);
	}
}

#if defined(CONFIG_WIFI_RPIPICOWCYW43_SHELL)
void pico_w_cyw43_shell_register(struct pico_w_cyw43_dev *dev);
#else
#define pico_w_cyw43_shell_register(dev)
#endif

void picw_w_cyw43_print_tasks(void);
