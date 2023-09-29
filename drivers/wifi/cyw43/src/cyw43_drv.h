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

#include "cyw43_offload.h"

#define MAX_DATA_SIZE 1600
#define PICOWCYW43_GPIO_INTERRUPT_PIN 24

enum cyw43_security_type {
	PICOWCYW43_SEC_OPEN,
	PICOWCYW43_SEC_WEP,
	PICOWCYW43_SEC_WPA,
	PICOWCYW43_SEC_WPA2_AES,
	PICOWCYW43_SEC_WPA2_MIXED,
	PICOWCYW43_SEC_MAX
};

enum cyw43_request {
	PICOWCYW43_REQ_ACTIVE_SCAN,
	PICOWCYW43_REQ_PASSIVE_SCAN,	
	PICOWCYW43_REQ_CONNECT,
	PICOWCYW43_REQ_DISCONNECT,
	PICOWCYW43_REQ_NONE
};

enum cyw43_role {
	PICOWCYW43_ROLE_CLIENT,
	PICOWCYW43_ROLE_AP,
};

struct cyw43_cfg {
	struct gpio_dt_spec data;
	struct gpio_dt_spec clock;
  	struct gpio_dt_spec power;
  	struct gpio_dt_spec irq_gpio;  
  	struct gpio_dt_spec wl_on_gpio;  
};



struct cyw43_sta {
	char ssid[WIFI_SSID_MAX_LEN + 1];
	enum cyw43_security_type security;
	char pass[65];
	bool connected;
	uint8_t channel;
	int rssi;
};

struct cyw43_bus_ops;


//TODO: check these for unnecessary/unused fields.
struct cyw43_dev {
	struct net_if *iface;
	struct cyw43_bus_ops *bus;
	scan_result_cb_t scan_cb;
	struct k_work_q work_q;
	struct k_work request_work;
	struct k_work_delayable status_work;
	struct cyw43_sta sta;
	enum cyw43_request req;
	enum cyw43_role role;
	uint8_t mac[6];
        struct net_stats_wifi stats;
        uint8_t frame_buf[NET_ETH_MAX_FRAME_SIZE];
	char buf[MAX_DATA_SIZE];
	struct k_mutex mutex;
	atomic_val_t mutex_owner;
	unsigned int mutex_depth;
	void *bus_data;
	struct cyw43_off_socket socket[PICOWCYW43_OFFLOAD_MAX_SOCKETS];
};


static inline void cyw43_lock(struct cyw43_dev *cyw43_device)
{
	/* Nested locking */
	if (atomic_get(&cyw43_device->mutex_owner) != (atomic_t)(uintptr_t)_current) {
		k_mutex_lock(&cyw43_device->mutex, K_FOREVER);
		atomic_set(&cyw43_device->mutex_owner, (atomic_t)(uintptr_t)_current);
		cyw43_device->mutex_depth = 1;
	} else {
		cyw43_device->mutex_depth++;
	}
}

static inline void cyw43_unlock(struct cyw43_dev *cyw43_device)
{
	if (!--cyw43_device->mutex_depth) {
		atomic_set(&cyw43_device->mutex_owner, -1);
		k_mutex_unlock(&cyw43_device->mutex);
	}
}

#if defined(CONFIG_WIFI_CYW43_SHELL)
void cyw43_shell_register(struct cyw43_dev *dev);
#endif
