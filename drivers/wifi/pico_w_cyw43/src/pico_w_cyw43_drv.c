/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include "pico_w_cyw43_drv.h"

#define DT_DRV_COMPAT pico_w_cyw43

#include "pico_w_cyw43_log.h"
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static const struct pico_w_cyw43_cfg pico_w_cyw43_cfg = {
      	.data = NULL,
        .clock = NULL,
	.power = NULL,
};

static struct pico_w_cyw43_dev pico_w_cyw43_0; /* static instance */

static int pico_w_cyw43_init(const struct device *dev)
{
  LOG_DBG("");
  
  pico_w_cyw43_shell_register(dev->data);

  return 0;
}

static void pico_w_cyw43_iface_init(struct net_if *iface)
{
  LOG_DBG("");

  return;
}

int pico_w_cyw43_iface_status(const struct device *dev,
			      struct wifi_iface_status *status)
{
  LOG_DBG("");
  return 0;
}

static int pico_w_cyw43_mgmt_scan(const struct device *dev, scan_result_cb_t cb)
{
  LOG_DBG("");
  return 0;
}

static int pico_w_cyw43_mgmt_connect(const struct device *dev,
				     struct wifi_connect_req_params *params)
{
  LOG_DBG("");
  return 0;
}

static int pico_w_cyw43_mgmt_disconnect(const struct device *dev)
{
  LOG_DBG("");
  return 0;
}

static int pico_w_cyw43_mgmt_ap_enable(const struct device *dev,
				       struct wifi_connect_req_params *params)
{
  LOG_DBG("");
  return 0;
}

static int pico_w_cyw43_mgmt_ap_disable(const struct device *dev)
{
  LOG_DBG("");
  return 0;
}

static const struct net_wifi_mgmt_offload pico_w_cyw43_callbacks = {
	.wifi_iface.iface_api.init = pico_w_cyw43_iface_init,
	.scan			   = pico_w_cyw43_mgmt_scan,
	.connect		   = pico_w_cyw43_mgmt_connect,
	.disconnect		   = pico_w_cyw43_mgmt_disconnect,
	.ap_enable		   = pico_w_cyw43_mgmt_ap_enable,
	.ap_disable		   = pico_w_cyw43_mgmt_ap_disable,
	.iface_status		   = pico_w_cyw43_iface_status, 
};

NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, pico_w_cyw43_init, NULL,
				  &pico_w_cyw43_0, &pico_w_cyw43_cfg,
				  CONFIG_WIFI_INIT_PRIORITY,
				  &pico_w_cyw43_callbacks,
				  1500);


