/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include "pico_w_cyw43_drv.h"
#include "picowi/picowi_defs.h"
#include "picowi/picowi_pio.h"
#include "picowi/picowi_wifi.h"
#include "picowi/picowi_init.h"
#include "picowi/picowi_pico.h"
#include "picowi/picowi_scan.h"

#define DT_DRV_COMPAT pico_w_cyw43

#include "pico_w_cyw43_log.h"
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define PICOWCYW43_WORKQUEUE_STACK_SIZE 1024
K_KERNEL_STACK_DEFINE(pico_w_cyw43_work_q_stack, PICOWCYW43_WORKQUEUE_STACK_SIZE);

static const struct pico_w_cyw43_cfg pico_w_cyw43_cfg = {
      	.data = NULL,
        .clock = NULL,
	.power = NULL,
};

static struct pico_w_cyw43_dev pico_w_cyw43_0; /* static instance */


static void pico_w_cyw43_scan(struct pico_w_cyw43_dev *pico_w_cyw43)
{
	char *data;
	int i, ret;

	uint32_t led_ticks, poll_ticks;
	bool ledon=false;

	
	LOG_DBG("");

	pico_w_cyw43_lock(pico_w_cyw43);
	LOG_DBG("Scanning for wifi networks.\n");
	printf("Scanning for wifi networks.\n");

	add_event_handler(scan_event_handler);
	
	if (!scan_start()) {
	  printf("Error: can't start scan\n");
	}
	else {
	  while (1) {
	    // Toggle LED at 1 Hz
	    if (mstimeout(&led_ticks, 50)) {
	      wifi_set_led(ledon = !ledon);
	    }
	    
            // Get any events
            if (wifi_get_irq() || mstimeout(&poll_ticks, 10)) {
	      printf("wifi_get_irq()\n");
	      if (event_poll() < 0) {
		printf("event_poll < 0()\n");
		//printf("Total time %lu msec\n", ustime()/1000);
		break;
	      }
            }
	  }
	}


#if 0
	ret = eswifi_at_cmd_rsp(eswifi, cmd, &data);
	if (ret < 0) {
		eswifi->scan_cb(eswifi->iface, -EIO, NULL);
		eswifi_unlock(eswifi);
		return;
	}
#endif
	
#if 0	
	for (i = 0; i < ret; i++) {
		if (data[i] == '#') {
			struct wifi_scan_result res = {0};

			__parse_scan_res(&data[i], &res);

			pico_w_cyw43->scan_cb(pico_w_cyw43->iface, 0, &res);
			k_yield();

			while (data[i] && data[i] != '\n') {
				i++;
			}
		}
	}
#endif
	/* WiFi scan is done. */
	pico_w_cyw43->scan_cb(pico_w_cyw43->iface, 0, NULL);

	pico_w_cyw43_unlock(pico_w_cyw43);
}

static int pico_w_cyw43_connect(struct pico_w_cyw43_dev *pico_w_cyw43)
{
	struct in_addr addr;
	int err;

	LOG_DBG("Connecting to %s (pass=%s)", pico_w_cyw43->sta.ssid,
		pico_w_cyw43->sta.pass);

	pico_w_cyw43_lock(pico_w_cyw43);

	LOG_DBG("Connected!");

	pico_w_cyw43_unlock(pico_w_cyw43);
	return 0;

error:
	pico_w_cyw43_unlock(pico_w_cyw43);
	return -EIO;
}

static int pico_w_cyw43_disconnect(struct pico_w_cyw43_dev *pico_w_cyw43)
{
	struct in_addr addr;
	int err;

	LOG_DBG("Disconnecting from %s", pico_w_cyw43->sta.ssid);

	pico_w_cyw43_lock(pico_w_cyw43);

	LOG_DBG("Disconnected!");

	pico_w_cyw43_unlock(pico_w_cyw43);
	return 0;

error:
	pico_w_cyw43_unlock(pico_w_cyw43);
	return -EIO;
}

static void pico_w_cyw43_status_work(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct pico_w_cyw43_dev *pico_w_cyw43;
	char status[] = "CS\r";
	char rssi[] = "CR\r";
	char *rsp;
	int ret;
	
	pico_w_cyw43 = CONTAINER_OF(dwork, struct pico_w_cyw43_dev, status_work);

	pico_w_cyw43_lock(pico_w_cyw43);

#if 0	
	if (eswifi->role == ESWIFI_ROLE_AP) {
		goto done;
	}

	ret = eswifi_at_cmd_rsp(eswifi, status, &rsp);
	if (ret < 1) {
		LOG_ERR("Unable to retrieve status");
		goto done;
	}

	if (rsp[0] == '0' && eswifi->sta.connected) {
		eswifi->sta.connected = false;
		wifi_mgmt_raise_disconnect_result_event(eswifi->iface, 0);
		goto done;
	} else if (rsp[0] == '1' && !eswifi->sta.connected) {
		eswifi->sta.connected = true;
		wifi_mgmt_raise_connect_result_event(eswifi->iface, 0);
	}

	ret = eswifi_at_cmd_rsp(eswifi, rssi, &rsp);
	if (ret < 1) {
		LOG_ERR("Unable to retrieve rssi");
		/* continue */
	} else {
		eswifi->sta.rssi = atoi(rsp);
	}

	k_work_reschedule_for_queue(&eswifi->work_q, &eswifi->status_work,
				    K_MSEC(1000 * 30));

#endif //0	
done:
	pico_w_cyw43_unlock(pico_w_cyw43);
}



static void pico_w_cyw43_request_work(struct k_work *item)
{
	struct pico_w_cyw43_dev *pico_w_cyw43;
	int err;

	LOG_DBG("");

	pico_w_cyw43 = CONTAINER_OF(item, struct pico_w_cyw43_dev, request_work);

	switch (pico_w_cyw43->req) {
	case PICOWCYW43_REQ_CONNECT:
		err = pico_w_cyw43_connect(pico_w_cyw43);
		wifi_mgmt_raise_connect_result_event(pico_w_cyw43->iface, err);
		k_work_reschedule_for_queue(&pico_w_cyw43->work_q, &pico_w_cyw43->status_work,
					    K_MSEC(1000));
		break;
	case PICOWCYW43_REQ_DISCONNECT:
		err = pico_w_cyw43_disconnect(pico_w_cyw43);
		wifi_mgmt_raise_disconnect_result_event(pico_w_cyw43->iface, err);
		break;
	case PICOWCYW43_REQ_SCAN:
		pico_w_cyw43_scan(pico_w_cyw43);
		break;
	case PICOWCYW43_REQ_NONE:	
	default:
		break;
	}
}


static void pico_w_cyw43_iface_init(struct net_if *iface)
{
  LOG_DBG("");
  printf("Calling pico_w_cyw43_iface_init()\n");
  return;
}

int pico_w_cyw43_iface_status(const struct device *dev,
			      struct wifi_iface_status *status)
{
  LOG_DBG("");
  printf("Calling ifsce_status()\n");
  return 0;
}

static int pico_w_cyw43_mgmt_scan(const struct device *dev, scan_result_cb_t cb)
{

  struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  
  LOG_DBG("");
  printf("Calling mgmt_scan()\n");
    
  pico_w_cyw43_lock(pico_w_cyw43);

  pico_w_cyw43->scan_cb = cb;
  pico_w_cyw43->req = PICOWCYW43_REQ_SCAN;
  k_work_submit_to_queue(&pico_w_cyw43->work_q, &pico_w_cyw43->request_work);
  
  pico_w_cyw43_unlock(pico_w_cyw43);
  
  return 0;
}

static int pico_w_cyw43_mgmt_connect(const struct device *dev,
				     struct wifi_connect_req_params *params)
{
  LOG_DBG("");
  printf("Calling mgmt_connect()\n");
  return 0;
}

static int pico_w_cyw43_mgmt_disconnect(const struct device *dev)
{
  LOG_DBG("");
  printf("Calling mgmt_disconnect()\n");
  return 0;
}

static int pico_w_cyw43_mgmt_ap_enable(const struct device *dev,
				       struct wifi_connect_req_params *params)
{
  LOG_DBG("");
  printf("Calling mgmt_ap_enable()\n");
  return 0;
}

static int pico_w_cyw43_mgmt_ap_disable(const struct device *dev)
{
  LOG_DBG("");
  printf("Calling mgmt_ap_disable()\n");
  return 0;
}

static int pico_w_cyw43_init(const struct device *dev)
{
  int rv;

  struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  const struct pico_w_cyw43_cfg *cfg = dev->config;

  
  LOG_DBG("");

  pico_w_cyw43->role = PICOWCYW43_ROLE_CLIENT;
  k_mutex_init(&pico_w_cyw43->mutex);

  
  //picw_w_cyw43_print_tasks();

  if (!wifi_setup()) {
    LOG_DBG("Error: SPI communication\n");
  }
  else if (!wifi_init()) {
    LOG_DBG("Error: can't initialise WiFi\n");
  }
  else {
    rv= 0;
  }
  printf("Made it through wifi_setup()\n");

  //picw_w_cyw43_print_tasks();

  k_work_queue_start(&pico_w_cyw43->work_q, pico_w_cyw43_work_q_stack,
		     K_KERNEL_STACK_SIZEOF(pico_w_cyw43_work_q_stack),
		     CONFIG_SYSTEM_WORKQUEUE_PRIORITY - 1, NULL);
  
  k_work_init(&pico_w_cyw43->request_work, pico_w_cyw43_request_work);
  k_work_init_delayable(&pico_w_cyw43->status_work, pico_w_cyw43_status_work);

  pico_w_cyw43_shell_register(dev->data);

  wifi_set_led(true);
  
  return rv;
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


