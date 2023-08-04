/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include <zephyr/drivers/spi.h>

#include "pico_w_cyw43_drv.h"

#include "georgerobotics/cyw43.h"
#include "georgerobotics/cyw43_country.h"

#define DT_DRV_COMPAT infineon_cyw43

#include "pico_w_cyw43_log.h"
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL);

#define PICOWCYW43_WORKQUEUE_STACK_SIZE 1024
K_KERNEL_STACK_DEFINE(pico_w_cyw43_work_q_stack, PICOWCYW43_WORKQUEUE_STACK_SIZE);

static const struct pico_w_cyw43_cfg pico_w_cyw43_cfg = {
#if 0  
        .data = NULL,
        .clock = NULL,
	.power = NULL,
#endif
	.irq_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(cyw43_int), gpios),
};



static struct pico_w_cyw43_dev pico_w_cyw43_0; /* static instance */

#if 0 // Just leaving this here for now in case interrupts give us trouble and we need to poll instead.  
#define EVENT_POLL_THREAD_STACK_SIZE 1024
#define EVENT_POLL_THREAD_PRIO 2
K_KERNEL_STACK_MEMBER(pico_w_cyw43_event_poll_stack, EVENT_POLL_THREAD_STACK_SIZE);

struct k_thread event_thread;
static void pico_w_cyw43_event_poll_thread(void *p1)
{
	struct pico_w_cyw43_dev *pico_w_cyw43 = p1;

	uint32_t led_ticks, poll_ticks;

	
	LOG_DBG("Starting pico_w_cyw43_event_poll_thread\n");

	k_thread_name_set(NULL, "pico_w_cyw43_event_poll_thread");
	
	while (1) {
	  //if (wifi_get_irq() || mstimeout(&poll_ticks, 10)) {
	    pico_w_cyw43_lock(pico_w_cyw43);

	    if (event_poll() < 0) {
	      LOG_DBG("event_poll() returns < 0\n");
	    }
	    //}
	    pico_w_cyw43_unlock(pico_w_cyw43);
	    k_sleep(K_MSEC(10));
	}
	
}
#endif

// Kind of ridiculous, but a NOP version of this never-called LWIP function needs to exist to avoid link error.
struct pbuf {}; // Even more ridiculous... needs a fake declaraion of an LWIP structure to avoid warning.
uint16_t pbuf_copy_partial(const struct pbuf *p, void *dataptr, uint16_t len, uint16_t offset) {
    return 0;
}

static int process_cyw43_scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result) {
        LOG_DBG("ssid: %-32s rssi: %4d chan: %3d mac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\n",
            result->ssid, result->rssi, result->channel,
            result->bssid[0], result->bssid[1], result->bssid[2], result->bssid[3], result->bssid[4], result->bssid[5],
            result->auth_mode);
    }
    //TODO: return this info to Zephyr
    return 0;
}

static void pico_w_cyw43_scan(struct pico_w_cyw43_dev *pico_w_cyw43, bool active)
{	

  	LOG_DBG("Scannings (%s).\n", active ? "active" : "passive");
        pico_w_cyw43_lock(pico_w_cyw43);


	if (!cyw43_wifi_scan_active(&cyw43_state)) {
	  cyw43_wifi_scan_options_t scan_options = {0};
	  scan_options.scan_type = (active ? 0 : 1);
	  
	  int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, process_cyw43_scan_result);
	  
	  if (err == 0) {
	    LOG_INF("\nPerforming wifi scan\n");
	  } else {
	    LOG_ERR("Failed to start scan: %d\n", err);
	  }
	}
	else {
	  LOG_INF("\nWifi_scan already active.\n");
	}

	/* WiFi scan is done. */
	
	LOG_DBG("calling scan_cb with interface=%s\n", pico_w_cyw43->iface->if_dev->dev->name);
	pico_w_cyw43->scan_cb(pico_w_cyw43->iface, 0, NULL);

	pico_w_cyw43_unlock(pico_w_cyw43);
}


static int pico_w_cyw43_connect(struct pico_w_cyw43_dev *pico_w_cyw43)
{

	LOG_DBG("Connecting to %s (pass=%s)\n", pico_w_cyw43->sta.ssid, pico_w_cyw43->sta.pass);
	pico_w_cyw43_lock(pico_w_cyw43);

	const uint8_t *ssid = (const uint8_t *)pico_w_cyw43->sta.ssid;
	const uint8_t *pass = (const uint8_t *)pico_w_cyw43->sta.pass;

	//TODO: might have to implement retry and timeout
	//if (cyw43_arch_wifi_connect_timeout_ms(pico_w_cyw43->sta.ssid, pico_w_cyw43->sta.pass, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
	if (cyw43_wifi_join(&cyw43_state, strlen(ssid), (const uint8_t *)ssid, pass ? strlen(pass) : 0, (const uint8_t *)pass, CYW43_AUTH_WPA2_AES_PSK, NULL, CYW43_CHANNEL_NONE)) {
	  LOG_ERR("failed to connect.\n");
	  return 1;
	} else {
	  LOG_DBG("Connected.\n");
	}


	LOG_DBG("Done Connecting to %s (pass=%s)\n", pico_w_cyw43->sta.ssid, pico_w_cyw43->sta.pass);
	pico_w_cyw43_unlock(pico_w_cyw43);
	return 0;
}

static int pico_w_cyw43_disconnect(struct pico_w_cyw43_dev *pico_w_cyw43)
{
	LOG_DBG("Disconnecting from %s", pico_w_cyw43->sta.ssid);

	pico_w_cyw43_lock(pico_w_cyw43);
	cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA); 

        if (false) { goto error; }
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
	
	pico_w_cyw43 = CONTAINER_OF(dwork, struct pico_w_cyw43_dev, status_work);

	pico_w_cyw43_lock(pico_w_cyw43);

	LOG_ERR("pico_w_cyw43_status_work not implemented\n");
	
	pico_w_cyw43_unlock(pico_w_cyw43);
}



static void pico_w_cyw43_request_work(struct k_work *item)
{
	struct pico_w_cyw43_dev *pico_w_cyw43;
	int err;

	pico_w_cyw43 = CONTAINER_OF(item, struct pico_w_cyw43_dev, request_work);

	LOG_DBG("(req=%d)", pico_w_cyw43->req);
	
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
	case PICOWCYW43_REQ_PASSIVE_SCAN:
	        pico_w_cyw43_scan(pico_w_cyw43, false);
		break;
	case PICOWCYW43_REQ_ACTIVE_SCAN:
	        pico_w_cyw43_scan(pico_w_cyw43, true);
		break;
	case PICOWCYW43_REQ_NONE:	
	default:
		break;
	}
}


static int pico_w_cyw43_send(const struct device *dev, struct net_pkt *pkt)
{
  int rv;
  struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  
  pico_w_cyw43_lock(pico_w_cyw43);
  
  // TODO: May want to do do the work in a work queue (example: eswifi_offload.c::eswifi_off_send())
  
  rv = cyw43_send_ethernet(&cyw43_state, CYW43_ITF_STA, net_pkt_get_len(pkt), (void *)(pkt->buffer->data), false);

  pico_w_cyw43_unlock(pico_w_cyw43);

  // TODO: Should also add statistic counters
  
  return (rv == 0 ? 0 : -EIO);
}


static void pico_w_cyw43_iface_init(struct net_if *iface)
{


  struct pico_w_cyw43_dev *pico_w_cyw43 = &pico_w_cyw43_0;
  struct ethernet_context *eth_ctx = net_if_l2_data(iface);
  
  LOG_DBG("Calling pico_w_cyw43_iface_init()\n");

  pico_w_cyw43_lock(pico_w_cyw43);
  
  eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
  pico_w_cyw43->iface = iface;
  
  //TODO: set MAC address
  
  ethernet_init(iface);
  
  pico_w_cyw43_unlock(pico_w_cyw43);

  return;
}

int pico_w_cyw43_iface_status(const struct device *dev,
			      struct wifi_iface_status *status)
{
  LOG_DBG("Calling iface_status()\n");
  return 0;
}

static int pico_w_cyw43_mgmt_scan(const struct device *dev,
				  struct wifi_scan_params *params,
				  scan_result_cb_t cb)
{

  struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  
  LOG_DBG("Calling mgmt_scan()");
    
  pico_w_cyw43_lock(pico_w_cyw43);

  pico_w_cyw43->scan_cb = cb;

  pico_w_cyw43->req = (params->scan_type == WIFI_SCAN_TYPE_ACTIVE ?
		       PICOWCYW43_REQ_ACTIVE_SCAN :
		       PICOWCYW43_REQ_PASSIVE_SCAN);

  k_work_submit_to_queue(&pico_w_cyw43->work_q, &pico_w_cyw43->request_work);
  
  pico_w_cyw43_unlock(pico_w_cyw43);
  
  return 0;
}

static int __pico_w_cyw43_dev_deconfig(struct pico_w_cyw43_dev *pico_w_cyw43,
                                       struct wifi_connect_req_params *params)
{
    memcpy(pico_w_cyw43->sta.ssid, params->ssid, params->ssid_length);
    pico_w_cyw43->sta.ssid[params->ssid_length] = '\0';

    switch (params->security) {
    case WIFI_SECURITY_TYPE_NONE:
        pico_w_cyw43->sta.pass[0] = '\0';
        pico_w_cyw43->sta.security = PICOWCYW43_SEC_OPEN;
        break;
    case WIFI_SECURITY_TYPE_PSK:
        memcpy(pico_w_cyw43->sta.pass, params->psk, params->psk_length);
        pico_w_cyw43->sta.pass[params->psk_length] = '\0';
        pico_w_cyw43->sta.security = PICOWCYW43_SEC_WPA2_MIXED;
        break;
    default:
        return -EINVAL;
    }

    if (params->channel == WIFI_CHANNEL_ANY) {
        pico_w_cyw43->sta.channel = 0U;
    } else {
        pico_w_cyw43->sta.channel = params->channel;
    }

    return 0;
}

static int pico_w_cyw43_mgmt_connect(const struct device *dev,
				     struct wifi_connect_req_params *params)
{
  struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  int err;
  LOG_DBG("");
  pico_w_cyw43_lock(pico_w_cyw43);
  err = __pico_w_cyw43_dev_deconfig(pico_w_cyw43, params);
  if (!err) {
    pico_w_cyw43->req = PICOWCYW43_REQ_CONNECT;
    k_work_submit_to_queue(&pico_w_cyw43->work_q, &pico_w_cyw43->request_work);
  }
  pico_w_cyw43_unlock(pico_w_cyw43);
  return 0;
}

static int pico_w_cyw43_mgmt_disconnect(const struct device *dev)
{
  struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  LOG_DBG("");
  pico_w_cyw43_lock(pico_w_cyw43);
  pico_w_cyw43->req = PICOWCYW43_REQ_DISCONNECT;
  k_work_submit_to_queue(&pico_w_cyw43->work_q, &pico_w_cyw43->request_work);
  pico_w_cyw43_unlock(pico_w_cyw43);
  return 0;
}


static int pico_w_cyw43_mgmt_ap_enable(const struct device *dev,
				       struct wifi_connect_req_params *params)
{
  struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  
  LOG_DBG("Calling mgmt_ap_enable()\n");

  pico_w_cyw43_lock(pico_w_cyw43);

  pico_w_cyw43_unlock(pico_w_cyw43);
  return 0;    
}

static int pico_w_cyw43_mgmt_ap_disable(const struct device *dev)
{
  LOG_DBG("Calling mgmt_ap_disable()\n");
  return 0;
}

void cyw43_cb_tcpip_deinit(cyw43_t *self, int itf)
{
  LOG_DBG("Calling cyw43_cb_tcpip_deinit(itf=%d)\n", itf);
  return;
}

void cyw43_cb_process_ethernet(void *cb_data, int itf, size_t len, const uint8_t *buf)
{
  struct net_pkt *pkt;
  struct pico_w_cyw43_dev *pico_w_cyw43 = &pico_w_cyw43_0;

  LOG_DBG("Calling cyw43_cb_process_ethernet(itf=%d)\n", itf);

  if (pico_w_cyw43->iface == NULL) {
    LOG_ERR("network interface unavailable");
    return;
  }

  pkt = net_pkt_rx_alloc_with_buffer(pico_w_cyw43->iface, len, AF_UNSPEC, 0, K_MSEC(100));

  if (!pkt) {
    LOG_ERR("Failed to get net buffer");
    goto pkt_processing_succeeded;
  }

  if (net_pkt_write(pkt, buf, len) < 0) {
    LOG_ERR("Failed to write pkt");
    goto pkt_processing_failed;
  }

  if (net_recv_data(pico_w_cyw43->iface, pkt) < 0) {
    LOG_ERR("Failed to push received data");
    goto pkt_processing_failed;
  }

  // TODO: Add statistic counters
  
  return;

 pkt_processing_failed:
  net_pkt_unref(pkt);

 pkt_processing_succeeded:
  return;
}

void cyw43_hal_generate_laa_mac(int idx, uint8_t buf[6])
{
  LOG_DBG("cyw43_hal_generate_laa_mac(idx=%d)", idx);
  return;
}

void cyw43_cb_tcpip_init(cyw43_t *self, int itf)
{
  LOG_DBG("cyw43_cb_tcpip_init(itf=%d)", itf);
  return;
}

void cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf)
{
  LOG_DBG("Calling cyw43_cb_tcpip_set_link_up(itf=%d)", itf);
  return;
}

void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf)
{
  LOG_DBG("Calling cyw43_cb_tcpip_set_link_down(itf=%d)", itf);
  return;
}

void cyw43_schedule_internal_poll_dispatch(void (*func)(void)) {
  LOG_DBG("Calling cyw43_schedule_internal_poll_dispatch()");
  assert(func == cyw43_poll);
  return;
}

static void cyw43_set_irq_enabled(bool enabled)
{
  LOG_DBG("Calling cyw43_set_irq_enabled(%s)", (enabled ? "true" : "false"));

  gpio_pin_interrupt_configure(pico_w_cyw43_cfg.irq_gpio.port, PICOWCYW43_GPIO_INTERRUPT_PIN, (enabled?(GPIO_INT_ENABLE|GPIO_INT_LEVEL_HIGH):GPIO_INT_DISABLE));
}


void cyw43_post_poll_hook(void)
{
  LOG_DBG("Calling cyw43_post_poll_hook()");
  cyw43_set_irq_enabled(true);
}

void cyw43_thread_enter(void)
{
  LOG_DBG("Calling cyw43_thread_enter()");
  pico_w_cyw43_lock(&pico_w_cyw43_0);
}

void cyw43_thread_exit(void)
{
  LOG_DBG("Calling cyw43_thread_exit()");
  pico_w_cyw43_unlock(&pico_w_cyw43_0);
}

void cyw43_delay_ms(uint32_t ms) {
  if (ms > 1000) {
    LOG_WRN("warning, calling cyw43_delay_ms() with value %d", ms); 
  }
  k_busy_wait(ms * 1000);
}

void sleep_ms(uint32_t ms) {
  if (ms > 1000) {
    LOG_WRN("warning, calling cyw43_delay_ms() with value %d\n", ms); 
  }

  k_sleep(K_MSEC(ms));
}

void cyw43_delay_us(uint32_t us) {
  if (us > 1000) {
    LOG_WRN("warning, calling cyw43_delay_ms() with value %d\n", us); 
  }

  k_busy_wait(us);
}

void cyw43_await_background_or_timeout_us(uint32_t timeout_us) {
  if (timeout_us > 1000) {
    LOG_WRN("warning, calling cyw43_delay_ms() with value %d\n", timeout_us); 
  }
  k_busy_wait(timeout_us);
}

struct gpio_callback pico_w_cyw43_gpio_cb;

static void pico_w_cyw43_isr(const struct device *port,
			    struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
  LOG_DBG("Calling pico_w_cyw43_isr()");

  // TODO: Should really find the Zephyr equivalent of gpio_get_irq_event_mask
  uint32_t events = gpio_get_irq_event_mask(PICOWCYW43_GPIO_INTERRUPT_PIN);
  if (events & GPIO_IRQ_LEVEL_HIGH) { 
    cyw43_set_irq_enabled(false);
  }
  
  //Schedule the rest of the work to be done
  if (cyw43_poll) {
    LOG_DBG("Calling cyw43_poll()");
    cyw43_poll();
  }
  else {
    LOG_DBG("Not calling cyw43_poll(), because it doesn't exist.");
  }
}


static void pico_w_cyw43_register_cb()
{
  int rv;
 
  //Note that this is technically a gpio_callback, not a true isr
  gpio_init_callback(&pico_w_cyw43_gpio_cb,
		     pico_w_cyw43_isr,
		     BIT(pico_w_cyw43_cfg.irq_gpio.pin));

  rv = gpio_add_callback(pico_w_cyw43_cfg.irq_gpio.port, &pico_w_cyw43_gpio_cb);

  LOG_DBG("gpio_add_callback() returned %d", rv);
  
  cyw43_set_irq_enabled(true);
}


static int pico_w_cyw43_init(const struct device *dev)
{
    struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  
    LOG_DBG("");

    k_mutex_init(&pico_w_cyw43->mutex);

    pico_w_cyw43->role = PICOWCYW43_ROLE_CLIENT;
    

    cyw43_init(&cyw43_state);
    // Based on example in ./pico-sdk/src/rp2_common/pico_cyw43_driver/cyw43_driver.c:cyw43_driver_init() IRQ setup happens next

    pico_w_cyw43_register_cb();

    cyw43_set_irq_enabled(true);

    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);


    k_work_queue_start(&pico_w_cyw43->work_q, pico_w_cyw43_work_q_stack,
		       K_KERNEL_STACK_SIZEOF(pico_w_cyw43_work_q_stack),
		       CONFIG_SYSTEM_WORKQUEUE_PRIORITY - 1, NULL);
   
    k_work_init(&pico_w_cyw43->request_work, pico_w_cyw43_request_work);
    k_work_init_delayable(&pico_w_cyw43->status_work, pico_w_cyw43_status_work);


    //wifi_set_led(true);
    
    pico_w_cyw43_shell_register(pico_w_cyw43);
    
    return 0;
  
}

static const struct wifi_mgmt_ops pico_w_cyw43_mgmt_api = {
	.scan			   = pico_w_cyw43_mgmt_scan,
	.connect		   = pico_w_cyw43_mgmt_connect,
	.disconnect		   = pico_w_cyw43_mgmt_disconnect,
	.ap_enable		   = pico_w_cyw43_mgmt_ap_enable,
	.ap_disable		   = pico_w_cyw43_mgmt_ap_disable,
	.iface_status		   = pico_w_cyw43_iface_status, 
};

static const struct net_wifi_mgmt_offload pico_w_cyw43_callbacks = {
	.wifi_iface.iface_api.init = pico_w_cyw43_iface_init,
	.wifi_iface.send           = pico_w_cyw43_send,
	.wifi_mgmt_api             = &pico_w_cyw43_mgmt_api,
};

NET_DEVICE_DT_INST_DEFINE(0,
			  pico_w_cyw43_init, NULL,
			  &pico_w_cyw43_0, &pico_w_cyw43_cfg, CONFIG_WIFI_INIT_PRIORITY,
			  &pico_w_cyw43_callbacks, ETHERNET_L2,
			  NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);


