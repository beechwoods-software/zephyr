/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include <zephyr/drivers/spi.h>

#include "cyw43_drv.h"

#include "georgerobotics/cyw43.h"
#include "georgerobotics/cyw43_country.h"

#define DT_DRV_COMPAT infineon_cyw43

#include "cyw43_log.h"
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL);

#define PICOWCYW43_WORKQUEUE_STACK_SIZE 1024
K_KERNEL_STACK_DEFINE(cyw43_work_q_stack, PICOWCYW43_WORKQUEUE_STACK_SIZE);

static const struct cyw43_cfg cyw43_cfg = {
#if 0  
        .data = NULL,
        .clock = NULL,
	.power = NULL,
#endif
#ifdef ISR_EVENT_PROCESSING
	.irq_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(cyw43_int), gpios),
#endif
  .wl_on_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(cyw43), wl_on_gpios),
};



static struct cyw43_dev cyw43_0; /* static instance */

#define POLLING_THREAD
#ifdef POLLING_THREAD // Just leaving this here for now in case interrupts give us trouble and we need to poll instead.  
#define EVENT_POLL_THREAD_STACK_SIZE 1024
#define EVENT_POLL_THREAD_PRIO 2
K_KERNEL_STACK_MEMBER(cyw43_event_poll_stack, EVENT_POLL_THREAD_STACK_SIZE);

struct k_thread event_thread;
static void cyw43_event_poll_thread(void *p1)
{
	LOG_DBG("Starting cyw43_event_poll_thread\n");

	k_thread_name_set(NULL, "cyw43_event_poll_thread");
	
	while (1) {
	    if (cyw43_poll) {
	      cyw43_poll();
	    }
	    else {
	      LOG_DBG("Not calling cyw43_poll() from poll_thread, because it doesn't exist.");
	    }
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
    //struct cyw43_dev *cyw43_device = &cyw43_0;
    
    if (result) {
        printf("ssid: %-32s rssi: %4d chan: %3d mac: %02x:%02x:%02x:%02x:%02x:%02x sec: %u\n",
            result->ssid, result->rssi, result->channel,
            result->bssid[0], result->bssid[1], result->bssid[2], result->bssid[3], result->bssid[4], result->bssid[5],
            result->auth_mode);
    }
    //TODO: return this info to Zephyr
    //LOG_DBG("calling scan_cb with interface=%s\n", cyw43_device->iface->if_dev->dev->name);
    //cyw43_device->scan_cb(cyw43_device->iface, 0, NULL);

    return 0;
}

static void cyw43_scan(struct cyw43_dev *cyw43_device, bool active)
{	

  	LOG_DBG("Scannings (%s).\n", active ? "active" : "passive");
        cyw43_lock(cyw43_device);


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
	
	LOG_DBG("calling scan_cb with interface=%s\n", cyw43_device->iface->if_dev->dev->name);
	cyw43_device->scan_cb(cyw43_device->iface, 0, NULL);

	cyw43_unlock(cyw43_device);
}


static int cyw43_connect(struct cyw43_dev *cyw43_device)
{

	LOG_DBG("Connecting to %s (pass=%s)\n", cyw43_device->sta.ssid, cyw43_device->sta.pass);
	cyw43_lock(cyw43_device);

	const uint8_t *ssid = (const uint8_t *)cyw43_device->sta.ssid;
	const uint8_t *pass = (const uint8_t *)cyw43_device->sta.pass;

	//TODO: might have to implement retry and timeout
	//if (cyw43_arch_wifi_connect_timeout_ms(cyw43_device->sta.ssid, cyw43_device->sta.pass, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
	if (cyw43_wifi_join(&cyw43_state, strlen(ssid), (const uint8_t *)ssid, pass ? strlen(pass) : 0, (const uint8_t *)pass, CYW43_AUTH_WPA2_AES_PSK, NULL, CYW43_CHANNEL_NONE)) {
	  LOG_ERR("failed to connect.\n");
	  return 1;
	} else {
	  LOG_DBG("Connected.\n");
	}

	net_if_carrier_on(cyw43_device->iface);
	LOG_DBG("Done Connecting to %s (pass=%s)\n", cyw43_device->sta.ssid, cyw43_device->sta.pass);
	cyw43_unlock(cyw43_device);
	return 0;
}

static int cyw43_disconnect(struct cyw43_dev *cyw43_device)
{
	LOG_DBG("Disconnecting from %s", cyw43_device->sta.ssid);

	cyw43_lock(cyw43_device);
	cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA); 

        if (false) { goto error; }
	LOG_DBG("Disconnected!");

	net_if_carrier_off(cyw43_device->iface);
	cyw43_unlock(cyw43_device);
	return 0;

error:
	cyw43_unlock(cyw43_device);
	return -EIO;
}



static void cyw43_request_work(struct k_work *item)
{
	struct cyw43_dev *cyw43_device;
	int err;

	cyw43_device = CONTAINER_OF(item, struct cyw43_dev, request_work);

	LOG_DBG("(req=%d)", cyw43_device->req);
	
	switch (cyw43_device->req) {
	case PICOWCYW43_REQ_CONNECT:
		err = cyw43_connect(cyw43_device);
		wifi_mgmt_raise_connect_result_event(cyw43_device->iface, err);
		break;
	case PICOWCYW43_REQ_DISCONNECT:
		err = cyw43_disconnect(cyw43_device);
		wifi_mgmt_raise_disconnect_result_event(cyw43_device->iface, err);
		break;
	case PICOWCYW43_REQ_PASSIVE_SCAN:
	        cyw43_scan(cyw43_device, false);
		break;
	case PICOWCYW43_REQ_ACTIVE_SCAN:
	        cyw43_scan(cyw43_device, true);
		break;
	case PICOWCYW43_REQ_NONE:	
	default:
		break;
	}
}


static int cyw43_send(const struct device *dev, struct net_pkt *pkt)
{
  int rv;
  struct cyw43_dev *cyw43_device = dev->data;
  
  cyw43_lock(cyw43_device);

  LOG_DBG("Calling cyw43_send()  -  packet length=%d)\n", net_pkt_get_len(pkt));
  
  // TODO: May want to do do the work in a work queue (example: eswifi_offload.c::eswifi_off_send())
  const int pkt_len = net_pkt_get_len(pkt);

  /* Read the packet payload */
  if (net_pkt_read(pkt, cyw43_device->frame_buf, pkt_len) < 0) {
    //TODO: what errishould the rv actually be? 
    rv = -99;
#if defined(CONFIG_NET_STATISTICS_WIFI)
    cyw43_device->stats.errors.tx++;
#endif

    goto out;
  }

  rv = cyw43_send_ethernet(&cyw43_state, CYW43_ITF_STA, pkt_len, (void *)(cyw43_device->frame_buf), false);

#if defined(CONFIG_NET_STATISTICS_WIFI)
	cyw43_device->stats.bytes.sent += pkt_len;
	cyw43_device->stats.pkts.tx++;
#endif

  cyw43_unlock(cyw43_device);

  // TODO: Should also add statistic counters
 out:  
  return (rv == 0 ? 0 : -EIO);
}


static void cyw43_iface_init(struct net_if *iface)
{


  struct cyw43_dev *cyw43_device = &cyw43_0;
  struct ethernet_context *eth_ctx = net_if_l2_data(iface);
  
  LOG_DBG("Calling cyw43_iface_init()\n");
  printk("Calling cyw43_iface_init()\n");

  cyw43_lock(cyw43_device);
  
  eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
  cyw43_device->iface = iface;
  
  net_if_set_link_addr(iface, cyw43_state.mac, 6, NET_LINK_ETHERNET);  
  
  ethernet_init(iface);
  net_if_carrier_off(iface);
  
  cyw43_unlock(cyw43_device);

  return;
}

int cyw43_iface_status(const struct device *dev,
			      struct wifi_iface_status *status)
{
  struct cyw43_dev *cyw43_device = &cyw43_0;
  
  LOG_DBG("Calling iface_status()\n");

  int link_status=cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
  switch (link_status) {
  case CYW43_LINK_JOIN:
    status->state = WIFI_STATE_COMPLETED;
    break;
  default:
    status->state = WIFI_STATE_DISCONNECTED;
  }


  cyw43_wifi_get_bssid(&cyw43_state, (char *) &(status->bssid[0]));

  int rssi;
  cyw43_wifi_get_rssi(&cyw43_state, (int *) &rssi);
  status->rssi = rssi;
  
  strcpy(status->ssid, (status->state == WIFI_STATE_COMPLETED ? cyw43_device->sta.ssid : ""));
  status->ssid_len = strlen(cyw43_device->sta.ssid);

  //Because right nbow, we're hard coded to CYW43_AUTH_WPA2_AES_PSK
  status->security = (status->state ==WIFI_STATE_COMPLETED ? WIFI_SECURITY_TYPE_PSK : WIFI_SECURITY_TYPE_NONE);
  
  return 0;
}

static int cyw43_mgmt_scan(const struct device *dev,
				  struct wifi_scan_params *params,
				  scan_result_cb_t cb)
{

  struct cyw43_dev *cyw43_device = dev->data;
  
  LOG_DBG("Calling mgmt_scan()");
    
  cyw43_lock(cyw43_device);

  cyw43_device->scan_cb = cb;

  cyw43_device->req = (params->scan_type == WIFI_SCAN_TYPE_ACTIVE ?
		       PICOWCYW43_REQ_ACTIVE_SCAN :
		       PICOWCYW43_REQ_PASSIVE_SCAN);

  k_work_submit_to_queue(&cyw43_device->work_q, &cyw43_device->request_work);
  
  cyw43_unlock(cyw43_device);
  
  return 0;
}

static int __cyw43_dev_deconfig(struct cyw43_dev *cyw43_device,
                                       struct wifi_connect_req_params *params)
{
    memcpy(cyw43_device->sta.ssid, params->ssid, params->ssid_length);
    cyw43_device->sta.ssid[params->ssid_length] = '\0';

    switch (params->security) {
    case WIFI_SECURITY_TYPE_NONE:
        cyw43_device->sta.pass[0] = '\0';
        cyw43_device->sta.security = PICOWCYW43_SEC_OPEN;
        break;
    case WIFI_SECURITY_TYPE_PSK:
        memcpy(cyw43_device->sta.pass, params->psk, params->psk_length);
        cyw43_device->sta.pass[params->psk_length] = '\0';
        cyw43_device->sta.security = PICOWCYW43_SEC_WPA2_MIXED;
        break;
    default:
        return -EINVAL;
    }

    if (params->channel == WIFI_CHANNEL_ANY) {
        cyw43_device->sta.channel = 0U;
    } else {
        cyw43_device->sta.channel = params->channel;
    }

    return 0;
}

static int cyw43_mgmt_connect(const struct device *dev,
				     struct wifi_connect_req_params *params)
{
  struct cyw43_dev *cyw43_device = dev->data;
  int err;
  LOG_DBG("");
  cyw43_lock(cyw43_device);
  err = __cyw43_dev_deconfig(cyw43_device, params);
  if (!err) {
    cyw43_device->req = PICOWCYW43_REQ_CONNECT;
    k_work_submit_to_queue(&cyw43_device->work_q, &cyw43_device->request_work);
  }
  cyw43_unlock(cyw43_device);
  return 0;
}

static int cyw43_mgmt_disconnect(const struct device *dev)
{
  struct cyw43_dev *cyw43_device = dev->data;
  LOG_DBG("");
  cyw43_lock(cyw43_device);
  cyw43_device->req = PICOWCYW43_REQ_DISCONNECT;
  k_work_submit_to_queue(&cyw43_device->work_q, &cyw43_device->request_work);
  cyw43_unlock(cyw43_device);
  return 0;
}


static int cyw43_mgmt_ap_enable(const struct device *dev,
				       struct wifi_connect_req_params *params)
{
  struct cyw43_dev *cyw43_device = dev->data;
  
  LOG_DBG("Calling mgmt_ap_enable()\n");

  cyw43_lock(cyw43_device);

  cyw43_unlock(cyw43_device);
  return 0;    
}

static int cyw43_mgmt_ap_disable(const struct device *dev)
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
  struct cyw43_dev *cyw43_device = &cyw43_0;

  LOG_DBG("Calling cyw43_cb_process_ethernet(itf=%d, len=%d)\n", itf, len);

  if (cyw43_device->iface == NULL) {
    LOG_ERR("network interface unavailable");
    return;
  }

  pkt = net_pkt_rx_alloc_with_buffer(cyw43_device->iface, len, AF_UNSPEC, 0, K_MSEC(100));

  if (!pkt) {
    LOG_ERR("Failed to get net buffer");
    goto pkt_processing_succeeded;
  }

  if (net_pkt_write(pkt, buf, len) < 0) {
    LOG_ERR("Failed to write pkt");
    goto pkt_processing_failed;
  }

  if (net_recv_data(cyw43_device->iface, pkt) < 0) {
    LOG_ERR("Failed to push received data");
    goto pkt_processing_failed;
  }

#if defined(CONFIG_NET_STATISTICS_WIFI)
	cyw43_device->stats.bytes.received += len;
	cyw43_device->stats.pkts.rx++;
#endif
   
  // TODO: Add statistic counters
  
  // TODO: Add statistic counters
  goto pkt_processing_succeeded;

 pkt_processing_failed:
#if defined(CONFIG_NET_STATISTICS_WIFI)
  cyw43_device->stats.errors.rx++;
#endif
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
  struct net_if *iface;
  LOG_DBG("Calling cyw43_cb_tcpip_set_link_up(itf=%d)", itf);

#if defined(CONFIG_NET_DHCPV4)    
  iface = net_if_get_default();
  net_dhcpv4_start(iface);
#endif  
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

void cyw43_thread_enter(void)
{
  LOG_DBG("Calling cyw43_thread_enter()");
  cyw43_lock(&cyw43_0);
}

void cyw43_thread_exit(void)
{
  LOG_DBG("Calling cyw43_thread_exit()");
  cyw43_unlock(&cyw43_0);
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

//#define ISR_EVENT_PROCESSING
#ifdef ISR_EVENT_PROCESSING
static void cyw43_set_irq_enabled(bool enabled)
{
  //LOG_DBG("Calling cyw43_set_irq_enabled(%s)", (enabled ? "true" : "false"));

  gpio_pin_interrupt_configure(cyw43_cfg.irq_gpio.port, PICOWCYW43_GPIO_INTERRUPT_PIN, (enabled?(GPIO_INT_ENABLE|GPIO_INT_LEVEL_HIGH):GPIO_INT_DISABLE));
}
#endif

void cyw43_post_poll_hook(void)
{
#ifdef ISR_EVENT_PROCESSING  
  cyw43_set_irq_enabled(true);
#endif
}


#ifdef ISR_EVENT_PROCESSING
struct gpio_callback cyw43_gpio_cb;

static void cyw43_isr(const struct device *port,
			    struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
  //LOG_DBG("Calling cyw43_isr()");

  // TODO: Should really find the Zephyr equivalent of gpio_get_irq_event_mask
  uint32_t events = gpio_get_irq_event_mask(PICOWCYW43_GPIO_INTERRUPT_PIN);
  if (events & GPIO_IRQ_LEVEL_HIGH) { 
    //cyw43_set_irq_enabled(false);
    cyw43_set_irq_enabled(true);
  }
  
  //Schedule the rest of the work to be done
  if (cyw43_poll) {
    //LOG_DBG("Calling cyw43_poll() from cyw43_isr");
    cyw43_poll();
  }
  else {
    //LOG_DBG("Not calling cyw43_poll() from cyw43_isr, because it doesn't exist.");
  }
}

static void cyw43_register_cb()
{
  int rv;
 
  //Note that this is technically a gpio_callback, not a true isr
  gpio_init_callback(&cyw43_gpio_cb,
		     cyw43_isr,
		     BIT(cyw43_cfg.irq_gpio.pin));

  rv = gpio_add_callback(cyw43_cfg.irq_gpio.port, &cyw43_gpio_cb);

  LOG_DBG("gpio_add_callback() returned %d", rv);
  
  cyw43_set_irq_enabled(true);
}
#endif

#if defined(CONFIG_NET_STATISTICS_WIFI)
static int cyw43_wifi_stats(const struct device *dev, struct net_stats_wifi *stats)
{
	struct cyw43_dev *cyw43_device = dev->data;
	
	stats->bytes.received = cyw43_device->stats.bytes.received;
	stats->bytes.sent = cyw43_device->stats.bytes.sent;
	stats->pkts.rx = cyw43_device->stats.pkts.rx;
	stats->pkts.tx = cyw43_device->stats.pkts.tx;
	stats->errors.rx = cyw43_device->stats.errors.rx;
	stats->errors.tx = cyw43_device->stats.errors.tx;
	stats->broadcast.rx = cyw43_device->stats.broadcast.rx;
	stats->broadcast.tx = cyw43_device->stats.broadcast.tx;
	stats->multicast.rx = cyw43_device->stats.multicast.rx;
	stats->multicast.tx = cyw43_device->stats.multicast.tx;
	stats->sta_mgmt.beacons_rx = cyw43_device->stats.sta_mgmt.beacons_rx;
	stats->sta_mgmt.beacons_miss = cyw43_device->stats.sta_mgmt.beacons_miss;

	return 0;
}
#endif

static int cyw43_device_init(const struct device *dev)
{
    struct cyw43_dev *cyw43_device = dev->data;
  
    LOG_DBG("");

    k_mutex_init(&cyw43_device->mutex);

    cyw43_device->role = PICOWCYW43_ROLE_CLIENT;
    

    cyw43_init(&cyw43_state);

#ifdef ISR_EVENT_PROCESSING
    // Based on example in ./pico-sdk/src/rp2_common/pico_cyw43_driver/cyw43_driver.c:cyw43_driver_init() IRQ setup happens next

    cyw43_register_cb();

    cyw43_set_irq_enabled(true);
#endif
    
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);


    k_work_queue_start(&cyw43_device->work_q, cyw43_work_q_stack,
		       K_KERNEL_STACK_SIZEOF(cyw43_work_q_stack),
		       CONFIG_SYSTEM_WORKQUEUE_PRIORITY - 1, NULL);
   
    k_work_init(&cyw43_device->request_work, cyw43_request_work);

#ifdef POLLING_THREAD
    k_thread_create(&event_thread, cyw43_event_poll_stack,
		    EVENT_POLL_THREAD_STACK_SIZE,
		    (k_thread_entry_t)cyw43_event_poll_thread, (void *)dev, NULL,
		    NULL, K_PRIO_COOP(EVENT_POLL_THREAD_PRIO), 0,
		    K_NO_WAIT);

#endif
    
    //wifi_set_led(true);
    
    cyw43_shell_register(cyw43_device);
    
    return 0;
  
}

static const struct wifi_mgmt_ops cyw43_mgmt_api = {
	.scan			   = cyw43_mgmt_scan,
	.connect		   = cyw43_mgmt_connect,
	.disconnect		   = cyw43_mgmt_disconnect,
	.ap_enable		   = cyw43_mgmt_ap_enable,
	.ap_disable		   = cyw43_mgmt_ap_disable,
	.iface_status		   = cyw43_iface_status,
#if defined(CONFIG_NET_STATISTICS_WIFI)
	.get_stats	           = cyw43_wifi_stats,
#endif	
};

static const struct net_wifi_mgmt_offload cyw43_callbacks = {
	.wifi_iface.iface_api.init = cyw43_iface_init,
	.wifi_iface.send           = cyw43_send,
	.wifi_mgmt_api             = &cyw43_mgmt_api,
};

NET_DEVICE_DT_INST_DEFINE(0,
			  cyw43_device_init, NULL,
			  &cyw43_0, &cyw43_cfg, CONFIG_WIFI_INIT_PRIORITY,
			  &cyw43_callbacks, ETHERNET_L2,
			  NET_L2_GET_CTX_TYPE(ETHERNET_L2), NET_ETH_MTU);
