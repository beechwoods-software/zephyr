/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#include "pico_w_cyw43_drv.h"

#if defined(CONFIG_BUILD_WITH_PICOWI)
#include "picowi/picowi_evtnum.h"
#include "picowi/picowi_ioctl.h"
#include "picowi/picowi_defs.h"
#include "picowi/picowi_pio.h"
#include "picowi/picowi_wifi.h"
#include "picowi/picowi_init.h"
#include "picowi/picowi_pico.h"
#include "picowi/picowi_scan.h"
#include "picowi/picowi_join.h"

#else
#include "georgerobotics/cyw43.h"
#include "georgerobotics/cyw43_country.h"
#define CYW43_GPIO_IRQ_HANDLER_PRIORITY 0x40
#endif // CONFIG_BUILD_WITH_PICOWI

#define DT_DRV_COMPAT pico_w_cyw43

#include "pico_w_cyw43_log.h"
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define PICOWCYW43_WORKQUEUE_STACK_SIZE 1024
K_KERNEL_STACK_DEFINE(pico_w_cyw43_work_q_stack, PICOWCYW43_WORKQUEUE_STACK_SIZE);

static const struct pico_w_cyw43_cfg pico_w_cyw43_cfg = {
      	.data = NULL,
        .clock = NULL,
	.power = NULL,
	.irq_gpio = NULL//= GPIO_DT_SPEC_INST_GET(0, mosi_gpios),
};


#define EVENT_POLL_THREAD_STACK_SIZE 1024
#define EVENT_POLL_THREAD_PRIO 2
K_KERNEL_STACK_MEMBER(pico_w_cyw43_event_poll_stack, EVENT_POLL_THREAD_STACK_SIZE);

struct k_thread event_thread;

static struct pico_w_cyw43_dev pico_w_cyw43_0; /* static instance */

static void pico_w_cyw43_event_poll_thread(void *p1)
{
	struct pico_w_cyw43_dev *pico_w_cyw43 = p1;

	uint32_t led_ticks, poll_ticks;

	
	printf("Starting pico_w_cyw43_event_poll_thread\n");

	k_thread_name_set(NULL, "pico_w_cyw43_event_poll_thread");
	
	while (1) {
	  //if (wifi_get_irq() || mstimeout(&poll_ticks, 10)) {
	    pico_w_cyw43_lock(pico_w_cyw43);
#if defined(CONFIG_BUILD_WITH_PICOWI)	    
	    if (event_poll() < 0) {
	      printf("event_poll() returns < 0\n");
	      //printf("Total time %lu msec\n", ustime()/1000);
	    }
	    //}
#else
	    if (cyw43_poll) {
	      cyw43_poll();
	    }
#endif // CONFIG_BUILD_WITH_PICOWI
	    pico_w_cyw43_unlock(pico_w_cyw43);
	    k_sleep(K_MSEC(10));
	}
	
}


static void pico_w_cyw43_scan(struct pico_w_cyw43_dev *pico_w_cyw43)
{
	char *data;
	int i, ret;

	uint32_t led_ticks, poll_ticks;
	bool ledon =false;

	
	LOG_DBG("");

	pico_w_cyw43_lock(pico_w_cyw43);
	LOG_DBG("Scanning for wifi networks.\n");
	printf("Scanning for wifi networks.\n");

#if defined(CONFIG_BUILD_WITH_PICOWI)	
	if (!scan_start()) {
	  printf("Error: can't start scan\n");
	  pico_w_cyw43->scan_cb(pico_w_cyw43->iface, -EIO, NULL);
	}
	wifi_set_led(true);	
#else
	printf ("George Robotics wifi scan not implemented\n");
#endif // CONFIG_BUILD_WITH_PICOWI

	/* WiFi scan is done. */
	
	printf("calling scan_cb with interface=%s\n", pico_w_cyw43->iface->if_dev->dev->name);
	pico_w_cyw43->scan_cb(pico_w_cyw43->iface, 0, NULL);

	pico_w_cyw43_unlock(pico_w_cyw43);
}


static int pico_w_cyw43_connect(struct pico_w_cyw43_dev *pico_w_cyw43)
{
    uint32_t led_ticks, poll_ticks;
    bool ledon=false;

	printf("Connecting to %s (pass=%s)\n", pico_w_cyw43->sta.ssid, pico_w_cyw43->sta.pass);
	pico_w_cyw43_lock(pico_w_cyw43);
#if defined(CONFIG_BUILD_WITH_PICOWI)    
    add_event_handler(my_join_event_handler);
    add_event_handler(join_event_handler);

    printf("io_init()\n");
    io_init();
    usdelay(1000);
    printf("join_start(%s, %s);\n", pico_w_cyw43->sta.ssid, pico_w_cyw43->sta.pass);
    if (!join_start(pico_w_cyw43->sta.ssid, pico_w_cyw43->sta.pass))
        printf("Error: can't start network join\n");
    else {
        while (1) {
            // Toggle LED at 1 Hz if joined, 5 Hz if not
            if (mstimeout(&led_ticks, 50))
            {
                // printf("wifi_set_led\n");
                wifi_set_led(ledon = !ledon);
            }

            // Get any events, poll the joining state machine
            if (wifi_get_irq() || mstimeout(&poll_ticks, 10)) {
                // printf("+++ EVENT \n");
                if (event_poll() < 0) break;
                join_state_poll(pico_w_cyw43->sta.ssid, pico_w_cyw43->sta.pass);
                mstimeout(&poll_ticks, 0);
            }
        }
    }
#else // CONFIG_BUILD_WITH_PICOWI
    printf("GR _connect function not implemented\n");
#endif
	printf("Done Connecting to %s (pass=%s)\n", pico_w_cyw43->sta.ssid, pico_w_cyw43->sta.pass);
	pico_w_cyw43_unlock(pico_w_cyw43);
}

static int pico_w_cyw43_disconnect(struct pico_w_cyw43_dev *pico_w_cyw43)
{
	struct in_addr addr;
	int err;

	LOG_DBG("Disconnecting from %s", pico_w_cyw43->sta.ssid);

	pico_w_cyw43_lock(pico_w_cyw43);
#if defined(CONFIG_BUILD_WITH_PICOWI)
	join_stop();
#endif // CONFIG_BUILD_WITH_PICOWI
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

#if defined(CONFIG_BUILD_WITH_PICOWI)
	printf("picowi implementation of _status_work not implemented\n");
#else // CONFIG_BUILD_WITH_PICOWI
	printf("GR implementation of _status_work not implemented\n");
#endif // CONFIG_BUILD_WITH_PICOWI
	
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


  struct pico_w_cyw43_dev *pico_w_cyw43 = &pico_w_cyw43_0;
  
  LOG_DBG("");
  printf("Calling pico_w_cyw43_iface_init()\n");

  pico_w_cyw43_lock(pico_w_cyw43);

  pico_w_cyw43->iface = iface;

  pico_w_cyw43_unlock(pico_w_cyw43);

  return;
}

int pico_w_cyw43_iface_status(const struct device *dev,
			      struct wifi_iface_status *status)
{
  LOG_DBG("");
  printf("Calling iface_status()\n");
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

static int __pico_w_cyw43_dev_deconfig(struct pico_w_cyw43_dev *pico_w_cyw43,
                                       struct wifi_connect_req_params *params)
{
  //printf("pre - memcpy 1\n");
    memcpy(pico_w_cyw43->sta.ssid, params->ssid, params->ssid_length);
    //printf("post - memcpy 1\n");
    pico_w_cyw43->sta.ssid[params->ssid_length] = '\0';

    switch (params->security) {
    case WIFI_SECURITY_TYPE_NONE:
        pico_w_cyw43->sta.pass[0] = '\0';
        pico_w_cyw43->sta.security = PICOWCYW43_SEC_OPEN;
        break;
    case WIFI_SECURITY_TYPE_PSK:
      //printf("pre - memcpy 2\n");
        memcpy(pico_w_cyw43->sta.pass, params->psk, params->psk_length);
	//printf("post - memcpy 2\n");
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
  
  LOG_DBG("");
  printf("Calling mgmt_ap_enable()\n");

  pico_w_cyw43_lock(pico_w_cyw43);

  pico_w_cyw43_unlock(pico_w_cyw43);
  return 0;    
}

static int pico_w_cyw43_mgmt_ap_disable(const struct device *dev)
{
  LOG_DBG("");
  printf("Calling mgmt_ap_disable()\n");
  return 0;
}

#if defined(CONFIG_BUILD_WITH_PICOWI)

#else

void cyw43_cb_tcpip_deinit(cyw43_t *self, int itf)
{
  printf("Calling cyw43_cb_tcpip_deinit(itf=%d)\n", itf);
  return;
}

void cyw43_cb_process_ethernet(void *cb_data, int itf, size_t len, const uint8_t *buf)
{
  printf("Calling cyw43_cb_process_ethernet(itf=%d)\n");
  return;
}

void cyw43_hal_generate_laa_mac(int idx, uint8_t buf[6])
{
  printf("cyw43_hal_generate_laa_mac(idx=%d)\n");
  return;
}

void cyw43_cb_tcpip_init(cyw43_t *self, int itf)
{
  printf("cyw43_cb_tcpip_init(itf=%d)\n");
  return;
}

void cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf)
{
  printf("Calling cyw43_cb_tcpip_set_link_up(itf=%d)\n");
  return;
}

void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf)
{
  printf("Calling cyw43_cb_tcpip_set_link_down(itf=%d)\n");
  return;
}

void cyw43_schedule_internal_poll_dispatch(void (*func)(void)) {
  printf("Calling cyw43_schedule_internal_poll_dispatch()\n");
  assert(func == cyw43_poll);
  return;
}

static void cyw43_set_irq_enabled(bool enabled)
{
  printf("Calling cyw43_set_irq_enabled(%s)\n", (enabled ? "true" : "false"));
  // ******   Probably should use the Zephyr rather than Pico SDK version of this *******//
  gpio_set_irq_enabled(CYW43_PIN_WL_HOST_WAKE, GPIO_IRQ_LEVEL_HIGH, enabled);
}

#if 0
static void cyw43_gpio_irq_handler(void)
{
    uint32_t events = gpio_get_irq_event_mask(CYW43_PIN_WL_HOST_WAKE);
    if (events & GPIO_IRQ_LEVEL_HIGH) {
        // As we use a high level interrupt, it will go off forever until it's serviced
        // So disable the interrupt until this is done. It's re-enabled again by CYW43_POST_POLL_HOOK
        // which is called at the end of cyw43_poll_func
        cyw43_set_irq_enabled(false);
        //async_context_set_work_pending(cyw43_async_context, &cyw43_poll_worker);
    }
}
#endif

void cyw43_post_poll_hook(void)
{
  printf("Calling cyw43_post_poll_hook()\n");
  cyw43_set_irq_enabled(true);
}

void cyw43_thread_enter(void)
{
  printf("Calling cyw43_thread_enter()\n");
  pico_w_cyw43_lock(&pico_w_cyw43_0);
}

void cyw43_thread_exit(void)
{
  printf("Calling cyw43_thread_exit()\n");
  pico_w_cyw43_unlock(&pico_w_cyw43_0);
}

void cyw43_delay_ms(uint32_t ms) {
  k_sleep(K_MSEC(ms));
}

void sleep_ms(uint32_t ms) {
  k_sleep(K_MSEC(ms));
}

void cyw43_delay_us(uint32_t us) {
  k_sleep(K_MSEC(us*1000));
}

void cyw43_await_background_or_timeout_us(uint32_t timeout_us) {
  k_sleep(K_MSEC(timeout_us*1000));
}

#endif // CONFIG_BUILD_WITH_PICOWI

struct gpio_callback pico_w_cyw43_gpio_cb;

static int pico_w_cyw43_isr(const struct device *port,
			    struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
  printf("Calling pico_w_cyw43_isr\n");
}

static void pico_w_cyw43_register_isr(const struct device *port)
{
  int rv;
  
  //Note that this is technically a gpio_callback, not a true isr
  gpio_init_callback(&pico_w_cyw43_gpio_cb,
		     pico_w_cyw43_isr,
		     BIT(24/*pico_w_cyw43_cfg.irq_gpio.pin*/));

  rv = gpio_add_callback(port/*pico_w_cyw43_cfg.irq_gpio.port*/, &pico_w_cyw43_gpio_cb);

  printf("gpio_add_callback() returned %d\n", rv);
  
  rv = gpio_pin_interrupt_configure(port/*pico_w_cyw43_cfg.irq_gpio.port*/, 24, GPIO_INT_EDGE_TO_ACTIVE);

  printf("gpio_pin_interrupt_configure(() returned %d\n", rv);
}


static int pico_w_cyw43_init(const struct device *dev)
{
  struct pico_w_cyw43_dev *pico_w_cyw43 = dev->data;
  const struct pico_w_cyw43_cfg *cfg = dev->config;

  
  LOG_DBG("");

  k_mutex_init(&pico_w_cyw43->mutex);

  //picw_w_cyw43_print_tasks();
  
  pico_w_cyw43->role = PICOWCYW43_ROLE_CLIENT;
    
  //k_msleep(1000);
  //k_msleep(10000);
  
#if defined(CONFIG_BUILD_WITH_PICOWI)  
  add_event_handler(scan_event_handler);

  if (!wifi_setup()) {
    LOG_DBG("Error: SPI communication\n");
    return -ENODEV;
  }
  else if (!wifi_init()) {
    LOG_DBG("Error: can't initialise WiFi\n");
    return -ENODEV;
  }
  printf("Made it through wifi_setup()\n");
#else
    /* event handling thread */

  cyw43_init(&cyw43_state);
  // Based on example in ./pico-sdk/src/rp2_common/pico_cyw43_driver/cyw43_driver.c:cyw43_driver_init() IRQ setup happens next
  
  pico_w_cyw43_register_isr(dev);

  /* If I try to enable irq this way, everything hangs (maybe because I'm using the rpi SDK
  // gpio_set_irq_enabled() routine in cyw43_set_irq_enabled() rather than Zephyr GPIO API? */
  // cyw43_set_irq_enabled(true);

  
#if 0
    k_thread_create(&event_thread, pico_w_cyw43_event_poll_stack,
		    EVENT_POLL_THREAD_STACK_SIZE,
		    (k_thread_entry_t)pico_w_cyw43_event_poll_thread, pico_w_cyw43, NULL,
		    NULL, K_PRIO_COOP(EVENT_POLL_THREAD_PRIO), 0,
		    K_NO_WAIT);
#endif    

  cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);
  return 0;

#endif // CONFIG_BUILD_WITH_PICOWI
  
    //picw_w_cyw43_print_tasks();
    
    k_work_queue_start(&pico_w_cyw43->work_q, pico_w_cyw43_work_q_stack,
		       K_KERNEL_STACK_SIZEOF(pico_w_cyw43_work_q_stack),
		       CONFIG_SYSTEM_WORKQUEUE_PRIORITY - 1, NULL);
    
    k_work_init(&pico_w_cyw43->request_work, pico_w_cyw43_request_work);
    k_work_init_delayable(&pico_w_cyw43->status_work, pico_w_cyw43_status_work);

    //k_sleep(K_MSEC(100));
    //wifi_set_led(true);
    
    pico_w_cyw43_shell_register(pico_w_cyw43);
    
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

#if 1
NET_DEVICE_OFFLOAD_INIT(pico_w_cyw43_0, "pico_w_cyw43", pico_w_cyw43_init, NULL,
			&pico_w_cyw43_0, NULL,
			CONFIG_WIFI_INIT_PRIORITY,
			&pico_w_cyw43_callbacks,
			1500);
#else
NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, pico_w_cyw43_init, NULL,
				  &pico_w_cyw43_0, &pico_w_cyw43_cfg,
				  CONFIG_WIFI_INIT_PRIORITY,
				  &pico_w_cyw43_callbacks,
				  1500);
#endif



