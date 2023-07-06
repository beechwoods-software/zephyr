/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_if.h>

#define WIFI_MGMT_EVENTS (NET_EVENT_WIFI_SCAN_RESULT |		\
			  NET_EVENT_WIFI_SCAN_DONE |			\
			  NET_EVENT_WIFI_CONNECT_RESULT |		\
			  NET_EVENT_WIFI_DISCONNECT_RESULT |		\
			  NET_EVENT_WIFI_TWT |				\
			  NET_EVENT_WIFI_RAW_SCAN_RESULT)
#if 0
static uint32_t scan_result;

static struct net_mgmt_event_callback sample_program_mgmt_cb;

static void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
  printf("Calling handle_wifi_scan_done(struct net_mgmt_event_callback *cb)");
}

static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
  const struct wifi_scan_result *entry =
    (const struct wifi_scan_result *)cb->info;
  uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];

  scan_result++;

  printf("Calling handle_wifi_scan_result(struct net_mgmt_event_callback *cb)");

  if (scan_result == 1U) {
    printf("\n%-4s | %-32s %-5s | %-13s | %-4s | %-15s | %-17s | %-8s\n",
	  "Num", "SSID", "(len)", "Chan (Band)", "RSSI", "Security", "BSSID", "MFP");
  }
  
  printf("%-4d | %-32s %-5u | %-4u (%-6s) | %-4d | %-15s | %-17s | %-8s\n",
	scan_result, entry->ssid, entry->ssid_length, entry->channel,
	wifi_band_txt(entry->band),
	entry->rssi,
	wifi_security_txt(entry->security),
	((entry->mac_length) ?
	 net_sprint_ll_addr_buf(entry->mac, WIFI_MAC_ADDR_LEN,
				mac_string_buf,
				sizeof(mac_string_buf)) : ""),
	wifi_mfp_txt(entry->mfp));
}

static void handle_wifi_raw_scan_result(struct net_mgmt_event_callback *cb)
{
  printf("Calling handle_wifi_raw_scan_result(struct net_mgmt_event_callback *cb)");
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint32_t mgmt_event, struct net_if *iface)
{
  printf("wifi_mgmt_event_handler handling event %d\n", mgmt_event);
	switch (mgmt_event) {
	case NET_EVENT_WIFI_SCAN_RESULT:
		handle_wifi_scan_result(cb);
		break;
	case NET_EVENT_WIFI_SCAN_DONE:
	        handle_wifi_scan_done(cb);
		break;
	case NET_EVENT_WIFI_CONNECT_RESULT:
	  //	handle_wifi_connect_result(cb);
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
	  //	handle_wifi_disconnect_result(cb);
		break;
	case NET_EVENT_WIFI_TWT:
	  //	handle_wifi_twt_event(cb);
		break;
#ifdef CONFIG_WIFI_MGMT_RAW_SCAN_RESULTS
	case NET_EVENT_WIFI_RAW_SCAN_RESULT:
	        handle_wifi_raw_scan_result(cb);
		break;
#endif /* CONFIG_WIFI_MGMT_RAW_SCAN_RESULTS */
	default:
		break;
	}
}
#endif

void wifi_scan(void) {

  //struct wifi_raw_scan_result scan_result;

  struct net_if *interface;

  interface = net_if_get_default();

  printf("Default interface is %s\n", interface->if_dev->dev->name);
  
  if (net_mgmt(NET_REQUEST_WIFI_SCAN, interface, NULL, 0)) {
    printf("Scan request failed\n");
  }
  
  printf("Finished calling net_mgmt(NET_REQUEST_WIFI_SCAN)\n");

  
}

void wifi_connect(char *ssid, char * passwd) {
    int ret;
    
    struct net_if *iface = net_if_get_default();
    printf("Default interface is %s\n", iface->if_dev->dev->name);
    static struct wifi_connect_req_params req_params = {
        .channel = 0,
        .security = WIFI_SECURITY_TYPE_PSK,
    };
    req_params.ssid = ssid;
    req_params.ssid_length = strlen(ssid);
    req_params.psk = passwd;
    req_params.psk_length = strlen(passwd);
    printf("\n\ncalling net_mgmt(NET_REQUEST_WIFI_CONNECT)\n");
    ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &req_params, sizeof(struct wifi_connect_req_params));
    printf("%d Finished calling net_mgmt(NET_REQUEST_WIFI_SCAN)\n", ret);
}

int main(void)
{
	/* NET_CONFIG_SETTINGS will init DHCP
	 * NET_SHELL is enabled to test driver and network stack
	 */

#if 0  
  net_mgmt_init_event_callback(&sample_program_mgmt_cb,
			       wifi_mgmt_event_handler,
			       WIFI_MGMT_EVENTS);

    net_mgmt_add_event_callback(&sample_program_mgmt_cb);


    printf("finished registering event callbacks.\n");

    //usleep(10000000);
    sleep(10);
    wifi_scan();
    //wifi_connect("ClarkeBelt_2Ghz", "TAKAPG43");

#else
  printf("Doing nothing\n");
#endif

  
  return 0;
}
