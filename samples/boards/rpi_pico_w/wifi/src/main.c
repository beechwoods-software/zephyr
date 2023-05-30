/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_if.h>


void wifi_scan(void) {

  struct wifi_raw_scan_result scan_result;

  struct net_if *interface;

  interface = net_if_get_default();

  printf("Default interface is %s\n", interface->if_dev->dev->name);
  
  net_mgmt(NET_REQUEST_WIFI_SCAN, interface, NULL, 0);
  printf("Finished calling net_mgmt(NET_REQUEST_WIFI_SCAN)\n");

  
}

void wifi_connect(void) {
    int ret;
  printf("\n\ncalling net_mgmt(NET_REQUEST_WIFI_CONNECT,)\n");
    // const uint8_t *ssid = (uint8_t*)"cyrus";
    struct net_if *iface = net_if_get_default();
    printf("Default interface is %s\n", iface->if_dev->dev->name);
    static struct wifi_connect_req_params req_params = {
        .ssid = "candance",
        .ssid_length = 5,
        .psk = "6173470125",
        .psk_length = 10,
        .channel = 0,
        .security = WIFI_SECURITY_TYPE_PSK,
    };

    ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &req_params, sizeof(struct wifi_connect_req_params));
  printf("%d Finished calling net_mgmt(NET_REQUEST_WIFI_SCAN)\n", ret);
}

int main(void)
{
	/* NET_CONFIG_SETTINGS will init DHCP
	 * NET_SHELL is enabled to test driver and network stack
	 */

  // wifi_scan();
  // wifi_connect(ssid, password);
  wifi_connect();
  return 0;
}
