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

int main(void)
{
	/* NET_CONFIG_SETTINGS will init DHCP
	 * NET_SHELL is enabled to test driver and network stack
	 */

  wifi_scan();
  return 0;
}
