/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_config.h>
#include <zephyr/drivers/led.h>

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

  if (strcmp(CONFIG_WIFI_SSID, "") && strcmp(CONFIG_WIFI_PSK, "")) {
    printf("Connecting to network \"%s\"\n", CONFIG_WIFI_SSID);
    wifi_connect(CONFIG_WIFI_SSID, CONFIG_WIFI_PSK);
  }
  else {
    printf("To configure wifi, either:\n");
    printf("\n");
    printf("  1. Run 'wifi connect \"<SSID>\" \"<passphrase>\"' on the command line.\n");
    printf("\n");
    printf("                        - or -\n");
    printf("\n");
    printf("  2. Define CONFIG_WIFI_SSID and CONFIG_WIFI_PSK in local.conf and rebuild.\n");
    printf("\n");
  }
  (void)net_config_init_app(NULL, "Initializing network");
  struct net_if *iface = net_if_get_default();
  if (NULL == iface) printf("NULL iface\n");
  const struct device *const wifidev = DEVICE_DT_GET_ANY(infineon_cyw43);
  if (NULL == wifidev) printf("NULL wifidev\n");
  else printf("+++ wifidev\n");
  const struct device *const led_dev = DEVICE_DT_GET_ANY(infineon_cyw43_led);
  if (NULL == led_dev) printf("NULL led_dev\n");
  else printf("+++ led_dev\n");
  int i = 0;
  while (i < 9)
  {
    if (i % 2 == 0) {
        printf("led_on\n");
        led_on(led_dev, 0);
    }
    else {
        printf("led_off\n");
        led_off(led_dev, 1);
    }
    sleep(2);
    i++;
  }
  printf("++++ DONE!\n");
  return 0;
}
