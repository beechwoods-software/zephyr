/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

int main(void)
{
  printf("Do nothing");
/*
  const struct device *const led = DEVICE_DT_GET_ANY(cyw43_led);
  if (led == NULL) printf("NULL LED\n");
  if (!device_is_ready(led)) {
    printf("LED device not ready\n");
    return 0;
  }
  else printf("LED ready\n");
  // led_on(led, 0);
*/
  return 0;
}
