/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include <zephyr/drivers/led.h>

int main(void)
{
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
