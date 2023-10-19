/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>

int main(void)
{
  const struct device *const gpio_dev = DEVICE_DT_GET_ANY(infineon_cyw43_gpio);
  if (NULL == gpio_dev) printf("NULL gpio_dev\n");
  else printf("+++ gpio_dev\n");
  int i = 0;
  while (i < 10)
  {
    if (i % 2 == 0) {
        printf("led_on\n");
        // led_on(gpio_dev, 0);
        gpio_pin_set(gpio_dev, 0, 1);
    }
    else {
        printf("led_off\n");
        // led_off(gpio_dev, 1);
        gpio_pin_set(gpio_dev, 0, 0);
    }
    sleep(2);
    i++;
  }
  printf("Setting pin 1, return value is: %d\n", gpio_pin_set(gpio_dev, 1, 1));
  printf("Setting pin 2, return value is: %d\n", gpio_pin_set(gpio_dev, 2, 1));
  printf("Setting pin 3, return value is: %d\n", gpio_pin_set(gpio_dev, 3, 1));
  gpio_port_value_t value;
  printf("+++ gpio_port_get\n");
  gpio_port_get(gpio_dev, &value);
  printf("+++ gpio_port_set_masked_raw\n");
  gpio_port_set_masked_raw(gpio_dev, 0xf, 1);
  printf("++++ DONE!\n");
  return 0;
}
