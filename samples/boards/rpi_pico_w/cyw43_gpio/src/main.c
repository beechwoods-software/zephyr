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
  gpio_port_value_t value = 0;
  gpio_port_clear_bits(gpio_dev, 7);
  gpio_port_get(gpio_dev, &value);
  printf("+++ gpio_port_get -> 0x%x\n", value);
  gpio_port_get_raw(gpio_dev, &value);
  printf("+++ gpio_port_get_raw -> 0x%x\n\n", value);

  printf("+++ Setting pin 0\n");
  gpio_pin_set(gpio_dev, 0, 1);
  gpio_port_get(gpio_dev, &value);
  printf("+++ gpio_port_get -> 0x%x\n", value);
  gpio_port_get_raw(gpio_dev, &value);
  printf("+++ gpio_port_get_raw -> 0x%x\n\n", value);

  printf("+++ Setting pin 1\n");
  gpio_pin_set(gpio_dev, 1, 1);
  gpio_port_get(gpio_dev, &value);
  printf("+++ gpio_port_get -> 0x%x\n", value);
  gpio_port_get_raw(gpio_dev, &value);
  printf("+++ gpio_port_get_raw -> 0x%x\n\n", value);

  printf("+++ Setting pin 2\n");
  gpio_pin_set(gpio_dev, 2, 1);
  gpio_port_get(gpio_dev, &value);
  printf("+++ gpio_port_get -> 0x%x\n", value);
  gpio_port_get_raw(gpio_dev, &value);
  printf("+++ gpio_port_get_raw -> 0x%x\n\n", value);

  printf("+++ Setting pin 3\n");
  gpio_pin_set(gpio_dev, 3, 1);
  gpio_port_get(gpio_dev, &value);
  printf("+++ gpio_port_get -> 0x%x\n", value);
  gpio_port_get_raw(gpio_dev, &value);
  printf("+++ gpio_port_get_raw -> 0x%x\n\n", value);

  printf("+++ gpio_port_set_masked_raw(gpio_dev, 0xf, 0);\n");
  gpio_port_set_masked_raw(gpio_dev, 0xf, 0);
  gpio_port_get(gpio_dev, &value);
  printf("+++ gpio_port_get -> 0x%x\n", value);
  gpio_port_get_raw(gpio_dev, &value);
  printf("+++ gpio_port_get_raw -> 0x%x\n\n", value);

  printf("++ DONE!\n");
  return 0;
}
