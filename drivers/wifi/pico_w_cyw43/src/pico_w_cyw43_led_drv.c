/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>

#include "georgerobotics/cyw43.h"
#include "georgerobotics/cyw43_country.h"

#define DT_DRV_COMPAT infineon_cyw43_led

#define PICOWCYW43_WORKQUEUE_STACK_SIZE 1024

static int cyw43_led_on(const struct device *dev, uint32_t led)
{
    printf("cyw43_led_on\n");
    cyw43_gpio_set(&cyw43_state, 0, true);
    return 0;
}

static int cyw43_led_off(const struct device *dev, uint32_t led)
{
    printf("+++ cyw43_led_off\n");
    cyw43_gpio_set(&cyw43_state, 0, false);
    return 0;
}

static int cyw43_led_init(const struct device *dev)
{
    printf("+++ cyw43_led_init\n");
    // cyw43_init(&cyw43_state);
    // cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_WORLDWIDE);
    printf("+++ ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    return 0;
}

static const struct led_driver_api cyw43_led_api = {
    .on = cyw43_led_on,
    .off = cyw43_led_off,
};

DEVICE_DT_INST_DEFINE(0, cyw43_led_init, NULL, NULL,
              NULL, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,
              &cyw43_led_api);

