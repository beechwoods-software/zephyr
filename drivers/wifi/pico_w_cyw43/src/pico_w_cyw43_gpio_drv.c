/*
 * Copyright (c) 2021, Yonatan Schachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/i2c.h>

#include <hardware/gpio.h>
#include <zephyr/logging/log.h>
#include "georgerobotics/cyw43.h"
#include "georgerobotics/cyw43_country.h"

#define DT_DRV_COMPAT infineon_cyw43_gpio

#define ALL_EVENTS (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE \
		| GPIO_IRQ_LEVEL_LOW | GPIO_IRQ_LEVEL_HIGH)

struct gpio_rpi_config {
	struct gpio_driver_config common;
};

struct gpio_rpi_data {
    struct gpio_driver_config common;
};

static int gpio_rpi_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
    printf("gpio_rpi_configure pin=%d flags=%x\n", pin, flags);
	return 0;
}


static int gpio_rpi_port_get_raw(const struct device *dev, uint32_t *value)
{
    printf("gpio_rpi_port_get_raw\n");
	return 0;
}

static int gpio_rpi_port_set_masked_raw(const struct device *port, uint32_t mask, uint32_t value)
{
    printf("gpio_rpi_port_set_masked_raw mask=%x value=%d\n", mask, value);
	return 0;
}

static int gpio_rpi_port_set_bits_raw(const struct device *port, uint32_t pins)
{
    printf("gpio_rpi_port_set_bits_raw pins=%x\n", pins);
    int ret = 0;
    for (int i = 0; i < NUM_BANK0_GPIOS; i++) {
        if (pins & (1<<i)) {
            if (i < CYW43_WL_GPIO_COUNT) {
                printf("Setting pin %d to 1\n", i);
                ret = cyw43_gpio_set(&cyw43_state, i, 1);
            } else {
                printf("pin %d is not supported in cyw43\n", i);
                return -ENOTSUP;
            }
        }
    }
	return ret;
}

static int gpio_rpi_port_clear_bits_raw(const struct device *port, uint32_t pins)
{
    printf("gpio_rpi_port_clear_bits_raw pins=%x\n", pins);
    int ret = 0;
    for (int i = 0; i < NUM_BANK0_GPIOS; i++) {
        if (pins & (1<<i)) {
            if (i < CYW43_WL_GPIO_COUNT) {
                printf("Clearing pin %d\n", i);
                ret = cyw43_gpio_set(&cyw43_state, i, 0);
            } else {
                printf("pin %d is not supported in cyw43\n", i);
                return -ENOTSUP;
            }
        }
    }
	return ret;
}

static int gpio_rpi_port_toggle_bits(const struct device *port, uint32_t pins)
{
    printf("gpio_rpi_port_toggle_bits pins=%x\n", pins);
	return 0;
}

static int gpio_rpi_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
    printf("gpio_rpi_pin_interrupt_configure pin=%d mode=%d trig=%d\n", pin, mode, trig);
	return 0;
}


static int gpio_rpi_manage_callback(const struct device *port, struct gpio_callback *cb, bool set)
{
    printf("gpio_rpi_manage_callback\n");
    return 0;
}

static int gpio_rpi_get_pending_int(const struct device *dev)
{
    printf("gpio_rpi_get_pending_int\n");
    return 0;
}

static const struct gpio_driver_api cyw43_gpio_api = {
	.pin_configure = gpio_rpi_configure,
	.port_get_raw = gpio_rpi_port_get_raw,
	.port_set_masked_raw = gpio_rpi_port_set_masked_raw,
	.port_set_bits_raw = gpio_rpi_port_set_bits_raw,
	.port_clear_bits_raw = gpio_rpi_port_clear_bits_raw,
	.port_toggle_bits = gpio_rpi_port_toggle_bits,
	.pin_interrupt_configure = gpio_rpi_pin_interrupt_configure,
    .manage_callback = gpio_rpi_manage_callback,
    .get_pending_int = gpio_rpi_get_pending_int,
};


static int cyw43_gpio_init(const struct device *dev)
{
    printf("gpio_rpi_init\n");
	return 0;
}

DEVICE_DT_INST_DEFINE(0, cyw43_gpio_init, NULL, NULL,
              NULL, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,
              &cyw43_gpio_api);
