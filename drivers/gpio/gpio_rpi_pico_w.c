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
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_sc18im, CONFIG_I2C_LOG_LEVEL);

#define DT_DRV_COMPAT raspberrypi_picow_gpio

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
    printf("gpio_rpi_configure\n");
	return 0;
}


static int gpio_rpi_port_get_raw(const struct device *dev, uint32_t *value)
{
    printf("gpio_rpi_port_get_raw\n");
	return 0;
}

static int gpio_rpi_port_set_masked_raw(const struct device *port, uint32_t mask, uint32_t value)
{
    printf("gpio_rpi_port_set_masked_raw\n");
	return 0;
}

static int gpio_rpi_port_set_bits_raw(const struct device *port, uint32_t pins)
{
    printf("gpio_rpi_port_set_bits_raw\n");
	return 0;
}

static int gpio_rpi_port_clear_bits_raw(const struct device *port, uint32_t pins)
{
    printf("gpio_rpi_port_clear_bits_raw\n");
	return 0;
}

static int gpio_rpi_port_toggle_bits(const struct device *port, uint32_t pins)
{
    printf("gpio_rpi_port_toggle_bits\n");
	return 0;
}

static int gpio_rpi_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
    printf("gpio_rpi_pin_interrupt_configure\n");
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

static const struct gpio_driver_api gpio_rpi_driver_api = {
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


static int gpio_rpi_init(const struct device *dev)
{
    printf("gpio_rpi_init\n");
	return 0;
}

#define GPIO_RPI_INIT(idx)							\
	static const struct gpio_rpi_config gpio_rpi_##idx##_config = {		\
		.bank_config_func = bank_##idx##_config_func,			\
		.common =							\
		{								\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(idx),	\
		}								\
	};									\
										\
	DEVICE_DT_INST_DEFINE(idx, gpio_rpi_init, NULL,			\
				&gpio_rpi_##idx##_data,				\
				&gpio_rpi_##idx##_config,			\
				POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,		\
				&gpio_rpi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_RPI_INIT)
