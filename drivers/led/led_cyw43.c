#define DT_DRV_COMPAT infineon_cyw43_led

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include "led_context.h"

struct cyw43_config {
    struct i2c_dt_spec bus;
};

struct cyw43_data {
    struct led_data dev_data;
};

static int cyw43_led_on(const struct device *dev, uint32_t led)
{
    // LOG_INF("cyw43_led_on\n");
    printf("cyw43_led_on\n");
    return 0;
}

static int cyw43_led_off(const struct device *dev, uint32_t led)
{
    // LOG_INF("cyw43_led_on\n");
    printf("cyw43_led_on\n");
    return 0;
}

static int cyw43_led_init(const struct device *dev)
{
    // LOG_INF("cyw43_led_init\n");
    printf("+++ cyw43_led_init\n");
	return 0;
}

static struct cyw43_data cyw43_led_data;

static const struct cyw43_config cyw43_led_config = {
    .bus = I2C_DT_SPEC_INST_GET(0),
};

static const struct led_driver_api cyw43_led_api = {
    .on = cyw43_led_on,
    .off = cyw43_led_off,
};

DEVICE_DT_INST_DEFINE(0, cyw43_led_init, NULL, &cyw43_led_data,
              NULL, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,
              &cyw43_led_api);

/*
#define LED_CYW43_DEFINE(n) \
  DEVICE_DT_INST_DEFINE(n, cyw43_led_init, NULL, NULL, NULL, \
			            POST_KERNEL, CONFIG_LED_INIT_PRIORITY, &cyw43_led_api);

DT_INST_FOREACH_STATUS_OKAY(LED_CYW43_DEFINE)
*/
