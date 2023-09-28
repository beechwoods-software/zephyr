/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include <zephyr/drivers/gpio.h>

int main(void)
{
    const struct device *const gpio_dev = DEVICE_DT_GET_ANY(raspberrypi_picow_gpio); 
    if (NULL == gpio_dev) {
        printf("NULL dev\n");
        return 0;
    }
    printf("got device \n");
//    gpio_port_set_bits(gpio_dev, 0);
}
