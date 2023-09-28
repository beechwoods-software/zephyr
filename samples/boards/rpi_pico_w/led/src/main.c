/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include "hardware/pio.h"

int main(void)
{

  int i = 0;
  while (i < 9)
  {
    if (i % 2 == 0) {
      printf("led_on\n");
      wifi_set_led(true);
    }
    else {
        printf("led_off\n");
      wifi_set_led(false);
    }
    sleep(2);
    i++;
  }
  printf("++++ DONE!\n");
  return 0;
}
