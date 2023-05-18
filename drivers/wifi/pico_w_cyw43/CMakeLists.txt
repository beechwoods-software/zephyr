# SPDX-License-Identifier: Apache-2.0


if(CONFIG_WIFI_RPIPICOWCYW43)

  zephyr_library_sources(
    src/pico_w_cyw43_drv.c
    src/pico_w_cyw43_shell.c
  )

endif()  
