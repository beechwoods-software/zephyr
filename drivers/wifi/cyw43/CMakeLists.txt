# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_CYW43)

  zephyr_library_include_directories(src/ src/georgerobotics/)

  zephyr_library_sources(
    src/cyw43_drv.c
    src/cyw43_bus_pio_spi.c
    src/georgerobotics/cyw43_ll.c
    src/georgerobotics/cyw43_ctrl.c
  )

  zephyr_library_sources_ifdef(CONFIG_WIFI_CYW43_SHELL src/cyw43_shell.c)

endif()  
