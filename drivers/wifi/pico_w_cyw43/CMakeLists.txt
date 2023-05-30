# SPDX-License-Identifier: Apache-2.0


if(CONFIG_WIFI_RPIPICOWCYW43)

  zephyr_library_sources(
    src/pico_w_cyw43_drv.c
    src/pico_w_cyw43_shell.c
    src/picowi/picowi_pio.c
    src/picowi/picowi_init.c
    src/picowi/picowi_wifi.c
    src/picowi/picowi_pico.c
    src/picowi/picowi_ioctl.c
    src/picowi/picowi_event.c
    src/picowi/picowi_scan.c
    src/picowi/picowi_join.c
    src/picowi/fw_43439.c
  )

endif()  
