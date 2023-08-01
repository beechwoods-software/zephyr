# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_RPIPICOWCYW43)
  if(CONFIG_BUILD_CYW43_WITH_ZEPHYR_PIO_SPI)
    zephyr_library_include_directories(src/ src/georgerobotics/)
    
    zephyr_library_sources(
      src/pico_w_cyw43_drv.c
      src/pico_w_cyw43_shell.c
      src/georgerobotics/cyw43_ll.c
      src/georgerobotics/cyw43_ctrl.c
      src/cyw43_bus_zephyr_pio_spi.c
    )
  elseif(CONFIG_USE_PICOWI_PIOSPI)
    zephyr_library_include_directories(src/ src/georgerobotics/)
    
    zephyr_library_sources(
      src/pico_w_cyw43_drv.c
      src/pico_w_cyw43_shell.c
      src/georgerobotics/cyw43_ll.c
      src/georgerobotics/cyw43_ctrl.c
      src/cyw43_bus_picowi_pio_spi.c
      src/picowi/picowi_pio.c
      src/picowi/picowi_pico.c
    )
  else()
    zephyr_library_include_directories(src/ src/rpi2_common/ src/georgerobotics/)
    
    zephyr_library_sources(
      src/pico_w_cyw43_drv.c
      src/pico_w_cyw43_shell.c
      src/georgerobotics/cyw43_ll.c
      src/georgerobotics/cyw43_ctrl.c
      src/rpi2_common/cyw43_bus_pio_spi.c
    )
  endif()
endif()  
