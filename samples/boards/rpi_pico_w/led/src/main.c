/*
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <unistd.h>
#include "hardware/pio.h"

void wifi_spi_write(uint8_t *dp, int nbits)
{
    pio_sm_clear_fifos(wifi_pio, wifi_sm);
    pio_sm_exec(wifi_pio, wifi_sm, pio_encode_jmp(picowi_pio_offset_writer));
    pio_sm_set_consecutive_pindirs(wifi_pio, wifi_sm, SD_CMD_PIN, 1, true);
    int n = 0;
    while (n < nbits)
    {
        if (!pio_sm_is_tx_fifo_full(wifi_pio, wifi_sm))
        {
            *wifi_txfifo = *dp++;
            n += 8;
        }
    }
    while (!pio_sm_is_tx_fifo_empty(wifi_pio, wifi_sm)) ;
    while (wifi_pio->sm[wifi_sm].addr != picowi_pio_offset_writer) ;
    pio_sm_set_consecutive_pindirs(wifi_pio, wifi_sm, SD_CMD_PIN, 1, false);
    pio_sm_exec(wifi_pio, wifi_sm, pio_encode_jmp(picowi_pio_offset_stall));
}

int wifi_data_write(int func, int addr, uint8_t *dp, int nbytes)
{
    SPI_MSG msg = {
        .hdr = {
         .wr = SD_WR,
        .incr = 1,
        .func = func&SD_FUNC_MASK,
        .addr = addr,
        .len = nbytes
    }
    };

    if (func & SD_FUNC_SWAP)
        msg.vals[0] = SWAP16_2(msg.vals[0]);
    io_out(SD_CS_PIN, 0);
    if (nbytes <= 4)
    {
        memcpy(&msg.bytes[4], dp, nbytes);
        wifi_spi_write((uint8_t *)&msg, 64);
    }
    else
    {
        wifi_spi_write((uint8_t *)&msg, 32);
        wifi_spi_write(dp, nbytes * 8);
    }
    io_out(SD_CS_PIN, 1);
    return (nbytes);
}

int wifi_reg_write(int func, uint32_t addr, uint32_t val, int nbytes)
{
    int ret;

    if (func & SD_FUNC_SWAP && nbytes > 1)
        val = SWAP16_2(val);
    ret = wifi_data_write(func, addr, (uint8_t *)&val, nbytes);
    display(DISP_REG,
        "Wr_reg   len %u %s 0x%04lx <- 0x%02lX\n",
        nbytes,
        wifi_func_str(func),
        addr&SB_ADDR_MASK,
        val);
    return (ret);
}

int wifi_bak_reg_write(uint32_t addr, uint32_t val, int nbytes)
{
    wifi_bak_window(addr);
    addr |= nbytes==4 ? SB_32BIT_WIN : 0;
    return(wifi_reg_write(SD_FUNC_BAK, addr, val, nbytes));
}

void wifi_set_led(bool on)
{
    static bool init=false;
    if (!init)
        wifi_bak_reg_write(BAK_GPIOOUTEN_REG, 1<<SD_LED_GPIO, 4);
    init = true;
    wifi_bak_reg_write(BAK_GPIOOUT_REG, on ? 1<<SD_LED_GPIO : 0, 4);
}

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
