/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define CYW43_USE_DMA 0

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "cyw43.h"
#include "cyw43_internal.h"
#include "cyw43_spi.h"
#include "cyw43_debug_pins.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "picowi/picowi_pio.h"

//#define DT_DRV_COMPAT infineon_cyw43

#define WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE        (4)

#if CYW43_SPI_PIO
#define WL_REG_ON 23
#define DATA_OUT_PIN 24u
#define DATA_IN_PIN 24u
#define IRQ_PIN 24u
// #define MONITOR_PIN 3u
#define CLOCK_PIN 29u
#define CS_PIN 25u
#define IRQ_SAMPLE_DELAY_NS 100

#define CLOCK_DIV 2
#define CLOCK_DIV_MINOR 0
#define PADS_DRIVE_STRENGTH PADS_BANK0_GPIO0_DRIVE_VALUE_12MA

#if !CYW43_USE_SPI
#error CYW43_USE_SPI should be true
#endif

#ifndef NDEBUG
#define ENABLE_SPI_DUMPING 1
#endif

// Set to 1 to enable
#if ENABLE_SPI_DUMPING //NDEBUG
#if 0
#define DUMP_SPI_TRANSACTIONS(A) A
#else
static bool enable_spi_packet_dumping=true; // set to true to dump
#define DUMP_SPI_TRANSACTIONS(A) if (enable_spi_packet_dumping) {A}
#endif

//static uint32_t counter = 0;
#else
#define DUMP_SPI_TRANSACTIONS(A)
#endif

//#define SWAP32(A) ((((A) & 0xff000000U) >> 8) | (((A) & 0xff0000U) << 8) | (((A) & 0xff00U) >> 8) | (((A) & 0xffU) << 8))
__force_inline static uint32_t __swap16x2(uint32_t a) {
    __asm ("rev16 %0, %0" : "+l" (a) : : );
    return a;
}
#define SWAP32(a) __swap16x2(a)

//static const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));


int cyw43_spi_init(cyw43_int_t *self) {

    // Only does something if CYW43_LOGIC_DEBUG=1
    logic_debug_init();

    cyw43_spi_gpio_setup();
    
    pio_init();
    
    assert(!self->bus_data);
    
    return 0;
}

void cyw43_spi_deinit(cyw43_int_t *self) {
  return;
}


#if ENABLE_SPI_DUMPING
static void dump_bytes(const uint8_t *bptr, uint32_t len) {
    unsigned int i = 0;

    for (i = 0; i < len;) {
      /* if ((i & 0x0f) == 0) {
            printf("\n");
	    } else */ if ((i & 0x07) == 0) {
            printf(" ");
        }
        printf("%02x ", bptr[i++]);
    }
}
#endif


int cyw43_spi_transfer(cyw43_int_t *self, const uint8_t *tx, size_t tx_length, uint8_t *rx,
                       size_t rx_length) {

  bool tx_only = false;
  
  CYW43_VDEBUG("Calling cyw43_spi_transfer(self=0x%lx, tx=0x%lx, tx_length=%d, rx=0x%lx, rx_length=%d)\n",
         (unsigned long)self, (unsigned long)tx, (unsigned int)tx_length,
  	       (unsigned long)rx, (unsigned int)rx_length);
  
    if ((tx == NULL) && (rx == NULL)) {
        return CYW43_FAIL_FAST_CHECK(-CYW43_EINVAL);
    }

    
    if (rx != NULL) {
      if (tx == NULL) {
	tx = rx;
	assert(tx_length && tx_length < rx_length);
      }
      else {
	tx_only=true;
      }
      
      gpio_put(CS_PIN, false);
      
      pio_spi_write((unsigned char *)tx, (int)tx_length);
      DUMP_SPI_TRANSACTIONS(
	    printf("TXed:");
            dump_bytes(tx, tx_length);
            printf("\n");
      )
    }
    
    if (!tx_only) {
      memset(rx, 0, rx_length);
      pio_spi_read((unsigned char *)rx, (int)rx_length);
      DUMP_SPI_TRANSACTIONS(
	    printf("RXed:");
            dump_bytes(rx, rx_length);
            printf("\n");
      )

    }
    
    gpio_put(CS_PIN, true);
    
    return 0;
}

// Initialise our gpios
void cyw43_spi_gpio_setup(void) {
  printf("cyw43_spi_gpio_setup()\n");
    // Setup WL_REG_ON (23)
    gpio_init(WL_REG_ON);
    gpio_set_dir(WL_REG_ON, GPIO_OUT);
    gpio_pull_up(WL_REG_ON);
    //gpio_put(WL_REG_ON, false);

    // Setup CS (25)
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    //gpio_pull_up(CS_PIN);
    gpio_put(CS_PIN, true);

#if 0    
    // Setup CLOCK (29)
    gpio_init(CLOCK_PIN);
    gpio_set_dir(CLOCK_PIN, GPIO_OUT);
    //gpio_pull_up(CLOCK_PIN);
    gpio_put(CLOCK_PIN, false);
#endif
    
    // Setup DO, DI and IRQ (24)
    gpio_init(DATA_OUT_PIN);
    gpio_set_dir(DATA_OUT_PIN, GPIO_OUT);
    //gpio_pull_up(DATA_OUT_PIN);
    gpio_put(DATA_OUT_PIN, false);

#if 0    
    k_sleep(K_MSEC(100));
    
    gpio_put(WL_REG_ON, false);

    k_sleep(K_MSEC(50));

    gpio_set_dir(DATA_OUT_PIN, GPIO_IN);
    gpio_set_dir(IRQ_PIN, GPIO_IN);
#endif
}

// Reset wifi chip
void cyw43_spi_reset(void) {
  printf("cyw43_spi_reset()\n");
    gpio_put(WL_REG_ON, false); // off
    k_sleep(K_MSEC(20));
    gpio_put(WL_REG_ON, true); // on
    k_sleep(K_MSEC(250));

    // Setup IRQ (24) - also used for DO, DI
    gpio_init(IRQ_PIN);
    //gpio_set_dir(DATA_OUT_PIN, GPIO_IN);
    gpio_set_dir(IRQ_PIN, GPIO_IN);
}

static inline uint32_t make_cmd(bool write, bool inc, uint32_t fn, uint32_t addr, uint32_t sz) {
    return write << 31 | inc << 30 | fn << 28 | (addr & 0x1ffff) << 11 | sz;
}

#if CYW43_VERBOSE_DEBUG
static const char *func_name(int fn) {
    switch (fn)
    {
        case BUS_FUNCTION:
            return "BUS_FUNCTION";
        case BACKPLANE_FUNCTION:
            return "BACKPLANE_FUNCTION";
        case WLAN_FUNCTION:
            return "WLAN_FUNCTION";
        default:
            return "UNKNOWN";
    }
}
#endif

uint32_t read_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
    uint32_t buf[2] = {0};
    assert(fn != BACKPLANE_FUNCTION);
    buf[0] = SWAP32(make_cmd(false, true, fn, reg, 4));
    int ret = cyw43_spi_transfer(self, NULL, 4, (uint8_t *)buf, 8);
    if (ret != 0) {
        return ret;
    }
    return SWAP32(buf[1]);
}

static inline uint32_t _cyw43_read_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint size) {
    // Padding plus max read size of 32 bits + another 4?
    static_assert(WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE % 4 == 0, "");
    uint32_t buf32[WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE/4 + 1 + 1];
    uint8_t *buf = (uint8_t *)buf32;
    const uint32_t padding = (fn == BACKPLANE_FUNCTION) ? WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE : 0; // Add response delay
    buf32[0] = make_cmd(false, true, fn, reg, size + padding);

    if (fn == BACKPLANE_FUNCTION) {
        logic_debug_set(pin_BACKPLANE_READ, 1);
    }
    int ret = cyw43_spi_transfer(self, NULL, 4, buf, 8 + padding);
    if (fn == BACKPLANE_FUNCTION) {
        logic_debug_set(pin_BACKPLANE_READ, 0);
    }

    if (ret != 0) {
        return ret;
    }
    uint32_t result = buf32[padding > 0 ? 2 : 1];
    CYW43_VDEBUG("cyw43_read_reg_u%d %s 0x%lx=0x%lx\n", size * 8,
		 func_name(fn), (unsigned long)reg, (unsigned long)result);
    return result;
}

uint32_t cyw43_read_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
    return _cyw43_read_reg(self, fn, reg, 4);
}

int cyw43_read_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
    return _cyw43_read_reg(self, fn, reg, 2);
}

int cyw43_read_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg) {
    return _cyw43_read_reg(self, fn, reg, 1);
}

// This is only used to switch the word order on boot
int write_reg_u32_swap(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
    uint32_t buf[2];
    // Boots up in little endian so command needs swapping too
    buf[0] = SWAP32(make_cmd(true, true, fn, reg, 4));
    buf[1] = SWAP32(val);
    int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 8, NULL, 0);
    CYW43_VDEBUG("write_reg_u32_swap %s 0x%lx=0x%lx\n",
		 func_name(fn), (unsigned long)reg, (unsigned long)val);
    return ret;
}

static inline int _cyw43_write_reg(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val, uint size) {
    uint32_t buf[2];
    buf[0] = make_cmd(true, true, fn, reg, size);
    buf[1] = val;
    if (fn == BACKPLANE_FUNCTION) {
        // In case of f1 overflow
        self->last_size = 8;
        self->last_header[0] = buf[0];
        self->last_header[1] = buf[1];
        self->last_backplane_window = self->cur_backplane_window;
    }

    if (fn == BACKPLANE_FUNCTION) {
        logic_debug_set(pin_BACKPLANE_WRITE, 1);
    }

    int ret = cyw43_spi_transfer(self, (uint8_t *)buf, 8, NULL, 0);

    if (fn == BACKPLANE_FUNCTION) {
        logic_debug_set(pin_BACKPLANE_WRITE, 0);
    }

    CYW43_VDEBUG("cyw43_write_reg_u%d %s 0x%lx=0x%lx\n",size * 8,
		 func_name(fn), (unsigned long)reg, (unsigned long)val);
    return ret;
}

int cyw43_write_reg_u32(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
    return _cyw43_write_reg(self, fn, reg, val, 4);
}

int cyw43_write_reg_u16(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint16_t val) {
    return _cyw43_write_reg(self, fn, reg, val, 2);
}

int cyw43_write_reg_u8(cyw43_int_t *self, uint32_t fn, uint32_t reg, uint32_t val) {
    return _cyw43_write_reg(self, fn, reg, val, 1);
}

#if MAX_BLOCK_SIZE > 0x7f8
#error Block size is wrong for SPI
#endif

int cyw43_read_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, uint8_t *buf) {
    assert(fn != BACKPLANE_FUNCTION || (len <= 64 && (addr + len) <= 0x8000));
    const uint32_t padding = (fn == BACKPLANE_FUNCTION) ? 4 : 0; // Add response delay
    size_t aligned_len = (len + 3) & ~3;
    assert(aligned_len > 0 && aligned_len <= 0x7f8);
    assert(buf == self->spid_buf || buf < self->spid_buf || buf >= (self->spid_buf + sizeof(self->spid_buf)));
    self->spi_header[padding > 0 ? 0 : 1] = make_cmd(false, true, fn, addr, len + padding);
    if (fn == WLAN_FUNCTION) {
        logic_debug_set(pin_WIFI_RX, 1);
    }
    int ret = cyw43_spi_transfer(self, NULL, 4, (uint8_t *)&self->spi_header[padding > 0 ? 0 : 1], aligned_len + 4 + padding);
    if (fn == WLAN_FUNCTION) {
        logic_debug_set(pin_WIFI_RX, 0);
    }
    if (ret != 0) {
        printf("cyw43_read_bytes error %d", ret);
        return ret;
    }
    if (buf != self->spid_buf) { // avoid a copy in the usual case just to add the header
        memcpy(buf, self->spid_buf, len);
    }
    return 0;
}

// See whd_bus_spi_transfer_bytes
// Note, uses spid_buf if src isn't using it already
// Apart from firmware download this appears to only be used for wlan functions?
int cyw43_write_bytes(cyw43_int_t *self, uint32_t fn, uint32_t addr, size_t len, const uint8_t *src) {
    assert(fn != BACKPLANE_FUNCTION || (len <= 64 && (addr + len) <= 0x8000));
    size_t aligned_len = (len + 3) & ~3u;
    assert(aligned_len > 0 && aligned_len <= 0x7f8);
    if (fn == WLAN_FUNCTION) {
        // Wait for FIFO to be ready to accept data
        int f2_ready_attempts = 1000;
        while (f2_ready_attempts-- > 0) {
            uint32_t bus_status = cyw43_read_reg_u32(self, BUS_FUNCTION, SPI_STATUS_REGISTER);
            if (bus_status & STATUS_F2_RX_READY) {
                logic_debug_set(pin_F2_RX_READY_WAIT, 0);
                break;
            } else {
                logic_debug_set(pin_F2_RX_READY_WAIT, 1);
            }
        }
        if (f2_ready_attempts <= 0) {
            printf("F2 not ready\n");
            return CYW43_FAIL_FAST_CHECK(-CYW43_EIO);
        }
    }
    if (src == self->spid_buf) { // avoid a copy in the usual case just to add the header
        self->spi_header[1] = make_cmd(true, true, fn, addr, len);
        logic_debug_set(pin_WIFI_TX, 1);
        int res = cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[1], aligned_len + 4, NULL, 0);
        logic_debug_set(pin_WIFI_TX, 0);
        return res;
    } else {
        // todo: would be nice to get rid of this. Only used for firmware download?
        assert(src < self->spid_buf || src >= (self->spid_buf + sizeof(self->spid_buf)));
        self->spi_header[1] = make_cmd(true, true, fn, addr, len);
        memcpy(self->spid_buf, src, len);
        return cyw43_spi_transfer(self, (uint8_t *)&self->spi_header[1], aligned_len + 4, NULL, 0);
    }
}
#endif
