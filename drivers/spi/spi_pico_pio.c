/*
 * Copyright (c) 2023 Stephen Boylan <stephen.boylan@beechwoods.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT raspberrypi_pico_spi_pio

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_pico_pio);

#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include "spi_context.h"

#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

#include <hardware/pio.h>
#include "hardware/clocks.h"

#define PIO_CYCLES		(4)
#define PIO_FIFO_DEPTH	(4)

struct spi_pico_pio_config {
	const struct device *piodev;
	const struct pinctrl_dev_config *pin_cfg;
	struct gpio_dt_spec clk_gpio;
	struct gpio_dt_spec mosi_gpio;
	struct gpio_dt_spec miso_gpio;
};

struct spi_pico_pio_data {
	struct spi_context spi_ctx;
	uint32_t tx_count;
	uint32_t rx_count;
	int cpol;
	int cpha;
	bool loop;
	PIO pio;
	size_t pio_sm;
	int bits;
	int dfs;
	bool half_duplex;
};

RPI_PICO_PIO_DEFINE_PROGRAM(spi_cpol_0_cpha_0, 0, 1,
            //     .wrap_target
    0x6101, //  0: out    pins, 1         side 0 [1] 
    0x5101, //  1: in     pins, 1         side 1 [1] 
            //     .wrap
);

RPI_PICO_PIO_DEFINE_PROGRAM(spi_cpol_1_cpha_1, 0, 2,
            //     .wrap_target
    0x7021, //  0: out    x, 1            side 1     
    0xa101, //  1: mov    pins, x         side 0 [1] 
    0x5001, //  2: in     pins, 1         side 1     
            //     .wrap
);

static float spi_pico_pio_clock_divisor(uint32_t spi_frequency) {
	return (float)clock_get_hz(clk_sys) / (float)(PIO_CYCLES * spi_frequency);
}

static uint32_t spi_pico_pio_maximum_clock_frequency() {
	return clock_get_hz(clk_sys) / PIO_CYCLES;
}

static uint32_t spi_pico_pio_minimum_clock_frequency() {
	return clock_get_hz(clk_sys) / (PIO_CYCLES * 65536);
}

static inline bool spi_pico_pio_transfer_ongoing(struct spi_pico_pio_data *data)
{
	return spi_context_tx_on(&data->spi_ctx) || spi_context_rx_on(&data->spi_ctx);
}

// TODO:  Add support for 16- and 32-bit writes
static inline void spi_pico_pio_sm_put8(PIO pio, uint sm, uint8_t data) {
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_8 *txfifo = (io_rw_8 *) &pio->txf[sm];

	*txfifo = data;
}

// TODO:  Add support for 16- and 32-bit reads
static inline uint8_t spi_pico_pio_sm_get8(PIO pio, uint sm) {
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_8 *rxfifo = (io_rw_8 *) &pio->rxf[sm];

	return *rxfifo;
}

static int spi_pico_pio_configure(const struct spi_pico_pio_config *dev_cfg,
			    struct spi_pico_pio_data *data,
			    const struct spi_config *spi_cfg)
{
	const struct gpio_dt_spec *miso = NULL;
	const struct gpio_dt_spec *mosi = NULL;
	const struct gpio_dt_spec *clk = NULL;
	pio_sm_config sm_config;
	uint32_t offset;
	uint32_t wrap_target;
	uint32_t wrap;
	const pio_program_t * program;
	int rc;

	// DEBUG
	printk("Entering spi_pico_pio_configure(dev_cfg=%p, data=%p, spi_cfg=%p)\n", dev_cfg, data, spi_cfg);
	printk("spi_cfg->operation=0x%X\n", spi_cfg->operation);
	// DEBUG END

	printk("&data->spi_ctx=%p, (&data->spi_ctx)->config=%p, spi_cfg=%p\n",
		&data->spi_ctx, (&data->spi_ctx)->config, spi_cfg);
	if (spi_context_configured(&data->spi_ctx, spi_cfg)) {
		return 0;
	}

	if (spi_cfg->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("Unsupported configuration");
		return -ENOTSUP;
	}

#if defined(CONFIG_SPI_EXTENDED_MODES)
	if (spi_cfg->operation & (SPI_LINES_DUAL | SPI_LINES_QUAD | SPI_LINES_OCTAL)) {
		LOG_ERR("Unsupported configuration");
		return -ENOTSUP;
	}
#endif /* CONFIG_SPI_EXTENDED_MODES */

	const int bits = SPI_WORD_SIZE_GET(spi_cfg->operation);

	if (bits != 8) {
		LOG_ERR("Only 8 bit word size is supported");
		return -ENOTSUP;
	}

	data->bits = bits;
	data->dfs = ((data->bits - 1) / 8) + 1;

	if ((spi_cfg->frequency > spi_pico_pio_maximum_clock_frequency())
			|| (spi_cfg->frequency < spi_pico_pio_minimum_clock_frequency())) {
		LOG_ERR("clock-frequency out of range");
		return -EINVAL;
	}

	float clock_div = spi_pico_pio_clock_divisor(spi_cfg->frequency);

	/* Half-duplex mode has not been implemented */
	if (spi_cfg->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL) {
		data->cpol = 1;
	}
	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA) {
		data->cpha = 1;
	}
	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_LOOP) {
		data->loop = true;
	}

	// DEBUG
	printk("data->bits=%d, data->dfs=%d\n", data->bits, data->dfs);
	printk("cpol=%d, cpha=%d, loop=%s\n", data->cpol, data->cpha, (data->loop ? "true" : "false"));
	// DEBUG END

	if (dev_cfg->mosi_gpio.port) {
		mosi = &dev_cfg->mosi_gpio;

		// DEBUG
		printk("mosi=%p, mosi->pin=%d\n", mosi, mosi->pin);
		// DEBUG END
	}

	if (dev_cfg->miso_gpio.port) {
		miso = &dev_cfg->miso_gpio;

		// DEBUG
		printk("miso=%p, miso->pin=%d\n", miso, miso->pin);
		// DEBUG END
	}

	clk = &dev_cfg->clk_gpio;

	// DEBUG
	printk("clk=%p, clk->pin=%d\n", clk, clk->pin);
	int quotient = (int)clock_div;
	int fraction = (clock_div - (float)quotient) * 256;
	printk("quotient=%d, fraction=%d\n", quotient, fraction);
	const struct device *pio_device = dev_cfg->piodev;
	printk("pio_device=%p\n", pio_device);
	if (pio_device->name != NULL) {
		printk("Device name=%s\n", pio_device->name);
	} else {
		printk("No device name\n");
	}
	printk("pio_device->config=%p, pio_device->data=%p, pio_device->api=%p)\n",
		pio_device->config, pio_device->data, pio_device->api);
	// DEBUG END

	data->pio = pio_rpi_pico_get_pio(dev_cfg->piodev);
	// DEBUG
	printk("data->pio=%p\n", data->pio);
	// DEBUG END

	rc = pio_rpi_pico_allocate_sm(dev_cfg->piodev, &data->pio_sm);
	// DEBUG
	printk("data->pio_sm=%d\n", data->pio_sm);
	// DEBUG END
	if (rc < 0) {
		return rc;
	}

	if ((data->cpol == 0) && (data->cpha == 0)) {
		program = RPI_PICO_PIO_GET_PROGRAM(spi_cpol_0_cpha_0);
		wrap_target = RPI_PICO_PIO_GET_WRAP_TARGET(spi_cpol_0_cpha_0);
		wrap = RPI_PICO_PIO_GET_WRAP(spi_cpol_0_cpha_0);
	} else if ((data->cpol == 1) && (data->cpha == 1)) {
		program = RPI_PICO_PIO_GET_PROGRAM(spi_cpol_1_cpha_1);
		wrap_target = RPI_PICO_PIO_GET_WRAP_TARGET(spi_cpol_1_cpha_1);
		wrap = RPI_PICO_PIO_GET_WRAP(spi_cpol_1_cpha_1);
	} else {
		printk("Not supported:  cpol=%d, cpha=%d\n", data->cpol, data->cpha);
		return -ENOTSUP;
	}

	if (!pio_can_add_program(data->pio, program)) {
		return -EBUSY;
	}

	offset = pio_add_program(data->pio, program);
	sm_config = pio_get_default_sm_config();
	// DEBUG
	printk("offset=%d\n", offset);
	// DEBUG END

	sm_config_set_clkdiv(&sm_config, clock_div);
	sm_config_set_in_pins(&sm_config, miso->pin);
	sm_config_set_in_shift(&sm_config, false, true, data->bits);
	sm_config_set_out_pins(&sm_config, mosi->pin, 1);
	sm_config_set_out_shift(&sm_config, false, true, data->bits);
	sm_config_set_sideset_pins(&sm_config, clk->pin);
	sm_config_set_sideset(&sm_config, 1, false, false);
	sm_config_set_wrap(&sm_config,
			   offset + wrap_target,
			   offset + wrap);

	// DEBUG
	printk("sm_config.clkdiv=0x%X, sm_config.execctrl=0x%X, sm_config.pinctrl=0x%X, sm_config.shiftctrl=0x%X\n",
		sm_config.clkdiv, sm_config.execctrl, sm_config.pinctrl, sm_config.shiftctrl);
	printk("pindirs: 0x%lX, pinval0x%X\n", (BIT(clk->pin) | BIT(mosi->pin)), (data->cpol << clk->pin));
	// DEBUG END

	pio_sm_set_consecutive_pindirs(data->pio, data->pio_sm, miso->pin, 1, false);
	pio_sm_set_pindirs_with_mask(data->pio, data->pio_sm, (BIT(clk->pin) | BIT(mosi->pin)), (BIT(clk->pin) | BIT(mosi->pin)));
	pio_sm_set_pins_with_mask(data->pio, data->pio_sm, (data->cpol << clk->pin), BIT(clk->pin) | BIT(mosi->pin));
    pio_gpio_init(data->pio, mosi->pin);
    pio_gpio_init(data->pio, miso->pin);
    pio_gpio_init(data->pio, clk->pin);

	pio_sm_init(data->pio, data->pio_sm, offset, &sm_config);
	pio_sm_set_enabled(data->pio, data->pio_sm, true);

	data->spi_ctx.config = spi_cfg;

	// DEBUG
	printk("PIO SPI device configured\n");
	// DEBUG END

	return 0;
}

static void spi_pico_pio_txrx(const struct device *dev)
{
	struct spi_pico_pio_data *data = dev->data;
	const size_t chunk_len = spi_context_max_continuous_chunk(&data->spi_ctx);
	const void *txbuf = data->spi_ctx.tx_buf;
	void *rxbuf = data->spi_ctx.rx_buf;
	uint32_t txrx;
	size_t fifo_cnt = 0;

	data->tx_count = 0;
	data->rx_count = 0;

	// DEBUG
	printk("spi_pico_pio_txrx() called\n");
	printk("  data->pio=%p, data->pio_sm=%d\n", data->pio, data->pio_sm);
	// DEBUG END

	pio_sm_clear_fifos(data->pio, data->pio_sm);

	while (data->rx_count < chunk_len || data->tx_count < chunk_len) {
		/* Fill up fifo with available TX data */
		while ((!pio_sm_is_tx_fifo_full(data->pio, data->pio_sm))
				&& data->tx_count < chunk_len
				&& fifo_cnt < PIO_FIFO_DEPTH) {
			/* Send 0 in the case of read only operation */
			txrx = 0;

			if (txbuf) {
				txrx = ((uint8_t *)txbuf)[data->tx_count];

				// DEBUG
				printk("  writing %d (0x%X)\n", txrx, txrx);
				// DEBUG END
			}
			spi_pico_pio_sm_put8(data->pio, data->pio_sm, txrx);
			data->tx_count++;
			fifo_cnt++;
		}

		// DEBUG
		printk("  data->tx_count=%d, fifo_cnt=%d\n", data->tx_count, fifo_cnt);
		// DEBUG END

		while ((!pio_sm_is_rx_fifo_empty(data->pio, data->pio_sm))
				&& data->rx_count < chunk_len
				&& fifo_cnt > 0) {
			txrx = spi_pico_pio_sm_get8(data->pio, data->pio_sm);

			// DEBUG
			printk("  read %d (0x%X)\n", txrx, txrx);
			// DEBUG END

			/* Discard received data if rx buffer not assigned */
			if (rxbuf) {
				((uint8_t *)rxbuf)[data->rx_count] = (uint8_t)txrx;
			}
			data->rx_count++;
			fifo_cnt--;
		}

		// DEBUG
		printk("  data->rx_count=%d, fifo_cnt=%d\n", data->rx_count, fifo_cnt);
		// DEBUG END
	}
}

static int spi_pico_pio_transceive_impl(const struct device *dev,
				     const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
					 bool asynchronous,
				     spi_callback_t cb,
				     void *userdata)
{
	const struct spi_pico_pio_config *dev_cfg = dev->config;
	struct spi_pico_pio_data *data = dev->data;
	struct spi_context *spi_ctx = &data->spi_ctx;
	int rc;

	// DEBUG
	printk("spi_pico_pio_transceive_impl() called\n");
	// DEBUG END

	spi_context_lock(spi_ctx, asynchronous, cb, userdata, spi_cfg);

	rc = spi_pico_pio_configure(dev_cfg, data, spi_cfg);
	if (rc < 0) {
		return rc;
	}

	spi_context_buffers_setup(spi_ctx, tx_bufs, rx_bufs, data->dfs);

	/* Set initial clock state before asserting CS */
	// DEBUG
	printk("data->cpol=%d\n", data->cpol);
	printk("Enable CS\n");
	// DEBUG END
	gpio_pin_set_dt(&dev_cfg->clk_gpio, data->cpol);

	spi_context_cs_control(spi_ctx, true);

	do {
		spi_pico_pio_txrx(dev);
		spi_context_update_tx(spi_ctx, 1, data->tx_count);
		spi_context_update_rx(spi_ctx, 1, data->rx_count);
	} while (spi_pico_pio_transfer_ongoing(data));

	// DEBUG
	printk("Disable CS\n");
	// DEBUG END
	spi_context_cs_control(spi_ctx, false);

	/* Set idle clock state after de-asserting CS */
	gpio_pin_set_dt(&dev_cfg->clk_gpio, data->cpol);

	spi_context_complete(spi_ctx, dev, 0);

	return 0;
}

static int spi_pico_pio_transceive(const struct device *dev,
				const struct spi_config *spi_cfg,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{

	// DEBUG
	printk("spi_pico_pio_transceive() called: bufs:  tx=%d, rx=%d\n",
		tx_bufs->count, rx_bufs->count);
	// DEBUG END

	// TODO:  Add interrupt support
	// TODO:  Add DMA support
	// TODO:  Ponder adding async operation
	return spi_pico_pio_transceive_impl(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

int spi_pico_pio_release(const struct device *dev,
			  const struct spi_config *spi_cfg)
{
	struct spi_pico_pio_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->spi_ctx);

	return 0;
}

static struct spi_driver_api spi_pico_pio_api = {
	.transceive = spi_pico_pio_transceive,
	.release = spi_pico_pio_release,
};

static int config_gpio(const struct gpio_dt_spec *gpio, const char *tag, int mode)
{
	int rc = 0;

	if (!device_is_ready(gpio->port)) {
		LOG_ERR("GPIO port for %s pin is not ready", tag);
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(gpio, mode);
	if (rc < 0) {
		LOG_ERR("Couldn't configure %s pin; (%d)", tag, rc);
		return rc;
	}

	return 0;
}

int spi_pico_pio_init(const struct device *dev)
{
	const struct spi_pico_pio_config *dev_cfg = dev->config;
	struct spi_pico_pio_data *data = dev->data;
	int rc;

	rc = pinctrl_apply_state(dev_cfg->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (rc) {
		LOG_ERR("Failed to apply pinctrl state");
		return rc;
	}

	rc = config_gpio(&dev_cfg->clk_gpio, "clk", GPIO_OUTPUT_ACTIVE);
	if (rc < 0) {
		return rc;
	}

	if (dev_cfg->mosi_gpio.port != NULL) {
		rc = config_gpio(&dev_cfg->mosi_gpio, "mosi", GPIO_OUTPUT);
		if (rc < 0) {
			return rc;
		}
	}

	if (dev_cfg->miso_gpio.port != NULL) {
		rc = config_gpio(&dev_cfg->miso_gpio, "miso", GPIO_INPUT);
		if (rc < 0) {
			return rc;
		}
	}

	rc = spi_context_cs_configure_all(&data->spi_ctx);
	if (rc < 0) {
		LOG_ERR("Failed to configure CS pins: %d", rc);
		return rc;
	}

	spi_context_unlock_unconditionally(&data->spi_ctx);

	return 0;
}

#define SPI_PICO_PIO_INIT(inst)						\
	PINCTRL_DT_INST_DEFINE(inst);								\
	static struct spi_pico_pio_config spi_pico_pio_config_##inst = {	\
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(inst)),					\
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),					\
		.clk_gpio = GPIO_DT_SPEC_INST_GET(inst, clk_gpios),	\
		.mosi_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mosi_gpios, {0}),	\
		.miso_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, miso_gpios, {0}),	\
	};								\
									\
	static struct spi_pico_pio_data spi_pico_pio_data_##inst = {	\
		SPI_CONTEXT_INIT_LOCK(spi_pico_pio_data_##inst, spi_ctx),	\
		SPI_CONTEXT_INIT_SYNC(spi_pico_pio_data_##inst, spi_ctx),	\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), spi_ctx)	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    spi_pico_pio_init,				\
			    NULL,					\
			    &spi_pico_pio_data_##inst,			\
			    &spi_pico_pio_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SPI_INIT_PRIORITY,			\
			    &spi_pico_pio_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_PICO_PIO_INIT)
