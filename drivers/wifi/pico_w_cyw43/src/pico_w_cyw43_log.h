/**
 * Copyright (c) 2023 Beechwoods Software
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME wifi_pico_w_cyw43
#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL

#if 1

#include <zephyr/logging/log.h>

#else // 0

#undef LOG_MODULE_REGISTER
#define LOG_MODULE_REGISTER(x)
#undef LOG_DBG
#define LOG_DBG(x, ...)
#undef LOG_WRN
#define LOG_WRN(x, ...)
#undef LOG_INF
#define LOG_INF(x, ...)
#undef LOG_ERR
#define LOG_ERR(x, ...)

#endif // 0
