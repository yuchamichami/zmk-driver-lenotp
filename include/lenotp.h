/*
 * Copyright (c) 2026
 */

#pragma once

#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

int lenotp_sleep(const struct device *dev);
int lenotp_wakeup(const struct device *dev);
int lenotp_reset(const struct device *dev);
int lenotp_enter_idle(const struct device *dev);
int lenotp_exit_idle(const struct device *dev);
int lenotp_set_speed(const struct device *dev, uint8_t up, uint8_t down, uint8_t left,
                     uint8_t right);

#ifdef __cplusplus
}
#endif
