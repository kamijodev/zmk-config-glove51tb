/*
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>

void zmk_input_processor_rotate_adjust_angle(const struct device *dev, int32_t delta);
int32_t zmk_input_processor_rotate_get_angle(const struct device *dev);
