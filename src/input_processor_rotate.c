/*
 * ZMK Input Processor: Skew Compensation
 *
 * Corrects asymmetric axis crosstalk on trackball sensors where
 * vertical movement bleeds into the X axis but not vice versa.
 *
 * Applies shear correction:  x' = x - (k / 100) * y,  y' = y
 *
 * Since X events arrive before Y, the correction from Y is deferred
 * to the next frame. At typical polling rates this is imperceptible.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_rotate

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/input_processor.h>

LOG_MODULE_REGISTER(skew, CONFIG_ZMK_LOG_LEVEL);

#define SCALE 100

struct rotate_config {
    uint8_t type;
    uint16_t x_code;
    uint16_t y_code;
    bool track_remainders;
};

struct rotate_data {
    int32_t skew_offset;
    int32_t pending_x_correction;
    int32_t remainder;
};

static int rotate_handle_event(const struct device *dev, struct input_event *event,
                               uint32_t param1, uint32_t param2,
                               struct zmk_input_processor_state *state) {
    const struct rotate_config *config = dev->config;
    struct rotate_data *data = dev->data;

    if (event->type != config->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == config->x_code) {
        /* DEBUG: test if pending persists across events */
        int32_t numerator = (int32_t)event->value * SCALE + data->pending_x_correction;
        event->value = numerator / SCALE;
        /* Set pending to fixed +500 → next X should be +5 */
        data->pending_x_correction = 500;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api rotate_driver_api = {
    .handle_event = rotate_handle_event,
};

void zmk_input_processor_rotate_adjust_angle(const struct device *dev, int32_t delta) {
    struct rotate_data *data = dev->data;
    data->skew_offset += delta;
    LOG_INF("Skew factor offset: %d (effective = param + %d)", data->skew_offset, data->skew_offset);
}

int32_t zmk_input_processor_rotate_get_angle(const struct device *dev) {
    struct rotate_data *data = dev->data;
    return data->skew_offset;
}

#define ROTATE_INST(n)                                                                             \
    static struct rotate_data rotate_data_##n = {};                                                \
    static const struct rotate_config rotate_config_##n = {                                        \
        .type = DT_INST_PROP(n, type),                                                             \
        .x_code = DT_INST_PROP(n, x_code),                                                        \
        .y_code = DT_INST_PROP(n, y_code),                                                         \
        .track_remainders = DT_INST_PROP(n, track_remainders),                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &rotate_data_##n, &rotate_config_##n, POST_KERNEL,        \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &rotate_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ROTATE_INST)
