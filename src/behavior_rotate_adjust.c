/*
 * ZMK Behavior: Rotate Adjust
 *
 * Adjusts the rotation angle of an input-processor-rotate at runtime.
 * Two zero-param behaviors (inc / dec) for keymap editor compatibility.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_rotate_adjust

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include "input_processor_rotate.h"

struct rotate_adjust_config {
    const struct device *processor;
    int32_t step;
};

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct rotate_adjust_config *config = dev->config;
    zmk_input_processor_rotate_adjust_angle(config->processor, config->step);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api rotate_adjust_behavior_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define ROTATE_ADJUST_INST(n)                                                                      \
    static const struct rotate_adjust_config rotate_adjust_config_##n = {                          \
        .processor = DEVICE_DT_GET(DT_INST_PHANDLE(n, processor)),                                 \
        .step = DT_INST_PROP(n, step),                                                             \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, &rotate_adjust_config_##n, POST_KERNEL,           \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &rotate_adjust_behavior_api);

DT_INST_FOREACH_STATUS_OKAY(ROTATE_ADJUST_INST)
