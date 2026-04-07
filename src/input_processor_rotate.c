/*
 * ZMK Input Processor: 2D Rotation
 *
 * Applies a rotation matrix to trackball X/Y input to correct
 * sensor axis misalignment.
 *
 * Since ZMK processes X and Y events independently, the cross-axis
 * correction term for X (-y * sin(θ)) is deferred to the next frame.
 * At typical polling rates (125Hz+) this one-frame delay is imperceptible.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_rotate

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/input_processor.h>

LOG_MODULE_REGISTER(rotate, CONFIG_ZMK_LOG_LEVEL);

#define SCALE 1024

/* sin(0°)..sin(90°) × 1024 */
static const int16_t sin_table[91] = {
       0,   18,   36,   54,   71,   89,  107,  125,  143,  160,
     178,  195,  213,  230,  248,  265,  282,  299,  316,  333,
     350,  367,  383,  400,  416,  433,  449,  465,  481,  496,
     512,  527,  543,  558,  573,  587,  602,  616,  630,  644,
     658,  672,  685,  698,  711,  724,  737,  749,  761,  773,
     784,  796,  807,  818,  828,  839,  849,  859,  869,  878,
     887,  896,  905,  913,  921,  929,  936,  943,  950,  956,
     963,  969,  974,  980,  985,  990,  994,  998, 1002, 1006,
    1009, 1012, 1014, 1017, 1019, 1020, 1022, 1023, 1023, 1024,
    1024,
};

static int32_t get_sin(int angle) {
    angle = ((angle % 360) + 360) % 360;
    if (angle <= 90) {
        return sin_table[angle];
    }
    if (angle <= 180) {
        return sin_table[180 - angle];
    }
    if (angle <= 270) {
        return -sin_table[angle - 180];
    }
    return -sin_table[360 - angle];
}

static int32_t get_cos(int angle) {
    return get_sin(angle + 90);
}

struct rotate_config {
    uint8_t type;
    uint16_t x_code;
    uint16_t y_code;
    bool track_remainders;
};

struct rotate_data {
    int32_t angle_offset;
    int32_t buffered_x;
    int32_t pending_x_correction;
    int32_t x_remainder;
    int32_t y_remainder;
};

static int rotate_handle_event(const struct device *dev, struct input_event *event,
                               uint32_t param1, uint32_t param2,
                               struct zmk_input_processor_state *state) {
    const struct rotate_config *config = dev->config;
    struct rotate_data *data = dev->data;

    if (event->type != config->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int angle = (int32_t)param1 + data->angle_offset;
    int32_t sin_val = get_sin(angle);
    int32_t cos_val = get_cos(angle);

    if (event->code == config->x_code) {
        data->buffered_x = event->value;

        /* x' = x·cos(θ) + pending correction from previous Y event */
        int32_t numerator = (int32_t)event->value * cos_val + data->pending_x_correction;
        if (config->track_remainders) {
            numerator += data->x_remainder;
        }
        event->value = numerator / SCALE;
        if (config->track_remainders) {
            data->x_remainder = numerator - event->value * SCALE;
        }
        data->pending_x_correction = 0;

    } else if (event->code == config->y_code) {
        int32_t raw_y = event->value;

        /* y' = x·sin(θ) + y·cos(θ) */
        int32_t numerator = (int32_t)data->buffered_x * sin_val + raw_y * cos_val;
        if (config->track_remainders) {
            numerator += data->y_remainder;
        }
        event->value = numerator / SCALE;
        if (config->track_remainders) {
            data->y_remainder = numerator - event->value * SCALE;
        }

        /* Deferred X correction: -y·sin(θ), applied to next frame's X event */
        data->pending_x_correction = -raw_y * sin_val;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api rotate_driver_api = {
    .handle_event = rotate_handle_event,
};

void zmk_input_processor_rotate_adjust_angle(const struct device *dev, int32_t delta) {
    struct rotate_data *data = dev->data;
    data->angle_offset += delta;
    LOG_INF("Rotation angle offset: %d", data->angle_offset);
}

int32_t zmk_input_processor_rotate_get_angle(const struct device *dev) {
    struct rotate_data *data = dev->data;
    return data->angle_offset;
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
