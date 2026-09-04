/*
 * Copyright (c) 2026
 */

#define DT_DRV_COMPAT lenotp

#include <errno.h>
#include <stdint.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include <lenotp.h>

LOG_MODULE_REGISTER(lenotp, CONFIG_INPUT_LOG_LEVEL);

#define LENOTP_REPORT_LEN 7
#define LENOTP_REPORT_ID 0x07
#define LENOTP_REPORT_BYTE1 0x00
#define LENOTP_REPORT_BYTE2 0x01

#define LENOTP_BUTTON_MASK GENMASK(2, 0)
#define LENOTP_LEFT_BUTTON BIT(0)
#define LENOTP_RIGHT_BUTTON BIT(1)
#define LENOTP_MIDDLE_BUTTON BIT(2)

#define LENOTP_SPEED_MIN 1
#define LENOTP_SPEED_MAX 100

struct lenotp_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
    struct gpio_dt_spec power_gpio;
    uint16_t startup_delay_ms;
    uint16_t poll_period_ms;
    uint8_t x_divisor;
    uint8_t y_divisor;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
    bool ignore_buttons;
    bool wakeup_on_init;
    bool reset_on_init;
    bool exit_idle_on_init;
    bool sleep_on_suspend;
    bool wakeup_on_resume;
    bool power_off_on_suspend;
};

struct lenotp_data {
    const struct device *dev;
    struct gpio_callback int_cb;
    struct k_work_delayable work;
    struct k_mutex lock;
    uint8_t buttons;
    bool suspended;
};

struct lenotp_report {
    uint8_t buttons;
    int16_t x;
    int16_t y;
};

static int lenotp_send_command(const struct device *dev, const uint8_t *command,
                               size_t command_len) {
    const struct lenotp_config *config = dev->config;
    struct lenotp_data *data = dev->data;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = i2c_write_dt(&config->i2c, command, command_len);
    k_mutex_unlock(&data->lock);

    return ret;
}

int lenotp_sleep(const struct device *dev) {
    static const uint8_t command[] = {0x22, 0x00, 0x01, 0x08};

    return lenotp_send_command(dev, command, sizeof(command));
}

int lenotp_wakeup(const struct device *dev) {
    static const uint8_t command[] = {0x22, 0x00, 0x00, 0x08};

    return lenotp_send_command(dev, command, sizeof(command));
}

int lenotp_reset(const struct device *dev) {
    static const uint8_t command[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x77, 0x77, 0x77};

    return lenotp_send_command(dev, command, sizeof(command));
}

int lenotp_enter_idle(const struct device *dev) {
    static const uint8_t command[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x06, 0x06, 0x01};

    return lenotp_send_command(dev, command, sizeof(command));
}

int lenotp_exit_idle(const struct device *dev) {
    static const uint8_t command[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x06, 0x06, 0x00};

    return lenotp_send_command(dev, command, sizeof(command));
}

int lenotp_set_speed(const struct device *dev, uint8_t up, uint8_t down, uint8_t left,
                     uint8_t right) {
    if (!IN_RANGE(up, LENOTP_SPEED_MIN, LENOTP_SPEED_MAX) ||
        !IN_RANGE(down, LENOTP_SPEED_MIN, LENOTP_SPEED_MAX) ||
        !IN_RANGE(left, LENOTP_SPEED_MIN, LENOTP_SPEED_MAX) ||
        !IN_RANGE(right, LENOTP_SPEED_MIN, LENOTP_SPEED_MAX)) {
        return -EINVAL;
    }

    uint8_t vertical_command[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x42, up, down};
    int ret = lenotp_send_command(dev, vertical_command, sizeof(vertical_command));
    if (ret < 0) {
        return ret;
    }

    uint8_t horizontal_command[] = {0x25, 0x00, 0x06, 0x00, 0x29, 0x43, left, right};

    return lenotp_send_command(dev, horizontal_command, sizeof(horizontal_command));
}

static int lenotp_interrupt_configure(const struct device *dev, gpio_flags_t flags) {
    const struct lenotp_config *config = dev->config;

    return gpio_pin_interrupt_configure_dt(&config->int_gpio, flags);
}

static int lenotp_interrupt_enable(const struct device *dev) {
    return lenotp_interrupt_configure(dev, GPIO_INT_EDGE_TO_ACTIVE);
}

static int lenotp_interrupt_disable(const struct device *dev) {
    return lenotp_interrupt_configure(dev, GPIO_INT_DISABLE);
}

static bool lenotp_has_power_gpio(const struct device *dev) {
    const struct lenotp_config *config = dev->config;

    return config->power_gpio.port != NULL;
}

static int lenotp_power_on(const struct device *dev) {
    const struct lenotp_config *config = dev->config;
    int ret;

    if (!lenotp_has_power_gpio(dev)) {
        return 0;
    }

    if (!gpio_is_ready_dt(&config->power_gpio)) {
        LOG_ERR("Power GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to enable power GPIO: %d", ret);
    }

    return ret;
}

static int lenotp_power_off(const struct device *dev) {
    const struct lenotp_config *config = dev->config;
    int ret;

    if (!config->power_off_on_suspend || !lenotp_has_power_gpio(dev)) {
        return 0;
    }

    ret = gpio_pin_set_dt(&config->power_gpio, 0);
    if (ret < 0) {
        LOG_ERR("Failed to disable power GPIO: %d", ret);
    }

    return ret;
}

static int lenotp_configure_int_input(const struct device *dev) {
    const struct lenotp_config *config = dev->config;

    return gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
}

static int lenotp_disconnect_int(const struct device *dev) {
    const struct lenotp_config *config = dev->config;
    int ret;

    ret = gpio_pin_configure(config->int_gpio.port, config->int_gpio.pin, GPIO_DISCONNECTED);
    if (ret == 0) {
        return 0;
    }

    LOG_WRN("Failed to disconnect INT GPIO, leaving it as input: %d", ret);

    return lenotp_configure_int_input(dev);
}

static int16_t lenotp_scale(int16_t value, uint8_t divisor) {
    if (divisor <= 1 || value == 0) {
        return value;
    }

    int16_t scaled = value / divisor;

    if (scaled == 0) {
        return value > 0 ? 1 : -1;
    }

    return scaled;
}

static int lenotp_read_report(const struct device *dev, struct lenotp_report *report) {
    const struct lenotp_config *config = dev->config;
    struct lenotp_data *data = dev->data;
    uint8_t raw[LENOTP_REPORT_LEN];
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    ret = i2c_read_dt(&config->i2c, raw, sizeof(raw));
    k_mutex_unlock(&data->lock);

    if (ret < 0) {
        return ret;
    }

    if (raw[0] != LENOTP_REPORT_ID || raw[1] != LENOTP_REPORT_BYTE1 ||
        raw[2] != LENOTP_REPORT_BYTE2) {
        LOG_WRN("Invalid report header: %02x %02x %02x", raw[0], raw[1], raw[2]);
        return -EBADMSG;
    }

    int16_t x = (int8_t)raw[4];
    int16_t y = (int8_t)raw[5];

    if (config->swap_xy) {
        int16_t tmp = x;

        x = y;
        y = tmp;
    }

    if (config->invert_x) {
        x = -x;
    }

    if (config->invert_y) {
        y = -y;
    }

    report->buttons = raw[3] & LENOTP_BUTTON_MASK;
    report->x = lenotp_scale(x, config->x_divisor);
    report->y = lenotp_scale(y, config->y_divisor);

    return 0;
}

static uint8_t lenotp_count_events(const struct lenotp_report *report, uint8_t changed_buttons,
                                   bool ignore_buttons) {
    uint8_t count = 0;

    if (!ignore_buttons) {
        for (int i = 0; i < 3; i++) {
            if (changed_buttons & BIT(i)) {
                count++;
            }
        }
    }

    if (report->x != 0) {
        count++;
    }

    if (report->y != 0) {
        count++;
    }

    return count;
}

static uint16_t lenotp_button_code(uint8_t button) {
    switch (button) {
    case LENOTP_LEFT_BUTTON:
        return INPUT_BTN_0;
    case LENOTP_RIGHT_BUTTON:
        return INPUT_BTN_1;
    case LENOTP_MIDDLE_BUTTON:
        return INPUT_BTN_2;
    default:
        return INPUT_BTN_0;
    }
}

static void lenotp_report_input(const struct device *dev, const struct lenotp_report *report) {
    const struct lenotp_config *config = dev->config;
    struct lenotp_data *data = dev->data;
    uint8_t changed_buttons = report->buttons ^ data->buttons;
    uint8_t event_count = lenotp_count_events(report, changed_buttons, config->ignore_buttons);
    uint8_t event_index = 0;
    int ret;

    if (event_count == 0) {
        return;
    }

    if (!config->ignore_buttons) {
        for (int i = 0; i < 3; i++) {
            uint8_t button = BIT(i);

            if ((changed_buttons & button) == 0) {
                continue;
            }

            ret = input_report_key(dev, lenotp_button_code(button), (report->buttons & button) != 0,
                                   ++event_index == event_count, K_FOREVER);
            if (ret < 0) {
                LOG_WRN("Failed to report button %d: %d", i, ret);
            }
        }

        data->buttons = report->buttons;
    }

    if (report->x != 0) {
        ret =
            input_report_rel(dev, INPUT_REL_X, report->x, ++event_index == event_count, K_FOREVER);
        if (ret < 0) {
            LOG_WRN("Failed to report X movement: %d", ret);
        }
    }

    if (report->y != 0) {
        ret =
            input_report_rel(dev, INPUT_REL_Y, report->y, ++event_index == event_count, K_FOREVER);
        if (ret < 0) {
            LOG_WRN("Failed to report Y movement: %d", ret);
        }
    }
}

static bool lenotp_reschedule_if_int_active(const struct device *dev) {
    const struct lenotp_config *config = dev->config;
    struct lenotp_data *data = dev->data;
    int active;

    if (data->suspended) {
        return false;
    }

    active = gpio_pin_get_dt(&config->int_gpio);
    if (active < 0) {
        LOG_WRN("Failed to read INT pin: %d", active);
        return false;
    }

    if (active == 0) {
        return false;
    }

    k_work_reschedule(&data->work, K_MSEC(config->poll_period_ms));

    return true;
}

static void lenotp_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct lenotp_data *data = CONTAINER_OF(dwork, struct lenotp_data, work);
    const struct device *dev = data->dev;
    const struct lenotp_config *config = dev->config;
    struct lenotp_report report;
    int active;
    int ret;

    if (data->suspended) {
        return;
    }

    active = gpio_pin_get_dt(&config->int_gpio);
    if (active < 0) {
        LOG_WRN("Failed to read INT pin: %d", active);
        goto enable_interrupt;
    }

    if (active == 0) {
        goto enable_interrupt;
    }

    ret = lenotp_read_report(dev, &report);
    if (ret < 0) {
        LOG_WRN("Failed to read report: %d", ret);
        if (lenotp_reschedule_if_int_active(dev)) {
            return;
        }
        goto enable_interrupt;
    }

    LOG_DBG("buttons=%02x x=%d y=%d", report.buttons, report.x, report.y);
    lenotp_report_input(dev, &report);

    if (lenotp_reschedule_if_int_active(dev)) {
        return;
    }

enable_interrupt:
    if (data->suspended) {
        return;
    }

    ret = lenotp_interrupt_enable(dev);
    if (ret < 0) {
        LOG_WRN("Failed to enable INT interrupt: %d", ret);
    }
}

static void lenotp_int_handler(const struct device *gpio_dev, struct gpio_callback *cb,
                               uint32_t pins) {
    struct lenotp_data *data = CONTAINER_OF(cb, struct lenotp_data, int_cb);

    ARG_UNUSED(gpio_dev);
    ARG_UNUSED(pins);

    if (data->suspended) {
        return;
    }

    lenotp_interrupt_disable(data->dev);
    k_work_reschedule(&data->work, K_NO_WAIT);
}

static int lenotp_run_init_commands(const struct device *dev) {
    const struct lenotp_config *config = dev->config;
    int ret;

    if (config->reset_on_init) {
        ret = lenotp_reset(dev);
        if (ret < 0) {
            return ret;
        }
        k_msleep(10);
    }

    if (config->wakeup_on_init) {
        ret = lenotp_wakeup(dev);
        if (ret < 0) {
            return ret;
        }
    }

    if (config->exit_idle_on_init) {
        ret = lenotp_exit_idle(dev);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

static int lenotp_init(const struct device *dev) {
    const struct lenotp_config *config = dev->config;
    struct lenotp_data *data = dev->data;
    int ret;

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&config->int_gpio)) {
        LOG_ERR("INT GPIO not ready");
        return -ENODEV;
    }

    if (config->x_divisor == 0 || config->y_divisor == 0 || config->poll_period_ms == 0) {
        LOG_ERR("Movement divisors and poll period must be greater than zero");
        return -EINVAL;
    }

    ret = lenotp_power_on(dev);
    if (ret < 0) {
        return ret;
    }

    data->dev = dev;
    data->buttons = 0;
    data->suspended = false;
    k_mutex_init(&data->lock);
    k_work_init_delayable(&data->work, lenotp_work_handler);

    ret = lenotp_configure_int_input(dev);
    if (ret < 0) {
        LOG_ERR("Failed to configure INT GPIO: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->int_cb, lenotp_int_handler, BIT(config->int_gpio.pin));
    ret = gpio_add_callback(config->int_gpio.port, &data->int_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add INT GPIO callback: %d", ret);
        return ret;
    }

    if (config->startup_delay_ms > 0) {
        k_msleep(config->startup_delay_ms);
    }

    ret = lenotp_run_init_commands(dev);
    if (ret < 0) {
        LOG_ERR("Failed to run init command: %d", ret);
        return ret;
    }

    ret = lenotp_interrupt_enable(dev);
    if (ret < 0) {
        LOG_ERR("Failed to enable INT interrupt: %d", ret);
        return ret;
    }

    if (gpio_pin_get_dt(&config->int_gpio) > 0) {
        lenotp_interrupt_disable(dev);
        k_work_reschedule(&data->work, K_NO_WAIT);
    }

    LOG_INF("lenoTP input initialized");

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int lenotp_pm_action(const struct device *dev, enum pm_device_action action) {
    const struct lenotp_config *config = dev->config;
    struct lenotp_data *data = dev->data;
    struct k_work_sync sync;
    int ret;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        data->suspended = true;
        k_work_cancel_delayable_sync(&data->work, &sync);

        ret = lenotp_interrupt_disable(dev);
        if (ret < 0) {
            LOG_ERR("Failed to disable INT interrupt: %d", ret);
            return ret;
        }

        if (config->sleep_on_suspend &&
            !(config->power_off_on_suspend && lenotp_has_power_gpio(dev))) {
            ret = lenotp_sleep(dev);
            if (ret < 0) {
                LOG_WRN("Failed to send sleep command: %d", ret);
            }
        }

        ret = lenotp_disconnect_int(dev);
        if (ret < 0) {
            LOG_ERR("Failed to disconnect INT GPIO: %d", ret);
            return ret;
        }

        return lenotp_power_off(dev);

    case PM_DEVICE_ACTION_RESUME:
        ret = lenotp_power_on(dev);
        if (ret < 0) {
            return ret;
        }

        if (config->startup_delay_ms > 0 &&
            (config->power_off_on_suspend || config->wakeup_on_resume ||
             config->sleep_on_suspend)) {
            k_msleep(config->startup_delay_ms);
        }

        ret = lenotp_configure_int_input(dev);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT GPIO: %d", ret);
            return ret;
        }

        if (config->wakeup_on_resume || config->sleep_on_suspend) {
            ret = lenotp_wakeup(dev);
            if (ret < 0) {
                LOG_WRN("Failed to send wakeup command: %d", ret);
            }
        }

        data->suspended = false;

        ret = lenotp_interrupt_enable(dev);
        if (ret < 0) {
            LOG_ERR("Failed to enable INT interrupt: %d", ret);
            return ret;
        }

        if (gpio_pin_get_dt(&config->int_gpio) > 0) {
            lenotp_interrupt_disable(dev);
            k_work_reschedule(&data->work, K_NO_WAIT);
        }

        return 0;

    default:
        return -ENOTSUP;
    }
}
#endif /* CONFIG_PM_DEVICE */

#define LENOTP_INIT(inst)                                                                          \
    static struct lenotp_data lenotp_data_##inst;                                                  \
    static const struct lenotp_config lenotp_config_##inst = {                                     \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                                        \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {0}),                            \
        .startup_delay_ms = DT_INST_PROP(inst, startup_delay_ms),                                  \
        .poll_period_ms = DT_INST_PROP(inst, poll_period_ms),                                      \
        .x_divisor = DT_INST_PROP(inst, x_divisor),                                                \
        .y_divisor = DT_INST_PROP(inst, y_divisor),                                                \
        .invert_x = DT_INST_PROP(inst, invert_x),                                                  \
        .invert_y = DT_INST_PROP(inst, invert_y),                                                  \
        .swap_xy = DT_INST_PROP(inst, swap_xy),                                                    \
        .ignore_buttons = DT_INST_PROP(inst, ignore_buttons),                                      \
        .wakeup_on_init = DT_INST_PROP(inst, wakeup_on_init),                                      \
        .reset_on_init = DT_INST_PROP(inst, reset_on_init),                                        \
        .exit_idle_on_init = DT_INST_PROP(inst, exit_idle_on_init),                                \
        .sleep_on_suspend = DT_INST_PROP(inst, sleep_on_suspend),                                  \
        .wakeup_on_resume = DT_INST_PROP(inst, wakeup_on_resume),                                  \
        .power_off_on_suspend = DT_INST_PROP(inst, power_off_on_suspend),                          \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(inst, lenotp_pm_action);                                              \
    DEVICE_DT_INST_DEFINE(inst, lenotp_init, PM_DEVICE_DT_INST_GET(inst), &lenotp_data_##inst,     \
                          &lenotp_config_##inst, POST_KERNEL, CONFIG_INPUT_LENOTP_INIT_PRIORITY,   \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(LENOTP_INIT)
