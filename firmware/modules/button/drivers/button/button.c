// Ties to the 'compatible = "custom,button"' node in the Devicetree
#define DT_DRV_COMPAT custom_button

#include <errno.h>
#include <zephyr/logging/log.h>

#include "button.h"

// Enable logging at CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(button);

//------------------------------------------------------------------------------
// Forward declarations

static int button_init(const struct device *dev);
static int button_state_get(const struct device *dev, uint8_t *state);

//------------------------------------------------------------------------------
// Private functions

// Initialize the button
static int button_init(const struct device *dev)
{
    int ret;

    // Cast device.config (declared const void *) to our button's config
    const struct button_config *cfg = (const struct button_config *)dev->config;

    // Get the button struct from the config
    const struct gpio_dt_spec *btn = &cfg->btn;

    // Print to console
    LOG_DBG("Initializing button (instance ID: %u)\r\n", cfg->id);

    // Check that the button device is ready
    if (!gpio_is_ready_dt(btn)) {
        LOG_ERR("GPIO is not ready\r\n");
        return -ENODEV;
    }

    // Set the button as input (apply extra flags if needed)
    ret = gpio_pin_configure_dt(btn, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Could not configure GPIO as input\r\n");
        return -ENODEV;
    }

    return 0;
}

//------------------------------------------------------------------------------
// Public functions (API)

// Get state from button
static int button_state_get(const struct device *dev, uint8_t *state)
{
    int ret;

    // Cast device config (declared const void *) to our button's config
    const struct button_config *cfg = (const struct button_config *)dev->config;

    // Get the button struct from the config
    const struct gpio_dt_spec *btn = &cfg->btn;

    // Poll button state
    ret = gpio_pin_get_dt(btn);
    if (ret < 0) {
        LOG_ERR("Error (%d): failed to read button pin\r\n", ret);
        return ret;
    } else {
        *state = ret;
    }

    return 0;
}

//------------------------------------------------------------------------------
// Devicetree handling

// Define the public API functions for the driver
static const struct button_api button_api_funcs = {
    .get = button_state_get,
};

// Expansion macro to define driver instances
#define BUTTON_DEFINE(inst)                                                 \
                                                                            \
    /* Create an instance of the config struct, populate with DT values */  \
    static const struct button_config button_config_##inst = {              \
        .btn = GPIO_DT_SPEC_GET(                                            \
            DT_PHANDLE(DT_INST(inst, custom_button), pin), gpios),          \
        .id = inst                                                          \
    };                                                                      \
                                                                            \
    /* Create a "device" instance from a Devicetree node identifier and */  \
    /* registers the init function to run during boot. */                   \
    DEVICE_DT_INST_DEFINE(inst,                                             \
                          button_init,                                      \
                          NULL,                                             \
                          NULL,                                             \
                          &button_config_##inst,                            \
                          POST_KERNEL,                                      \
                          CONFIG_GPIO_INIT_PRIORITY,                        \
                          &button_api_funcs);                               \

// The Devicetree build process calls this to create an instance of structs for
// each device (button) defined in the Devicetree
DT_INST_FOREACH_STATUS_OKAY(BUTTON_DEFINE)
