#include <stdio.h>
#include <zephyr/kernel.h>

#include "button.h"

// Settings
static const int32_t sleep_time_ms = 50;
static const struct device *btn_1 = DEVICE_DT_GET(DT_ALIAS(my_button_1));
static const struct device *btn_2 = DEVICE_DT_GET(DT_ALIAS(my_button_2));

int main(void)
{
    int ret;
    uint8_t state_1;
    uint8_t state_2;

    // Make sure that the button was initialized
    if (!(device_is_ready(btn_1) && device_is_ready(btn_2))) {
        printk("Error: buttons are not ready\r\n");
        return 0;
    }

    // Get the API from one of the buttons
    const struct button_api *btn_api = (const struct button_api *)btn_1->api;

    // Do forever
    while (1) {

        // Poll button 1 state
        ret = btn_api->get(btn_1, &state_1);
        if (ret < 0) {
            printk("Error (%d): failed to read button 1 pin\r\n", ret);
            continue;
        }

        // Poll button 2 state
        ret = btn_api->get(btn_2, &state_2);
        if (ret < 0) {
            printk("Error (%d): failed to read button 2 pin\r\n", ret);
            continue;
        }

        // Print button states
        printk("%u %u\r\n", state_1, state_2);

        // Sleep
        k_msleep(sleep_time_ms);
    }

    return 0;
}