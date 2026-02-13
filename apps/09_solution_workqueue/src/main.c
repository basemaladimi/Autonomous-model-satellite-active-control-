#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// Settings
#define DEBOUNCE_DELAY_MS 50

// Button struct (from Devicetree)
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(my_button), gpios);

// Struct for holding GPIO-related callback functions
static struct gpio_callback btn_cb_data;

// Struct for holding the workqueue
static struct k_work_delayable button_work;

// GPIO callback (ISR)
void button_isr(const struct device *dev, 
                struct gpio_callback *cb, 
                uint32_t pins)
{
    // Add work to the workqueue
    k_work_reschedule(&button_work, K_MSEC(DEBOUNCE_DELAY_MS));
}

// Work handler: button pressed
void button_work_handler(struct k_work *work)
{
    int state;

    // Read the state of the button (after the debounce delay)
    state = gpio_pin_get_dt(&btn);
    if (state < 0) {
        printk("Error (%d): failed to read button pin\r\n", state);
    } else if (state) {
        printk("Doing some work...now with debounce!\r\n");
    }
}

int main(void)
{
    int ret;

    // Initialize work item
    k_work_init_delayable(&button_work, button_work_handler);

    // Make sure that the button was initialized
    if (!gpio_is_ready_dt(&btn)) {
        printk("ERROR: button not ready\r\n");
        return 0;
    }

    // Set the button as input (apply extra flags if needed)
    ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
    if (ret < 0) {
        printk("ERROR: could not set button to input\r\n");
        return 0;
    }

    // Configure the interrupt
    ret = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        printk("ERROR: could not configure button as interrupt source\r\n");
        return 0;
    }

    // Connect callback function (ISR) to interrupt source
    gpio_init_callback(&btn_cb_data, button_isr, BIT(btn.pin));
	gpio_add_callback(btn.port, &btn_cb_data);

    // Do nothing
    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}