#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>

// Sleep settings
static const int32_t blink_max_ms = 2000;
static const int32_t blink_min_ms = 0;

// Stack size settings
#define BLINK_THREAD_STACK_SIZE 512
#define INPUT_THREAD_STACK_SIZE 512

// Define stack areas for the threads
K_THREAD_STACK_DEFINE(blink_stack, BLINK_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(input_stack, INPUT_THREAD_STACK_SIZE);

// Declare thread data structs
static struct k_thread blink_thread;
static struct k_thread input_thread;

// Define mutex
K_MUTEX_DEFINE(my_mutex);

// Define shared blink sleep value
static int32_t blink_sleep_ms = 500;

// Get LED struct from Devicetree
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(my_led), gpios);

// Console input thread entry point
void input_thread_start(void *arg_1, void *arg_2, void *arg_3)
{
    int8_t inc;

    printk("Starting input thread\r\n");

    while (1) {

        // Get line from console (blocking)
        const char *line = console_getline();

        // See if first character is + or -
        if (line[0] == '+') {
            inc = 1;
        } else if (line[0] == '-') {
            inc = -1;
        } else {
            continue;
        }

        // Increase or decrease wait time and bound the value
        // Use the mutex to prvent other threads from accessing the variable
        // during the update
        k_mutex_lock(&my_mutex, K_FOREVER);
        blink_sleep_ms += (int32_t)inc * 100;
        if (blink_sleep_ms > blink_max_ms) {
            blink_sleep_ms = blink_max_ms;
        } else if (blink_sleep_ms < blink_min_ms) {
            blink_sleep_ms = blink_min_ms;
        }
        k_mutex_unlock(&my_mutex);

        // Print the new sleep time
        printk("Updating blink sleep to: %d\r\n", blink_sleep_ms);
    }
}

// Blink thread entry point
void blink_thread_start(void *arg_1, void *arg_2, void *arg_3)
{
    int ret;
    int state = 0;
    int32_t sleep_ms;

    printk("Starting blink thread\r\n");

    while (1) {

        // Update the sleep time. Protected with a mutex.
        k_mutex_lock(&my_mutex, K_FOREVER);
        sleep_ms = blink_sleep_ms;
        k_mutex_unlock(&my_mutex);

        // Change the state of the pin
        state = !state;

        // Set pin state
        ret = gpio_pin_set_dt(&led, state);
        if (ret < 0) {
            printk("Error: could not toggle pin\r\n");
        }

        k_msleep(sleep_ms);
    }
}

int main(void)
{
    int ret;
    k_tid_t input_tid;
    k_tid_t blink_tid;

    // Make sure that the GPIO was initialized
    if (!gpio_is_ready_dt(&led)) {
        printk("Error: GPIO pin not ready\r\n");
        return 0;
    }

    // Set the GPIO as output
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
    if (ret < 0) {
        printk("Error: Could not configure GPIO\r\n");
        return 0;
    }

    // Initialize the console
    console_getline_init();

    // Start the input thread
    input_tid = k_thread_create(&input_thread,          // Thread struct
                                input_stack,            // Stack
                                K_THREAD_STACK_SIZEOF(input_stack),
                                input_thread_start,     // Entry point
                                NULL,                   // arg_1
                                NULL,                   // arg_2
                                NULL,                   // arg_3
                                7,                      // Priority
                                0,                      // Options
                                K_NO_WAIT);             // Delay

    // Start the blink thread
    blink_tid = k_thread_create(&blink_thread,          // Thread struct
                                blink_stack,            // Stack
                                K_THREAD_STACK_SIZEOF(blink_stack),
                                blink_thread_start,     // Entry point
                                NULL,                   // arg_1
                                NULL,                   // arg_2
                                NULL,                   // arg_3
                                8,                      // Priority
                                0,                      // Options
                                K_NO_WAIT);             // Delay

    // Do forever
    while (1) {
        k_msleep(1000);
    }

    return 0;
}