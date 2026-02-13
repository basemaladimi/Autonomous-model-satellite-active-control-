#include <stdio.h>
#include <zephyr/kernel.h>

// Settings
#define TIMER_MS 1000

// Struct for holding timer information
static struct k_timer my_timer;

// Timer callback
void timer_callback(struct k_timer *timer)
{
    // Check to make sure the correct timer triggered
    if (timer == &my_timer) {
        printk("Timer!\r\n");
    }
}

int main(void)
{
    // Initialize the timer
    k_timer_init(&my_timer, timer_callback, NULL);

    // Wait 1 sec (duration) before calling the callback and then call the
    // callback every 1 sec after (period)
    k_timer_start(&my_timer, K_MSEC(TIMER_MS), K_MSEC(TIMER_MS));

    // Do nothing
    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}