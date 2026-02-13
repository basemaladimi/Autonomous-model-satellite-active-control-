#include <zephyr/random/random.h>

#ifdef CONFIG_SAY_HELLO
#include "say_hello.h"
#endif

// Settings
static const int32_t sleep_time_ms = 1000;

int main(void)
{
    uint32_t rnd;

    // Do forever
    while (1) {

        // Print random number
        rnd = sys_rand32_get();
        printk("Random value: %u\r\n", rnd);

#ifdef CONFIG_SAY_HELLO
        say_hello();
#endif

        // Sleep
        k_msleep(sleep_time_ms);
    }

    return 0;
}