#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "say_hello.h"

// Settings
static const int32_t sleep_time_ms = 1000;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(my_led), gpios);

int main(void)
{
	int ret;
	int state = 0;

	// Make sure that the GPIO was initialized
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	// Set the GPIO as output
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	if (ret < 0) {
		return 0;
	}

	// Do forever
	while (1) {

		// Toggle the LED
		state = !state;
		ret = gpio_pin_set_dt(&led, state);
		if (ret < 0) {
			return 0;
		}

		// Say hello
		say_hello();
		
		// Sleep
		k_msleep(sleep_time_ms);
	}
	
	return 0;
}
