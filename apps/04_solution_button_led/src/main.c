#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// Settings
static const int32_t sleep_time_ms = 100;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(my_led), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(my_button), gpios);

int main(void)
{
	int ret;
	int state;

	// Make sure that the LED was initialized
	if (!gpio_is_ready_dt(&led)) {
		printk("ERROR: LED not ready\r\n");
		return 0;
	}

	// Make sure that the button was initialized
	if (!gpio_is_ready_dt(&btn)) {
		printk("ERROR: button not ready\r\n");
		return 0;
	}

	// Set the LED as output (apply extra flags if needed)
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	if (ret < 0) {
		return 0;
	}

	// Set the button as input (apply extra flags if needed)
	ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
	if (ret < 0) {
		return 0;
	}

	// Print out flags
	printk("LED spec flags: 0x%x\r\n", led.dt_flags);
	printk("Button spec flags: 0x%x\r\n", btn.dt_flags);
	
	// Do forever
	while (1) {

		// Poll button state
		state = gpio_pin_get_dt(&btn);
        if (state < 0) {
            printk("Error %d: failed to read button pin\r\n", state);
        } else {
            printk("Button state: %d\r\n", state);
        }
		
		// Set LED state
		ret = gpio_pin_set_dt(&led, state);
		if (ret < 0) {
			return 0;
		}

		// Sleep
		k_msleep(sleep_time_ms);
	}

	return 0;
}
