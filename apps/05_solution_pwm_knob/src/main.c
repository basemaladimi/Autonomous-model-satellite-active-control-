#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>

// Settings
static const int32_t sleep_time_ms = 10;

// Get Devicetree configurations
#define MY_ADC_CH DT_ALIAS(my_adc_channel)
static const struct device *adc = DEVICE_DT_GET(DT_ALIAS(my_adc));
static const struct adc_channel_cfg adc_ch = ADC_CHANNEL_CFG_DT(MY_ADC_CH);
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(led_0));

int main(void)
{
	int ret;
	uint16_t buf;
	int32_t vref_mv;
	uint32_t pulse_ns;

	// Get Vref (mV) from Devicetree property
	vref_mv = DT_PROP(MY_ADC_CH, zephyr_vref_mv);

	// Buffer and options for ADC (defined in adc.h)
	struct adc_sequence seq = {
		.channels = BIT(adc_ch.channel_id),
		.buffer = &buf,
		.buffer_size = sizeof(buf),
		.resolution = DT_PROP(MY_ADC_CH, zephyr_resolution)
	};

	// Make sure that the ADC was initialized
	if (!device_is_ready(adc)) {
		printk("ADC peripheral is not ready\r\n");
		return 0;
	}

	// Make sure that the PWM LED was initialized
	if (!pwm_is_ready_dt(&pwm_led)) {
		printk("PWM is not ready\r\n");
		return 0;
	}

	// Configure ADC channel
	ret = adc_channel_setup(adc, &adc_ch);
	if (ret < 0) {
		printk("Could not set up ADC\r\n");
		return 0;
	}

	// Do forever
	while (1) {

		// Sample ADC
		ret = adc_read(adc, &seq);
		if (ret < 0) {
			printk("Could not read ADC: %d\r\n", ret);
			continue;
		}

		// Calculate pulse width
		pulse_ns = pwm_led.period / ((1 << seq.resolution) / (float)buf);
		printk("Pulse: %u ns\r\n", pulse_ns);

		// Set LED PWM pulse width
		ret = pwm_set_dt(&pwm_led, pwm_led.period, pulse_ns);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return 0;
		}

		// Sleep
		k_msleep(sleep_time_ms);
	}

	return 0;
}
