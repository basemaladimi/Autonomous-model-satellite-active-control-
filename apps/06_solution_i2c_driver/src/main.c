#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

// Settings
static const int32_t sleep_time_ms = 1000;

// Get Devicetree configurations
static const struct device *const mcp = DEVICE_DT_GET(DT_ALIAS(my_mcp9808));

int main(void)
{
	
	int ret;

    // Check if the MCP9808 has been initialized (init function called)
	if (!device_is_ready(mcp)) {
		printk("Device %s is not ready.\n", mcp->name);
		return 0;
	}

    // Do forever
    while (1) 
    {
        struct sensor_value tmp;

        // Fetch the temperature value from the sensor into the device's data structure
        ret = sensor_sample_fetch(mcp);
        if (ret < 0) {
            printk("Sample fetch error: %d\n", ret);
            return 0;
        }

        // Copy the temperature value from the device's data structure into the tmp struct
        ret = sensor_channel_get(mcp, SENSOR_CHAN_AMBIENT_TEMP, &tmp);
        if (ret < 0) {
            printk("Channel get error: %d\n", ret);
            return 0;
        }

        // Print the temperature value 
        printk("Temperature: %d.%06d\n", tmp.val1, tmp.val2);

        // Sleep
        k_msleep(sleep_time_ms);
    }
    
    return 0;
}
