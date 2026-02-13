#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

// Sleep settings
static const int32_t sensor_sleep_ms = 500;
static const int32_t output_sleep_ms = 5000;

// Stack size settings
#define SENSOR_THREAD_STACK_SIZE 512
#define OUTPUT_THREAD_STACK_SIZE 1024
#define QUEUE_SIZE 20

// Define stack areas for the threads
K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(output_stack, OUTPUT_THREAD_STACK_SIZE);

// Declare thread data structs
static struct k_thread sensor_thread;
static struct k_thread output_thread;

// Define queue
K_MSGQ_DEFINE(my_queue, sizeof(struct sensor_value), QUEUE_SIZE, 4);

// Sensor thread entry point
void sensor_thread_start(void *sensor, void *arg2, void *arg3)
{
	int ret;
	struct sensor_value temperature;

	// Cast sensor handle back to device struct pointer
	const struct device *mcp = (const struct device *)sensor;

	// Check if the MCP9808 has been initialized (init function called)
	if (!device_is_ready(mcp)) {
		printk("Device %s is not ready.\n", mcp->name);
		return;
	}

	printk("Starting sensor thread\r\n");

	while (1) {

		// Sleep early to prevent fast loop on failure
		k_msleep(sensor_sleep_ms);

		// Fetch the value from the sensor into the device's data struct
        ret = sensor_sample_fetch(mcp);
        if (ret < 0) {
            printk("Sample fetch error: %d\n", ret);
            continue;
        }

        // Copy the value from the device's data struct into the local variable
        ret = sensor_channel_get(mcp, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
        if (ret < 0) {
            printk("Channel get error: %d\n", ret);
            continue;
        }

		// Put message on queue
		printk("Putting message on queue\r\n");
		ret = k_msgq_put(&my_queue, &temperature, K_NO_WAIT);
		if (ret < 0) {
			printk("Queue full\r\n");
		}
	}
}

// Output thread entry point
void output_thread_start(void *arg1, void *arg2, void *arg3)
{
	struct sensor_value temperature;

	printk("Starting output thread\r\n");

	while (1) {

		// Let the queue fill up
		k_msleep(output_sleep_ms);

		// Print all messages in queue
		while (k_msgq_num_used_get(&my_queue) > 0) {
			if (k_msgq_get(&my_queue, &temperature, K_FOREVER) == 0) {
				printk("Temperature: %d.%06d\n", 
					   temperature.val1, 
					   temperature.val2);
			}
		}
	}
}

int main(void) 
{
	k_tid_t sensor_tid;
	k_tid_t output_tid;

	// Get Devicetree configuration for sensor
	const struct device *const mcp = DEVICE_DT_GET(DT_ALIAS(my_mcp9808));

	// Start the sensor thread
	sensor_tid = k_thread_create(&sensor_thread,		// Thread struct
								sensor_stack,			// Stack
								K_THREAD_STACK_SIZEOF(sensor_stack),
								sensor_thread_start,	// Entry point
								(void *)mcp,			// Pass in sensor handle
								NULL,					// arg_2
								NULL,					// arg_3
								8,						// Priority
								0,						// Options
								K_NO_WAIT);				// Delay

	// Start the output thread
	output_tid = k_thread_create(&output_thread,		// Thread struct
								output_stack,			// Stack
								K_THREAD_STACK_SIZEOF(output_stack),
								output_thread_start,	// Entry point
								NULL,					// arg_1
								NULL,					// arg_2
								NULL,					// arg_3
								7,						// Priority
								0,						// Options
								K_NO_WAIT);				// Delay

	// Do nothing
	while (1) {
		k_msleep(1000);
	}

	return 0;
}