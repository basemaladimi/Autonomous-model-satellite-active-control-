#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

float A_inv[3][3] = {
    {1.011020f, -0.007368f, 0.008895f}, 
    {-0.007368f, 1.001592f, 0.002973f}, 
    {0.008895f, 0.002979f, 1.000076f}
};
float b_accel[3] = {0.145774f, -0.047434f, -0.799729f};

void trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    struct sensor_value accel[3];
    float raw_a[3], cal_a[3], shifted_a[3];

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

    for (int i = 0; i < 3; i++) {
        raw_a[i] = (float)sensor_value_to_double(&accel[i]);
        shifted_a[i] = raw_a[i] - b_accel[i];
    }

    for (int i = 0; i < 3; i++) {
        cal_a[i] = A_inv[i][0] * shifted_a[0] + A_inv[i][1] * shifted_a[1] + A_inv[i][2] * shifted_a[2];
    }

    printk("%f,%f,%f\n", 
           (double)(cal_a[0]), 
           (double)(cal_a[1]), 
           (double)(cal_a[2]));
}

int main(void) {
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(my_mpu));
    k_msleep(500);
    if (!device_is_ready(dev)) return 0;

    struct sensor_trigger trig = {.type = SENSOR_TRIG_DATA_READY, .chan = SENSOR_CHAN_ALL};
    sensor_trigger_set(dev, &trig, trigger_handler);

    while (1) k_sleep(K_FOREVER);
}