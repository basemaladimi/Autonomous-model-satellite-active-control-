#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <time.h>

float A_inv[3][3] = {
    {1.011020f, -0.007368f, 0.008895f}, 
    {-0.007368f, 1.001592f, 0.002973f}, 
    {0.008895f, 0.002979f, 1.000076f}
};
float b_accel[3] = {0.145774f, -0.047434f, -0.799729f};
float pitcha = 0.0f;
float rolla = 0.0f;
float rad_to_deg = 57.29577951f;
int64_t last_time = 0;
float dt = 0.0f;
float rollg = 0.0f, pitchg = 0.0f, yawg = 0.0f;
float rollc = 0.0f, pitchc = 0.0f;
float cnfdne_wght_rol = 0.073f;
float cnfdne_wght_pitch = 0.066f;
float level_offsets[2] = {0.0f, 0.0f};

void trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    int64_t current_time = k_ticks_to_us_near64(k_uptime_ticks());
    if(last_time !=0){
        dt = (current_time - last_time) / 1000000.0f;   // Convert microseconds to seconds
    }else{
        dt = 0.0f;                                      // First reading, no time difference
    }

    last_time = current_time;

    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    float raw_a[3], raw_g[3], cal_a[3], shifted_a[3];

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);

    for (int i = 0; i < 3; i++) {
        raw_a[i] = (float)sensor_value_to_double(&accel[i]);
        raw_g[i] = (float)sensor_value_to_double(&gyro[i])*rad_to_deg;
        shifted_a[i] = raw_a[i] - b_accel[i];
    }

    for (int i = 0; i < 3; i++) {
        cal_a[i] = A_inv[i][0] * shifted_a[0] + A_inv[i][1] * shifted_a[1] + A_inv[i][2] * shifted_a[2];
    }


    rollg = rollg + (-raw_g[1])* dt;
    pitchg = pitchg + raw_g[0] * dt;
    yawg = yawg + raw_g[2] * dt;

    rolla = atan2f((cal_a[0] - level_offsets[0]), cal_a[2])*rad_to_deg;
    pitcha = atan2f((cal_a[1] - level_offsets[1]), cal_a[2])*rad_to_deg;

    rollc = rolla * cnfdne_wght_rol + (1.0f - cnfdne_wght_rol) * (rollc + (-raw_g[1])* dt);
    pitchc = pitcha * cnfdne_wght_pitch + (1.0f - cnfdne_wght_pitch) * (pitchc + raw_g[0] * dt);

    printk("rolla:%f, pitcha:%f, rollc:%f, pitchc:%f\n",
            (double)(rolla), 
            (double)(pitcha), 
            (double)(rollc),
            (double)(pitchc));
}

int main(void) {
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(my_mpu));
    struct sensor_value a_tmp[3];
    float a_sum[3] = {0.0f, 0.0f, 0.0f};
    int s = 200;

    k_msleep(500);
    if (!device_is_ready(dev)) return 0;
    for (int i = 0; i < s; i++) {
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, a_tmp);
        a_sum[0] += (float)sensor_value_to_double(&a_tmp[0]);
        a_sum[1] += (float)sensor_value_to_double(&a_tmp[1]);
        a_sum[2] += (float)sensor_value_to_double(&a_tmp[2]);
        k_msleep(5);
    }
    float ax = (a_sum[0] / s) - b_accel[0], ay = (a_sum[1] / s) - b_accel[1], az = (a_sum[2] / s) - b_accel[2];
    level_offsets[0] = A_inv[0][0] * ax + A_inv[0][1] * ay + A_inv[0][2] * az;
    level_offsets[1] = A_inv[1][0] * ax + A_inv[1][1] * ay + A_inv[1][2] * az;
    struct sensor_trigger trig = {.type = SENSOR_TRIG_DATA_READY, .chan = SENSOR_CHAN_ALL};
    sensor_trigger_set(dev, &trig, trigger_handler);

    while (1) k_sleep(K_FOREVER);
}