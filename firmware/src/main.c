    #include <zephyr/kernel.h>
    #include <zephyr/device.h>
    #include <zephyr/drivers/sensor.h>
    #include <math.h>
    #include <time.h>
    #include <zephyr/drivers/i2c.h>
    #define N 5
    #define BARO_LOOP_MS 10
    #define BARO_FC_HZ           1.5f       // 1.0 or less for more filtering, 2.0 Hz or less filtering
    #define BARO_ALPHA_MIN       0.01f
    #define BARO_ALPHA_MAX       0.80f
    #define M_PI 3.14159265358979323846
    #define BARO_BETA            0.01f    
    #define BARO_MIN_STABLE_TIME 300
    #define GYRO_STILL_THRESHOLD      2.5f
    #define ACCEL_ERROR_THRESHOLD     0.35f       
    #define STILL_ENTER_COUNT         20
    #define STILL_EXIT_COUNT          8
    #define GROUND_CAL_TIMEOUT_MS     20000
    #define THERMAL_K1_HPA_PER_C   (-0.170894f)   // from your regression
    #define THERMAL_MIN_PRESSURE   300.0f         // safety clamp
    #define THERMAL_MAX_PRESSURE   1200.0f        // safety clamp


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
    float cnfdne_wght_rol = 0.004f;
    float cnfdne_wght_pitch = 0.004f;
    float level_offsets[2] = {0.0f, 0.0f};
    float gx_cal = 0.0f, gy_cal = 0.0f, gz_cal = 0.0f;
    static float baro_ring[N];
    static int baro_idx = 0;
    static bool baro_full = false;
    static bool baro_lpf_init = false;
    static float p_lpf = 0.0f;
    static int64_t baro_last_time_us = 0;

    /* ===== Global IMU Values (updated by trigger_handler at 200 Hz) ===== */
    static float raw_g_global[3] = {0.0f, 0.0f, 0.0f};
    static float cal_a_global[3] = {0.0f, 0.0f, 0.0f};

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

        rollg = rollg + (-(raw_g[1] - gy_cal))* dt;
        pitchg = pitchg + (raw_g[0] - gx_cal)* dt;
        yawg = yawg + raw_g[2] * dt;

        rolla = atan2f((cal_a[0] - level_offsets[0]), cal_a[2])*rad_to_deg;
        pitcha = atan2f((cal_a[1] - level_offsets[1]), cal_a[2])*rad_to_deg;

        rollc = (rolla * cnfdne_wght_rol + (1.0f - cnfdne_wght_rol) * (rollc + (-(raw_g[1] - gy_cal))* dt));
        pitchc = (pitcha * cnfdne_wght_pitch + (1.0f - cnfdne_wght_pitch) * (pitchc + (raw_g[0] - gx_cal)* dt));

        for (int i = 0; i < 3; i++) {
            raw_g_global[i] = raw_g[i];
            cal_a_global[i] = cal_a[i];
        }
        // printk("rolla:%f, pitcha:%f, rollc:%f, pitchc:%f\n",
        //     (double)(rolla),
        //     (double)(pitcha),
        //     (double)(rollc),
        //     (double)(pitchc));

    }

    // Median filter to reduce sudent spikes in barometer readings
    static float median5_from_ring(const float ring[N], bool full, int idx)
    {
        float w[N];
        int count = full ? N : idx;
        if (count <= 0) {
            return 0.0f;
        }
        for (int i = 0; i < count; i++) {
            w[i] = ring[i];
        }
        for (int i = 0; i < count - 1; i++) {
            for (int j = 0; j < count - 1 - i; j++) {
                if (w[j] > w[j + 1]) {
                    float tmp = w[j];
                    w[j] = w[j + 1];
                    w[j + 1] = tmp;
                }
            }
        }
        return w[count / 2];
    }
    static float median5_sliding_update(float p_new)
    {
        baro_ring[baro_idx] = p_new;
        baro_idx = (baro_idx + 1) % N;

        if (baro_idx == 0) {
            baro_full = true;
        }

        return median5_from_ring(baro_ring, baro_full, baro_idx);
    }

    /* ---------- Barometer LPF ---------- */

    // Compute alpha from cutoff and dt: alpha = 1 - exp(-2*pi*fc*dt)
    static float baro_compute_alpha(float fc_hz, float dt_s)
    {
        float alpha = 1.0f - expf(-2.0f * (float)M_PI * fc_hz * dt_s);

        if (alpha < BARO_ALPHA_MIN) {
            alpha = BARO_ALPHA_MIN;
        }
        if (alpha > BARO_ALPHA_MAX) {
            alpha = BARO_ALPHA_MAX;
        }
        return alpha;
    }
    // One-pole IIR LPF update in pressure domain: p_lpf = alpha * p_med + (1-alpha) * p_lpf
    static float baro_lpf_update(float p_med, float fc_hz, float dt_s)
    {
        float alpha = baro_compute_alpha(fc_hz, dt_s);

        if (!baro_lpf_init) {
            p_lpf = p_med;          /* clean startup */
            baro_lpf_init = true;
        } else {
            p_lpf += alpha * (p_med - p_lpf);
        }

        return p_lpf;
    }

    /* Stationarity update: returns true when platform is considered still */
    static bool stationarity_update(int32_t *still_count,
                                    int32_t *moving_count,
                                    bool is_stationary_locked)
    {
        float g_snapshot[3];
        float a_snapshot[3];
        g_snapshot[0] = raw_g_global[0];
        g_snapshot[1] = raw_g_global[1];
        g_snapshot[2] = raw_g_global[2];
        a_snapshot[0] = cal_a_global[0];
        a_snapshot[1] = cal_a_global[1];
        a_snapshot[2] = cal_a_global[2];

        float gyro_norm = sqrtf((g_snapshot[0] - gx_cal) * (g_snapshot[0] - gx_cal) +
                                (g_snapshot[1] - gy_cal) * (g_snapshot[1] - gy_cal) +
                                (g_snapshot[2] - gz_cal) * (g_snapshot[2] - gz_cal));
        float accel_magnitude = sqrtf(a_snapshot[0] * a_snapshot[0] +
                                    a_snapshot[1] * a_snapshot[1] +
                                    a_snapshot[2] * a_snapshot[2]);
        float accel_error = fabsf(accel_magnitude - 9.81f);

        bool still_now = (gyro_norm < GYRO_STILL_THRESHOLD) &&
                        (accel_error < ACCEL_ERROR_THRESHOLD);

        if (!is_stationary_locked) {
            if (still_now) {
                (*still_count)++;
                *moving_count = 0;
                if (*still_count >= STILL_ENTER_COUNT) {
                    is_stationary_locked = true;
                    *still_count = 0;
                }
            } else {
                *still_count = 0;
            }
        } else {
            if (!still_now) {
                (*moving_count)++;
                *still_count = 0;
                if (*moving_count >= STILL_EXIT_COUNT) {
                    is_stationary_locked = false;
                    *moving_count = 0;
                }
            } else {
                *moving_count = 0;
            }
        }

        return is_stationary_locked;
    }

    /* Ground calibration update: adapts and locks baseline pressure */
    static void ground_cal_update(bool is_stationary_locked,
                                float p_lpf_sample,
                                int64_t now_us,
                                int64_t ground_cal_start_us,
                                bool *in_GROUND_CAL,
                                bool *baseline_locked,
                                float *p_ref_adaptive,
                                int32_t *ground_cal_samples)
    {
        if (!(*in_GROUND_CAL) || *baseline_locked) {
            return;
        }

        if (is_stationary_locked) {
            *p_ref_adaptive += BARO_BETA * (p_lpf_sample - *p_ref_adaptive);
            (*ground_cal_samples)++;
        } else {
            *ground_cal_samples = 0;
            printk("GROUND_CAL reset due to movement\n");
        }

        int64_t cal_elapsed_ms = (now_us - ground_cal_start_us) / 1000;
        if (*ground_cal_samples >= BARO_MIN_STABLE_TIME || cal_elapsed_ms >= GROUND_CAL_TIMEOUT_MS) {
            *baseline_locked = true;
            *in_GROUND_CAL = false;
            printk("GROUND_CAL complete. Baseline: %f hPa, elapsed:%lld ms\n",
                (double)*p_ref_adaptive,
                (long long)cal_elapsed_ms);
        }
    }

    int main(void) {
        const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(my_mpu));
        const struct device *dev_baro = DEVICE_DT_GET(DT_NODELABEL(my_ms));
        struct sensor_value a_tmp[3], g_tmp[3];
        float a_sum[3] = {0.0f, 0.0f, 0.0f}, g_sum[3] = {0.0f, 0.0f, 0.0f};
        float p_ref = 0.0f;
        int s = 200;
        struct sensor_value pressure, temperature;
        float AltitudeBarometer;
        float R = 287.05; // Specific gas constant for dry air in J/(kg*K)
        float g = 9.80665; // Standard gravity in m/s^2
        float p_av = 0.0f;

        static bool in_GROUND_CAL = true;
        static bool baseline_locked = false;
        static int32_t ground_cal_samples = 0;
        static float p_ref_adaptive = 0.0f;
        static bool is_stationary_locked = false;
        static int32_t still_count = 0;
        static int32_t moving_count = 0;
        static bool thermal_ref_locked = false;
        static float thermal_t_ref_c = 0.0f;

        //Kalman Filter variables
        static float kf_initialized = false;
        // System state: X = [h, v]
        static float h_est = 0.0f;
        static float v_est = 0.0f;

        // Convariance P 2*2
        static float p00 = 1.0f, p01 = 0.0f, p10 = 0.0f, p11 = 1.0f;
        static float sigma_a = 1.2f;
        static float r_baro = 0.08f;

        k_msleep(500);

        if (!device_is_ready(dev)){
            printk("MPU6050 not ready!\n");
            return 0;
        }
        if (!device_is_ready(dev_baro)){
            printk("Barometer not ready!\n");
            return 0;
        }

        // 1. Get the I2C bus device that the MPU6050 is connected to
        const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
        if (!device_is_ready(i2c_dev)) {
            printk("I2C bus not ready!\n");
            return 0;
        }
        
        // 2. Set DLPF to Level 1 (Sets base clock to 1000 Hz and filters noise)
        i2c_reg_write_byte(i2c_dev, 0x68, 0x1A, 0x01);

        // 3. Set Sample Rate Divider to 4 (1000 Hz / (1 + 4) = 200 Hz)
        i2c_reg_write_byte(i2c_dev, 0x68, 0x19, 0x04);
        
        for (int i = 0; i < 50; i++){
            sensor_sample_fetch(dev);
            k_msleep(3);
        }
        for (int i = 0; i < 50; i++){
            sensor_sample_fetch(dev_baro);
            k_msleep(10);
        }

        for (int i = 0; i < s; i++) {
            sensor_sample_fetch(dev);
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, a_tmp);
            sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, g_tmp);
            for (int j = 0; j < 3; j++) {
                a_sum[j] += (float)sensor_value_to_double(&a_tmp[j]);
                g_sum[j] += (float)sensor_value_to_double(&g_tmp[j]);
            }
            k_msleep(3);
        }

        float ax = (a_sum[0] / s) - b_accel[0], ay = (a_sum[1] / s) - b_accel[1], az = (a_sum[2] / s) - b_accel[2];
        gx_cal = (g_sum[0] / s) * rad_to_deg, gy_cal = (g_sum[1] / s) * rad_to_deg, gz_cal = (g_sum[2] / s) * rad_to_deg;
        level_offsets[0] = A_inv[0][0] * ax + A_inv[0][1] * ay + A_inv[0][2] * az;
        level_offsets[1] = A_inv[1][0] * ax + A_inv[1][1] * ay + A_inv[1][2] * az;
        struct sensor_trigger trig = {.type = SENSOR_TRIG_DATA_READY, .chan = SENSOR_CHAN_ALL};
        sensor_trigger_set(dev, &trig, trigger_handler);
        
        // Compputing initial reference pressure for altitude calculation by taking median of N samples to reduce effect of spiks
        float samples[9];
        for (int i = 0; i < 9; i++) {
            sensor_sample_fetch(dev_baro);
            sensor_channel_get(dev_baro, SENSOR_CHAN_PRESS, &pressure);
            samples[i] = (float)sensor_value_to_double(&pressure);
            k_msleep(10);
        }
        for (int i = 0; i < 9 - 1; i++) {
            for (int j = i + 1; j < 9; j++) {
                if (samples[j] < samples[i]) {
                    float tmp = samples[i];
                    samples[i] = samples[j];
                    samples[j] = tmp;
                }
            }
        }
        for (int i = 4; i < 7; i++) {
            p_av += samples[i];
        }
        p_ref = p_av / 3.0f;
        // for (int i = 0; i < 9    ; i++) {
        //     printk("%f\n", (double)samples[i]);
        // }
        // printk("%f\n", (double)p_ref);
        p_ref_adaptive = p_ref;
        baro_last_time_us = k_ticks_to_us_near64(k_uptime_ticks());
        int64_t ground_cal_start_us = baro_last_time_us;
        
        while (1) {
            int64_t now_us = k_ticks_to_us_near64(k_uptime_ticks());
            float dt_meas_s = (baro_last_time_us > 0)
                ? ((float)(now_us - baro_last_time_us) / 1000000.0f)
                : 0.02f;
            baro_last_time_us = now_us;
            if (dt_meas_s < 0.010f) dt_meas_s = 0.010f;
            if (dt_meas_s > 0.100f) dt_meas_s = 0.100f;

            is_stationary_locked = stationarity_update(&still_count, &moving_count, is_stationary_locked);

            sensor_sample_fetch(dev_baro);
            sensor_channel_get(dev_baro, SENSOR_CHAN_PRESS, &pressure);
            sensor_channel_get(dev_baro, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
            float p_raw = (float)sensor_value_to_double(&pressure);
            float t = (float)sensor_value_to_double(&temperature);

            float p_med = median5_sliding_update(p_raw);
            float p_lpf = baro_lpf_update(p_med, BARO_FC_HZ, dt_meas_s);
            ground_cal_update(is_stationary_locked, p_lpf, now_us, ground_cal_start_us, &in_GROUND_CAL, &baseline_locked, &p_ref_adaptive,
                            &ground_cal_samples);
            if (baseline_locked && !thermal_ref_locked) {
                thermal_t_ref_c = t;       // current ambient temp from sensor
                thermal_ref_locked = true;
                printk("THERMAL_REF locked: %f C\n", (double)thermal_t_ref_c);
            }
            float p_used = p_lpf;
            if (thermal_ref_locked) {
                float delta_t_c = t - thermal_t_ref_c;   // current temp - reference temp
                // printk("delta_t:%f\n", (double)delta_t_c);
                float p_thermal_bias = THERMAL_K1_HPA_PER_C * delta_t_c;
                float p_corr = p_lpf - p_thermal_bias;
                // Safety clamp before logf()
                if (p_corr < THERMAL_MIN_PRESSURE) p_corr = THERMAL_MIN_PRESSURE;
                if (p_corr > THERMAL_MAX_PRESSURE) p_corr = THERMAL_MAX_PRESSURE;

                p_used = p_corr;
            }

            float t_k = t + 273.15f; // Convert to Kelvin 
            float a = (R * t_k) / g; // Scale height
            AltitudeBarometer = a * logf(p_ref_adaptive / p_used); // Altitude calculation using barometric formula
            if(baseline_locked && thermal_ref_locked && !kf_initialized){
                h_est = AltitudeBarometer;
                v_est = 0.0f;
                p00 = 0.5f, p01 = 0.0f, p10 = 0.0f, p11 = 1.0f;
                kf_initialized = true;
                printk("KF init: h: %f, v: %f\n", (double)h_est, (double)v_est);
            }
            // AltitudeBarometer = 44330.0f * (1.0f - powf(p_lpf / p_ref_adaptive, 0.1903f));       // Alttude calculation using simplified barometric formula
            if (baseline_locked) {
                // printk("alt:%f\n", (double)AltitudeBarometer);
            }
            k_msleep(BARO_LOOP_MS);
        }
    }adfads
    
