#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#define I2C_SDA_GPIO GPIO_NUM_21
#define I2C_SCL_GPIO GPIO_NUM_22
#define I2C_FREQ_HZ  400000

// MPU-6050 registers (RM-MPU-6000A-00 rev 4.0)
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

#define ACCEL_LSB_PER_G   16384.0f // AFS_SEL=0, ±2 g
#define GYRO_LSB_PER_DPS  131.0f   // FS_SEL=0, ±250 °/s

#define LOOP_PERIOD_MS    10       // 100 Hz (FreeRTOS tick is 100 Hz, so 10 ms is the minimum)
#define COMP_ALPHA        0.98f    // complementary filter: weight on gyro vs accel
#define PRINT_EVERY       10       // print every 10th loop = 10 Hz
#define HEADER_EVERY      20       // reprint the table header every 20 rows

// Startup calibration: robust (median/MAD outlier-rejected) bias estimate
#define CALIB_SAMPLES     200      // 2 s at 100 Hz, keep the board still
#define CALIB_MAX_STD_DPS 0.5f     // if inlier spread is above this, the board moved -> retry
#define CALIB_MAX_TRIES   5

// Per-axis Kalman filter, state = [true rate, gyro bias]
#define KF_Q_RATE   0.02f    // how fast the true rate may change per step (dps^2); lower = smoother, more lag
#define KF_Q_BIAS   1e-6f    // how fast the bias may drift per step (dps^2), e.g. with temperature
#define KF_R_GYRO   0.03f    // gyro measurement noise variance (dps^2)
#define KF_R_ZUPT   0.001f   // "rate is zero" pseudo-measurement variance when still

// Stillness detection (enables zero-rate updates -> live bias tracking)
#define STILL_EMA_ALPHA   0.05f   // smoothing for running mean/variance of raw gyro
#define STILL_MAX_VAR     0.05f   // dps^2, max gyro variance per axis
#define STILL_MAX_RATE    1.5f    // dps, max |gyro - bias| per axis
#define STILL_ACCEL_TOL   0.05f   // g, |accel| must be within 1 +/- this
#define STILL_MIN_SAMPLES 50      // must be still this long (0.5 s) before trusting it

#define RAD_TO_DEG (180.0f / (float)M_PI)

static const char *TAG = "MPU6050";

typedef struct {
    float ax, ay, az; // g
    float g[3];       // °/s, raw (x, y, z)
    float temp_c;
} mpu_sample_t;

typedef struct {
    float rate;    // estimated true rotation rate (dps)
    float bias;    // estimated gyro bias (dps)
    float P[2][2]; // state covariance
} gyro_kf_t;

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100);
}

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static void mpu_init(i2c_master_dev_handle_t dev)
{
    // Chip powers up asleep (PWR_MGMT_1 reset value 0x40). Reset, then wake
    // with the X-gyro PLL as clock source (recommended over internal 8 MHz).
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_PWR_MGMT_1, 0x80));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_PWR_MGMT_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(10));

    // DLPF 20 Hz (gyro) / 21 Hz (accel). Must sit well below the 50 Hz Nyquist limit of our
    // 100 Hz loop, otherwise vibration aliases into slow fake rotation (seen with 44 Hz).
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_CONFIG, 0x04));
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_SMPLRT_DIV, 0x00));   // sample rate 1 kHz
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_GYRO_CONFIG, 0x00));  // ±250 °/s
    ESP_ERROR_CHECK(reg_write(dev, MPU_REG_ACCEL_CONFIG, 0x00)); // ±2 g
}

// Burst read 0x3B..0x48: accel XYZ, temp, gyro XYZ (all from the same sample instant)
static esp_err_t mpu_read(i2c_master_dev_handle_t dev, mpu_sample_t *s)
{
    uint8_t d[14];
    esp_err_t err = reg_read(dev, MPU_REG_ACCEL_XOUT_H, d, sizeof(d));
    if (err != ESP_OK) {
        return err;
    }
    s->ax = (int16_t)((d[0] << 8) | d[1]) / ACCEL_LSB_PER_G;
    s->ay = (int16_t)((d[2] << 8) | d[3]) / ACCEL_LSB_PER_G;
    s->az = (int16_t)((d[4] << 8) | d[5]) / ACCEL_LSB_PER_G;
    s->temp_c = (int16_t)((d[6] << 8) | d[7]) / 340.0f + 36.53f;
    s->g[0] = (int16_t)((d[8] << 8) | d[9]) / GYRO_LSB_PER_DPS;
    s->g[1] = (int16_t)((d[10] << 8) | d[11]) / GYRO_LSB_PER_DPS;
    s->g[2] = (int16_t)((d[12] << 8) | d[13]) / GYRO_LSB_PER_DPS;
    return ESP_OK;
}

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static float median(float *v, int n)
{
    qsort(v, n, sizeof(float), cmp_float);
    return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

// Robust mean: drop samples more than 3 sigma (estimated via MAD) from the median,
// then average the rest. Immune to bumps/spikes that would skew a plain average.
static float robust_mean(const float *x, int n, float *std_out)
{
    static float tmp[CALIB_SAMPLES];
    for (int i = 0; i < n; i++) tmp[i] = x[i];
    float med = median(tmp, n);

    for (int i = 0; i < n; i++) tmp[i] = fabsf(x[i] - med);
    float sigma = 1.4826f * median(tmp, n); // MAD -> std for Gaussian noise
    if (sigma < 0.01f) sigma = 0.01f;

    float sum = 0, sum2 = 0;
    int k = 0;
    for (int i = 0; i < n; i++) {
        if (fabsf(x[i] - med) <= 3.0f * sigma) {
            sum += x[i];
            sum2 += x[i] * x[i];
            k++;
        }
    }
    float mean = sum / k;
    *std_out = sqrtf(fmaxf(sum2 / k - mean * mean, 0.0f));
    return mean;
}

// Collect samples while still and compute a robust per-axis bias. Retries if the board moved.
static void calibrate_gyro(i2c_master_dev_handle_t dev, float bias[3])
{
    static float buf[3][CALIB_SAMPLES];
    mpu_sample_t s;

    for (int attempt = 1; attempt <= CALIB_MAX_TRIES; attempt++) {
        ESP_LOGI(TAG, "Calibrating gyro (try %d), keep the board still...", attempt);
        int n = 0;
        while (n < CALIB_SAMPLES) {
            if (mpu_read(dev, &s) == ESP_OK) {
                for (int a = 0; a < 3; a++) buf[a][n] = s.g[a];
                n++;
            }
            vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
        }

        float std[3];
        bool ok = true;
        for (int a = 0; a < 3; a++) {
            bias[a] = robust_mean(buf[a], n, &std[a]);
            if (std[a] > CALIB_MAX_STD_DPS) ok = false;
        }
        ESP_LOGI(TAG, "Bias [dps]: %.3f %.3f %.3f  noise std: %.3f %.3f %.3f",
                 bias[0], bias[1], bias[2], std[0], std[1], std[2]);
        if (ok) return;
        ESP_LOGW(TAG, "Too much motion during calibration, retrying");
    }
    ESP_LOGW(TAG, "Calibration never settled; using last estimate (Kalman will refine it when still)");
}

static void kf_init(gyro_kf_t *kf, float bias0)
{
    kf->rate = 0;
    kf->bias = bias0;
    kf->P[0][0] = 1.0f;  kf->P[0][1] = 0;
    kf->P[1][0] = 0;     kf->P[1][1] = 0.01f; // fairly confident in the startup bias
}

// Scalar measurement update: z = h0*rate + h1*bias + noise(R)
static void kf_update(gyro_kf_t *kf, float h0, float h1, float z, float R)
{
    float Ph0 = kf->P[0][0] * h0 + kf->P[0][1] * h1;
    float Ph1 = kf->P[1][0] * h0 + kf->P[1][1] * h1;
    float S = h0 * Ph0 + h1 * Ph1 + R;
    float K0 = Ph0 / S, K1 = Ph1 / S;
    float y = z - (h0 * kf->rate + h1 * kf->bias);

    kf->rate += K0 * y;
    kf->bias += K1 * y;

    kf->P[0][0] -= K0 * Ph0;  kf->P[0][1] -= K0 * Ph1;
    kf->P[1][0] -= K1 * Ph0;  kf->P[1][1] -= K1 * Ph1;
}

static void kf_step(gyro_kf_t *kf, float gyro_raw, bool still)
{
    // Predict: rate and bias are random walks
    kf->P[0][0] += KF_Q_RATE;
    kf->P[1][1] += KF_Q_BIAS;

    // Gyro measures rate + bias
    kf_update(kf, 1.0f, 1.0f, gyro_raw, KF_R_GYRO);

    // Zero-velocity update: when still, true rate is 0, which makes bias observable
    if (still) {
        kf_update(kf, 1.0f, 0.0f, 0.0f, KF_R_ZUPT);
    }
}

// Column groups are 24 chars wide (" %7.2f %7.2f %7.2f") so headers and rows line up
static void print_header(void)
{
    printf("\n %-23s | %-23s | %-23s | %-23s |\n",
           "ANGLE (deg)", "RATE raw (dps)", "RATE stable (dps)", "GYRO BIAS (dps)");
    printf(" %7s %7s %7s | %7s %7s %7s | %7s %7s %7s | %7s %7s %7s | %s\n",
           "roll", "pitch", "yaw", "x", "y", "z", "x", "y", "z", "x", "y", "z", "state");
    printf("-------------------------+-------------------------+-------------------------+-------------------------+-------\n");
}

void app_main(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    // Address is 0x68 with AD0 low (default on most breakouts), 0x69 with AD0 high
    uint8_t addr = 0;
    while (addr == 0) {
        if (i2c_master_probe(bus, 0x68, 50) == ESP_OK) {
            addr = 0x68;
        } else if (i2c_master_probe(bus, 0x69, 50) == ESP_OK) {
            addr = 0x69;
        } else {
            ESP_LOGW(TAG, "MPU-6050 not found at 0x68/0x69. Check VCC->3V3, GND->GND, SDA->D21, SCL->D22");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t mpu;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &mpu));

    uint8_t who = 0;
    ESP_ERROR_CHECK(reg_read(mpu, MPU_REG_WHO_AM_I, &who, 1));
    ESP_LOGI(TAG, "Found device at 0x%02X, WHO_AM_I = 0x%02X%s", addr, who,
             who == 0x68 ? " (MPU-6050)" : " (unexpected, expected 0x68)");

    mpu_init(mpu);
    vTaskDelay(pdMS_TO_TICKS(100)); // let the gyro settle after wake-up

    float bias0[3];
    calibrate_gyro(mpu, bias0);

    gyro_kf_t kf[3];
    float ema_mean[3], ema_var[3];
    for (int a = 0; a < 3; a++) {
        kf_init(&kf[a], bias0[a]);
        ema_mean[a] = bias0[a];
        ema_var[a] = 0;
    }

    // Seed roll/pitch from the accelerometer so the filter starts at the true tilt
    mpu_sample_t s;
    float roll = 0, pitch = 0, yaw = 0;
    if (mpu_read(mpu, &s) == ESP_OK) {
        roll = atan2f(s.ay, s.az) * RAD_TO_DEG;
        pitch = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEG;
    }

    int64_t last_us = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();
    int loop = 0;
    int still_count = 0;

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));

        if (mpu_read(mpu, &s) != ESP_OK) {
            ESP_LOGW(TAG, "read failed");
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) / 1e6f;
        last_us = now_us;

        // --- Stillness detection: low gyro variance, small rate, accel ~1 g ---
        bool quiet = true;
        for (int a = 0; a < 3; a++) {
            float d = s.g[a] - ema_mean[a];
            ema_mean[a] += STILL_EMA_ALPHA * d;
            ema_var[a] = (1.0f - STILL_EMA_ALPHA) * (ema_var[a] + STILL_EMA_ALPHA * d * d);
            if (ema_var[a] > STILL_MAX_VAR || fabsf(s.g[a] - kf[a].bias) > STILL_MAX_RATE) {
                quiet = false;
            }
        }
        float acc_norm = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
        if (fabsf(acc_norm - 1.0f) > STILL_ACCEL_TOL) quiet = false;
        still_count = quiet ? still_count + 1 : 0;
        bool still = still_count >= STILL_MIN_SAMPLES;

        // --- Kalman: stable rate + live bias tracking ---
        for (int a = 0; a < 3; a++) {
            kf_step(&kf[a], s.g[a], still);
        }

        // Bias-corrected rotation rates (live bias from the Kalman filter, unsmoothed)
        float gx = s.g[0] - kf[0].bias;
        float gy = s.g[1] - kf[1].bias;
        float gz = s.g[2] - kf[2].bias;

        // Tilt from gravity: noisy and disturbed by acceleration, but never drifts
        float acc_roll = atan2f(s.ay, s.az) * RAD_TO_DEG;
        float acc_pitch = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * RAD_TO_DEG;

        // Complementary filter: integrate gyro (smooth, fast) and pull slowly toward accel (no drift)
        roll = COMP_ALPHA * (roll + gx * dt) + (1.0f - COMP_ALPHA) * acc_roll;
        pitch = COMP_ALPHA * (pitch + gy * dt) + (1.0f - COMP_ALPHA) * acc_pitch;

        // Yaw has no gravity reference, so it's gyro-only and will slowly drift
        yaw += gz * dt;

        if (++loop % PRINT_EVERY == 0) {
            if ((loop / PRINT_EVERY) % HEADER_EVERY == 1) {
                print_header();
            }
            printf(" %7.2f %7.2f %7.2f | %7.2f %7.2f %7.2f | %7.2f %7.2f %7.2f | %7.3f %7.3f %7.3f | %s\n",
                   roll, pitch, yaw, gx, gy, gz,
                   kf[0].rate, kf[1].rate, kf[2].rate,
                   kf[0].bias, kf[1].bias, kf[2].bias,
                   still ? "STILL" : "moving");
        }
    }
}
