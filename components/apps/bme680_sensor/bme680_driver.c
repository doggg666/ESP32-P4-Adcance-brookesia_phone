/*
 * BME680 I2C Driver for ESP32-P4 CrowPanel Advanced
 * Reads temperature, humidity, pressure, and gas resistance via I2C
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bme680_driver.h"

static const char *TAG = "BME680";

/* ---- Low-level I2C helpers ---- */

static esp_err_t bme680_read_reg(bme680_dev_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buf, len, 1000);
}

static esp_err_t bme680_write_reg(bme680_dev_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t write_buf[2] = {reg, val};
    return i2c_master_transmit(dev->i2c_dev, write_buf, sizeof(write_buf), 1000);
}

/* ---- Calibration data parsing ---- */

static esp_err_t bme680_read_calibration(bme680_dev_t *dev)
{
    uint8_t coeff1[BME680_COEFF_ADDR1_LEN] = {0};
    uint8_t coeff2[BME680_COEFF_ADDR2_LEN] = {0};
    esp_err_t ret;

    ret = bme680_read_reg(dev, BME680_COEFF_ADDR1, coeff1, BME680_COEFF_ADDR1_LEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data block 1");
        return ret;
    }

    ret = bme680_read_reg(dev, BME680_COEFF_ADDR2, coeff2, BME680_COEFF_ADDR2_LEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data block 2");
        return ret;
    }

    /* Temperature calibration */
    dev->calib.par_t1 = (uint16_t)(((uint16_t)coeff2[9] << 8) | coeff2[8]);
    dev->calib.par_t2 = (int16_t)(((uint16_t)coeff1[2] << 8) | coeff1[1]);
    dev->calib.par_t3 = (int8_t)coeff1[3];

    /* Pressure calibration */
    dev->calib.par_p1  = (uint16_t)(((uint16_t)coeff1[6] << 8) | coeff1[5]);
    dev->calib.par_p2  = (int16_t)(((uint16_t)coeff1[8] << 8) | coeff1[7]);
    dev->calib.par_p3  = (int8_t)coeff1[9];
    dev->calib.par_p4  = (int16_t)(((uint16_t)coeff1[12] << 8) | coeff1[11]);
    dev->calib.par_p5  = (int16_t)(((uint16_t)coeff1[14] << 8) | coeff1[13]);
    dev->calib.par_p6  = (int8_t)coeff1[16];
    dev->calib.par_p7  = (int8_t)coeff1[15];
    dev->calib.par_p8  = (int16_t)(((uint16_t)coeff1[20] << 8) | coeff1[19]);
    dev->calib.par_p9  = (int16_t)(((uint16_t)coeff1[22] << 8) | coeff1[21]);
    dev->calib.par_p10 = coeff1[23];

    /* Humidity calibration */
    dev->calib.par_h1 = (uint16_t)(((uint16_t)coeff2[2] << 4) | (coeff2[1] & 0x0F));
    dev->calib.par_h2 = (uint16_t)(((uint16_t)coeff2[0] << 4) | ((coeff2[1] >> 4) & 0x0F));
    dev->calib.par_h3 = (int8_t)coeff2[3];
    dev->calib.par_h4 = (int8_t)coeff2[4];
    dev->calib.par_h5 = (int8_t)coeff2[5];
    dev->calib.par_h6 = coeff2[6];
    dev->calib.par_h7 = (int8_t)coeff2[7];

    /* Gas heater calibration */
    dev->calib.par_gh1 = (int8_t)coeff2[12];
    dev->calib.par_gh2 = (int16_t)(((uint16_t)coeff2[11] << 8) | coeff2[10]);
    dev->calib.par_gh3 = (int8_t)coeff2[13];

    /* Read heater resistance range */
    uint8_t tmp = 0;
    ret = bme680_read_reg(dev, 0x02, &tmp, 1);
    if (ret != ESP_OK) return ret;
    dev->calib.res_heat_range = (tmp >> 4) & 0x03;

    /* Read heater resistance correction */
    ret = bme680_read_reg(dev, 0x00, &tmp, 1);
    if (ret != ESP_OK) return ret;
    dev->calib.res_heat_val = (int8_t)tmp;

    /* Read range switching error */
    ret = bme680_read_reg(dev, 0x04, &tmp, 1);
    if (ret != ESP_OK) return ret;
    dev->calib.range_sw_err = ((int8_t)tmp) / 16;

    ESP_LOGI(TAG, "Calibration data read successfully");
    return ESP_OK;
}

/* ---- Compensation calculations (from BME680 datasheet) ---- */

static float bme680_calc_temperature(bme680_dev_t *dev, uint32_t temp_adc)
{
    float var1, var2, calc_temp;

    var1 = ((((float)temp_adc / 16384.0f) - ((float)dev->calib.par_t1 / 1024.0f))
            * ((float)dev->calib.par_t2));
    var2 = (((((float)temp_adc / 131072.0f) - ((float)dev->calib.par_t1 / 8192.0f))
            * (((float)temp_adc / 131072.0f) - ((float)dev->calib.par_t1 / 8192.0f)))
            * ((float)dev->calib.par_t3 * 16.0f));

    dev->t_fine = (int32_t)(var1 + var2);
    calc_temp = (var1 + var2) / 5120.0f;

    return calc_temp;
}

static float bme680_calc_pressure(bme680_dev_t *dev, uint32_t pres_adc)
{
    float var1, var2, var3, calc_pres;

    var1 = (((float)dev->t_fine / 2.0f) - 64000.0f);
    var2 = var1 * var1 * (((float)dev->calib.par_p6) / 131072.0f);
    var2 = var2 + (var1 * ((float)dev->calib.par_p5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)dev->calib.par_p4) * 65536.0f);
    var1 = (((((float)dev->calib.par_p3 * var1 * var1) / 16384.0f)
            + ((float)dev->calib.par_p2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)dev->calib.par_p1));

    calc_pres = (1048576.0f - ((float)pres_adc));

    if ((int)var1 != 0) {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
        var1 = (((float)dev->calib.par_p9) * calc_pres * calc_pres) / 2147483648.0f;
        var2 = calc_pres * (((float)dev->calib.par_p8) / 32768.0f);
        var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f)
                * (dev->calib.par_p10 / 131072.0f));
        calc_pres = (calc_pres + (var1 + var2 + var3
                    + ((float)dev->calib.par_p7 * 128.0f)) / 16.0f);
    } else {
        calc_pres = 0;
    }

    return calc_pres / 100.0f;  /* Convert Pa to hPa */
}

static float bme680_calc_humidity(bme680_dev_t *dev, uint16_t hum_adc)
{
    float var1, var2, var3, var4, calc_hum, temp_comp;

    temp_comp = ((float)dev->t_fine) / 5120.0f;

    var1 = (float)((float)hum_adc)
           - (((float)dev->calib.par_h1 * 16.0f)
           + (((float)dev->calib.par_h3 / 2.0f) * temp_comp));

    var2 = var1 * ((float)(((float)dev->calib.par_h2 / 262144.0f)
           * (1.0f + (((float)dev->calib.par_h4 / 16384.0f) * temp_comp)
           + (((float)dev->calib.par_h5 / 1048576.0f) * temp_comp * temp_comp))));

    var3 = (float)dev->calib.par_h6 / 16384.0f;
    var4 = (float)dev->calib.par_h7 / 2097152.0f;

    calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    if (calc_hum > 100.0f) {
        calc_hum = 100.0f;
    } else if (calc_hum < 0.0f) {
        calc_hum = 0.0f;
    }

    return calc_hum;
}

static float bme680_calc_gas_resistance(bme680_dev_t *dev, uint16_t gas_adc, uint8_t gas_range)
{
    /* Gas range lookup table from BME680 datasheet */
    static const float lookup_k1_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f,
        0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
    };
    static const float lookup_k2_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f,
        -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    float var1, var2, var3, calc_gas;

    var1 = (1340.0f + (5.0f * dev->calib.range_sw_err));
    var2 = var1 * (1.0f + lookup_k1_range[gas_range] / 100.0f);
    var3 = 1.0f + (lookup_k2_range[gas_range] / 100.0f);

    calc_gas = 1.0f / (var3 * (0.000000125f) * (float)(1 << gas_range)
               * (((((float)gas_adc) - 512.0f) / var2) + 1.0f));

    return calc_gas;
}

/* ---- Gas heater target resistance calculation ---- */

static uint8_t bme680_calc_heater_res(bme680_dev_t *dev, uint16_t target_temp)
{
    float var1, var2, var3, var4, var5;
    uint8_t res_heat;

    if (target_temp > 400) {
        target_temp = 400;
    }

    var1 = (((float)dev->calib.par_gh1 / 16.0f) + 49.0f);
    var2 = ((((float)dev->calib.par_gh2 / 32768.0f) * 0.0005f) + 0.00235f);
    var3 = ((float)dev->calib.par_gh3 / 1024.0f);
    var4 = var1 * (1.0f + (var2 * (float)target_temp));
    var5 = var4 + (var3 * 25.0f);   /* Assume ambient temp = 25C */

    res_heat = (uint8_t)(3.4f * ((var5 * (4.0f / (4.0f + (float)dev->calib.res_heat_range))
               * (1.0f / (1.0f + ((float)dev->calib.res_heat_val * 0.002f)))) - 25.0f));

    return res_heat;
}

/* ---- Gas heater duration calculation ---- */

static uint8_t bme680_calc_heater_dur(uint16_t dur_ms)
{
    uint8_t factor = 0;
    uint8_t dur_reg;

    if (dur_ms >= 0xFC0) {
        dur_reg = 0xFF; /* Max duration */
    } else {
        while (dur_ms > 0x3F) {
            dur_ms = dur_ms / 4;
            factor += 1;
        }
        dur_reg = (uint8_t)(dur_ms + (factor * 64));
    }

    return dur_reg;
}

/* ---- Public API ---- */

esp_err_t bme680_soft_reset(bme680_dev_t *dev)
{
    esp_err_t ret = bme680_write_reg(dev, BME680_REG_RESET, BME680_SOFT_RESET_CMD);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ret;
}

esp_err_t bme680_init(bme680_dev_t *dev, i2c_master_bus_handle_t i2c_bus, uint8_t addr)
{
    esp_err_t ret;

    memset(dev, 0, sizeof(bme680_dev_t));
    dev->i2c_addr = addr;
    dev->initialized = false;

    /* Register device on I2C bus */
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    ret = i2c_master_bus_add_device(i2c_bus, &cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BME680 to I2C bus (addr=0x%02X)", addr);
        return ret;
    }

    /* Read and verify chip ID */
    uint8_t chip_id = 0;
    ret = bme680_read_reg(dev, BME680_REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }

    if (chip_id != BME680_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, BME680_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BME680 detected (chip ID: 0x%02X)", chip_id);

    /* Soft reset */
    ret = bme680_soft_reset(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed");
        return ret;
    }

    /* Read calibration data */
    ret = bme680_read_calibration(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    /* Configure oversampling: temp x2, pressure x4, humidity x2 */
    ret = bme680_write_reg(dev, BME680_REG_CTRL_HUM, BME680_OS_2X);
    if (ret != ESP_OK) return ret;

    /* Set IIR filter coefficient to 3 */
    ret = bme680_write_reg(dev, BME680_REG_CONFIG, BME680_FILTER_COEFF_3 << 2);
    if (ret != ESP_OK) return ret;

    /* Configure gas heater: 320 deg C target, 150ms duration */
    uint8_t res_heat = bme680_calc_heater_res(dev, 320);
    ret = bme680_write_reg(dev, BME680_REG_RES_HEAT_0, res_heat);
    if (ret != ESP_OK) return ret;

    uint8_t gas_wait = bme680_calc_heater_dur(150);
    ret = bme680_write_reg(dev, BME680_REG_GAS_WAIT_0, gas_wait);
    if (ret != ESP_OK) return ret;

    /* Enable gas measurement, select heater set-point 0 */
    ret = bme680_write_reg(dev, BME680_REG_CTRL_GAS_1, BME680_RUN_GAS_ENABLE);
    if (ret != ESP_OK) return ret;

    dev->initialized = true;
    ESP_LOGI(TAG, "BME680 initialized successfully");
    return ESP_OK;
}

esp_err_t bme680_read_sensor_data(bme680_dev_t *dev, bme680_data_t *data)
{
    esp_err_t ret;
    uint8_t status;

    if (!dev->initialized) {
        ESP_LOGE(TAG, "BME680 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    memset(data, 0, sizeof(bme680_data_t));

    /* Trigger forced mode measurement: temp OS x2, pressure OS x4, forced mode */
    uint8_t ctrl_meas = (BME680_OS_2X << 5) | (BME680_OS_4X << 2) | BME680_MODE_FORCED;
    ret = bme680_write_reg(dev, BME680_REG_CTRL_MEAS, ctrl_meas);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger measurement");
        return ret;
    }

    /* Wait for measurement to complete (polling the new_data bit) */
    int retries = 50;
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = bme680_read_reg(dev, BME680_REG_MEAS_STATUS_0, &status, 1);
        if (ret != ESP_OK) return ret;
        retries--;
    } while (!(status & 0x80) && retries > 0);

    if (retries == 0) {
        ESP_LOGW(TAG, "Measurement timeout");
        return ESP_ERR_TIMEOUT;
    }

    data->sensor_ready = true;

    /* Read raw temperature, pressure, and humidity */
    uint8_t raw_data[8] = {0};
    ret = bme680_read_reg(dev, BME680_REG_PRES_MSB, raw_data, 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }

    uint32_t pres_adc = ((uint32_t)raw_data[0] << 12) | ((uint32_t)raw_data[1] << 4) | ((uint32_t)raw_data[2] >> 4);
    uint32_t temp_adc = ((uint32_t)raw_data[3] << 12) | ((uint32_t)raw_data[4] << 4) | ((uint32_t)raw_data[5] >> 4);
    uint16_t hum_adc  = ((uint16_t)raw_data[6] << 8) | raw_data[7];

    /* Compensate temperature first (needed for pressure and humidity) */
    data->temperature = bme680_calc_temperature(dev, temp_adc);
    data->pressure    = bme680_calc_pressure(dev, pres_adc);
    data->humidity    = bme680_calc_humidity(dev, hum_adc);

    /* Read gas resistance data */
    uint8_t gas_data[2] = {0};
    ret = bme680_read_reg(dev, BME680_REG_GAS_R_MSB, gas_data, 2);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read gas data");
        data->gas_valid = false;
        return ESP_OK; /* Non-fatal: we still have temp/hum/press */
    }

    uint16_t gas_adc   = ((uint16_t)gas_data[0] << 2) | ((gas_data[1] >> 6) & 0x03);
    uint8_t  gas_range = gas_data[1] & 0x0F;
    bool gas_valid_bit = (gas_data[1] >> 5) & 0x01;

    if (gas_valid_bit) {
        data->gas_resistance = bme680_calc_gas_resistance(dev, gas_adc, gas_range);
        data->gas_valid = true;
    } else {
        data->gas_valid = false;
    }

    ESP_LOGD(TAG, "T=%.2f C, H=%.2f %%, P=%.2f hPa, Gas=%.0f Ohms (valid=%d)",
             data->temperature, data->humidity, data->pressure,
             data->gas_resistance, data->gas_valid);

    return ESP_OK;
}
