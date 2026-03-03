/*
 * BME680 I2C Driver for ESP32-P4 CrowPanel Advanced
 * Reads temperature, humidity, pressure, and gas resistance via I2C
 */

#ifndef _BME680_DRIVER_H_
#define _BME680_DRIVER_H_

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BME680 I2C address (SDO to GND = 0x76, SDO to VCC = 0x77) */
#define BME680_I2C_ADDR_PRIMARY     0x76
#define BME680_I2C_ADDR_SECONDARY   0x77

/* BME680 Register addresses */
#define BME680_REG_CHIP_ID          0xD0
#define BME680_REG_RESET            0xE0
#define BME680_REG_CTRL_HUM         0x72
#define BME680_REG_STATUS           0x73
#define BME680_REG_CTRL_MEAS        0x74
#define BME680_REG_CONFIG           0x75
#define BME680_REG_CTRL_GAS_0       0x70
#define BME680_REG_CTRL_GAS_1       0x71
#define BME680_REG_GAS_WAIT_0       0x64
#define BME680_REG_RES_HEAT_0       0x5A
#define BME680_REG_MEAS_STATUS_0    0x1D
#define BME680_REG_PRES_MSB         0x1F
#define BME680_REG_PRES_LSB         0x20
#define BME680_REG_PRES_XLSB        0x21
#define BME680_REG_TEMP_MSB         0x22
#define BME680_REG_TEMP_LSB         0x23
#define BME680_REG_TEMP_XLSB        0x24
#define BME680_REG_HUM_MSB          0x25
#define BME680_REG_HUM_LSB          0x26
#define BME680_REG_GAS_R_MSB        0x2A
#define BME680_REG_GAS_R_LSB        0x2B

/* BME680 expected chip ID */
#define BME680_CHIP_ID              0x61

/* Calibration data register ranges */
#define BME680_COEFF_ADDR1          0x89
#define BME680_COEFF_ADDR1_LEN      25
#define BME680_COEFF_ADDR2          0xE1
#define BME680_COEFF_ADDR2_LEN      16

/* Oversampling settings */
#define BME680_OS_NONE              0x00
#define BME680_OS_1X                0x01
#define BME680_OS_2X                0x02
#define BME680_OS_4X                0x03
#define BME680_OS_8X                0x04
#define BME680_OS_16X               0x05

/* IIR filter settings */
#define BME680_FILTER_OFF           0x00
#define BME680_FILTER_COEFF_1       0x01
#define BME680_FILTER_COEFF_3       0x02
#define BME680_FILTER_COEFF_7       0x03
#define BME680_FILTER_COEFF_15      0x04
#define BME680_FILTER_COEFF_31      0x05
#define BME680_FILTER_COEFF_63      0x06
#define BME680_FILTER_COEFF_127     0x07

/* Mode settings */
#define BME680_MODE_SLEEP           0x00
#define BME680_MODE_FORCED          0x01

/* Gas measurement enable */
#define BME680_RUN_GAS_ENABLE       0x10

/* Soft reset command */
#define BME680_SOFT_RESET_CMD       0xB6

/* Calibration parameters structure */
typedef struct {
    /* Temperature calibration */
    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;

    /* Pressure calibration */
    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int16_t  par_p4;
    int16_t  par_p5;
    int8_t   par_p6;
    int8_t   par_p7;
    int16_t  par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;

    /* Humidity calibration */
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t   par_h3;
    int8_t   par_h4;
    int8_t   par_h5;
    uint8_t  par_h6;
    int8_t   par_h7;

    /* Gas heater calibration */
    int8_t   par_gh1;
    int16_t  par_gh2;
    int8_t   par_gh3;

    /* Heater resistance range and switching error */
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;
} bme680_calib_t;

/* BME680 sensor data output */
typedef struct {
    float temperature;      /* Temperature in degrees Celsius */
    float humidity;         /* Relative humidity in % */
    float pressure;         /* Pressure in hPa (mbar) */
    float gas_resistance;   /* Gas resistance in Ohms */
    bool  gas_valid;        /* Whether gas measurement is valid */
    bool  sensor_ready;     /* Whether sensor data is available */
} bme680_data_t;

/* BME680 device handle */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    bme680_calib_t calib;
    int32_t t_fine;             /* Fine temperature for compensation */
    uint8_t i2c_addr;
    bool initialized;
} bme680_dev_t;

/**
 * @brief Initialize the BME680 sensor
 *
 * @param dev Pointer to BME680 device structure
 * @param i2c_bus I2C master bus handle (from bsp_i2c_get_handle())
 * @param addr I2C address of the BME680 (0x76 or 0x77)
 * @return ESP_OK on success
 */
esp_err_t bme680_init(bme680_dev_t *dev, i2c_master_bus_handle_t i2c_bus, uint8_t addr);

/**
 * @brief Trigger a forced measurement and read sensor data
 *
 * @param dev Pointer to BME680 device structure
 * @param data Pointer to data structure to store results
 * @return ESP_OK on success
 */
esp_err_t bme680_read_sensor_data(bme680_dev_t *dev, bme680_data_t *data);

/**
 * @brief Perform soft reset of the BME680
 *
 * @param dev Pointer to BME680 device structure
 * @return ESP_OK on success
 */
esp_err_t bme680_soft_reset(bme680_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* _BME680_DRIVER_H_ */
