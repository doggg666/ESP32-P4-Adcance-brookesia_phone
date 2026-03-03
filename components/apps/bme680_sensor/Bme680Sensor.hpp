/*
 * BME680 Sensor App for ESP_Brookesia Phone
 * Displays temperature, humidity, pressure, and gas resistance
 * from a BME680 module connected via I2C
 */

#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"
#include "bme680_driver.h"

class Bme680Sensor : public ESP_Brookesia_PhoneApp
{
public:
    Bme680Sensor();
    ~Bme680Sensor();

    bool run(void);
    bool back(void);
    bool close(void);
    bool init(void) override;

private:
    /* LVGL UI elements */
    lv_obj_t *_main_panel;

    /* Sensor value labels */
    lv_obj_t *_lbl_temp_value;
    lv_obj_t *_lbl_hum_value;
    lv_obj_t *_lbl_pres_value;
    lv_obj_t *_lbl_gas_value;

    /* Status label */
    lv_obj_t *_lbl_status;

    /* Refresh timer */
    lv_timer_t *_refresh_timer;

    /* BME680 device */
    bme680_dev_t _bme680_dev;
    bool _sensor_ok;

    /* Screen dimensions */
    uint16_t _width;
    uint16_t _height;

    /* UI creation helpers */
    void createUI(void);
    lv_obj_t *createSensorCard(lv_obj_t *parent, const char *title, const char *unit,
                               lv_color_t accent_color, lv_obj_t **value_label_out);

    /* Timer callback for periodic sensor reads */
    static void refreshTimerCb(lv_timer_t *timer);
};
