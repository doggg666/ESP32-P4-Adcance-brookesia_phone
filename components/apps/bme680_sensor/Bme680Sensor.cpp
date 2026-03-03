/*
 * BME680 Sensor App for ESP_Brookesia Phone
 * Displays temperature, humidity, pressure, and gas resistance
 * from a BME680 module connected via I2C on the CrowPanel Advanced 10.1"
 */

#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "Bme680Sensor.hpp"

static const char *TAG = "BME680_App";

LV_IMG_DECLARE(img_app_bme680);

/* Sensor read interval in milliseconds */
#define BME680_REFRESH_INTERVAL_MS      2000

/* Color palette for the sensor cards */
#define COLOR_TEMP      lv_color_make(0xFF, 0x6B, 0x35)   /* Warm orange */
#define COLOR_HUM       lv_color_make(0x00, 0xB4, 0xD8)   /* Sky blue */
#define COLOR_PRES      lv_color_make(0x2E, 0xCC, 0x71)   /* Green */
#define COLOR_GAS       lv_color_make(0xAF, 0x7A, 0xC5)   /* Purple */
#define COLOR_BG_CARD   lv_color_make(0x1E, 0x1E, 0x2E)   /* Dark card background */
#define COLOR_TEXT       lv_color_make(0xE0, 0xE0, 0xE0)   /* Light text */
#define COLOR_UNIT       lv_color_make(0x90, 0x90, 0x90)   /* Dimmer text for units */

/* Font definitions */
#define FONT_TITLE      &lv_font_montserrat_20
#define FONT_VALUE      &lv_font_montserrat_40
#define FONT_UNIT       &lv_font_montserrat_18
#define FONT_STATUS     &lv_font_montserrat_16

/* ---- Constructor / Destructor ---- */

Bme680Sensor::Bme680Sensor() :
    ESP_Brookesia_PhoneApp("BME680", &img_app_bme680, true),
    _main_panel(nullptr),
    _lbl_temp_value(nullptr),
    _lbl_hum_value(nullptr),
    _lbl_pres_value(nullptr),
    _lbl_gas_value(nullptr),
    _lbl_status(nullptr),
    _ui_refresh_timer(nullptr),
    _sensor_ok(false),
    _cached_data_valid(false),
    _sensor_task_handle(nullptr),
    _task_running(false),
    _width(0),
    _height(0)
{
    memset(&_bme680_dev, 0, sizeof(bme680_dev_t));
    memset(&_cached_data, 0, sizeof(bme680_data_t));
}

Bme680Sensor::~Bme680Sensor()
{
}

/* ---- App lifecycle ---- */

bool Bme680Sensor::init(void)
{
    /* Try to initialize BME680 on the shared I2C bus */
    i2c_master_bus_handle_t i2c_bus = bsp_i2c_get_handle();

    /* Try primary address first (0x76), then secondary (0x77) */
    esp_err_t ret = bme680_init(&_bme680_dev, i2c_bus, BME680_I2C_ADDR_PRIMARY);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BME680 not found at 0x76, trying 0x77...");
        ret = bme680_init(&_bme680_dev, i2c_bus, BME680_I2C_ADDR_SECONDARY);
    }

    if (ret == ESP_OK) {
        _sensor_ok = true;
        ESP_LOGI(TAG, "BME680 sensor ready");
    } else {
        _sensor_ok = false;
        ESP_LOGE(TAG, "BME680 sensor not found on I2C bus");
    }

    return true;
}

bool Bme680Sensor::run(void)
{
    lv_area_t area = getVisualArea();
    _width  = area.x2 - area.x1;
    _height = area.y2 - area.y1;

    createUI();

    /* Start background FreeRTOS task for non-blocking sensor reads */
    if (_sensor_ok) {
        _task_running = true;
        xTaskCreate(sensorReadTask, "bme680_read", 4096, this, 3, &_sensor_task_handle);
    }

    /* Start LVGL timer to update UI from cached data (lightweight, non-blocking) */
    _ui_refresh_timer = lv_timer_create(uiRefreshTimerCb, 500, this);

    return true;
}

bool Bme680Sensor::back(void)
{
    notifyCoreClosed();
    return true;
}

bool Bme680Sensor::close(void)
{
    /* Stop the background sensor read task */
    _task_running = false;
    if (_sensor_task_handle) {
        /* Give the task time to exit its loop */
        vTaskDelay(pdMS_TO_TICKS(100));
        _sensor_task_handle = nullptr;
    }

    /* Stop the UI refresh timer */
    if (_ui_refresh_timer) {
        lv_timer_del(_ui_refresh_timer);
        _ui_refresh_timer = nullptr;
    }
    return true;
}

/* ---- UI Creation ---- */

lv_obj_t *Bme680Sensor::createSensorCard(lv_obj_t *parent, const char *title,
                                          const char *unit, lv_color_t accent_color,
                                          lv_obj_t **value_label_out)
{
    /* Card container */
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_style_bg_color(card, COLOR_BG_CARD, 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(card, 16, 0);
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, accent_color, 0);
    lv_obj_set_style_border_opa(card, LV_OPA_60, 0);
    lv_obj_set_style_pad_all(card, 12, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    /* Accent bar at the top */
    lv_obj_t *accent_bar = lv_obj_create(card);
    lv_obj_set_size(accent_bar, lv_pct(100), 4);
    lv_obj_align(accent_bar, LV_ALIGN_TOP_MID, 0, -6);
    lv_obj_set_style_bg_color(accent_bar, accent_color, 0);
    lv_obj_set_style_bg_opa(accent_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(accent_bar, 2, 0);
    lv_obj_set_style_border_width(accent_bar, 0, 0);
    lv_obj_clear_flag(accent_bar, LV_OBJ_FLAG_SCROLLABLE);

    /* Title label */
    lv_obj_t *lbl_title = lv_label_create(card);
    lv_label_set_text(lbl_title, title);
    lv_obj_set_style_text_font(lbl_title, FONT_TITLE, 0);
    lv_obj_set_style_text_color(lbl_title, accent_color, 0);
    lv_obj_align(lbl_title, LV_ALIGN_TOP_LEFT, 0, 6);

    /* Value label (large number) */
    lv_obj_t *lbl_value = lv_label_create(card);
    lv_label_set_text(lbl_value, "--.-");
    lv_obj_set_style_text_font(lbl_value, FONT_VALUE, 0);
    lv_obj_set_style_text_color(lbl_value, COLOR_TEXT, 0);
    lv_obj_align(lbl_value, LV_ALIGN_CENTER, 0, 6);

    /* Unit label */
    lv_obj_t *lbl_unit = lv_label_create(card);
    lv_label_set_text(lbl_unit, unit);
    lv_obj_set_style_text_font(lbl_unit, FONT_UNIT, 0);
    lv_obj_set_style_text_color(lbl_unit, COLOR_UNIT, 0);
    lv_obj_align(lbl_unit, LV_ALIGN_BOTTOM_RIGHT, 0, -2);

    *value_label_out = lbl_value;
    return card;
}

void Bme680Sensor::createUI(void)
{
    /* Main panel fills the visual area */
    _main_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(_main_panel, _width, _height);
    lv_obj_align(_main_panel, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(_main_panel, lv_color_make(0x12, 0x12, 0x1A), 0);
    lv_obj_set_style_bg_opa(_main_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(_main_panel, 0, 0);
    lv_obj_set_style_radius(_main_panel, 0, 0);
    lv_obj_set_style_pad_all(_main_panel, 10, 0);
    lv_obj_clear_flag(_main_panel, LV_OBJ_FLAG_SCROLLABLE);

    /* Title bar */
    lv_obj_t *title_bar = lv_obj_create(_main_panel);
    lv_obj_set_size(title_bar, lv_pct(100), 40);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_opa(title_bar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl_app_title = lv_label_create(title_bar);
    lv_label_set_text(lbl_app_title, "BME680 Environment Sensor");
    lv_obj_set_style_text_font(lbl_app_title, FONT_TITLE, 0);
    lv_obj_set_style_text_color(lbl_app_title, COLOR_TEXT, 0);
    lv_obj_align(lbl_app_title, LV_ALIGN_LEFT_MID, 0, 0);

    /* Status label in title bar */
    _lbl_status = lv_label_create(title_bar);
    if (_sensor_ok) {
        lv_label_set_text(_lbl_status, "Sensor: Connected");
        lv_obj_set_style_text_color(_lbl_status, lv_color_make(0x2E, 0xCC, 0x71), 0);
    } else {
        lv_label_set_text(_lbl_status, "Sensor: Not Found");
        lv_obj_set_style_text_color(_lbl_status, lv_color_make(0xE7, 0x4C, 0x3C), 0);
    }
    lv_obj_set_style_text_font(_lbl_status, FONT_STATUS, 0);
    lv_obj_align(_lbl_status, LV_ALIGN_RIGHT_MID, 0, 0);

    /* Calculate card layout - 2x2 grid */
    int card_area_h = _height - 60;    /* Space below title */
    int card_w = (_width - 30) / 2;    /* Two columns with spacing */
    int card_h = (card_area_h - 20) / 2; /* Two rows with spacing */
    int start_y = 50;

    /* Temperature card (top-left) */
    lv_obj_t *card_temp = createSensorCard(_main_panel, "Temperature", "\xC2\xB0" "C",
                                           COLOR_TEMP, &_lbl_temp_value);
    lv_obj_set_size(card_temp, card_w, card_h);
    lv_obj_align(card_temp, LV_ALIGN_TOP_LEFT, 0, start_y);

    /* Humidity card (top-right) */
    lv_obj_t *card_hum = createSensorCard(_main_panel, "Humidity", "% RH",
                                          COLOR_HUM, &_lbl_hum_value);
    lv_obj_set_size(card_hum, card_w, card_h);
    lv_obj_align(card_hum, LV_ALIGN_TOP_RIGHT, 0, start_y);

    /* Pressure card (bottom-left) */
    lv_obj_t *card_pres = createSensorCard(_main_panel, "Pressure", "hPa",
                                           COLOR_PRES, &_lbl_pres_value);
    lv_obj_set_size(card_pres, card_w, card_h);
    lv_obj_align(card_pres, LV_ALIGN_BOTTOM_LEFT, 0, -5);

    /* Gas Resistance card (bottom-right) */
    lv_obj_t *card_gas = createSensorCard(_main_panel, "Gas Resistance", "kOhm",
                                          COLOR_GAS, &_lbl_gas_value);
    lv_obj_set_size(card_gas, card_w, card_h);
    lv_obj_align(card_gas, LV_ALIGN_BOTTOM_RIGHT, 0, -5);
}

/* ---- Background sensor read task (runs on its own FreeRTOS task, non-blocking to LVGL) ---- */

void Bme680Sensor::sensorReadTask(void *param)
{
    Bme680Sensor *app = (Bme680Sensor *)param;

    while (app->_task_running) {
        bme680_data_t data;
        esp_err_t ret = bme680_read_sensor_data(&app->_bme680_dev, &data);

        if (ret == ESP_OK && data.sensor_ready) {
            /* Update cached data (atomic-ish copy for simple struct) */
            app->_cached_data = data;
            app->_cached_data_valid = true;
        } else {
            app->_cached_data_valid = false;
            ESP_LOGW(TAG, "Failed to read BME680 sensor data");
        }

        /* Wait before next read */
        vTaskDelay(pdMS_TO_TICKS(BME680_REFRESH_INTERVAL_MS));
    }

    /* Task self-deletes when _task_running is set to false */
    vTaskDelete(NULL);
}

/* ---- LVGL timer callback: updates UI from cached sensor data (non-blocking) ---- */

void Bme680Sensor::uiRefreshTimerCb(lv_timer_t *timer)
{
    Bme680Sensor *app = (Bme680Sensor *)lv_timer_get_user_data(timer);
    if (app == nullptr) return;

    if (!app->_sensor_ok) {
        lv_label_set_text(app->_lbl_temp_value, "--.-");
        lv_label_set_text(app->_lbl_hum_value, "--.-");
        lv_label_set_text(app->_lbl_pres_value, "----");
        lv_label_set_text(app->_lbl_gas_value, "--.-");
        lv_label_set_text(app->_lbl_status, "Sensor: Not Found");
        lv_obj_set_style_text_color(app->_lbl_status, lv_color_make(0xE7, 0x4C, 0x3C), 0);
        return;
    }

    if (app->_cached_data_valid) {
        char buf[32];

        /* Update temperature */
        snprintf(buf, sizeof(buf), "%.1f", app->_cached_data.temperature);
        lv_label_set_text(app->_lbl_temp_value, buf);

        /* Update humidity */
        snprintf(buf, sizeof(buf), "%.1f", app->_cached_data.humidity);
        lv_label_set_text(app->_lbl_hum_value, buf);

        /* Update pressure */
        snprintf(buf, sizeof(buf), "%.1f", app->_cached_data.pressure);
        lv_label_set_text(app->_lbl_pres_value, buf);

        /* Update gas resistance (convert to kOhm) */
        if (app->_cached_data.gas_valid) {
            snprintf(buf, sizeof(buf), "%.1f", app->_cached_data.gas_resistance / 1000.0f);
            lv_label_set_text(app->_lbl_gas_value, buf);
        } else {
            lv_label_set_text(app->_lbl_gas_value, "N/A");
        }

        /* Update status */
        lv_label_set_text(app->_lbl_status, "Sensor: Connected");
        lv_obj_set_style_text_color(app->_lbl_status, lv_color_make(0x2E, 0xCC, 0x71), 0);
    } else {
        lv_label_set_text(app->_lbl_status, "Sensor: Reading...");
        lv_obj_set_style_text_color(app->_lbl_status, lv_color_make(0xFF, 0xC1, 0x07), 0);
    }
}
