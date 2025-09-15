/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/uart.h"

#include "ssd1306.h"
// #include "font8x8_basic.h"

const static char *TAG = "WPS";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_4
#else
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2
#endif

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

#define ADC_VALUE_MAX               4095
#define PRESSURE_MIN_PA             0
#define PRESSURE_MAX_PA             1200000
#define PRESSURE_M                  116.59f
#define PRESSURE_N                  41.687f

/* Static ADC calibration values for pressure sensor
    Vout,adc = Vin × (R2 / (R1 + R2))
    Voltage divider: 5V to 3.3V → Vin/Vout = 3.3/5 = 0.66.
    Vmin = 0.5V → Vadc,min = 0.5 × 0.66 = 0.33V
    Vmax = 4.5V → Vadc,max = 4.5 × 0.66 = 2.97V
*/
#define ADC_MIN    410    // Vmin,adc = 0.5V (0 MPa) --> ADCmin = 0.33V / 3.3V × 4095 ≈ 410
#define ADC_MAX    3687   // Vmax,adc = 4.5V (1.2 MPa) --> ADCmax = 2.97V / 3.3V × 4095 ≈ 3687
#define PRESSURE_MAX_MPA 1.2f

static int adc_raw;
static float pressure_bar;

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static float adc_raw_to_mpa(int raw);
static float mpa_to_bar(float mpa);
static float adc_raw_to_bar(int raw);

// Deprecated in favor of adc_raw_to_bar()
static float adc_raw_to_mpa(int raw)
{
    if (raw < ADC_MIN) raw = ADC_MIN;
    if (raw > ADC_MAX) raw = ADC_MAX;
    return ((float)(raw - ADC_MIN) * PRESSURE_MAX_MPA) / (ADC_MAX - ADC_MIN);
}

// Deprecated in favor of adc_raw_to_bar()
static float mpa_to_bar(float mpa)
{
    return mpa * 10.0f;
}

static float adc_raw_to_bar(int raw)
{
    // Convert ADC raw to pressure in Bar using linear equation from calibration
    float result = (raw - PRESSURE_N) / PRESSURE_M;
    if (result < 0.0f) {
        result = 0.0f;
    }
    return result;
}

void app_main(void)
{
    //-------------SSD1306 Init---------------//
    SSD1306_t dev;
    char buffer1[30], buffer2[30];
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);

    //-------------UART0 Init---------------//
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, 1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // TXD0 = GPIO14

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);

    while (1) {
        //-------------Read ADC value---------------//
        int sum = 0;
        for (int i = 0; i < 10; i++) {
            int sample = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &sample));
            sum += sample;
        }
        adc_raw = sum / 10;
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw);
        pressure_bar = adc_raw_to_bar(adc_raw);
        // pressure_bar = mpa_to_bar(adc_raw_to_mpa(adc_raw));

        //-------------Send value via UART---------------//
        char uart_msg[32];
        int uart_msg_len = snprintf(uart_msg, sizeof(uart_msg), "P=%.3f\n\r", pressure_bar);
        uart_write_bytes(uart_num, uart_msg, uart_msg_len);
        snprintf((char *)buffer1, sizeof(buffer1), "(Raw: %d/4095)", adc_raw);
        // TODO: check if pressure is valid before displaying
        snprintf((char *)buffer2, sizeof(buffer2), "%.3f Bar\n", pressure_bar);

        //-------------Display on SSD1306---------------//
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, "Water pressure:", 16, false);
        ssd1306_display_text(&dev, 2, buffer1, strlen(buffer1), false);
        ssd1306_display_text_x3(&dev, 4, buffer2, strlen(buffer2), false);
        ssd1306_display_text(&dev, 7, "Bar", 3, false);

        //-------------End cycle---------------//
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
