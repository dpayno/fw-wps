/*
 * WPS - Water Pressure Sensor
 *
 * This application reads pressure data from a water pressure sensor
 * connected to an ADC channel, displays the readings on an SSD1306 OLED
 * display, and stores it in a SD card.
 *
 */

/* Standard libraries */
 #include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <time.h>

/* ESP-IDF libraries */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

/* ADC */
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/* Display */
#include "ssd1306.h"

/* SD card */
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

const static char *TAG = "WPS";
#define INIT_STEP_DELAY_MS 250
#define REBOOT_DELAY_S 5

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0    ADC_CHANNEL_4
#else
#define EXAMPLE_ADC1_CHAN0    ADC_CHANNEL_2
#endif

#define EXAMPLE_ADC_ATTEN     ADC_ATTEN_DB_12

#define ADC_VALUE_MAX         4095
#define PRESSURE_MIN_PA       0
#define PRESSURE_MAX_PA       1200000
#define PRESSURE_M            126.5454f
#define PRESSURE_N            39.8727f

/*---------------------------------------------------------------
        SD Card General Macros
---------------------------------------------------------------*/
#define EXAMPLE_MAX_CHAR_SIZE 64
#define MOUNT_POINT           "/sdcard"

#define PIN_NUM_MISO          19
#define PIN_NUM_MOSI          23
#define PIN_NUM_CLK           18
#define PIN_NUM_CS            2

/*---------------------------------------------------------------
        Global Variables
---------------------------------------------------------------*/
static int adc_raw;
static float pressure_bar;

typedef enum {
    WPS_STATUS_OK = 0,
    WPS_STATUS_SD_MOUNT_FAIL,
    WPS_STATUS_SD_FILE_FAIL,
    WPS_STATUS_ADC_FAIL,
    WPS_STATUS_DISPLAY_FAIL,
    WPS_STATUS_UNKNOWN_FAIL
} wps_status_t;

/*---------------------------------------------------------------
        Function Prototypes
---------------------------------------------------------------*/
static float adc_raw_to_bar(int raw);
static esp_err_t write_file(const char *path, char *data, const char* type);
static esp_err_t read_file(const char *path);
static void restart_on_error(SSD1306_t dev, wps_status_t status);

/*---------------------------------------------------------------
        Function Definitions
---------------------------------------------------------------*/
static float adc_raw_to_bar(int raw)
{
    // Convert ADC raw to pressure in Bar using linear equation from calibration
    float result = (raw - PRESSURE_N) / PRESSURE_M;
    if (result < 0.0f) {
        result = 0.0f;
    }
    return result;
}

static esp_err_t write_file(const char *path, char *data, const char* type)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, type);
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

static void restart_on_error(SSD1306_t dev, wps_status_t status)
{
    //-------------Check status---------------//
    switch (status) {
        case WPS_STATUS_OK:
            ssd1306_display_text(&dev, 5, "Status: OK", 10, false);
            break;
        case WPS_STATUS_SD_MOUNT_FAIL:
            ssd1306_display_text(&dev, 5, "SD Mount Fail", 14, false);
            break;
        case WPS_STATUS_SD_FILE_FAIL:
            ssd1306_display_text(&dev, 5, "SD File Fail", 13, false);
            break;
        case WPS_STATUS_ADC_FAIL:
            ssd1306_display_text(&dev, 5, "ADC Fail", 9, false);
            break;
        case WPS_STATUS_DISPLAY_FAIL:
            ssd1306_display_text(&dev, 5, "Display Fail", 12, false);
            break;
        default:
            ssd1306_display_text(&dev, 5, "Unknown Fail", 12, false);
            break;
    };
    vTaskDelay(pdMS_TO_TICKS(10));
    if (status != WPS_STATUS_OK) {
        for (int i = REBOOT_DELAY_S; i > 0; i--) {
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "Restarting in %d...", i);
            ssd1306_display_text(&dev, 6, buffer, strlen(buffer), false);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        esp_restart();
    }
}

static void error_handler(SSD1306_t dev, int samples, int* n_errors) {
    (*n_errors)++;
    char error_str[30];
    snprintf(error_str, sizeof(error_str), "E: %05d S:%05d", *n_errors, samples);
    ssd1306_clear_line(&dev, 7, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    ssd1306_display_text(&dev, 7, error_str, strlen(error_str), false);
    vTaskDelay(pdMS_TO_TICKS(10));
    
}

/*---------------------------------------------------------------
        Application Main
---------------------------------------------------------------*/
void app_main(void)
{
    //-------------SSD1306 Init---------------//
    SSD1306_t dev;
    char buffer1[30], buffer2[30], buffer3[64];
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, 0, "WPS init", 9, false);
    vTaskDelay(pdMS_TO_TICKS(INIT_STEP_DELAY_MS));
    ssd1306_display_text(&dev, 2, "Init screen...", 15, false);
    vTaskDelay(pdMS_TO_TICKS(INIT_STEP_DELAY_MS));

    //-------------ADC1 Init & Config---------------//
    ssd1306_display_text(&dev, 3, "Init adc...", 12, false);
    vTaskDelay(pdMS_TO_TICKS(INIT_STEP_DELAY_MS));
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    if (adc_oneshot_new_unit(&init_config1, &adc1_handle) != ESP_OK) {
        restart_on_error(dev, WPS_STATUS_ADC_FAIL);
        return;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config) != ESP_OK) {
        restart_on_error(dev, WPS_STATUS_ADC_FAIL);
        return;
    }

    //-------------SD card init & test---------------//
    ssd1306_display_text(&dev, 4, "Init sd card...", 16, false);
    vTaskDelay(pdMS_TO_TICKS(INIT_STEP_DELAY_MS));
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SPI peripheral");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        restart_on_error(dev, WPS_STATUS_SD_MOUNT_FAIL);
        return;
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. ");
            restart_on_error(dev, WPS_STATUS_SD_MOUNT_FAIL);
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.",
                esp_err_to_name(ret));
            restart_on_error(dev, WPS_STATUS_SD_MOUNT_FAIL);

        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    sdmmc_card_print_info(stdout, card);

    // Create a file if it does not exist.
    const char *file_pressure = MOUNT_POINT"/data.csv";
    struct stat st;
    if (stat(file_pressure, &st) != 0) {
        ESP_LOGI(TAG, "File %s does not exist, creating file", file_pressure);
        char data[EXAMPLE_MAX_CHAR_SIZE];
        snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "TS, PRAW, PBAR\n");
        ret = write_file(file_pressure, data, "w");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create file %s", file_pressure);
            restart_on_error(dev, WPS_STATUS_SD_FILE_FAIL);
            return;
        }
    } else {
        ESP_LOGI(TAG, "File %s already exists, skipping creation",
            file_pressure);
    }

    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");
    // spi_bus_free(host.slot);
    restart_on_error(dev, WPS_STATUS_OK);

    //-------------Main loop---------------//
    TickType_t xLastWakeTime;
    int samples = 0;
    int errors = 0;
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        //-------------Start cycle---------------//
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000));

        //-------------Read ADC value---------------//
        esp_err_t adc_stat;
        int sum = 0;
        samples++;
        for (int i = 0; i < 10; i++) {
            int sample = 0;
            adc_stat = adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &sample);
            if (adc_stat != ESP_OK) {
                break;
            }
            sum += sample;
        }
        if (adc_stat != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed");
            error_handler(dev, samples, &errors);
            continue;
        }
        adc_raw = sum / 10;
        pressure_bar = adc_raw_to_bar(adc_raw);
        ESP_LOGI(TAG, "ADC Raw: %d", adc_raw);
        ESP_LOGI(TAG, "Pressure: %.2f Bar", pressure_bar);
        snprintf((char *)buffer1, sizeof(buffer1), "(Raw: %d/4095)", adc_raw);
        snprintf((char *)buffer2, sizeof(buffer2), "%.3f Bar\n", pressure_bar);

        //-------------Store data in SD card---------------//
        ESP_LOGI(TAG, "Mounting filesystem");
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
        if (ret != ESP_OK) {
            if (ret == ESP_FAIL) {
                ESP_LOGE(TAG, "Failed to mount filesystem. ");
            } else {
                ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                    "Make sure SD card lines have pull-up resistors in place.",
                    esp_err_to_name(ret));
            }
            error_handler(dev, samples, &errors);
            continue;
        }
        ESP_LOGI(TAG, "Filesystem mounted");
        time_t now;
        time(&now);
        snprintf(buffer3, EXAMPLE_MAX_CHAR_SIZE, "%lld,%d,%.3f\n", now, adc_raw, pressure_bar);
        ret = write_file(file_pressure, buffer3, "a");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create file %s", file_pressure);
            error_handler(dev, samples, &errors);
            continue;
        }

        // All done, unmount partition and disable SPI peripheral
        esp_vfs_fat_sdcard_unmount(mount_point, card);
        ESP_LOGI(TAG, "Card unmounted");
        // spi_bus_free(host.slot);

        //-------------Calculate uptime---------------//
        int uptime_sec = esp_timer_get_time() / 1000000;
        int hours = uptime_sec / 3600;
        int minutes = (uptime_sec % 3600) / 60;
        int seconds = uptime_sec % 60;
        char uptime_str[23];
        snprintf(uptime_str, sizeof(uptime_str), " WPS   %02d:%02d:%02d", hours, minutes, seconds);

        //-------------Prepare error count string---------------//
        char error_str[30];
        snprintf(error_str, sizeof(error_str), "E:%05d  S:%05d", errors, samples);

        //-------------Display on SSD1306---------------//
        ssd1306_clear_screen(&dev, false);
        vTaskDelay(pdMS_TO_TICKS(10));
        ssd1306_display_text(&dev, 0, uptime_str, strlen(uptime_str), false);
        vTaskDelay(pdMS_TO_TICKS(10));
        ssd1306_display_text(&dev, 1, "(Bar)", 6, false);
        vTaskDelay(pdMS_TO_TICKS(10));
        ssd1306_display_text_x3(&dev, 2, buffer2, strlen(buffer2), false);
        vTaskDelay(pdMS_TO_TICKS(10));
        ssd1306_display_text(&dev, 5, buffer1, strlen(buffer1), false);
        vTaskDelay(pdMS_TO_TICKS(10));
        ssd1306_display_text(&dev, 7, error_str, strlen(error_str), false);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}
