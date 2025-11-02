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
#define PRESSURE_M            128.84f
#define PRESSURE_N            4.7915f

/* Static ADC calibration values for pressure sensor
    Vout,adc = Vin × (R2 / (R1 + R2))
    Voltage divider: 5V to 3.3V → Vin/Vout = 3.3/5 = 0.66.
    Vmin = 0.5V → Vadc,min = 0.5 × 0.66 = 0.33V
    Vmax = 4.5V → Vadc,max = 4.5 × 0.66 = 2.97V
*/
// Vmin,adc = 0.5V (0 MPa) --> ADCmin = 0.33V / 3.3V × 4095 ≈ 410
#define ADC_MIN               410
// Vmax,adc = 4.5V (1.2 MPa) --> ADCmax = 2.97V / 3.3V × 4095 ≈ 3687
#define ADC_MAX               3687
#define PRESSURE_MAX_MPA      1.2f

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

/*---------------------------------------------------------------
        Function Prototypes
---------------------------------------------------------------*/
static float adc_raw_to_bar(int raw);
static esp_err_t write_file(const char *path, char *data, const char* type);
static esp_err_t read_file(const char *path);

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

    //-------------ADC1 Init & Config---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    //-------------SD card init & test---------------//
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
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.",
                esp_err_to_name(ret));
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
            return;
        }
    } else {
        ESP_LOGI(TAG, "File %s already exists, skipping creation",
            file_pressure);
    }

    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");
    spi_bus_free(host.slot);

    //-------------Main loop---------------//
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        //-------------Read ADC value---------------//
        int sum = 0;
        for (int i = 0; i < 10; i++) {
            int sample = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &sample));
            sum += sample;
        }
        adc_raw = sum / 10;
        pressure_bar = adc_raw_to_bar(adc_raw);
        ESP_LOGI(TAG, "ADC Raw: %d", adc_raw);
        ESP_LOGI(TAG, "Pressure: %.2f Bar", pressure_bar);
        snprintf((char *)buffer1, sizeof(buffer1), "(Raw: %d/4095)", adc_raw);
        snprintf((char *)buffer2, sizeof(buffer2), "%.3f Bar\n", pressure_bar);

        //-------------Store data in SD card---------------//
        ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize bus.");
            return;
        }
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
            return;
        }
        ESP_LOGI(TAG, "Filesystem mounted");
        time_t now;
        time(&now);
        snprintf(buffer3, EXAMPLE_MAX_CHAR_SIZE, "%lld,%d,%.3f\n", now, adc_raw, pressure_bar);
        ret = write_file(file_pressure, buffer3, "a");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create file %s", file_pressure);
            return;
        }

        // All done, unmount partition and disable SPI peripheral
        esp_vfs_fat_sdcard_unmount(mount_point, card);
        ESP_LOGI(TAG, "Card unmounted");
        spi_bus_free(host.slot);

        //-------------Display on SSD1306---------------//
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, "Water pres (Bar):", 18, false);
        ssd1306_display_text_x3(&dev, 2, buffer2, strlen(buffer2), false);
        ssd1306_display_text(&dev, 5, buffer1, strlen(buffer1), false);

        //-------------End cycle---------------//
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000));
    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}
