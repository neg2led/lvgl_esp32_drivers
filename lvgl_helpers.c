/**
 * @file lvgl_helpers.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lvgl_helpers.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "lvgl_tft/disp_spi.h"
#include "lvgl_touch/tp_spi.h"

#include "lvgl_i2c_conf.h"
#include "lvgl_spi_conf.h"

#include "driver/i2c.h"

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
    #include "src/lv_core/lv_refr.h"
#else
    #include "lvgl/src/lv_core/lv_refr.h"
#endif

/*********************
 *      DEFINES
 *********************/

#define TAG                       "lvgl_helpers"
#define I2C_MASTER_TX_BUF_DISABLE 0 /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /* I2C master doesn't need buffer */

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/* Interface and driver initialization */
esp_err_t lvgl_driver_init(void) {
    esp_err_t err = ESP_OK;
    ESP_LOGI(TAG, "Display hor size: %d, ver size: %d", LV_HOR_RES_MAX, LV_VER_RES_MAX);
    ESP_LOGI(TAG, "Display buffer size: %d", DISP_BUF_SIZE);

#if defined(CONFIG_LV_TFT_DISPLAY_CONTROLLER_FT81X)
    ESP_LOGI(TAG, "Initializing SPI master for FT81X");

    err = lvgl_spi_driver_init(TFT_SPI_HOST,
                               DISP_SPI_MISO, DISP_SPI_MOSI, DISP_SPI_CLK,
                               SPI_BUS_MAX_TRANSFER_SZ, 1,
                               DISP_SPI_IO2, DISP_SPI_IO3);
    if (err != ESP_OK) return err;
    disp_spi_add_device(TFT_SPI_HOST);
    err = disp_driver_init();

    #if defined(CONFIG_LV_TOUCH_CONTROLLER_FT81X)
    touch_driver_init();
    #endif

    return err;
#endif

#if defined(SHARED_SPI_BUS)
    ESP_LOGI(TAG, "Initializing shared SPI master");

    err = lvgl_spi_driver_init(TFT_SPI_HOST,
                               TP_SPI_MISO, DISP_SPI_MOSI, DISP_SPI_CLK,
                               SPI_BUS_MAX_TRANSFER_SZ, 1,
                               -1, -1);
    if (err != ESP_OK) return err;

    disp_spi_add_device(TFT_SPI_HOST);
    tp_spi_add_device(TOUCH_SPI_HOST);

    err = disp_driver_init();
    touch_driver_init();

    return err;
#endif

#if defined(SHARED_I2C_BUS)
    ESP_LOGI(TAG, "Initializing shared I2C master");

    err = lvgl_i2c_driver_init(DISP_I2C_PORT,
                               DISP_I2C_SDA, DISP_I2C_SCL,
                               DISP_I2C_SPEED_HZ);
    if (err != ESP_OK) return err;

    err = disp_driver_init();
    touch_driver_init();

    return err;
#endif

/* Display controller initialization */
#if defined CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
    ESP_LOGI(TAG, "Initializing SPI master for display");

    err = lvgl_spi_driver_init(TFT_SPI_HOST,
                               DISP_SPI_MISO, DISP_SPI_MOSI, DISP_SPI_CLK,
                               SPI_BUS_MAX_TRANSFER_SZ, 1,
                               DISP_SPI_IO2, DISP_SPI_IO3);
    if (err != ESP_OK) return err;

    disp_spi_add_device(TFT_SPI_HOST);

    err = disp_driver_init();
#elif defined(CONFIG_LV_TFT_DISPLAY_PROTOCOL_I2C)
    ESP_LOGI(TAG, "Initializing I2C master for display");
    /* Init the i2c master on the display driver code */
    err = lvgl_i2c_driver_init(DISP_I2C_PORT,
                               DISP_I2C_SDA, DISP_I2C_SCL,
                               DISP_I2C_SPEED_HZ);
    if (err != ESP_OK) return err;

    err = disp_driver_init();
#else
    #error "No protocol defined for display controller"
#endif

/* Touch controller initialization */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    #if defined(CONFIG_LV_TOUCH_DRIVER_PROTOCOL_SPI)
    ESP_LOGI(TAG, "Initializing SPI master for touch");

    err = lvgl_spi_driver_init(TOUCH_SPI_HOST,
                               TP_SPI_MISO, TP_SPI_MOSI, TP_SPI_CLK,
                               0 /* Defaults to 4094 */, 2,
                               -1, -1);
    if (err != ESP_OK) return err;

    tp_spi_add_device(TOUCH_SPI_HOST);

    touch_driver_init();
    #elif defined(CONFIG_LV_TOUCH_DRIVER_PROTOCOL_I2C)
    ESP_LOGI(TAG, "Initializing I2C master for touch");

    err = lvgl_i2c_driver_init(TOUCH_I2C_PORT,
                               TOUCH_I2C_SDA, TOUCH_I2C_SCL,
                               TOUCH_I2C_SPEED_HZ);
    if (err != ESP_OK) return err;

    touch_driver_init();
    #elif defined(CONFIG_LV_TOUCH_DRIVER_ADC)
    touch_driver_init();
    #elif defined(CONFIG_LV_TOUCH_DRIVER_DISPLAY)
    touch_driver_init();
    #else
        #error "No protocol defined for touch controller"
    #endif
#else
#endif
    return err;
}

/* Config the i2c master
 *
 * This should init the i2c master to be used on display and touch controllers.
 * So we should be able to know if the display and touch controllers shares the
 * same i2c master.
 */
esp_err_t lvgl_i2c_driver_init(int port, int sda_pin, int scl_pin, int speed_hz) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing I2C master port %d...", port);
    ESP_LOGI(TAG, "SDA pin: %d, SCL pin: %d, Speed: %d (Hz)",
             sda_pin, scl_pin, speed_hz);

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda_pin,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = scl_pin,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = speed_hz,
    };

    ESP_LOGI(TAG, "Setting I2C master configuration...");
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));

    ESP_LOGI(TAG, "Installing I2C master driver...");
    err = i2c_driver_install(port,
                             I2C_MODE_MASTER,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE,
                             0 /* intr_alloc_flags */);

    return err;
}

/* Initialize spi bus master */
esp_err_t lvgl_spi_driver_init(int host,
                               int miso_pin, int mosi_pin, int sclk_pin,
                               int max_transfer_sz,
                               int dma_channel,
                               int quadwp_pin, int quadhd_pin) {
#if defined(CONFIG_IDF_TARGET_ESP32)
    assert((SPI_HOST <= host) && (VSPI_HOST >= host));
    const char *spi_names[] = {
        "SPI_HOST", "HSPI_HOST", "VSPI_HOST"};
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    assert((SPI_HOST <= host) && (HSPI_HOST >= host));
    const char *spi_names[] = {
        "SPI_HOST", "", ""};
#endif

    ESP_LOGI(TAG, "Configuring SPI host %s (%d)", spi_names[host], host);
    ESP_LOGI(TAG, "MISO pin: %d, MOSI pin: %d, SCLK pin: %d, IO2/WP pin: %d, IO3/HD pin: %d",
             miso_pin, mosi_pin, sclk_pin, quadwp_pin, quadhd_pin);

    ESP_LOGI(TAG, "Max transfer size: %d (bytes)", max_transfer_sz);

    spi_bus_config_t buscfg = {
        .miso_io_num     = miso_pin,
        .mosi_io_num     = mosi_pin,
        .sclk_io_num     = sclk_pin,
        .quadwp_io_num   = quadwp_pin,
        .quadhd_io_num   = quadhd_pin,
        .max_transfer_sz = max_transfer_sz};

    ESP_LOGI(TAG, "Initializing SPI bus...");
    esp_err_t err = spi_bus_initialize(host, &buscfg, dma_channel);

    return err;
}
