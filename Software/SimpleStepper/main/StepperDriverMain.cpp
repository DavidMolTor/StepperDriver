#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#include "sdkconfig.h"

#include "ManagerWifi.h"
#include "ManagerOTA.h"

#include "TMC2209.h"

// User accessible LED GPIO definition
#define USER_LED_GPIO (gpio_num_t)CONFIG_USER_LED_GPIO

// TMC2209 driver pin definitions
#define TMC2209_VREF_GPIO    (gpio_num_t)CONFIG_TMC2209_VREF_GPIO
#define TMC2209_STEP_GPIO    (gpio_num_t)CONFIG_TMC2209_STEP_GPIO
#define TMC2209_DIR_GPIO     (gpio_num_t)CONFIG_TMC2209_DIR_GPIO
#define TMC2209_EN_GPIO      (gpio_num_t)CONFIG_TMC2209_EN_GPIO
#define TMC2209_RX_GPIO      (gpio_num_t)CONFIG_TMC2209_RX_GPIO
#define TMC2209_TX_GPIO      (gpio_num_t)CONFIG_TMC2209_TX_GPIO
#define TMC2209_DIAG_GPIO    (gpio_num_t)CONFIG_TMC2209_DIAG_GPIO

// I2C bus pin definitions
#define I2C_SDA_GPIO CONFIG_I2C_SDA_GPIO
#define I2C_SCL_GPIO CONFIG_I2C_SCL_GPIO

// Board switches pin definitions
#define SWM_GPIO CONFIG_SWM_GPIO
#define SWC_GPIO CONFIG_SWC_GPIO
#define SWP_GPIO CONFIG_SWP_GPIO

static void init_console()
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));

    usb_serial_jtag_vfs_use_driver();

    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

static void init_gpio()
{
    // Configure the user LED
    gpio_reset_pin(USER_LED_GPIO);
    gpio_set_direction(USER_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(USER_LED_GPIO, 1);

    // Default button configuration
    gpio_config_t io_conf = {
        .pin_bit_mask   = 1ULL,
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };

    // Configure each button
    io_conf.pin_bit_mask = 1ULL << CONFIG_SWM_GPIO;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    io_conf.pin_bit_mask = 1ULL << CONFIG_SWC_GPIO;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    io_conf.pin_bit_mask = 1ULL << CONFIG_SWP_GPIO;
    ESP_ERROR_CHECK(gpio_config(&io_conf));    
}

static void run_maintenance(ManagerWifi &wifi, ManagerOTA &ota)
{
    ota.startServiceMDNS();
    ota.startWebServer();

    while (true) {
        printf("OTA web server active, waiting for firmware...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void run_app(ManagerWifi &wifi)
{
    ManagerWifi::Credentials credentials = wifi.loadCredentials();

    // Connect to the given wifi network
    if (credentials.valid() && !wifi.connect(credentials)) {
        printf("Unable to connect to WiFi, running application...\n");
    }

    TMC2209 tmc2209;
    tmc2209.setup(UART_NUM_0, TMC2209_TX_GPIO, TMC2209_RX_GPIO);
    tmc2209.setGPIO(TMC2209_EN_GPIO, TMC2209_STEP_GPIO, TMC2209_DIR_GPIO);

    while (true) {
        printf("Main app running...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    init_console();
    init_gpio();

    // Manager definitions
    ManagerWifi managerWifi;
    ManagerOTA managerOTA(SWC_GPIO, USER_LED_GPIO);

    // Check for OTA button
    bool ota_mode = managerOTA.isOtaButtonPressed();

    // Initialize the wifi
    managerWifi.init();

    // Check for saved credentials
    ManagerWifi::Credentials credentials;
    do
    {
        if (!managerWifi.hasSavedCredentials()) {
            credentials = managerWifi.serialProvisioning();
        } else {
            credentials = managerWifi.loadCredentials();
        }
    } while (!credentials.valid());

    // Connect to the given wifi network
    if (!managerWifi.connect(credentials) && ota_mode) {
        do
        {
            credentials = managerWifi.serialProvisioning();
        } while (!managerWifi.connect(credentials));
    }
    
    // Run OTA mode if requested
    if (ota_mode) {
        run_maintenance(managerWifi, managerOTA);
    } else {
        run_app(managerWifi);
    }
}
