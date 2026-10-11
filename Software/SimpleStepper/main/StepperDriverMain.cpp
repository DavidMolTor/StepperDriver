#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
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

// TMC2209 serial address definition
#define TMC2209_SERIAL_ADDRESS 0

// I2C bus pin definitions
#define I2C_SDA_GPIO CONFIG_I2C_SDA_GPIO
#define I2C_SCL_GPIO CONFIG_I2C_SCL_GPIO

// Board switches pin definitions
#define SWM_GPIO CONFIG_SWM_GPIO
#define SWC_GPIO CONFIG_SWC_GPIO
#define SWP_GPIO CONFIG_SWP_GPIO

// NEMA17 specification definitions
#define STEPS_PER_REV   200
#define CURRENT_RMS_MA  1000
#define SENSE_RESISTOR  0.1f

// Speed control definitions
#define MAX_SPEED_HZ        10.0f
#define HZ_STEPPING         1.0f


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

static void IRAM_ATTR gpioCallback(void *arg) {
    //Get the GPIO number
    uint32_t gpio_num = (uint32_t)arg;
    switch (gpio_num) {
        case SWM_GPIO:

            break;
        case SWC_GPIO:

            break;
        case SWP_GPIO:
        
            break;
    }
}

static void init_gpio()
{
    // Configure the user LED
    gpio_reset_pin(USER_LED_GPIO);
    gpio_set_direction(USER_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(USER_LED_GPIO, 1);

    // Default button configuration
    gpio_config_t io_conf = {
        .pin_bit_mask   = (1ULL << SWM_GPIO) | (1ULL << SWC_GPIO) | (1ULL << SWP_GPIO),
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_POSEDGE 
    };

    // Configure all buttons
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Set the ISR service
    gpio_install_isr_service(0);

    // Add all GPIO callbacks
    gpio_isr_handler_add((gpio_num_t)SWM_GPIO, gpioCallback, (void*)SWM_GPIO);
    gpio_isr_handler_add((gpio_num_t)SWC_GPIO, gpioCallback, (void*)SWC_GPIO);
    gpio_isr_handler_add((gpio_num_t)SWP_GPIO, gpioCallback, (void*)SWP_GPIO);
}

static void init_timers()
{

}

void stepper_run()
{
    vTaskDelay(pdMS_TO_TICKS(5000));
}

static void run_maintenance(ManagerWifi &wifi, ManagerOTA &ota)
{
    ota.startServiceMDNS();
    ota.startWebServer();

    while (true) {
        printf("OTA web server active, waiting for firmware...\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void run_app(ManagerWifi &wifi)
{
    TMC2209 stepper_driver;
    stepper_driver.setStepsPerRev(STEPS_PER_REV);
    
    stepper_driver.setGPIO(TMC2209_EN_GPIO, TMC2209_STEP_GPIO, TMC2209_DIR_GPIO);
    vTaskDelay(pdMS_TO_TICKS(100));

    stepper_driver.setup(UART_NUM_0, TMC2209_TX_GPIO, TMC2209_RX_GPIO, TMC2209_SERIAL_ADDRESS);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check if the driver is setup and working
    TMC2209::Settings settings = stepper_driver.getSettings();
    if (settings.is_setup && settings.is_communicating) {
        // Enable the stepper driver
        stepper_driver.setEnabled(true);

        // Set the initial configuration
        stepper_driver.setExternalSenseResistors(true);
        stepper_driver.setStealthChop(false);
        stepper_driver.setMicrosteps(0);

        // Set the current control configuration
        stepper_driver.setCurrentRMS(CURRENT_RMS_MA, SENSE_RESISTOR);
        stepper_driver.setAutomaticCurrentScaling(true);
        stepper_driver.setAutomaticGradientAdaptation(true);

        // Set the initial direction
        stepper_driver.setInverseDirection(false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    stepper_driver.setStepPWM(true, 1.0f);
    
    while (true) {
        printf("Main app running...\n");
        stepper_run();
    }
}

extern "C" void app_main(void)
{
    init_console();
    init_gpio();
    init_timers();

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
