#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// User accessible LED GPIO definition
#define USER_LED_GPIO CONFIG_USER_LED_GPIO

// TMC2209 driver pin definitions
#define TMC2209_VREF_GPIO    CONFIG_TMC2209_VREF_GPIO
#define TMC2209_STEP_GPIO    CONFIG_TMC2209_STEP_GPIO
#define TMC2209_DIR_GPIO     CONFIG_TMC2209_DIR_GPIO
#define TMC2209_EN_GPIO      CONFIG_TMC2209_EN_GPIO
#define TMC2209_RX_GPIO      CONFIG_TMC2209_RX_GPIO
#define TMC2209_TX_GPIO      CONFIG_TMC2209_TX_GPIO
#define TMC2209_DIAG_GPIO    CONFIG_TMC2209_DIAG_GPIO

// I2C bus pin definitions
#define I2C_SDA_GPIO CONFIG_I2C_SDA_GPIO
#define I2C_SCL_GPIO CONFIG_I2C_SCL_GPIO

// Board switches pin definitions
#define SWM_GPIO CONFIG_SWM_GPIO
#define SWC_GPIO CONFIG_SWC_GPIO
#define SWP_GPIO CONFIG_SWP_GPIO

static void setup()
{
    
}

void app_main()
{
    setup();

    while (1) {

        printf("\n -- PROJECT DEFINITIONS -- \n");
        printf("\n");

        printf("USER_LED_GPIO: %d\n", USER_LED_GPIO);
        printf("\n");

        printf("TMC2209_VREF_GPIO: %d\n", TMC2209_VREF_GPIO);
        printf("TMC2209_STEP_GPIO: %d\n", TMC2209_STEP_GPIO);
        printf("TMC2209_DIR_GPIO: %d\n", TMC2209_DIR_GPIO);
        printf("TMC2209_EN_GPIO: %d\n", TMC2209_EN_GPIO);
        printf("TMC2209_RX_GPIO: %d\n", TMC2209_RX_GPIO);
        printf("TMC2209_TX_GPIO: %d\n", TMC2209_TX_GPIO);
        printf("TMC2209_DIAG_GPIO: %d\n", TMC2209_DIAG_GPIO);
        printf("\n");

        printf("I2C_SDA_GPIO: %d\n", I2C_SDA_GPIO);
        printf("I2C_SCL_GPIO: %d\n", I2C_SCL_GPIO);
        printf("\n");

        printf("SWM_GPIO: %d\n", SWM_GPIO);
        printf("SWC_GPIO: %d\n", SWC_GPIO);
        printf("SWP_GPIO: %d\n", SWP_GPIO);
        printf("\n");

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
