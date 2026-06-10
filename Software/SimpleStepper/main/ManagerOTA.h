#ifndef MANAGER_OTA_H
#define MANAGER_OTA_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <cstring>

#include "esp_http_server.h"
#include "esp_app_format.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "mdns.h"

#include "driver/gpio.h"


class ManagerOTA {
public:
    explicit ManagerOTA(int ota_gpio, int led_gpio);

    esp_err_t startWebServer();
    void stopWebServer();
    void startServiceMDNS();

    bool isOtaButtonPressed();

private:
    int button_gpio;
    int led_gpio;

    static void blinkTask(void* pvParameters);

    inline static httpd_handle_t server = nullptr;

    static esp_err_t rootGetHandler(httpd_req_t *req);
    static esp_err_t updatePostHandler(httpd_req_t *req);
};

#endif
