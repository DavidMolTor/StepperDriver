#ifndef SIMPLE_STEPPER_WIFI_H
#define SIMPLE_STEPPER_WIFI_H

#include <string>

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "nvs_flash.h"
#include "nvs.h"

class ManagerWifi {
public:
    struct Credentials {
        std::string ssid;
        std::string password;

        bool valid() const { return !ssid.empty(); }
    };

    ManagerWifi();

    void init();
    bool hasSavedCredentials();

    Credentials loadCredentials();
    void saveCredentials(const Credentials& creds);
    void clearCredentials();

    Credentials serialProvisioning();
    bool connect(const Credentials& creds);

private:
    void initNvs();
    std::string readLine(const char* prompt, bool allow_empty = false);

    static void blinkTask(void* pvParameters);
};

#endif
